#include "cartographer/serialization/swmap.h"

#include <cstring>
#include <filesystem>
#include <set>

#include "cartographer/backend/trajectory_backend_2d.h"
#include "cartographer/mapping/submap_2d.h"
#include "glog/logging.h"
#include "sqlite3.h"

namespace cartographer {
namespace io {
namespace {

constexpr int kApplicationId = 0x53574d50;  // SWMP
constexpr int kSchemaVersion = 4;

void Check(int result, sqlite3* db, const char* operation) {
  CHECK(result == SQLITE_OK || result == SQLITE_DONE || result == SQLITE_ROW)
      << operation << ": " << (db ? sqlite3_errmsg(db) : "no database");
}

void Exec(sqlite3* db, const char* sql) {
  char* error = nullptr;
  const int result = sqlite3_exec(db, sql, nullptr, nullptr, &error);
  if (result != SQLITE_OK) {
    const std::string message = error ? error : "unknown SQLite error";
    sqlite3_free(error);
    LOG(FATAL) << message;
  }
}

class Statement {
 public:
  Statement(sqlite3* db, const char* sql) : db_(db) {
    Check(sqlite3_prepare_v2(db, sql, -1, &statement_, nullptr), db, "prepare");
  }
  ~Statement() { sqlite3_finalize(statement_); }
  sqlite3_stmt* get() { return statement_; }
  void Run() {
    Check(sqlite3_step(statement_), db_, "step");
    Check(sqlite3_reset(statement_), db_, "reset");
    Check(sqlite3_clear_bindings(statement_), db_, "clear bindings");
  }

 private:
  sqlite3* db_;
  sqlite3_stmt* statement_ = nullptr;
};

void AppendU32(std::string* out, uint32 value) {
  for (int i = 0; i != 4; ++i) out->push_back((value >> (8 * i)) & 0xff);
}
void AppendFloat(std::string* out, float value) {
  uint32 bits;
  std::memcpy(&bits, &value, sizeof(bits));
  AppendU32(out, bits);
}
uint32 ReadU32(const std::string& data, size_t* offset) {
  CHECK_LE(*offset + 4, data.size());
  uint32 value = 0;
  for (int i = 0; i != 4; ++i)
    value |= static_cast<uint32>(static_cast<uint8>(data[(*offset)++])) << (8 * i);
  return value;
}
float ReadFloat(const std::string& data, size_t* offset) {
  const uint32 bits = ReadU32(data, offset);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

std::string EncodeCells(const std::vector<uint16>& cells) {
  std::string result;
  result.reserve(cells.size() * 2);
  for (uint16 value : cells) {
    result.push_back(value & 0xff);
    result.push_back(value >> 8);
  }
  return result;
}
std::vector<uint16> DecodeCells(const void* bytes, int size) {
  CHECK_EQ(size % 2, 0);
  const auto* data = static_cast<const uint8*>(bytes);
  std::vector<uint16> result(size / 2);
  for (int i = 0; i != size / 2; ++i)
    result[i] = data[2 * i] | (static_cast<uint16>(data[2 * i + 1]) << 8);
  return result;
}

std::string EncodePointCloud(const sensor::PointCloud& cloud) {
  std::string result;
  AppendU32(&result, cloud.size());
  for (const auto& point : cloud.points()) {
    AppendFloat(&result, point.position.x());
    AppendFloat(&result, point.position.y());
    AppendFloat(&result, point.position.z());
  }
  AppendU32(&result, cloud.intensities().size());
  for (float intensity : cloud.intensities()) AppendFloat(&result, intensity);
  return result;
}
sensor::PointCloud DecodePointCloud(const void* bytes, int size) {
  const std::string data(static_cast<const char*>(bytes), size);
  size_t offset = 0;
  std::vector<sensor::RangefinderPoint> points;
  const uint32 point_count = ReadU32(data, &offset);
  points.reserve(point_count);
  for (size_t i = 0; i != point_count; ++i) {
    points.push_back({Eigen::Vector3f(ReadFloat(data, &offset),
                                     ReadFloat(data, &offset),
                                     ReadFloat(data, &offset))});
  }
  std::vector<float> intensities;
  const uint32 intensity_count = ReadU32(data, &offset);
  intensities.reserve(intensity_count);
  for (uint32 i = 0; i != intensity_count; ++i)
    intensities.push_back(ReadFloat(data, &offset));
  CHECK_EQ(offset, data.size());
  return sensor::PointCloud(std::move(points), std::move(intensities));
}

void BindPose(sqlite3_stmt* s, int first, const transform::Rigid2d& pose) {
  const auto& t = pose.translation();
  sqlite3_bind_double(s, first, t.x());
  sqlite3_bind_double(s, first + 1, t.y());
  sqlite3_bind_double(s, first + 2, pose.rotation().angle());
}
transform::Rigid2d ReadPose(sqlite3_stmt* s, int first) {
  return transform::Rigid2d(
      Eigen::Vector2d(sqlite3_column_double(s, first),
                      sqlite3_column_double(s, first + 1)),
      sqlite3_column_double(s, first + 2));
}
void BindBlob(sqlite3_stmt* s, int index, const std::string& value) {
  Check(sqlite3_bind_blob(s, index, value.data(), value.size(), SQLITE_TRANSIENT),
        sqlite3_db_handle(s), "bind blob");
}

sqlite3* Open(const std::string& filename, int flags) {
  sqlite3* db = nullptr;
  const int result = sqlite3_open_v2(filename.c_str(), &db, flags, nullptr);
  Check(result, db, "open swmap");
  return db;
}

void Validate(sqlite3* db) {
  Statement app(db, "PRAGMA application_id");
  CHECK_EQ(sqlite3_step(app.get()), SQLITE_ROW);
  CHECK_EQ(sqlite3_column_int(app.get(), 0), kApplicationId);
  Statement version(db, "PRAGMA user_version");
  CHECK_EQ(sqlite3_step(version.get()), SQLITE_ROW);
  CHECK_EQ(sqlite3_column_int(version.get(), 0), kSchemaVersion)
      << "Unsupported .swmap schema";
}

}  // namespace

bool WriteSwMap(const std::string& filename, const mapping::TrajectoryBackend2D& pose_graph,
                bool include_unfinished_submaps) {
  std::filesystem::remove(filename);
  sqlite3* db = Open(filename, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE);
  Exec(db, "PRAGMA application_id=1398230352; PRAGMA user_version=4;"
           "PRAGMA journal_mode=DELETE; BEGIN IMMEDIATE;"
           "CREATE TABLE metadata(key TEXT PRIMARY KEY,value TEXT NOT NULL);"
           "INSERT INTO metadata VALUES('format','sweepnav_2d_map');"
           "INSERT INTO metadata VALUES('schema_version','4');"
           "CREATE TABLE trajectories(trajectory_id INTEGER PRIMARY KEY);"
           "CREATE TABLE submaps(trajectory_id INTEGER,submap_index INTEGER,"
           "global_x REAL,global_y REAL,global_yaw REAL,"
           "local_x REAL,local_y REAL,local_yaw REAL,"
           "num_range_data INTEGER,finished INTEGER,resolution REAL,max_x REAL,max_y REAL,"
           "num_x INTEGER,num_y INTEGER,min_cost REAL,max_cost REAL,"
           "box_min_x INTEGER,box_min_y INTEGER,box_max_x INTEGER,box_max_y INTEGER,cells BLOB,"
           "PRIMARY KEY(trajectory_id,submap_index));"
           "CREATE TABLE nodes(trajectory_id INTEGER,node_index INTEGER,time INTEGER,"
           "global_x REAL,global_y REAL,global_yaw REAL,"
           "local_x REAL,local_y REAL,local_yaw REAL,filtered BLOB,"
           "PRIMARY KEY(trajectory_id,node_index));"
           "CREATE TABLE constraints(sequence INTEGER PRIMARY KEY,submap_trajectory_id INTEGER,"
           "submap_index INTEGER,node_trajectory_id INTEGER,node_index INTEGER,"
           "x REAL,y REAL,yaw REAL,"
           "translation_weight REAL,rotation_weight REAL,tag INTEGER);"
           "");

  std::set<int> trajectory_ids;
  Statement submap(db, "INSERT INTO submaps VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)");
  std::set<mapping::SubmapId> included_submaps;
  for (const auto& item : pose_graph.GetAllSubmapData()) {
    const auto* value = dynamic_cast<const mapping::Submap2D*>(item.data.submap.get());
    CHECK(value != nullptr);
    if (!include_unfinished_submaps && !value->insertion_finished()) continue;
    trajectory_ids.insert(item.id.trajectory_id);
    included_submaps.insert(item.id);
    auto* s = submap.get();
    sqlite3_bind_int(s, 1, item.id.trajectory_id); sqlite3_bind_int(s, 2, item.id.submap_index);
    BindPose(s, 3, item.data.pose); BindPose(s, 6, value->local_pose());
    sqlite3_bind_int(s, 9, value->num_range_data()); sqlite3_bind_int(s, 10, value->insertion_finished());
    const auto* grid = value->grid(); CHECK(grid != nullptr);
    sqlite3_bind_double(s, 11, grid->limits().resolution());
    sqlite3_bind_double(s, 12, grid->limits().max().x()); sqlite3_bind_double(s, 13, grid->limits().max().y());
    sqlite3_bind_int(s, 14, grid->limits().cell_limits().num_x_cells); sqlite3_bind_int(s, 15, grid->limits().cell_limits().num_y_cells);
    sqlite3_bind_double(s, 16, grid->GetMinCorrespondenceCost()); sqlite3_bind_double(s, 17, grid->GetMaxCorrespondenceCost());
    const auto& box = grid->known_cells_box_for_serialization();
    sqlite3_bind_int(s, 18, box.isEmpty() ? 0 : box.min().x()); sqlite3_bind_int(s, 19, box.isEmpty() ? 0 : box.min().y());
    sqlite3_bind_int(s, 20, box.isEmpty() ? -1 : box.max().x()); sqlite3_bind_int(s, 21, box.isEmpty() ? -1 : box.max().y());
    BindBlob(s, 22, EncodeCells(grid->cells_for_serialization())); submap.Run();
  }

  Statement node(db, "INSERT INTO nodes VALUES(?,?,?,?,?,?,?,?,?,?)");
  for (const auto& item : pose_graph.GetTrajectoryNodes()) {
    trajectory_ids.insert(item.id.trajectory_id);
    auto* s = node.get(); const auto& d = *item.data.constant_data;
    sqlite3_bind_int(s, 1, item.id.trajectory_id); sqlite3_bind_int(s, 2, item.id.node_index);
    sqlite3_bind_int64(s, 3, common::ToUniversal(d.time)); BindPose(s, 4, item.data.global_pose); BindPose(s, 7, d.local_pose);
    BindBlob(s, 10, EncodePointCloud(d.filtered_point_cloud)); node.Run();
  }

  Statement constraint(db, "INSERT INTO constraints VALUES(?,?,?,?,?,?,?,?,?,?,?)");
  int sequence = 0;
  for (const auto& value : pose_graph.constraints()) {
    if (!included_submaps.count(value.submap_id)) continue;
    auto* s = constraint.get(); sqlite3_bind_int(s, 1, sequence++);
    sqlite3_bind_int(s, 2, value.submap_id.trajectory_id); sqlite3_bind_int(s, 3, value.submap_id.submap_index);
    sqlite3_bind_int(s, 4, value.node_id.trajectory_id); sqlite3_bind_int(s, 5, value.node_id.node_index);
    BindPose(s, 6, value.pose.zbar_ij); sqlite3_bind_double(s, 9, value.pose.translation_weight);
    sqlite3_bind_double(s, 10, value.pose.rotation_weight); sqlite3_bind_int(s, 11, value.tag); constraint.Run();
  }

  Statement trajectory(db, "INSERT INTO trajectories VALUES(?)");
  for (int id : trajectory_ids) { sqlite3_bind_int(trajectory.get(), 1, id); trajectory.Run(); }
  Exec(db, "COMMIT;");
  return sqlite3_close_v2(db) == SQLITE_OK;
}

SerializedState ReadSwMap(const std::string& filename) {
  sqlite3* db = Open(filename, SQLITE_OPEN_READONLY); Validate(db);
  SerializedState result;
  Statement trajectories(db, "SELECT trajectory_id FROM trajectories ORDER BY trajectory_id");
  while (sqlite3_step(trajectories.get()) == SQLITE_ROW) result.trajectory_ids.push_back(sqlite3_column_int(trajectories.get(), 0));
  Statement submaps(db, "SELECT * FROM submaps ORDER BY trajectory_id,submap_index");
  while (sqlite3_step(submaps.get()) == SQLITE_ROW) {
    auto* s = submaps.get(); SerializedSubmap2D value;
    value.id = {sqlite3_column_int(s, 0), sqlite3_column_int(s, 1)}; value.global_pose = ReadPose(s, 2); value.local_pose = ReadPose(s, 5);
    value.num_range_data = sqlite3_column_int(s, 8); value.finished = sqlite3_column_int(s, 9);
    value.grid.resolution = sqlite3_column_double(s, 10); value.grid.max = {sqlite3_column_double(s, 11), sqlite3_column_double(s, 12)};
    value.grid.cell_limits = {sqlite3_column_int(s, 13), sqlite3_column_int(s, 14)};
    value.grid.min_correspondence_cost = sqlite3_column_double(s, 15); value.grid.max_correspondence_cost = sqlite3_column_double(s, 16);
    const Eigen::Vector2i box_min(sqlite3_column_int(s, 17), sqlite3_column_int(s, 18)); const Eigen::Vector2i box_max(sqlite3_column_int(s, 19), sqlite3_column_int(s, 20));
    if ((box_min.array() <= box_max.array()).all()) value.grid.known_cells_box = Eigen::AlignedBox2i(box_min, box_max);
    value.grid.cells = DecodeCells(sqlite3_column_blob(s, 21), sqlite3_column_bytes(s, 21)); result.submaps.push_back(std::move(value));
  }
  Statement nodes(db, "SELECT * FROM nodes ORDER BY trajectory_id,node_index");
  while (sqlite3_step(nodes.get()) == SQLITE_ROW) {
    auto* s = nodes.get(); SerializedNode value;
    value.id = {sqlite3_column_int(s, 0), sqlite3_column_int(s, 1)}; value.data.time = common::FromUniversal(sqlite3_column_int64(s, 2));
    value.global_pose = ReadPose(s, 3); value.data.local_pose = ReadPose(s, 6);
    value.data.filtered_point_cloud = DecodePointCloud(sqlite3_column_blob(s, 9), sqlite3_column_bytes(s, 9)); result.nodes.push_back(std::move(value));
  }
  Statement constraints(db, "SELECT * FROM constraints ORDER BY sequence");
  while (sqlite3_step(constraints.get()) == SQLITE_ROW) {
    auto* s = constraints.get(); result.constraints.push_back({
      {sqlite3_column_int(s, 1), sqlite3_column_int(s, 2)}, {sqlite3_column_int(s, 3), sqlite3_column_int(s, 4)},
      {ReadPose(s, 5), sqlite3_column_double(s, 8), sqlite3_column_double(s, 9)},
      static_cast<mapping::Constraint::Tag>(sqlite3_column_int(s, 10))});
  }
  CHECK_EQ(sqlite3_close_v2(db), SQLITE_OK); return result;
}

}  // namespace io
}  // namespace cartographer
