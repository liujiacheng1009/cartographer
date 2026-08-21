#include "cartographer/state/map_database.h"

#include <cstring>
#include <filesystem>
#include <set>

#include "cartographer/slam/pose_graph.h"
#include "cartographer/slam/submap_2d.h"
#include "glog/logging.h"
#include "sqlite3.h"

namespace cartographer {
namespace io {
namespace {

constexpr int kApplicationId = 0x53574d50;  // SWMP
constexpr int kSchemaVersion = 2;

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

std::string EncodeVector(const Eigen::VectorXf& values) {
  std::string result;
  AppendU32(&result, values.size());
  for (int i = 0; i != values.size(); ++i) AppendFloat(&result, values[i]);
  return result;
}
Eigen::VectorXf DecodeVector(const void* bytes, int size) {
  const std::string data(static_cast<const char*>(bytes), size);
  size_t offset = 0;
  Eigen::VectorXf result(ReadU32(data, &offset));
  for (int i = 0; i != result.size(); ++i) result[i] = ReadFloat(data, &offset);
  CHECK_EQ(offset, data.size());
  return result;
}

void BindPose(sqlite3_stmt* s, int first, const transform::Rigid3d& pose) {
  const auto& t = pose.translation();
  const auto& q = pose.rotation();
  sqlite3_bind_double(s, first, t.x());
  sqlite3_bind_double(s, first + 1, t.y());
  sqlite3_bind_double(s, first + 2, t.z());
  sqlite3_bind_double(s, first + 3, q.w());
  sqlite3_bind_double(s, first + 4, q.x());
  sqlite3_bind_double(s, first + 5, q.y());
  sqlite3_bind_double(s, first + 6, q.z());
}
transform::Rigid3d ReadPose(sqlite3_stmt* s, int first) {
  return transform::Rigid3d(
      Eigen::Vector3d(sqlite3_column_double(s, first),
                      sqlite3_column_double(s, first + 1),
                      sqlite3_column_double(s, first + 2)),
      Eigen::Quaterniond(sqlite3_column_double(s, first + 3),
                         sqlite3_column_double(s, first + 4),
                         sqlite3_column_double(s, first + 5),
                         sqlite3_column_double(s, first + 6)));
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

bool WriteSwMap(const std::string& filename, const mapping::PoseGraph& pose_graph,
                bool include_unfinished_submaps) {
  std::filesystem::remove(filename);
  sqlite3* db = Open(filename, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE);
  Exec(db, "PRAGMA application_id=1398230352; PRAGMA user_version=2;"
           "PRAGMA journal_mode=DELETE; BEGIN IMMEDIATE;"
           "CREATE TABLE metadata(key TEXT PRIMARY KEY,value TEXT NOT NULL);"
           "INSERT INTO metadata VALUES('format','sweepnav_2d_map');"
           "INSERT INTO metadata VALUES('schema_version','2');"
           "CREATE TABLE trajectories(trajectory_id INTEGER PRIMARY KEY);"
           "CREATE TABLE submaps(trajectory_id INTEGER,submap_index INTEGER,"
           "gtx REAL,gty REAL,gtz REAL,gqw REAL,gqx REAL,gqy REAL,gqz REAL,"
           "ltx REAL,lty REAL,ltz REAL,lqw REAL,lqx REAL,lqy REAL,lqz REAL,"
           "num_range_data INTEGER,finished INTEGER,resolution REAL,max_x REAL,max_y REAL,"
           "num_x INTEGER,num_y INTEGER,min_cost REAL,max_cost REAL,"
           "box_min_x INTEGER,box_min_y INTEGER,box_max_x INTEGER,box_max_y INTEGER,cells BLOB,"
           "PRIMARY KEY(trajectory_id,submap_index));"
           "CREATE TABLE nodes(trajectory_id INTEGER,node_index INTEGER,time INTEGER,"
           "gtx REAL,gty REAL,gtz REAL,gqw REAL,gqx REAL,gqy REAL,gqz REAL,"
           "ltx REAL,lty REAL,ltz REAL,lqw REAL,lqx REAL,lqy REAL,lqz REAL,"
           "aqw REAL,aqx REAL,aqy REAL,aqz REAL,filtered BLOB,high_resolution BLOB,"
           "low_resolution BLOB,histogram BLOB,PRIMARY KEY(trajectory_id,node_index));"
           "CREATE TABLE constraints(sequence INTEGER PRIMARY KEY,submap_trajectory_id INTEGER,"
           "submap_index INTEGER,node_trajectory_id INTEGER,node_index INTEGER,"
           "tx REAL,ty REAL,tz REAL,qw REAL,qx REAL,qy REAL,qz REAL,"
           "translation_weight REAL,rotation_weight REAL,tag INTEGER);"
           "CREATE TABLE trajectory_data(trajectory_id INTEGER PRIMARY KEY,gravity_constant REAL,"
           "iqw REAL,iqx REAL,iqy REAL,iqz REAL,has_origin INTEGER,"
           "tx REAL,ty REAL,tz REAL,qw REAL,qx REAL,qy REAL,qz REAL);"
           "CREATE TABLE landmarks(landmark_id TEXT PRIMARY KEY,tx REAL,ty REAL,tz REAL,"
           "qw REAL,qx REAL,qy REAL,qz REAL);");

  std::set<int> trajectory_ids;
  Statement submap(db, "INSERT INTO submaps VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)");
  std::set<mapping::SubmapId> included_submaps;
  for (const auto& item : pose_graph.GetAllSubmapData()) {
    const auto* value = dynamic_cast<const mapping::Submap2D*>(item.data.submap.get());
    CHECK(value != nullptr);
    if (!include_unfinished_submaps && !value->insertion_finished()) continue;
    trajectory_ids.insert(item.id.trajectory_id);
    included_submaps.insert(item.id);
    auto* s = submap.get();
    sqlite3_bind_int(s, 1, item.id.trajectory_id); sqlite3_bind_int(s, 2, item.id.submap_index);
    BindPose(s, 3, item.data.pose); BindPose(s, 10, value->local_pose());
    sqlite3_bind_int(s, 17, value->num_range_data()); sqlite3_bind_int(s, 18, value->insertion_finished());
    const auto* grid = value->grid(); CHECK(grid != nullptr);
    sqlite3_bind_double(s, 19, grid->limits().resolution());
    sqlite3_bind_double(s, 20, grid->limits().max().x()); sqlite3_bind_double(s, 21, grid->limits().max().y());
    sqlite3_bind_int(s, 22, grid->limits().cell_limits().num_x_cells); sqlite3_bind_int(s, 23, grid->limits().cell_limits().num_y_cells);
    sqlite3_bind_double(s, 24, grid->GetMinCorrespondenceCost()); sqlite3_bind_double(s, 25, grid->GetMaxCorrespondenceCost());
    const auto& box = grid->known_cells_box_for_serialization();
    sqlite3_bind_int(s, 26, box.isEmpty() ? 0 : box.min().x()); sqlite3_bind_int(s, 27, box.isEmpty() ? 0 : box.min().y());
    sqlite3_bind_int(s, 28, box.isEmpty() ? -1 : box.max().x()); sqlite3_bind_int(s, 29, box.isEmpty() ? -1 : box.max().y());
    BindBlob(s, 30, EncodeCells(grid->cells_for_serialization())); submap.Run();
  }

  Statement node(db, "INSERT INTO nodes VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)");
  for (const auto& item : pose_graph.GetTrajectoryNodes()) {
    trajectory_ids.insert(item.id.trajectory_id);
    auto* s = node.get(); const auto& d = *item.data.constant_data;
    sqlite3_bind_int(s, 1, item.id.trajectory_id); sqlite3_bind_int(s, 2, item.id.node_index);
    sqlite3_bind_int64(s, 3, common::ToUniversal(d.time)); BindPose(s, 4, item.data.global_pose); BindPose(s, 11, d.local_pose);
    sqlite3_bind_double(s, 18, d.gravity_alignment.w()); sqlite3_bind_double(s, 19, d.gravity_alignment.x());
    sqlite3_bind_double(s, 20, d.gravity_alignment.y()); sqlite3_bind_double(s, 21, d.gravity_alignment.z());
    BindBlob(s, 22, EncodePointCloud(d.filtered_gravity_aligned_point_cloud));
    BindBlob(s, 23, EncodePointCloud(d.high_resolution_point_cloud)); BindBlob(s, 24, EncodePointCloud(d.low_resolution_point_cloud));
    BindBlob(s, 25, EncodeVector(d.rotational_scan_matcher_histogram)); node.Run();
  }

  Statement constraint(db, "INSERT INTO constraints VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)");
  int sequence = 0;
  for (const auto& value : pose_graph.constraints()) {
    if (!included_submaps.count(value.submap_id)) continue;
    auto* s = constraint.get(); sqlite3_bind_int(s, 1, sequence++);
    sqlite3_bind_int(s, 2, value.submap_id.trajectory_id); sqlite3_bind_int(s, 3, value.submap_id.submap_index);
    sqlite3_bind_int(s, 4, value.node_id.trajectory_id); sqlite3_bind_int(s, 5, value.node_id.node_index);
    BindPose(s, 6, value.pose.zbar_ij); sqlite3_bind_double(s, 13, value.pose.translation_weight);
    sqlite3_bind_double(s, 14, value.pose.rotation_weight); sqlite3_bind_int(s, 15, value.tag); constraint.Run();
  }

  Statement trajectory_data(db, "INSERT INTO trajectory_data VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)");
  for (const auto& item : pose_graph.GetTrajectoryData()) {
    trajectory_ids.insert(item.first); auto* s = trajectory_data.get();
    sqlite3_bind_int(s, 1, item.first); sqlite3_bind_double(s, 2, item.second.gravity_constant);
    for (int i = 0; i != 4; ++i) sqlite3_bind_double(s, 3 + i, item.second.imu_calibration[i]);
    sqlite3_bind_int(s, 7, item.second.fixed_frame_origin_in_map.has_value());
    if (item.second.fixed_frame_origin_in_map) BindPose(s, 8, *item.second.fixed_frame_origin_in_map);
    trajectory_data.Run();
  }
  Statement landmark(db, "INSERT INTO landmarks VALUES(?,?,?,?,?,?,?,?)");
  for (const auto& item : pose_graph.GetLandmarkPoses()) {
    sqlite3_bind_text(landmark.get(), 1, item.first.c_str(), -1, SQLITE_TRANSIENT);
    BindPose(landmark.get(), 2, item.second); landmark.Run();
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
    value.id = {sqlite3_column_int(s, 0), sqlite3_column_int(s, 1)}; value.global_pose = ReadPose(s, 2); value.local_pose = ReadPose(s, 9);
    value.num_range_data = sqlite3_column_int(s, 16); value.finished = sqlite3_column_int(s, 17);
    value.grid.resolution = sqlite3_column_double(s, 18); value.grid.max = {sqlite3_column_double(s, 19), sqlite3_column_double(s, 20)};
    value.grid.cell_limits = {sqlite3_column_int(s, 21), sqlite3_column_int(s, 22)};
    value.grid.min_correspondence_cost = sqlite3_column_double(s, 23); value.grid.max_correspondence_cost = sqlite3_column_double(s, 24);
    const Eigen::Vector2i box_min(sqlite3_column_int(s, 25), sqlite3_column_int(s, 26)); const Eigen::Vector2i box_max(sqlite3_column_int(s, 27), sqlite3_column_int(s, 28));
    if ((box_min.array() <= box_max.array()).all()) value.grid.known_cells_box = Eigen::AlignedBox2i(box_min, box_max);
    value.grid.cells = DecodeCells(sqlite3_column_blob(s, 29), sqlite3_column_bytes(s, 29)); result.submaps.push_back(std::move(value));
  }
  Statement nodes(db, "SELECT * FROM nodes ORDER BY trajectory_id,node_index");
  while (sqlite3_step(nodes.get()) == SQLITE_ROW) {
    auto* s = nodes.get(); SerializedNode value;
    value.id = {sqlite3_column_int(s, 0), sqlite3_column_int(s, 1)}; value.data.time = common::FromUniversal(sqlite3_column_int64(s, 2));
    value.global_pose = ReadPose(s, 3); value.data.local_pose = ReadPose(s, 10);
    value.data.gravity_alignment = Eigen::Quaterniond(sqlite3_column_double(s, 17), sqlite3_column_double(s, 18), sqlite3_column_double(s, 19), sqlite3_column_double(s, 20));
    value.data.filtered_gravity_aligned_point_cloud = DecodePointCloud(sqlite3_column_blob(s, 21), sqlite3_column_bytes(s, 21));
    value.data.high_resolution_point_cloud = DecodePointCloud(sqlite3_column_blob(s, 22), sqlite3_column_bytes(s, 22));
    value.data.low_resolution_point_cloud = DecodePointCloud(sqlite3_column_blob(s, 23), sqlite3_column_bytes(s, 23));
    value.data.rotational_scan_matcher_histogram = DecodeVector(sqlite3_column_blob(s, 24), sqlite3_column_bytes(s, 24)); result.nodes.push_back(std::move(value));
  }
  Statement constraints(db, "SELECT * FROM constraints ORDER BY sequence");
  while (sqlite3_step(constraints.get()) == SQLITE_ROW) {
    auto* s = constraints.get(); result.constraints.push_back({
      {sqlite3_column_int(s, 1), sqlite3_column_int(s, 2)}, {sqlite3_column_int(s, 3), sqlite3_column_int(s, 4)},
      {ReadPose(s, 5), sqlite3_column_double(s, 12), sqlite3_column_double(s, 13)},
      static_cast<mapping::PoseGraphInterface::Constraint::Tag>(sqlite3_column_int(s, 14))});
  }
  Statement trajectory_data(db, "SELECT * FROM trajectory_data ORDER BY trajectory_id");
  while (sqlite3_step(trajectory_data.get()) == SQLITE_ROW) {
    auto* s = trajectory_data.get(); mapping::PoseGraphInterface::TrajectoryData value; value.gravity_constant = sqlite3_column_double(s, 1);
    for (int i = 0; i != 4; ++i) value.imu_calibration[i] = sqlite3_column_double(s, 2 + i);
    if (sqlite3_column_int(s, 6)) value.fixed_frame_origin_in_map = ReadPose(s, 7);
    result.trajectory_data.emplace(sqlite3_column_int(s, 0), value);
  }
  Statement landmarks(db, "SELECT * FROM landmarks ORDER BY landmark_id");
  while (sqlite3_step(landmarks.get()) == SQLITE_ROW)
    result.landmark_poses.emplace(reinterpret_cast<const char*>(sqlite3_column_text(landmarks.get(), 0)), ReadPose(landmarks.get(), 1));
  CHECK_EQ(sqlite3_close_v2(db), SQLITE_OK); return result;
}

}  // namespace io
}  // namespace cartographer
