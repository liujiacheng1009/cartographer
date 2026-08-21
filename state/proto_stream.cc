/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/state/proto_stream.h"

#include <filesystem>

#include "glog/logging.h"
#include "sqlite3.h"

namespace cartographer {
namespace io {

namespace {

constexpr int kApplicationId = 0x53574d50;  // "SWMP"
constexpr int kSchemaVersion = 1;

uint64 Checksum(const std::string& data) {
  uint64 value = 1469598103934665603ULL;
  for (const unsigned char byte : data) {
    value ^= byte;
    value *= 1099511628211ULL;
  }
  return value;
}

void CheckSqlite(const int result, sqlite3* database, const char* operation) {
  CHECK(result == SQLITE_OK || result == SQLITE_DONE || result == SQLITE_ROW)
      << operation << ": " << sqlite3_errmsg(database);
}

void Execute(sqlite3* database, const char* sql) {
  char* error = nullptr;
  const int result = sqlite3_exec(database, sql, nullptr, nullptr, &error);
  if (result != SQLITE_OK) {
    const std::string message = error == nullptr ? "unknown error" : error;
    sqlite3_free(error);
    LOG(FATAL) << "SQLite statement failed: " << message;
  }
}

}  // namespace

ProtoStreamWriter::ProtoStreamWriter(const std::string& filename)
    : database_(nullptr) {
  std::filesystem::remove(filename);
  const int result = sqlite3_open(filename.c_str(), &database_);
  CheckSqlite(result, database_, "open map");
  Execute(database_, "PRAGMA application_id=1398230352;");
  Execute(database_, "PRAGMA user_version=1;");
  Execute(database_, "PRAGMA journal_mode=DELETE;");
  Execute(database_,
          "CREATE TABLE metadata(key TEXT PRIMARY KEY,value TEXT NOT NULL);"
          "INSERT INTO metadata VALUES('format','sweepnav_2d_map');"
          "INSERT INTO metadata VALUES('schema_version','1');"
          "CREATE TABLE records(sequence INTEGER PRIMARY KEY AUTOINCREMENT,"
          "record_type TEXT NOT NULL,payload BLOB NOT NULL,"
          "checksum INTEGER NOT NULL);BEGIN IMMEDIATE;");
  CheckSqlite(sqlite3_prepare_v2(
                  database_,
                  "INSERT INTO records(record_type,payload,checksum) VALUES(?,?,?)",
                  -1, &insert_, nullptr),
              database_, "prepare map insert");
}

void ProtoStreamWriter::WriteProto(const google::protobuf::Message& proto) {
  std::string uncompressed_data;
  proto.SerializeToString(&uncompressed_data);
  const std::string type = proto.GetTypeName();
  CheckSqlite(sqlite3_bind_text(insert_, 1, type.c_str(), type.size(),
                                SQLITE_TRANSIENT),
              database_, "bind record type");
  CheckSqlite(sqlite3_bind_blob(insert_, 2, uncompressed_data.data(),
                                uncompressed_data.size(), SQLITE_TRANSIENT),
              database_, "bind record payload");
  CheckSqlite(sqlite3_bind_int64(
                  insert_, 3,
                  static_cast<sqlite3_int64>(Checksum(uncompressed_data))),
              database_, "bind record checksum");
  const int result = sqlite3_step(insert_);
  ok_ = ok_ && result == SQLITE_DONE;
  CheckSqlite(result, database_, "insert map record");
  CheckSqlite(sqlite3_reset(insert_), database_, "reset map insert");
  CheckSqlite(sqlite3_clear_bindings(insert_), database_,
              "clear map insert bindings");
}

bool ProtoStreamWriter::Close() {
  if (insert_ != nullptr) sqlite3_finalize(insert_);
  insert_ = nullptr;
  Execute(database_, ok_ ? "COMMIT;" : "ROLLBACK;");
  ok_ = ok_ && sqlite3_close(database_) == SQLITE_OK;
  database_ = nullptr;
  return ok_;
}

ProtoStreamReader::ProtoStreamReader(const std::string& filename)
    : database_(nullptr) {
  const int result = sqlite3_open_v2(filename.c_str(), &database_,
                                     SQLITE_OPEN_READONLY, nullptr);
  CheckSqlite(result, database_, "open map");
  sqlite3_stmt* pragma = nullptr;
  CheckSqlite(sqlite3_prepare_v2(database_, "PRAGMA application_id", -1,
                                 &pragma, nullptr),
              database_, "read application id");
  CHECK_EQ(sqlite3_step(pragma), SQLITE_ROW);
  CHECK_EQ(sqlite3_column_int(pragma, 0), kApplicationId)
      << "Not a SweepNav map: " << filename;
  sqlite3_finalize(pragma);
  CheckSqlite(sqlite3_prepare_v2(database_, "PRAGMA user_version", -1,
                                 &pragma, nullptr),
              database_, "read schema version");
  CHECK_EQ(sqlite3_step(pragma), SQLITE_ROW);
  CHECK_EQ(sqlite3_column_int(pragma, 0), kSchemaVersion)
      << "Unsupported SweepNav map schema";
  sqlite3_finalize(pragma);
  CheckSqlite(sqlite3_prepare_v2(
                  database_,
                  "SELECT record_type,payload,checksum FROM records ORDER BY sequence",
                  -1, &select_, nullptr),
              database_, "prepare map read");
}

bool ProtoStreamReader::ReadProto(google::protobuf::Message* proto) {
  const int result = sqlite3_step(select_);
  if (result == SQLITE_DONE) {
    eof_ = true;
    sqlite3_finalize(select_);
    select_ = nullptr;
    sqlite3_close(database_);
    database_ = nullptr;
    return false;
  }
  CheckSqlite(result, database_, "read map record");
  const char* type =
      reinterpret_cast<const char*>(sqlite3_column_text(select_, 0));
  CHECK_EQ(proto->GetTypeName(), type) << "Unexpected map record type";
  const void* payload = sqlite3_column_blob(select_, 1);
  const int size = sqlite3_column_bytes(select_, 1);
  const std::string data(static_cast<const char*>(payload), size);
  CHECK_EQ(static_cast<uint64>(sqlite3_column_int64(select_, 2)),
           Checksum(data))
      << "Map record checksum mismatch";
  return proto->ParseFromString(data);
}

bool ProtoStreamReader::eof() const { return eof_; }

}  // namespace io
}  // namespace cartographer
