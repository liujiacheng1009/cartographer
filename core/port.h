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

#ifndef CARTOGRAPHER_COMMON_PORT_H_
#define CARTOGRAPHER_COMMON_PORT_H_

#include <array>
#include <cinttypes>
#include <cmath>
#include <stdexcept>
#include <string>
#include <zlib.h>

namespace cartographer {

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

namespace common {

inline int RoundToInt(const float x) { return std::lround(x); }

inline int RoundToInt(const double x) { return std::lround(x); }

inline int64 RoundToInt64(const float x) { return std::lround(x); }

inline int64 RoundToInt64(const double x) { return std::lround(x); }

inline void FastGzipString(const std::string& uncompressed,
                           std::string* compressed) {
  z_stream stream{};
  if (deflateInit2(&stream, Z_BEST_SPEED, Z_DEFLATED, MAX_WBITS + 16, 8,
                   Z_DEFAULT_STRATEGY) != Z_OK) {
    throw std::runtime_error("Failed to initialize gzip compressor.");
  }
  compressed->clear();
  stream.next_in = reinterpret_cast<Bytef*>(
      const_cast<char*>(uncompressed.data()));
  stream.avail_in = static_cast<uInt>(uncompressed.size());
  std::array<char, 16384> buffer;
  int result = Z_OK;
  while (result == Z_OK) {
    stream.next_out = reinterpret_cast<Bytef*>(buffer.data());
    stream.avail_out = static_cast<uInt>(buffer.size());
    result = deflate(&stream, Z_FINISH);
    compressed->append(buffer.data(), buffer.size() - stream.avail_out);
  }
  deflateEnd(&stream);
  if (result != Z_STREAM_END) {
    throw std::runtime_error("Failed to compress gzip stream.");
  }
}

inline void FastGunzipString(const std::string& compressed,
                             std::string* decompressed) {
  z_stream stream{};
  if (inflateInit2(&stream, MAX_WBITS + 16) != Z_OK) {
    throw std::runtime_error("Failed to initialize gzip decompressor.");
  }
  decompressed->clear();
  stream.next_in =
      reinterpret_cast<Bytef*>(const_cast<char*>(compressed.data()));
  stream.avail_in = static_cast<uInt>(compressed.size());
  std::array<char, 16384> buffer;
  int result = Z_OK;
  while (result == Z_OK) {
    stream.next_out = reinterpret_cast<Bytef*>(buffer.data());
    stream.avail_out = static_cast<uInt>(buffer.size());
    result = inflate(&stream, Z_NO_FLUSH);
    decompressed->append(buffer.data(), buffer.size() - stream.avail_out);
  }
  inflateEnd(&stream);
  if (result != Z_STREAM_END) {
    throw std::runtime_error("Failed to decompress gzip stream.");
  }
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_PORT_H_
