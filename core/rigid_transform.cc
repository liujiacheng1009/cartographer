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

#include "cartographer/core/rigid_transform.h"
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


#include <time.h>

#include <cerrno>
#include <cstring>
#include <string>

#include "glog/logging.h"

namespace cartographer {
namespace common {

Duration FromSeconds(const double seconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::duration<double>(seconds));
}

double ToSeconds(const Duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

double ToSeconds(const std::chrono::steady_clock::duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

Time FromUniversal(const int64 ticks) { return Time(Duration(ticks)); }

int64 ToUniversal(const Time time) { return time.time_since_epoch().count(); }

std::ostream& operator<<(std::ostream& os, const Time time) {
  os << std::to_string(ToUniversal(time));
  return os;
}

common::Duration FromMilliseconds(const int64 milliseconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::milliseconds(milliseconds));
}

double GetThreadCpuTimeSeconds() {
#ifndef WIN32
  struct timespec thread_cpu_time;
  CHECK(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &thread_cpu_time) == 0)
      << std::strerror(errno);
  return thread_cpu_time.tv_sec + 1e-9 * thread_cpu_time.tv_nsec;
#else
  return 0.;
#endif
}

}  // namespace common
}  // namespace cartographer

namespace cartographer {
namespace transform {

TimestampedTransform Interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const common::Time time) {
  CHECK_LE(start.time, time);
  CHECK_GE(end.time, time);
  const double duration = common::ToSeconds(end.time - start.time);
  const double factor = common::ToSeconds(time - start.time) / duration;
  const Eigen::Vector3d origin =
      start.transform.translation() +
      (end.transform.translation() - start.transform.translation()) * factor;
  const Eigen::Quaterniond rotation =
      Eigen::Quaterniond(start.transform.rotation())
          .slerp(factor, Eigen::Quaterniond(end.transform.rotation()));
  return TimestampedTransform{time, transform::Rigid3d(origin, rotation)};
}

}  // namespace transform
}  // namespace cartographer

#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/core/parameter_dictionary.h"
#include "glog/logging.h"

namespace cartographer {
namespace transform {

namespace {

Eigen::Vector3d TranslationFromDictionary(
    common::ParameterDictionary* dictionary) {
  const std::vector<double> translation = dictionary->GetArrayValuesAsDoubles();
  CHECK_EQ(3, translation.size()) << "Need (x, y, z) for translation.";
  return Eigen::Vector3d(translation[0], translation[1], translation[2]);
}

}  // namespace

Eigen::Quaterniond RollPitchYaw(const double roll, const double pitch,
                                const double yaw) {
  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}

transform::Rigid3d FromDictionary(common::ParameterDictionary* dictionary) {
  const Eigen::Vector3d translation =
      TranslationFromDictionary(dictionary->GetDictionary("translation").get());

  auto rotation_dictionary = dictionary->GetDictionary("rotation");
  if (rotation_dictionary->HasKey("w")) {
    const Eigen::Quaterniond rotation(rotation_dictionary->GetDouble("w"),
                                      rotation_dictionary->GetDouble("x"),
                                      rotation_dictionary->GetDouble("y"),
                                      rotation_dictionary->GetDouble("z"));
    CHECK_NEAR(rotation.norm(), 1., 1e-9);
    return transform::Rigid3d(translation, rotation);
  } else {
    const std::vector<double> rotation =
        rotation_dictionary->GetArrayValuesAsDoubles();
    CHECK_EQ(3, rotation.size()) << "Need (roll, pitch, yaw) for rotation.";
    return transform::Rigid3d(
        translation, RollPitchYaw(rotation[0], rotation[1], rotation[2]));
  }
}

}  // namespace transform
}  // namespace cartographer
