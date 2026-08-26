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

#ifndef CARTOGRAPHER_FOUNDATION_UTILS_H_
#define CARTOGRAPHER_FOUNDATION_UTILS_H_

#include <cmath>

namespace cartographer {
namespace common {

// Brings an angular difference into [-pi, pi]. Kept templated for Ceres Jets.
template <typename T>
T NormalizeAngleDifference(T difference) {
  const T kPi = T(M_PI);
  while (difference > kPi) difference -= T(2.) * kPi;
  while (difference < -kPi) difference += T(2.) * kPi;
  return difference;
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_FOUNDATION_UTILS_H_
