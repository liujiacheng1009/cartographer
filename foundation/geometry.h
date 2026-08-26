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

#ifndef CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_
#define CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "sophus/se2.hpp"
#include "sophus/se3.hpp"

namespace cartographer {
namespace transform {

using Rigid2d = Sophus::SE2d;
using Rigid2f = Sophus::SE2f;
using Rigid3f = Sophus::SE3f;

template <typename T>
Sophus::SE2<T> MakeRigid2(const Eigen::Matrix<T, 2, 1>& translation,
                          const T& yaw) {
  return Sophus::SE2<T>(yaw, translation);
}

template <typename Derived>
Sophus::SE2<typename Derived::Scalar> MakeRigid2Translation(
    const Eigen::MatrixBase<Derived>& translation) {
  using T = typename Derived::Scalar;
  return Sophus::SE2<T>::trans(translation);
}

template <typename T>
T Yaw(const Sophus::SE2<T>& transform) {
  return transform.so2().log();
}

template <typename T>
Sophus::SE3<T> Embed3D(const Sophus::SE2<T>& transform) {
  const Eigen::Matrix<T, 3, 1> translation(
      transform.translation().x(), transform.translation().y(), T(0));
  return Sophus::SE3<T>(
      Sophus::SO3<T>(Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
          transform.so2().log(), Eigen::Matrix<T, 3, 1>::UnitZ()))),
      translation);
}

template <typename T>
Sophus::SE3<T> MakeRigid3Rotation(const Eigen::AngleAxis<T>& rotation) {
  return Sophus::SE3<T>(
      Sophus::SO3<T>(Eigen::Quaternion<T>(rotation)),
      Eigen::Matrix<T, 3, 1>::Zero());
}

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_
