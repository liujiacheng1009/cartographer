/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/slam/pose_extrapolator_interface.h"

#include "cartographer/core/time.h"

namespace cartographer {
namespace mapping {

namespace {

proto::ConstantVelocityPoseExtrapolatorOptions
CreateConstantVelocityPoseExtrapolatorOptions(
    common::ParameterDictionary* const parameter_dictionary) {
  proto::ConstantVelocityPoseExtrapolatorOptions options;
  options.set_pose_queue_duration(
      parameter_dictionary->GetDouble("pose_queue_duration"));
  options.set_imu_gravity_time_constant(
      parameter_dictionary->GetDouble("imu_gravity_time_constant"));
  return options;
}

}  // namespace

proto::PoseExtrapolatorOptions CreatePoseExtrapolatorOptions(
    common::ParameterDictionary* const parameter_dictionary) {
  proto::PoseExtrapolatorOptions options;
  *options.mutable_constant_velocity() =
      CreateConstantVelocityPoseExtrapolatorOptions(
          parameter_dictionary->GetDictionary("constant_velocity").get());
  return options;
}

}  // namespace mapping
}  // namespace cartographer
