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

#include "cartographer/slam/trajectory_builder_interface.h"

#include "cartographer/slam/options.h"

namespace cartographer {
namespace mapping {
namespace {

void PopulatePureLocalizationTrimmerOptions(
    TrajectoryBuilderOptions* const trajectory_builder_options,
    common::ParameterDictionary* const parameter_dictionary) {
  constexpr char kDictionaryKey[] = "pure_localization_trimmer";
  if (!parameter_dictionary->HasKey(kDictionaryKey)) return;

  auto options_dictionary = parameter_dictionary->GetDictionary(kDictionaryKey);
  auto* options =
      trajectory_builder_options->mutable_pure_localization_trimmer();
  options->set_max_submaps_to_keep(
      options_dictionary->GetInt("max_submaps_to_keep"));
}

void PopulatePoseGraphOdometryMotionFilterOptions(
    TrajectoryBuilderOptions* const trajectory_builder_options,
    common::ParameterDictionary* const parameter_dictionary) {
  constexpr char kDictionaryKey[] = "pose_graph_odometry_motion_filter";
  if (!parameter_dictionary->HasKey(kDictionaryKey)) return;

  auto options_dictionary = parameter_dictionary->GetDictionary(kDictionaryKey);
  auto* options =
      trajectory_builder_options->mutable_pose_graph_odometry_motion_filter();
  options->set_max_time_seconds(
      options_dictionary->GetDouble("max_time_seconds"));
  options->set_max_distance_meters(
      options_dictionary->GetDouble("max_distance_meters"));
  options->set_max_angle_radians(
      options_dictionary->GetDouble("max_angle_radians"));
}

}  // namespace

TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    common::ParameterDictionary* const parameter_dictionary) {
  TrajectoryBuilderOptions options;
  *options.mutable_trajectory_builder_2d_options() =
      CreateLocalTrajectoryBuilderOptions2D(
          parameter_dictionary->GetDictionary("trajectory_builder_2d").get());
  PopulatePureLocalizationTrimmerOptions(&options, parameter_dictionary);
  PopulatePoseGraphOdometryMotionFilterOptions(&options, parameter_dictionary);
  return options;
}

}  // namespace mapping
}  // namespace cartographer
