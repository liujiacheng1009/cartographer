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

#include "cartographer/slam/map_builder.h"

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/core/time.h"
#include "cartographer/state/swmap.h"
#include "cartographer/slam/local_trajectory_builder_2d.h"
#include "cartographer/slam/pose_graph.h"
#include "cartographer/slam/collated_trajectory_builder.h"
#include "cartographer/slam/global_trajectory_builder.h"
#include "cartographer/slam/motion_filter.h"
#include "cartographer/core/collator.h"
#include "cartographer/core/trajectory_collator.h"
#include "cartographer/core/voxel_filter.h"
#include "cartographer/core/rigid_transform.h"
#include "cartographer/core/transform.h"

namespace cartographer {
namespace mapping {
namespace {


std::vector<std::string> SelectRangeSensorIds(
    const std::set<MapBuilder::SensorId>& expected_sensor_ids) {
  std::vector<std::string> range_sensor_ids;
  for (const MapBuilder::SensorId& sensor_id : expected_sensor_ids) {
    if (sensor_id.type == MapBuilder::SensorId::SensorType::RANGE) {
      range_sensor_ids.push_back(sensor_id.id);
    }
  }
  return range_sensor_ids;
}

void MaybeAddPureLocalizationTrimmer(
    const int trajectory_id,
    const TrajectoryBuilderOptions& trajectory_options,
    PoseGraph* pose_graph) {
  if (trajectory_options.pure_localization()) {
    LOG(WARNING)
        << "'TrajectoryBuilderOptions::pure_localization' field is deprecated. "
           "Use 'TrajectoryBuilderOptions::pure_localization_trimmer' instead.";
    pose_graph->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
        trajectory_id, 3 /* max_submaps_to_keep */));
    return;
  }
  if (trajectory_options.has_pure_localization_trimmer()) {
    pose_graph->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
        trajectory_id,
        trajectory_options.pure_localization_trimmer().max_submaps_to_keep()));
  }
}

}  // namespace

MapBuilder::MapBuilder(const MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) {
  CHECK(options.use_trajectory_builder_2d());
  pose_graph_ = absl::make_unique<PoseGraph>(
      options_.pose_graph_options(),
      absl::make_unique<optimization::OptimizationProblem2D>(
          options_.pose_graph_options().optimization_problem_options()),
      &thread_pool_);
  if (options.collate_by_trajectory()) {
    sensor_collator_ = absl::make_unique<sensor::TrajectoryCollator>();
  } else {
    sensor_collator_ = absl::make_unique<sensor::Collator>();
  }
}

int MapBuilder::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {
  const int trajectory_id = trajectory_builders_.size();

  absl::optional<MotionFilter> pose_graph_odometry_motion_filter;
  if (trajectory_options.has_pose_graph_odometry_motion_filter()) {
    LOG(INFO) << "Using a motion filter for adding odometry to the pose graph.";
    pose_graph_odometry_motion_filter.emplace(
        MotionFilter(trajectory_options.pose_graph_odometry_motion_filter()));
  }

  std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder;
  if (trajectory_options.has_trajectory_builder_2d_options()) {
    local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder2D>(
        trajectory_options.trajectory_builder_2d_options(),
        SelectRangeSensorIds(expected_sensor_ids));
  }
  DCHECK(dynamic_cast<PoseGraph*>(pose_graph_.get()));
  trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(
      trajectory_options, sensor_collator_.get(), trajectory_id,
      expected_sensor_ids,
      CreateGlobalTrajectoryBuilder2D(
          std::move(local_trajectory_builder), trajectory_id,
          static_cast<PoseGraph*>(pose_graph_.get()),
          local_slam_result_callback, pose_graph_odometry_motion_filter)));
  MaybeAddPureLocalizationTrimmer(trajectory_id, trajectory_options,
                                  pose_graph_.get());

  if (trajectory_options.has_initial_trajectory_pose()) {
    const auto& initial_trajectory_pose =
        trajectory_options.initial_trajectory_pose();
    pose_graph_->SetInitialTrajectoryPose(
        trajectory_id, initial_trajectory_pose.to_trajectory_id,
        initial_trajectory_pose.relative_pose, initial_trajectory_pose.timestamp);
  }
  return trajectory_id;
}

int MapBuilder::AddTrajectoryForDeserialization() {
  const int trajectory_id = trajectory_builders_.size();
  trajectory_builders_.emplace_back();
  return trajectory_id;
}

void MapBuilder::FinishTrajectory(const int trajectory_id) {
  sensor_collator_->FinishTrajectory(trajectory_id);
  pose_graph_->FinishTrajectory(trajectory_id);
}

std::string MapBuilder::GetSubmapTexture(
    const SubmapId& submap_id, SubmapTextureResponse* const response) {
  if (submap_id.trajectory_id < 0 ||
      submap_id.trajectory_id >= num_trajectory_builders()) {
    return "Requested submap from trajectory " +
           std::to_string(submap_id.trajectory_id) + " but there are only " +
           std::to_string(num_trajectory_builders()) + " trajectories.";
  }

  const auto submap_data = pose_graph_->GetSubmapData(submap_id);
  if (submap_data.submap == nullptr) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but it does not exist: maybe it has been trimmed.";
  }
  submap_data.submap->ToSubmapTextureResponse(submap_data.pose, response);
  return "";
}

bool MapBuilder::SerializeStateToFile(bool include_unfinished_submaps,
                                      const std::string& filename) {
  return io::WriteSwMap(filename, *pose_graph_, include_unfinished_submaps);
}

std::map<int, int> MapBuilder::LoadStateFromFile(
    const std::string& state_filename, const bool load_frozen_state) {
  const std::string suffix = ".swmap";
  if (state_filename.substr(
          std::max<int>(state_filename.size() - suffix.size(), 0)) != suffix) {
    LOG(WARNING) << "The file containing the state should be a .swmap file.";
  }
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  CHECK(load_frozen_state)
      << "Native .swmap loading currently supports frozen maps only.";
  io::SerializedState state = io::ReadSwMap(state_filename);
  std::map<int, int> trajectory_remapping;
  for (int old_id : state.trajectory_ids) {
    const int new_id = AddTrajectoryForDeserialization();
    CHECK(trajectory_remapping.emplace(old_id, new_id).second);
    pose_graph_->FreezeTrajectory(new_id);
  }
  for (const auto& landmark : state.landmark_poses) {
    pose_graph_->SetLandmarkPose(landmark.first, landmark.second, true);
  }
  for (auto& submap : state.submaps) {
    submap.id.trajectory_id = trajectory_remapping.at(submap.id.trajectory_id);
    pose_graph_->AddSerializedSubmap(submap);
  }
  for (auto& node : state.nodes) {
    node.id.trajectory_id = trajectory_remapping.at(node.id.trajectory_id);
    pose_graph_->AddSerializedNode(node);
  }
  for (const auto& item : state.trajectory_data) {
    pose_graph_->SetSerializedTrajectoryData(
        trajectory_remapping.at(item.first), item.second);
  }
  for (auto constraint : state.constraints) {
    constraint.submap_id.trajectory_id =
        trajectory_remapping.at(constraint.submap_id.trajectory_id);
    constraint.node_id.trajectory_id =
        trajectory_remapping.at(constraint.node_id.trajectory_id);
    if (constraint.tag == PoseGraph::Constraint::INTRA_SUBMAP) {
      pose_graph_->AddNodeToSubmap(constraint.node_id, constraint.submap_id);
    }
  }
  return trajectory_remapping;
}

std::unique_ptr<MapBuilderInterface> CreateMapBuilder(
    const MapBuilderOptions& options) {
  return absl::make_unique<MapBuilder>(options);
}

}  // namespace mapping
}  // namespace cartographer
