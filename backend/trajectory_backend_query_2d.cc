// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#include "cartographer/backend/trajectory_backend_2d.h"

#include <iterator>

#include "cartographer/foundation/math.h"
#include "cartographer/foundation/transform.h"

namespace cartographer {
namespace mapping {

MapById<NodeId, TrajectoryNode> TrajectoryBackend2D::GetTrajectoryNodes() const {
  absl::MutexLock locker(&mutex_);
  return data_.trajectory_nodes;
}

MapById<NodeId, TrajectoryNodePose> TrajectoryBackend2D::GetTrajectoryNodePoses() const {
  MapById<NodeId, TrajectoryNodePose> node_poses;
  absl::MutexLock locker(&mutex_);
  for (const auto& node_id_data : data_.trajectory_nodes) {
    absl::optional<TrajectoryNodePose::ConstantPoseData> constant_pose_data;
    if (node_id_data.data.constant_data != nullptr) {
      constant_pose_data = TrajectoryNodePose::ConstantPoseData{
          node_id_data.data.constant_data->time,
          node_id_data.data.constant_data->local_pose};
    }
    node_poses.Insert(
        node_id_data.id,
        TrajectoryNodePose{node_id_data.data.global_pose, constant_pose_data});
  }
  return node_poses;
}

std::map<int, TrajectoryState>
TrajectoryBackend2D::GetTrajectoryStates() const {
  std::map<int, TrajectoryState> trajectories_state;
  absl::MutexLock locker(&mutex_);
  for (const auto& it : data_.trajectories_state) {
    trajectories_state[it.first] = it.second.state;
  }
  return trajectories_state;
}

sensor::MapByTime<sensor::OdometryData> TrajectoryBackend2D::GetOdometryData() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->odometry_data();
}

std::vector<Constraint> TrajectoryBackend2D::constraints() const {
  std::vector<Constraint> result;
  absl::MutexLock locker(&mutex_);
  result = data_.constraints;
  return result;
}

void TrajectoryBackend2D::SetInitialTrajectoryPose(const int from_trajectory_id,
                                         const int to_trajectory_id,
                                         const transform::Rigid2d& pose,
                                         const common::Time time) {
  absl::MutexLock locker(&mutex_);
  data_.initial_trajectory_poses[from_trajectory_id] =
      InitialTrajectoryPose{to_trajectory_id, pose, time};
}

transform::Rigid2d TrajectoryBackend2D::GetInterpolatedGlobalTrajectoryPose(
    const int trajectory_id, const common::Time time) const {
  CHECK_GT(data_.trajectory_nodes.SizeOfTrajectoryOrZero(trajectory_id), 0);
  const auto it = data_.trajectory_nodes.lower_bound(trajectory_id, time);
  if (it == data_.trajectory_nodes.BeginOfTrajectory(trajectory_id)) {
    return it->data.global_pose;
  }
  if (it == data_.trajectory_nodes.EndOfTrajectory(trajectory_id)) {
    return std::prev(it)->data.global_pose;
  }
  const auto& start = *std::prev(it);
  const auto& end = *it;
  const double factor = common::ToSeconds(time - start.data.time()) /
                        common::ToSeconds(end.data.time() - start.data.time());
  const Eigen::Vector2d translation =
      start.data.global_pose.translation() +
      factor * (end.data.global_pose.translation() -
                start.data.global_pose.translation());
  const double delta_angle = common::NormalizeAngleDifference(
      transform::Yaw(end.data.global_pose) -
      transform::Yaw(start.data.global_pose));
  return transform::MakeRigid2(
      translation, transform::Yaw(start.data.global_pose) + factor * delta_angle);
}

transform::Rigid2d TrajectoryBackend2D::GetLocalToGlobalTransform(
    const int trajectory_id) const {
  absl::MutexLock locker(&mutex_);
  return ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                       trajectory_id);
}

SubmapData TrajectoryBackend2D::GetSubmapData(
    const SubmapId& submap_id) const {
  absl::MutexLock locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

MapById<SubmapId, SubmapData>
TrajectoryBackend2D::GetAllSubmapData() const {
  absl::MutexLock locker(&mutex_);
  return GetSubmapDataUnderLock();
}

MapById<SubmapId, SubmapPose>
TrajectoryBackend2D::GetAllSubmapPoses() const {
  absl::MutexLock locker(&mutex_);
  MapById<SubmapId, SubmapPose> submap_poses;
  for (const auto& submap_id_data : data_.submap_data) {
    auto submap_data = GetSubmapDataUnderLock(submap_id_data.id);
    submap_poses.Insert(
        submap_id_data.id,
        SubmapPose{submap_data.submap->num_range_data(), submap_data.pose});
  }
  return submap_poses;
}

transform::Rigid2d TrajectoryBackend2D::ComputeLocalToGlobalTransform(
    const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
    const int trajectory_id) const {
  auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
  auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);
  if (begin_it == end_it) {
    const auto it = data_.initial_trajectory_poses.find(trajectory_id);
    if (it == data_.initial_trajectory_poses.end()) {
      return transform::Rigid2d();
    }
    return GetInterpolatedGlobalTrajectoryPose(it->second.to_trajectory_id,
                                               it->second.time) *
           it->second.relative_pose;
  }
  const SubmapId last_optimized_submap_id = std::prev(end_it)->id;
  return global_submap_poses.at(last_optimized_submap_id).global_pose *
         data_.submap_data.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}

SubmapData TrajectoryBackend2D::GetSubmapDataUnderLock(
    const SubmapId& submap_id) const {
  const auto it = data_.submap_data.find(submap_id);
  if (it == data_.submap_data.end()) return {};
  auto submap = it->data.submap;
  if (data_.global_submap_poses_2d.Contains(submap_id)) {
    return {submap,
            data_.global_submap_poses_2d.at(submap_id).global_pose};
  }
  return {submap, ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                                submap_id.trajectory_id) *
                      submap->local_pose()};
}

}  // namespace mapping
}  // namespace cartographer
