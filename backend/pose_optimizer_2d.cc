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

#include "cartographer/backend/pose_optimizer_2d.h"

#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <vector>

#include "cartographer/application/slam_options.h"
#include "cartographer/foundation/math.h"
#include "cartographer/foundation/sensor_data.h"
#include "cartographer/foundation/transform.h"
#include "cartographer/scan_matching/cost_helpers.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

class SpaCostFunction2D {
 public:
  explicit SpaCostFunction2D(
      const Constraint::Pose& observed_relative_pose)
      : observed_relative_pose_(observed_relative_pose) {}

  template <typename T>
  bool operator()(const T* const start_pose, const T* const end_pose,
                  T* e) const {
    const std::array<T, 3> error =
        ScaleError(ComputeUnscaledError(
                       observed_relative_pose_.zbar_ij,
                       start_pose, end_pose),
                   observed_relative_pose_.translation_weight,
                   observed_relative_pose_.rotation_weight);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  const Constraint::Pose observed_relative_pose_;
};

ceres::CostFunction* CreateAutoDiffSpaCostFunction(
    const Constraint::Pose& observed_relative_pose) {
  return new ceres::AutoDiffCostFunction<SpaCostFunction2D, 3 /* residuals */,
                                         3 /* start pose variables */,
                                         3 /* end pose variables */>(
      new SpaCostFunction2D(observed_relative_pose));
}

// Converts a pose into the 3 optimization variable format used for Ceres:
// translation in x and y, followed by the rotation angle representing the
// orientation.
std::array<double, 3> FromPose(const transform::Rigid2d& pose) {
  return {{pose.translation().x(), pose.translation().y(),
           transform::Yaw(pose)}};
}

// Converts a pose as represented for Ceres back to an transform::Rigid2d pose.
transform::Rigid2d ToPose(const std::array<double, 3>& values) {
  return transform::MakeRigid2(Eigen::Vector2d(values[0], values[1]),
                               values[2]);
}

}  // namespace

PoseOptimizer2D::PoseOptimizer2D(
    const OptimizationProblemOptions& options)
    : options_(options) {}

PoseOptimizer2D::~PoseOptimizer2D() {}

void PoseOptimizer2D::AddOdometryData(
    const int trajectory_id, const sensor::OdometryData& odometry_data) {
  odometry_data_.Append(trajectory_id, odometry_data);
}

void PoseOptimizer2D::AddTrajectoryNode(const int trajectory_id,
                                        const NodeSpec2D& node_data) {
  node_data_.Append(trajectory_id, node_data);
}

void PoseOptimizer2D::InsertTrajectoryNode(const NodeId& node_id,
                                           const NodeSpec2D& node_data) {
  node_data_.Insert(node_id, node_data);
}

void PoseOptimizer2D::TrimTrajectoryNode(const NodeId& node_id) {
  odometry_data_.Trim(node_data_, node_id);
  node_data_.Trim(node_id);
}

void PoseOptimizer2D::AddSubmap(
    const int trajectory_id, const transform::Rigid2d& global_submap_pose) {
  submap_data_.Append(trajectory_id, SubmapSpec2D{global_submap_pose});
}

void PoseOptimizer2D::InsertSubmap(
    const SubmapId& submap_id, const transform::Rigid2d& global_submap_pose) {
  submap_data_.Insert(submap_id, SubmapSpec2D{global_submap_pose});
}

void PoseOptimizer2D::TrimSubmap(const SubmapId& submap_id) {
  submap_data_.Trim(submap_id);
}

void PoseOptimizer2D::SetMaxNumIterations(
    const int32 max_num_iterations) {
  options_.mutable_ceres_solver_options()->set_max_num_iterations(
      max_num_iterations);
}

void PoseOptimizer2D::Solve(
    const std::vector<Constraint>& constraints,
    const std::map<int, TrajectoryState>& trajectories_state) {
  if (node_data_.empty()) {
    // Nothing to optimize.
    return;
  }

  std::set<int> frozen_trajectories;
  for (const auto& it : trajectories_state) {
    if (it.second == TrajectoryState::FROZEN) {
      frozen_trajectories.insert(it.first);
    }
  }

  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  // Set the starting point.
  // TODO(hrapp): Move ceres data into SubmapSpec.
  MapById<SubmapId, std::array<double, 3>> C_submaps;
  MapById<NodeId, std::array<double, 3>> C_nodes;
  bool first_submap = true;
  for (const auto& submap_id_data : submap_data_) {
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
    C_submaps.Insert(submap_id_data.id,
                     FromPose(submap_id_data.data.global_pose));
    problem.AddParameterBlock(C_submaps.at(submap_id_data.id).data(), 3);
    if (first_submap || frozen) {
      first_submap = false;
      // Fix the pose of the first submap or all submaps of a frozen
      // trajectory.
      problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
    }
  }
  for (const auto& node_id_data : node_data_) {
    const bool frozen =
        frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
    C_nodes.Insert(node_id_data.id, FromPose(node_id_data.data.global_pose_2d));
    problem.AddParameterBlock(C_nodes.at(node_id_data.id).data(), 3);
    if (frozen) {
      problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).data());
    }
  }
  // Add cost functions for intra- and inter-submap constraints.
  for (const Constraint& constraint : constraints) {
    problem.AddResidualBlock(
        CreateAutoDiffSpaCostFunction(constraint.pose),
        // Loop closure constraints should have a loss function.
        constraint.tag == Constraint::INTER_SUBMAP
            ? new ceres::HuberLoss(options_.huber_scale())
            : nullptr,
        C_submaps.at(constraint.submap_id).data(),
        C_nodes.at(constraint.node_id).data());
  }
  // Add penalties for violating odometry or changes between consecutive nodes
  // if odometry is not available.
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    const int trajectory_id = node_it->id.trajectory_id;
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    if (frozen_trajectories.count(trajectory_id) != 0) {
      node_it = trajectory_end;
      continue;
    }

    auto prev_node_it = node_it;
    for (++node_it; node_it != trajectory_end; ++node_it) {
      const NodeId first_node_id = prev_node_it->id;
      const NodeSpec2D& first_node_data = prev_node_it->data;
      prev_node_it = node_it;
      const NodeId second_node_id = node_it->id;
      const NodeSpec2D& second_node_data = node_it->data;

      if (second_node_id.node_index != first_node_id.node_index + 1) {
        continue;
      }

      // Add a relative pose constraint based on the odometry (if available).
      std::unique_ptr<transform::Rigid2d> relative_odometry =
          CalculateOdometryBetweenNodes(trajectory_id, first_node_data,
                                        second_node_data);
      if (relative_odometry != nullptr) {
        problem.AddResidualBlock(
            CreateAutoDiffSpaCostFunction(Constraint::Pose{
                *relative_odometry,
                options_.odometry_translation_weight(),
                options_.odometry_rotation_weight()}),
            nullptr /* loss function */, C_nodes.at(first_node_id).data(),
            C_nodes.at(second_node_id).data());
      }

      // Add a relative pose constraint based on consecutive local SLAM poses.
      const transform::Rigid2d relative_local_slam_pose =
          first_node_data.local_pose_2d.inverse() *
          second_node_data.local_pose_2d;
      problem.AddResidualBlock(
          CreateAutoDiffSpaCostFunction(
              Constraint::Pose{relative_local_slam_pose,
                               options_.local_slam_pose_translation_weight(),
                               options_.local_slam_pose_rotation_weight()}),
          nullptr /* loss function */, C_nodes.at(first_node_id).data(),
          C_nodes.at(second_node_id).data());
    }
  }

  // Solve.
  ceres::Solver::Summary summary;
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);
  if (options_.log_solver_summary()) {
    LOG(INFO) << summary.FullReport();
  }

  // Store the result.
  for (const auto& C_submap_id_data : C_submaps) {
    submap_data_.at(C_submap_id_data.id).global_pose =
        ToPose(C_submap_id_data.data);
  }
  for (const auto& C_node_id_data : C_nodes) {
    node_data_.at(C_node_id_data.id).global_pose_2d =
        ToPose(C_node_id_data.data);
  }
}

std::unique_ptr<transform::Rigid2d> PoseOptimizer2D::InterpolateOdometry(
    const int trajectory_id, const common::Time time) const {
  const auto it = odometry_data_.lower_bound(trajectory_id, time);
  if (it == odometry_data_.EndOfTrajectory(trajectory_id)) {
    return nullptr;
  }
  if (it == odometry_data_.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return absl::make_unique<transform::Rigid2d>(it->pose);
    }
    return nullptr;
  }
  const auto prev_it = std::prev(it);
  const double factor = common::ToSeconds(time - prev_it->time) /
                        common::ToSeconds(it->time - prev_it->time);
  const Eigen::Vector2d translation =
      prev_it->pose.translation() +
      factor * (it->pose.translation() - prev_it->pose.translation());
  const double yaw = transform::Yaw(prev_it->pose) +
      factor * common::NormalizeAngleDifference(
                   transform::Yaw(it->pose) - transform::Yaw(prev_it->pose));
  return absl::make_unique<transform::Rigid2d>(yaw, translation);
}

std::unique_ptr<transform::Rigid2d>
PoseOptimizer2D::CalculateOdometryBetweenNodes(
    const int trajectory_id, const NodeSpec2D& first_node_data,
    const NodeSpec2D& second_node_data) const {
  if (odometry_data_.HasTrajectory(trajectory_id)) {
    const std::unique_ptr<transform::Rigid2d> first_node_odometry =
        InterpolateOdometry(trajectory_id, first_node_data.time);
    const std::unique_ptr<transform::Rigid2d> second_node_odometry =
        InterpolateOdometry(trajectory_id, second_node_data.time);
    if (first_node_odometry != nullptr && second_node_odometry != nullptr) {
      transform::Rigid2d relative_odometry =
          first_node_odometry->inverse() * (*second_node_odometry);
      return absl::make_unique<transform::Rigid2d>(relative_odometry);
    }
  }
  return nullptr;
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
