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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_2D_H_

#include <array>
#include <deque>
#include <map>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/core/port.h"
#include "cartographer/core/rigid_transform.h"
#include "cartographer/pose_graph/id.h"
#include "cartographer/pose_graph/pose_graph_types.h"
#include "cartographer/trajectory/options.h"
#include "cartographer/core/sensor_data.h"
#include "cartographer/core/map_by_time.h"
#include "cartographer/core/sensor_data.h"
#include "cartographer/core/rigid_transform.h"

namespace cartographer {
namespace mapping {
namespace optimization {

struct NodeSpec2D {
  common::Time time;
  transform::Rigid2d local_pose_2d;
  transform::Rigid2d global_pose_2d;
  Eigen::Quaterniond gravity_alignment;
};

struct SubmapSpec2D {
  transform::Rigid2d global_pose;
};

class OptimizationProblem2D {
 public:
  using Constraint = ::cartographer::mapping::Constraint;
  explicit OptimizationProblem2D(
      const OptimizationProblemOptions& options);
  ~OptimizationProblem2D();

  OptimizationProblem2D(const OptimizationProblem2D&) = delete;
  OptimizationProblem2D& operator=(const OptimizationProblem2D&) = delete;

  void AddOdometryData(int trajectory_id,
                       const sensor::OdometryData& odometry_data);
  void AddTrajectoryNode(int trajectory_id,
                         const NodeSpec2D& node_data);
  void InsertTrajectoryNode(const NodeId& node_id,
                            const NodeSpec2D& node_data);
  void TrimTrajectoryNode(const NodeId& node_id);
  void AddSubmap(int trajectory_id,
                 const transform::Rigid2d& global_submap_pose);
  void InsertSubmap(const SubmapId& submap_id,
                    const transform::Rigid2d& global_submap_pose);
  void TrimSubmap(const SubmapId& submap_id);
  void SetMaxNumIterations(int32 max_num_iterations);

  void Solve(
      const std::vector<Constraint>& constraints,
      const std::map<int, TrajectoryState>& trajectories_state);

  const MapById<NodeId, NodeSpec2D>& node_data() const {
    return node_data_;
  }
  const MapById<SubmapId, SubmapSpec2D>& submap_data() const {
    return submap_data_;
  }
  const sensor::MapByTime<sensor::OdometryData>& odometry_data()
      const {
    return odometry_data_;
  }

  void SetTrajectoryData(
      int trajectory_id,
      const TrajectoryData& trajectory_data);
  const std::map<int, TrajectoryData>& trajectory_data()
      const {
    return trajectory_data_;
  }

 private:
  std::unique_ptr<transform::Rigid3d> InterpolateOdometry(
      int trajectory_id, common::Time time) const;
  // Computes the relative pose between two nodes based on odometry data.
  std::unique_ptr<transform::Rigid3d> CalculateOdometryBetweenNodes(
      int trajectory_id, const NodeSpec2D& first_node_data,
      const NodeSpec2D& second_node_data) const;

  OptimizationProblemOptions options_;
  MapById<NodeId, NodeSpec2D> node_data_;
  MapById<SubmapId, SubmapSpec2D> submap_data_;
  sensor::MapByTime<sensor::OdometryData> odometry_data_;
  std::map<int, TrajectoryData> trajectory_data_;
};

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_2D_H_
