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
#include "cartographer/foundation/time.h"
#include "cartographer/foundation/geometry.h"
#include "cartographer/backend/map_by_id.h"
#include "cartographer/backend/backend_types.h"
#include "cartographer/application/slam_options.h"
#include "cartographer/foundation/sensor_data.h"
#include "cartographer/backend/trajectory_history.h"
#include "cartographer/foundation/sensor_data.h"
#include "cartographer/foundation/geometry.h"

namespace cartographer {
namespace mapping {
namespace optimization {

struct NodeSpec2D {
  common::Time time;
  transform::Rigid2d local_pose_2d;
  transform::Rigid2d global_pose_2d;
};

struct SubmapSpec2D {
  transform::Rigid2d global_pose;
};

class PoseOptimizer2D {
 public:
  using Constraint = ::cartographer::mapping::Constraint;
  explicit PoseOptimizer2D(
      const OptimizationProblemOptions& options);
  ~PoseOptimizer2D();

  PoseOptimizer2D(const PoseOptimizer2D&) = delete;
  PoseOptimizer2D& operator=(const PoseOptimizer2D&) = delete;

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

 private:
  std::unique_ptr<transform::Rigid2d> InterpolateOdometry(
      int trajectory_id, common::Time time) const;
  // Computes the relative pose between two nodes based on odometry data.
  std::unique_ptr<transform::Rigid2d> CalculateOdometryBetweenNodes(
      int trajectory_id, const NodeSpec2D& first_node_data,
      const NodeSpec2D& second_node_data) const;

  OptimizationProblemOptions options_;
  MapById<NodeId, NodeSpec2D> node_data_;
  MapById<SubmapId, SubmapSpec2D> submap_data_;
  sensor::MapByTime<sensor::OdometryData> odometry_data_;
};

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_2D_H_
