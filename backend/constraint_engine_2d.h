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

#ifndef CARTOGRAPHER_BACKEND_CONSTRAINT_ENGINE_2D_H_
#define CARTOGRAPHER_BACKEND_CONSTRAINT_ENGINE_2D_H_

#include <cstddef>
#include <map>
#include <optional>
#include <vector>

#include "cartographer/application/slam_options.h"
#include "cartographer/backend/backend_types.h"
#include "cartographer/foundation/sampling.h"
#include "cartographer/foundation/sensor_data.h"
#include "cartographer/mapping/submap_2d.h"
#include "cartographer/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/scan_matching/fast_correlative_scan_matcher_2d.h"

namespace cartographer {
namespace mapping {
namespace constraints {

// Computes constraints synchronously on the serial backend worker.
class ConstraintEngine2D {
 public:
  using Constraint = ::cartographer::mapping::Constraint;
  using Result = std::vector<Constraint>;

  explicit ConstraintEngine2D(const ConstraintBuilderOptions& options);
  ~ConstraintEngine2D() = default;

  ConstraintEngine2D(const ConstraintEngine2D&) = delete;
  ConstraintEngine2D& operator=(const ConstraintEngine2D&) = delete;

  // Explores a new constraint between 'submap' identified by
  // 'submap_id', and the 'compressed_point_cloud' for 'node_id'. The
  // 'initial_relative_pose' is relative to the 'submap'.
  void MaybeAddConstraint(const SubmapId& submap_id, const Submap2D* submap,
                          const NodeId& node_id,
                          const TrajectoryNode::Data* const constant_data,
                          const transform::Rigid2d& initial_relative_pose);

  // Explores a new constraint between 'submap' identified by
  // 'submap_id' and the 'compressed_point_cloud' for 'node_id'.
  // This performs full-submap matching.
  void MaybeAddGlobalConstraint(
      const SubmapId& submap_id, const Submap2D* submap, const NodeId& node_id,
      const TrajectoryNode::Data* const constant_data);

  // Returns all constraints accumulated since the previous call.
  Result TakeConstraints();

  // Delete data related to 'submap_id'.
  void DeleteScanMatcher(const SubmapId& submap_id);

 private:
  struct SubmapScanMatcher {
    const Grid2D* grid = nullptr;
    std::unique_ptr<scan_matching::FastCorrelativeScanMatcher2D>
        fast_correlative_scan_matcher;
  };

  const SubmapScanMatcher& GetOrCreateScanMatcher(
      const SubmapId& submap_id, const Grid2D* grid);

  std::optional<Constraint> ComputeConstraint(
      const SubmapId& submap_id, const Submap2D* submap,
      const NodeId& node_id, bool match_full_submap,
      const TrajectoryNode::Data* constant_data,
      const transform::Rigid2d& initial_relative_pose,
      const SubmapScanMatcher& submap_scan_matcher);

  const ConstraintBuilderOptions options_;
  Result constraints_;
  std::size_t num_computations_ = 0;

  std::map<SubmapId, SubmapScanMatcher> submap_scan_matchers_;
  std::map<SubmapId, common::FixedRatioSampler> per_submap_sampler_;

  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;
};

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_BACKEND_CONSTRAINT_ENGINE_2D_H_
