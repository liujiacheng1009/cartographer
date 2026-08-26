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

#include "cartographer/backend/constraint_engine_2d.h"

#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

#include "absl/memory/memory.h"
#include "cartographer/foundation/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace constraints {

ConstraintEngine2D::ConstraintEngine2D(
    const ConstraintBuilderOptions& options)
    : options_(options),
      ceres_scan_matcher_(options.ceres_scan_matcher_options()) {}

void ConstraintEngine2D::MaybeAddConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose) {
  if (initial_relative_pose.translation().norm() >
      options_.max_constraint_distance()) {
    return;
  }
  if (!per_submap_sampler_
           .emplace(std::piecewise_construct, std::forward_as_tuple(submap_id),
                    std::forward_as_tuple(options_.sampling_ratio()))
           .first->second.Pulse()) {
    return;
  }

  const auto& scan_matcher =
      GetOrCreateScanMatcher(submap_id, submap->grid());
  ++num_computations_;
  auto constraint = ComputeConstraint(
      submap_id, submap, node_id, false, /* match_full_submap */ constant_data,
      initial_relative_pose, scan_matcher);
  if (constraint) constraints_.push_back(std::move(*constraint));
}

void ConstraintEngine2D::MaybeAddGlobalConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data) {
  const auto& scan_matcher =
      GetOrCreateScanMatcher(submap_id, submap->grid());
  ++num_computations_;
  auto constraint = ComputeConstraint(
      submap_id, submap, node_id, true, /* match_full_submap */ constant_data,
      transform::Rigid2d(), scan_matcher);
  if (constraint) constraints_.push_back(std::move(*constraint));
}

const ConstraintEngine2D::SubmapScanMatcher&
ConstraintEngine2D::GetOrCreateScanMatcher(const SubmapId& submap_id,
                                           const Grid2D* const grid) {
  CHECK(grid);
  auto [it, inserted] = submap_scan_matchers_.try_emplace(submap_id);
  if (inserted) {
    it->second.grid = grid;
    it->second.fast_correlative_scan_matcher =
        absl::make_unique<scan_matching::FastCorrelativeScanMatcher2D>(
            *grid, options_.fast_correlative_scan_matcher_options());
  }
  CHECK_EQ(it->second.grid, grid);
  return it->second;
}

std::optional<ConstraintEngine2D::Constraint>
ConstraintEngine2D::ComputeConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, bool match_full_submap,
    const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const SubmapScanMatcher& submap_scan_matcher) {
  CHECK(submap_scan_matcher.fast_correlative_scan_matcher);
  const transform::Rigid2d initial_pose =
      submap->local_pose() * initial_relative_pose;

  // The 'constraint_transform' (submap i <- node j) is computed from:
  // - a filtered point cloud in node j,
  // - the initial guess 'initial_pose' for (map <- node j),
  // - the result 'pose_estimate' of Match() (map <- node j).
  // - the ComputeSubmapPose() (map <- submap i)
  float score = 0.;
  transform::Rigid2d pose_estimate;

  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  if (match_full_submap) {
    if (submap_scan_matcher.fast_correlative_scan_matcher->MatchFullSubmap(
            constant_data->filtered_point_cloud,
            options_.global_localization_min_score(), &score, &pose_estimate)) {
      CHECK_GT(score, options_.global_localization_min_score());
      CHECK_GE(node_id.trajectory_id, 0);
      CHECK_GE(submap_id.trajectory_id, 0);
    } else {
      return std::nullopt;
    }
  } else {
    if (submap_scan_matcher.fast_correlative_scan_matcher->Match(
            initial_pose, constant_data->filtered_point_cloud,
            options_.min_score(), &score, &pose_estimate)) {
      // We've reported a successful local match.
      CHECK_GT(score, options_.min_score());
    } else {
      return std::nullopt;
    }
  }
  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.
  ceres::Solver::Summary unused_summary;
  ceres_scan_matcher_.Match(pose_estimate.translation(), pose_estimate,
                            constant_data->filtered_point_cloud,
                            *submap_scan_matcher.grid, &pose_estimate,
                            &unused_summary);

  const transform::Rigid2d constraint_transform =
      submap->local_pose().inverse() * pose_estimate;
  Constraint constraint{submap_id,
                        node_id,
                        {constraint_transform,
                         options_.loop_closure_translation_weight(),
                         options_.loop_closure_rotation_weight()},
                        Constraint::INTER_SUBMAP};

  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->filtered_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      const transform::Rigid2d difference =
          initial_pose.inverse() * pose_estimate;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << std::abs(transform::Yaw(difference));
    }
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
  return constraint;
}

ConstraintEngine2D::Result ConstraintEngine2D::TakeConstraints() {
  if (options_.log_matches()) {
    LOG(INFO) << num_computations_ << " computations resulted in "
              << constraints_.size() << " additional constraints.";
  }
  num_computations_ = 0;
  Result result;
  result.swap(constraints_);
  return result;
}

void ConstraintEngine2D::DeleteScanMatcher(const SubmapId& submap_id) {
  submap_scan_matchers_.erase(submap_id);
  per_submap_sampler_.erase(submap_id);
}

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer
