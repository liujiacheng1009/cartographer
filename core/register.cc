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

#include "cartographer/core/register.h"

#include "cartographer/slam/local_trajectory_builder_2d.h"
#include "cartographer/slam/pose_graph_2d.h"
#include "cartographer/slam/constraint_builder_2d.h"
#include "cartographer/slam/global_trajectory_builder.h"
#include "cartographer/core/trajectory_collator.h"

namespace cartographer {
namespace metrics {

void RegisterAllMetrics(FamilyFactory* registry) {
  mapping::constraints::ConstraintBuilder2D::RegisterMetrics(registry);
  mapping::GlobalTrajectoryBuilderRegisterMetrics(registry);
  mapping::LocalTrajectoryBuilder2D::RegisterMetrics(registry);
  mapping::PoseGraph2D::RegisterMetrics(registry);
  sensor::TrajectoryCollator::RegisterMetrics(registry);
}

}  // namespace metrics
}  // namespace cartographer
