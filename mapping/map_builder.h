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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "cartographer/core/parameter_dictionary.h"
#include "cartographer/core/thread_pool.h"
#include "cartographer/core/collator.h"
#include "cartographer/pose_graph/id.h"
#include "cartographer/trajectory/options.h"
#include "cartographer/pose_graph/pose_graph.h"
#include "cartographer/mapping/submap_texture.h"
#include "cartographer/trajectory/trajectory_builder_interface.h"

namespace cartographer {
namespace mapping {

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
MapBuilderOptions CreateMapBuilderOptions(
    common::ParameterDictionary* parameter_dictionary);

class MapBuilder {
 public:
  using LocalSlamResultCallback =
      TrajectoryBuilderInterface::LocalSlamResultCallback;
  using SensorId = TrajectoryBuilderInterface::SensorId;

  explicit MapBuilder(const MapBuilderOptions &options);
  ~MapBuilder() = default;

  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;

  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback);

  int AddTrajectoryForDeserialization();

  void FinishTrajectory(int trajectory_id);

  std::string GetSubmapTexture(const SubmapId &submap_id,
                               SubmapTextureResponse *response);

  bool SerializeStateToFile(bool include_unfinished_submaps,
                            const std::string &filename);

  std::map<int, int> LoadStateFromFile(const std::string &filename,
                                       bool load_frozen_state);

  PoseGraph* pose_graph() { return pose_graph_.get(); }

  int num_trajectory_builders() const {
    return trajectory_builders_.size();
  }

  mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(
      int trajectory_id) const {
    return trajectory_builders_.at(trajectory_id).get();
  }

 private:
  const MapBuilderOptions options_;
  common::ThreadPool thread_pool_;

  std::unique_ptr<PoseGraph> pose_graph_;

  std::unique_ptr<sensor::Collator> sensor_collator_;
  std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
      trajectory_builders_;
};

std::unique_ptr<MapBuilder> CreateMapBuilder(const MapBuilderOptions& options);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
