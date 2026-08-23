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

#ifndef CARTOGRAPHER_APPLICATION_SLAM_SYSTEM_H_
#define CARTOGRAPHER_APPLICATION_SLAM_SYSTEM_H_

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "cartographer/application/config.h"
#include "cartographer/backend/task_executor.h"
#include "cartographer/frontend/data_dispatcher.h"
#include "cartographer/backend/map_by_id.h"
#include "cartographer/application/slam_options.h"
#include "cartographer/backend/trajectory_backend_2d.h"
#include "cartographer/mapping/submap_texture.h"
#include "cartographer/frontend/frontend_2d.h"

namespace cartographer {
namespace mapping {

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a TrajectoryBackend2D for loop closure.
SlamSystemOptions CreateSlamSystemOptions(
    common::ParameterDictionary* parameter_dictionary);

class SlamSystem {
 public:
  using LocalSlamResultCallback = Frontend2D::LocalSlamResultCallback;
  using SensorId = Frontend2D::SensorId;

  explicit SlamSystem(const SlamSystemOptions &options);
  ~SlamSystem() = default;

  SlamSystem(const SlamSystem &) = delete;
  SlamSystem &operator=(const SlamSystem &) = delete;

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

  TrajectoryBackend2D* backend() { return backend_.get(); }

  int num_trajectory_builders() const {
    return trajectory_builders_.size();
  }

  Frontend2D *GetTrajectoryBuilder(
      int trajectory_id) const {
    return trajectory_builders_.at(trajectory_id).get();
  }

 private:
  const SlamSystemOptions options_;
  common::TaskExecutor task_executor_;

  std::unique_ptr<TrajectoryBackend2D> backend_;

  std::unique_ptr<sensor::DataDispatcher> data_dispatcher_;
  std::vector<std::unique_ptr<Frontend2D>> trajectory_builders_;
};

std::unique_ptr<SlamSystem> CreateSlamSystem(const SlamSystemOptions& options);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_APPLICATION_SLAM_SYSTEM_H_
