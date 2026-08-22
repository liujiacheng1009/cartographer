/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/backend/trajectory_connectivity_state.h"

#include <algorithm>

#include "absl/container/flat_hash_map.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

void ConnectedComponents::Add(const int trajectory_id) {
  absl::MutexLock locker(&lock_);
  forest_.emplace(trajectory_id, trajectory_id);
}

void ConnectedComponents::Connect(const int trajectory_id_a,
                                  const int trajectory_id_b) {
  absl::MutexLock locker(&lock_);
  Union(trajectory_id_a, trajectory_id_b);
  ++connection_map_[std::minmax(trajectory_id_a, trajectory_id_b)];
}

void ConnectedComponents::Union(const int trajectory_id_a,
                                const int trajectory_id_b) {
  forest_.emplace(trajectory_id_a, trajectory_id_a);
  forest_.emplace(trajectory_id_b, trajectory_id_b);
  const int representative_a = FindSet(trajectory_id_a);
  const int representative_b = FindSet(trajectory_id_b);
  forest_[representative_a] = representative_b;
}

int ConnectedComponents::FindSet(const int trajectory_id) {
  auto it = forest_.find(trajectory_id);
  CHECK(it != forest_.end());
  if (it->first != it->second) it->second = FindSet(it->second);
  return it->second;
}

bool ConnectedComponents::TransitivelyConnected(const int trajectory_id_a,
                                                 const int trajectory_id_b) {
  if (trajectory_id_a == trajectory_id_b) return true;
  absl::MutexLock locker(&lock_);
  if (!forest_.count(trajectory_id_a) || !forest_.count(trajectory_id_b)) {
    return false;
  }
  return FindSet(trajectory_id_a) == FindSet(trajectory_id_b);
}

std::vector<std::vector<int>> ConnectedComponents::Components() {
  absl::flat_hash_map<int, std::vector<int>> components;
  absl::MutexLock locker(&lock_);
  for (const auto& entry : forest_) {
    components[FindSet(entry.first)].push_back(entry.first);
  }
  std::vector<std::vector<int>> result;
  result.reserve(components.size());
  for (auto& component : components) {
    result.emplace_back(std::move(component.second));
  }
  return result;
}

std::vector<int> ConnectedComponents::GetComponent(const int trajectory_id) {
  absl::MutexLock locker(&lock_);
  const int set_id = FindSet(trajectory_id);
  std::vector<int> trajectory_ids;
  for (const auto& entry : forest_) {
    if (FindSet(entry.first) == set_id) trajectory_ids.push_back(entry.first);
  }
  return trajectory_ids;
}

void TrajectoryConnectivityState::Add(const int trajectory_id) {
  connected_components_.Add(trajectory_id);
}

void TrajectoryConnectivityState::Connect(const int trajectory_id_a,
                                          const int trajectory_id_b,
                                          const common::Time time) {
  if (TransitivelyConnected(trajectory_id_a, trajectory_id_b)) {
    // The trajectories are transitively connected, i.e. they belong to the same
    // connected component. In this case we only update the last connection time
    // of those two trajectories.
    auto sorted_pair = std::minmax(trajectory_id_a, trajectory_id_b);
    if (last_connection_time_map_[sorted_pair] < time) {
      last_connection_time_map_[sorted_pair] = time;
    }
  } else {
    // The connection between these two trajectories is about to join to
    // connected components. Here we update all bipartite trajectory pairs for
    // the two connected components with the connection time. This is to quickly
    // change to a more efficient loop closure search (by constraining the
    // search window) when connected components are joined.
    std::vector<int> component_a =
        connected_components_.GetComponent(trajectory_id_a);
    std::vector<int> component_b =
        connected_components_.GetComponent(trajectory_id_b);
    for (const auto id_a : component_a) {
      for (const auto id_b : component_b) {
        auto id_pair = std::minmax(id_a, id_b);
        last_connection_time_map_[id_pair] = time;
      }
    }
  }
  connected_components_.Connect(trajectory_id_a, trajectory_id_b);
}

bool TrajectoryConnectivityState::TransitivelyConnected(
    const int trajectory_id_a, const int trajectory_id_b) const {
  return connected_components_.TransitivelyConnected(trajectory_id_a,
                                                     trajectory_id_b);
}

std::vector<std::vector<int>> TrajectoryConnectivityState::Components() const {
  return connected_components_.Components();
}

common::Time TrajectoryConnectivityState::LastConnectionTime(
    const int trajectory_id_a, const int trajectory_id_b) {
  const auto sorted_pair = std::minmax(trajectory_id_a, trajectory_id_b);
  return last_connection_time_map_[sorted_pair];
}

}  // namespace mapping
}  // namespace cartographer
