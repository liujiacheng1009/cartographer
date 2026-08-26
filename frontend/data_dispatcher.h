// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef CARTOGRAPHER_CORE_DATA_DISPATCHER_H_
#define CARTOGRAPHER_CORE_DATA_DISPATCHER_H_

#include <compare>
#include <concepts>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "cartographer/foundation/transform.h"
#include "cartographer/foundation/sensor_data.h"

namespace cartographer::sensor {

using SensorData = std::variant<TimedPointCloudData, OdometryData>;

template <typename T>
concept DispatchableSensorData =
    std::same_as<std::remove_cvref_t<T>, TimedPointCloudData> ||
    std::same_as<std::remove_cvref_t<T>, OdometryData>;

// Synchronously merges each trajectory's sensor streams by timestamp. The bag
// reader is the only producer, so no locks or blocking API are needed.
class DataDispatcher {
 public:
  using Callback = std::function<void(const std::string&, SensorData)>;

  DataDispatcher() = default;
  ~DataDispatcher();
  DataDispatcher(const DataDispatcher&) = delete;
  DataDispatcher& operator=(const DataDispatcher&) = delete;

  void AddTrajectory(
      int trajectory_id,
      const absl::flat_hash_set<std::string>& expected_sensor_ids,
      Callback callback);

  template <DispatchableSensorData T>
  void AddSensorData(int trajectory_id, const std::string& sensor_id,
                     T&& data) {
    Add(QueueKey{trajectory_id, sensor_id},
        SensorData(std::forward<T>(data)));
  }

  void FinishTrajectory(int trajectory_id);

 private:
  struct QueueKey {
    int trajectory_id;
    std::string sensor_id;
    auto operator<=>(const QueueKey&) const = default;
  };

  struct Queue {
    std::deque<SensorData> data;
    Callback callback;
    bool finished = false;
  };

  static common::Time GetTime(const SensorData& data);
  void Add(const QueueKey& key, SensorData data);
  void Dispatch();
  common::Time GetCommonStartTime(int trajectory_id);
  void ReportBlocker(const QueueKey& key) const;

  common::Time last_dispatched_time_ = common::Time::min();
  std::map<int, common::Time> common_start_times_;
  std::map<QueueKey, Queue> queues_;
  absl::flat_hash_map<int, std::vector<QueueKey>> trajectory_queues_;
};

}  // namespace cartographer::sensor

#endif  // CARTOGRAPHER_CORE_DATA_DISPATCHER_H_
