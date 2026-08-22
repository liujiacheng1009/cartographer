// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#include "cartographer/core/data_dispatcher.h"

#include <algorithm>
#include <utility>

#include "glog/logging.h"

namespace cartographer::sensor {
namespace {

constexpr std::size_t kQueueWarningSize = 500;

}  // namespace

DataDispatcher::~DataDispatcher() {
  CHECK(queues_.empty())
      << "All trajectories must be finished before destroying DataDispatcher.";
}

common::Time DataDispatcher::GetTime(const SensorData& data) {
  return std::visit([](const auto& value) { return value.time; }, data);
}

void DataDispatcher::AddTrajectory(
    const int trajectory_id,
    const absl::flat_hash_set<std::string>& expected_sensor_ids,
    Callback callback) {
  CHECK(!expected_sensor_ids.empty());
  CHECK(!trajectory_queues_.contains(trajectory_id))
      << "Trajectory already registered: " << trajectory_id;
  auto& keys = trajectory_queues_[trajectory_id];
  keys.reserve(expected_sensor_ids.size());
  for (const auto& sensor_id : expected_sensor_ids) {
    QueueKey key{trajectory_id, sensor_id};
    CHECK(queues_.emplace(key, Queue{{}, callback, false}).second)
        << "Sensor queue already registered: " << sensor_id;
    keys.push_back(std::move(key));
  }
}

void DataDispatcher::Add(const QueueKey& key, SensorData data) {
  const auto found = queues_.find(key);
  if (found == queues_.end()) {
    LOG_EVERY_N(WARNING, 1000)
        << "Ignored data for unknown or finished sensor '" << key.sensor_id
        << "' on trajectory " << key.trajectory_id;
    return;
  }
  Queue& queue = found->second;
  CHECK(!queue.finished) << "Cannot add data to a finished sensor queue.";
  if (!queue.data.empty()) {
    CHECK_LE(GetTime(queue.data.back()), GetTime(data))
        << "Sensor data must be ordered within each stream: " << key.sensor_id;
  }
  queue.data.push_back(std::move(data));
  Dispatch();
}

void DataDispatcher::FinishTrajectory(const int trajectory_id) {
  const auto trajectory = trajectory_queues_.find(trajectory_id);
  CHECK(trajectory != trajectory_queues_.end())
      << "Unknown trajectory: " << trajectory_id;
  for (const auto& key : trajectory->second) {
    const auto queue = queues_.find(key);
    if (queue != queues_.end()) {
      CHECK(!queue->second.finished);
      queue->second.finished = true;
    }
  }
  Dispatch();
  trajectory_queues_.erase(trajectory);
  common_start_times_.erase(trajectory_id);
}

void DataDispatcher::Dispatch() {
  while (true) {
    auto next = queues_.end();
    for (auto queue = queues_.begin(); queue != queues_.end();) {
      if (queue->second.data.empty()) {
        if (queue->second.finished) {
          queue = queues_.erase(queue);
          continue;
        }
        ReportBlocker(queue->first);
        return;
      }
      if (next == queues_.end() ||
          GetTime(queue->second.data.front()) <
              GetTime(next->second.data.front())) {
        next = queue;
      }
      ++queue;
    }
    if (next == queues_.end()) return;

    const common::Time time = GetTime(next->second.data.front());
    CHECK_LE(last_dispatched_time_, time)
        << "Merged sensor data moved backwards in time.";
    const common::Time common_start =
        GetCommonStartTime(next->first.trajectory_id);
    if (time >= common_start) {
      SensorData data = std::move(next->second.data.front());
      next->second.data.pop_front();
      last_dispatched_time_ = time;
      next->second.callback(next->first.sensor_id, std::move(data));
      continue;
    }

    if (next->second.data.size() < 2) {
      if (!next->second.finished) {
        ReportBlocker(next->first);
        return;
      }
      SensorData data = std::move(next->second.data.front());
      next->second.data.pop_front();
      last_dispatched_time_ = time;
      next->second.callback(next->first.sensor_id, std::move(data));
      continue;
    }

    SensorData data = std::move(next->second.data.front());
    next->second.data.pop_front();
    if (GetTime(next->second.data.front()) > common_start) {
      last_dispatched_time_ = time;
      next->second.callback(next->first.sensor_id, std::move(data));
    }
  }
}

common::Time DataDispatcher::GetCommonStartTime(const int trajectory_id) {
  const auto existing = common_start_times_.find(trajectory_id);
  if (existing != common_start_times_.end()) return existing->second;

  common::Time common_start = common::Time::min();
  for (const auto& [key, queue] : queues_) {
    if (key.trajectory_id == trajectory_id) {
      CHECK(!queue.data.empty());
      common_start = std::max(common_start, GetTime(queue.data.front()));
    }
  }
  common_start_times_.emplace(trajectory_id, common_start);
  LOG(INFO) << "All sensor data for trajectory " << trajectory_id
            << " is available starting at '" << common_start << "'.";
  return common_start;
}

void DataDispatcher::ReportBlocker(const QueueKey& key) const {
  for (const auto& [unused_key, queue] : queues_) {
    if (queue.data.size() > kQueueWarningSize) {
      LOG_EVERY_N(WARNING, 60)
          << "Sensor queues are waiting for trajectory " << key.trajectory_id
          << ", sensor '" << key.sensor_id << "'.";
      return;
    }
  }
}

}  // namespace cartographer::sensor
