// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#include "cartographer/trajectory/trajectory_frontend_2d.h"

#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace cartographer::mapping {
namespace {

constexpr double kSensorDataRatesLoggingPeriodSeconds = 15.;

}  // namespace

TrajectoryFrontend2D::TrajectoryFrontend2D(
    const TrajectoryBuilderOptions& options,
    sensor::DataDispatcher* const data_dispatcher, const int trajectory_id,
    const std::set<SensorId>& expected_sensor_ids, std::string range_sensor_id,
    TrajectoryBackend2D* const backend,
    LocalSlamResultCallback local_slam_result_callback,
    const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter)
    : data_dispatcher_(data_dispatcher),
      trajectory_id_(trajectory_id),
      backend_(backend),
      local_slam_(absl::make_unique<LocalSlam2D>(
          options.trajectory_builder_2d_options(),
          std::move(range_sensor_id))),
      local_slam_result_callback_(std::move(local_slam_result_callback)),
      pose_graph_odometry_motion_filter_(pose_graph_odometry_motion_filter),
      last_logging_time_(std::chrono::steady_clock::now()) {
  absl::flat_hash_set<std::string> sensor_ids;
  for (const SensorId& sensor_id : expected_sensor_ids) {
    sensor_ids.insert(sensor_id.id);
  }
  data_dispatcher_->AddTrajectory(
      trajectory_id_, sensor_ids,
      [this](const std::string& sensor_id, sensor::SensorData data) {
        HandleSensorData(sensor_id, std::move(data));
      });
}

void TrajectoryFrontend2D::AddSensorData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& data) {
  data_dispatcher_->AddSensorData(trajectory_id_, sensor_id, data);
}

void TrajectoryFrontend2D::AddSensorData(
    const std::string& sensor_id, const sensor::OdometryData& data) {
  data_dispatcher_->AddSensorData(trajectory_id_, sensor_id, data);
}

void TrajectoryFrontend2D::HandleSensorData(
    const std::string& sensor_id, sensor::SensorData data) {
  auto it = rate_timers_
                .try_emplace(sensor_id, common::FromSeconds(
                                            kSensorDataRatesLoggingPeriodSeconds))
                .first;
  it->second.Pulse(std::visit(
      [](const auto& sensor_data) { return sensor_data.time; }, data));

  if (std::chrono::steady_clock::now() - last_logging_time_ >
      common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
    for (const auto& [id, timer] : rate_timers_) {
      LOG(INFO) << id << " rate: " << timer.DebugString();
    }
    last_logging_time_ = std::chrono::steady_clock::now();
  }

  std::visit(
      [this, &sensor_id](const auto& sensor_data) {
        ProcessSensorData(sensor_id, sensor_data);
      },
      data);
}

void TrajectoryFrontend2D::ProcessSensorData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& data) {
  auto matching_result = local_slam_->AddRangeData(sensor_id, data);
  if (matching_result == nullptr) return;

  std::unique_ptr<InsertionResult> insertion_result;
  if (matching_result->insertion_result != nullptr) {
    const NodeId node_id = backend_->AddNode(
        matching_result->insertion_result->constant_data, trajectory_id_,
        matching_result->insertion_result->insertion_submaps);
    CHECK_EQ(node_id.trajectory_id, trajectory_id_);
    insertion_result = absl::make_unique<InsertionResult>(InsertionResult{
        node_id, matching_result->insertion_result->constant_data,
        std::vector<std::shared_ptr<const Submap>>(
            matching_result->insertion_result->insertion_submaps.begin(),
            matching_result->insertion_result->insertion_submaps.end())});
  }
  if (local_slam_result_callback_) {
    local_slam_result_callback_(
        trajectory_id_, matching_result->time, matching_result->local_pose,
        std::move(matching_result->range_data_in_local),
        std::move(insertion_result));
  }
}

void TrajectoryFrontend2D::ProcessSensorData(
    const std::string&, const sensor::OdometryData& data) {
  CHECK(data.pose.IsValid()) << data.pose;
  local_slam_->AddOdometryData(data);
  if (pose_graph_odometry_motion_filter_.has_value() &&
      pose_graph_odometry_motion_filter_->IsSimilar(data.time, data.pose)) {
    return;
  }
  backend_->AddOdometryData(trajectory_id_, data);
}

}  // namespace cartographer::mapping
