/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "cartographer/frontend/pose_extrapolator.h"

#include "cartographer/foundation/math.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

PoseExtrapolator::PoseExtrapolator(
    const common::Duration pose_queue_duration)
    : pose_queue_duration_(pose_queue_duration),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid2d()} {}

common::Time PoseExtrapolator::GetLastPoseTime() const {
  return timed_pose_queue_.empty() ? common::Time::min()
                                   : timed_pose_queue_.back().time;
}

common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
  return cached_extrapolated_pose_.time;
}

void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid2d& pose) {
  timed_pose_queue_.push_back({time, pose});
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  UpdateVelocitiesFromPoses();
  TrimOdometryData();
  cached_extrapolated_pose_ = {time, pose};
}

void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  odometry_data_.push_back(odometry_data);
  TrimOdometryData();
  if (odometry_data_.size() < 2) return;

  const auto& oldest = odometry_data_.front();
  const auto& newest = odometry_data_.back();
  const double time_delta = common::ToSeconds(oldest.time - newest.time);
  const transform::Rigid2d pose_delta = newest.pose.inverse() * oldest.pose;
  angular_velocity_from_odometry_ = transform::Yaw(pose_delta) / time_delta;
  if (timed_pose_queue_.empty()) return;

  const Eigen::Vector2d velocity_in_tracking_frame =
      pose_delta.translation() / time_delta;
  const double extrapolation_delta =
      common::ToSeconds(newest.time - timed_pose_queue_.back().time);
  const double orientation_at_newest =
      transform::Yaw(timed_pose_queue_.back().pose) +
      extrapolation_delta * angular_velocity_from_odometry_;
  linear_velocity_from_odometry_ =
      Eigen::Rotation2Dd(orientation_at_newest) * velocity_in_tracking_frame;
}

transform::Rigid2d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  CHECK(!timed_pose_queue_.empty());
  const TimedPose& newest = timed_pose_queue_.back();
  CHECK_GE(time, newest.time);
  if (cached_extrapolated_pose_.time == time) {
    return cached_extrapolated_pose_.pose;
  }
  const double delta = common::ToSeconds(time - newest.time);
  const bool use_odometry = odometry_data_.size() >= 2;
  const Eigen::Vector2d translation =
      newest.pose.translation() +
      delta * (use_odometry ? linear_velocity_from_odometry_
                            : linear_velocity_from_poses_);
  const double yaw = common::NormalizeAngleDifference(
      transform::Yaw(newest.pose) +
      delta * (use_odometry ? angular_velocity_from_odometry_
                            : angular_velocity_from_poses_));
  cached_extrapolated_pose_ = {
      time, transform::MakeRigid2(translation, yaw)};
  return cached_extrapolated_pose_.pose;
}

void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) return;
  const TimedPose& oldest = timed_pose_queue_.front();
  const TimedPose& newest = timed_pose_queue_.back();
  const double delta = common::ToSeconds(newest.time - oldest.time);
  if (delta < common::ToSeconds(pose_queue_duration_)) {
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << delta << " s";
    return;
  }
  linear_velocity_from_poses_ =
      (newest.pose.translation() - oldest.pose.translation()) / delta;
  angular_velocity_from_poses_ =
      transform::Yaw(oldest.pose.inverse() * newest.pose) / delta;
}

void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}

}  // namespace mapping
}  // namespace cartographer
