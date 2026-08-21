// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef CARTOGRAPHER_CORE_SENSOR_DATA_H_
#define CARTOGRAPHER_CORE_SENSOR_DATA_H_

#include <string>
#include <vector>

#include "Eigen/Core"
#include "absl/types/optional.h"
#include "cartographer/core/point_cloud.h"
#include "cartographer/core/rigid_transform.h"
#include "cartographer/core/time.h"

namespace cartographer {
namespace sensor {

struct ImuData {
  common::Time time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};

struct OdometryData {
  common::Time time;
  transform::Rigid3d pose;
};

struct FixedFramePoseData {
  common::Time time;
  absl::optional<transform::Rigid3d> pose;
};

struct LandmarkObservation {
  std::string id;
  transform::Rigid3d landmark_to_tracking_transform;
  double translation_weight;
  double rotation_weight;
};

struct LandmarkData {
  common::Time time;
  std::vector<LandmarkObservation> landmark_observations;
};

struct TimedPointCloudData {
  common::Time time;
  Eigen::Vector3f origin;
  TimedPointCloud ranges;
  std::vector<float> intensities;
};

struct TimedPointCloudOriginData {
  struct RangeMeasurement {
    TimedRangefinderPoint point_time;
    float intensity;
    size_t origin_index;
  };
  common::Time time;
  std::vector<Eigen::Vector3f> origins;
  std::vector<RangeMeasurement> ranges;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_CORE_SENSOR_DATA_H_
