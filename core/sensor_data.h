// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef CARTOGRAPHER_CORE_SENSOR_DATA_H_
#define CARTOGRAPHER_CORE_SENSOR_DATA_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/core/point_cloud.h"
#include "cartographer/core/rigid_transform.h"
#include "cartographer/core/rigid_transform.h"

namespace cartographer {
namespace sensor {

struct OdometryData {
  common::Time time;
  transform::Rigid3d pose;
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
