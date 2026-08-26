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

#ifndef CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/foundation/time.h"
#include "cartographer/foundation/geometry.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

struct RangefinderPoint {
  Eigen::Vector3f position;
};

struct TimedRangefinderPoint {
  Eigen::Vector3f position;
  float time;
};

template <class T>
inline RangefinderPoint operator*(const Sophus::SE3<T>& lhs,
                                  const RangefinderPoint& rhs) {
  return {lhs * rhs.position};
}

template <class T>
inline TimedRangefinderPoint operator*(const Sophus::SE3<T>& lhs,
                                       const TimedRangefinderPoint& rhs) {
  return {lhs * rhs.position, rhs.time};
}

inline bool operator==(const RangefinderPoint& lhs,
                       const RangefinderPoint& rhs) {
  return lhs.position == rhs.position;
}

inline bool operator==(const TimedRangefinderPoint& lhs,
                       const TimedRangefinderPoint& rhs) {
  return lhs.position == rhs.position && lhs.time == rhs.time;
}

inline RangefinderPoint ToRangefinderPoint(
    const TimedRangefinderPoint& point) {
  return {point.position};
}

inline TimedRangefinderPoint ToTimedRangefinderPoint(
    const RangefinderPoint& point, float time) {
  return {point.position, time};
}

// Stores 3D positions of points together with some additional data, e.g.
// intensities.
class PointCloud {
 public:
  using PointType = RangefinderPoint;

  PointCloud();
  explicit PointCloud(std::vector<PointType> points);
  PointCloud(std::vector<PointType> points, std::vector<float> intensities);

  // Returns the number of points in the point cloud.
  size_t size() const;
  // Checks whether there are any points in the point cloud.
  bool empty() const;

  const std::vector<PointType>& points() const;
  const std::vector<float>& intensities() const;
  const PointType& operator[](const size_t index) const;

  // Iterator over the points in the point cloud.
  using ConstIterator = std::vector<PointType>::const_iterator;
  ConstIterator begin() const;
  ConstIterator end() const;

  void push_back(PointType value);

  // Creates a PointCloud consisting of all the points for which `predicate`
  // returns true, together with the corresponding intensities.
  template <class UnaryPredicate>
  PointCloud copy_if(UnaryPredicate predicate) const {
    std::vector<PointType> points;
    std::vector<float> intensities;

    // Note: benchmarks show that it is better to have this conditional outside
    // the loop.
    if (intensities_.empty()) {
      for (size_t index = 0; index < size(); ++index) {
        const PointType& point = points_[index];
        if (predicate(point)) {
          points.push_back(point);
        }
      }
    } else {
      for (size_t index = 0; index < size(); ++index) {
        const PointType& point = points_[index];
        if (predicate(point)) {
          points.push_back(point);
          intensities.push_back(intensities_[index]);
        }
      }
    }

    return PointCloud(points, intensities);
  }

 private:
  // For 2D points, the third entry is 0.f.
  std::vector<PointType> points_;
  // Intensities are optional. If non-empty, they must have the same size as
  // points.
  std::vector<float> intensities_;
};

// Stores 3D positions of points with their relative measurement time in the
// fourth entry. Time is in seconds, increasing and relative to the moment when
// the last point was acquired. So, the fourth entry for the last point is 0.f.
// If timing is not available, all fourth entries are 0.f. For 2D points, the
// third entry is 0.f (and the fourth entry is time).
using TimedPointCloud = std::vector<TimedRangefinderPoint>;

// TODO(wohe): Retained for cartographer_ros. To be removed once it is no
// longer used there.
struct PointCloudWithIntensities {
  TimedPointCloud points;
  std::vector<float> intensities;
};

// Transforms 'point_cloud' according to 'transform'.
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);

// Transforms 'point_cloud' according to 'transform'.
TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
PointCloud CropPointCloud(const PointCloud& point_cloud, float min_z,
                          float max_z);

// A batch of laser rays expressed in one common frame. Returns end at an
// obstacle; misses end at the configured free-space insertion distance.
struct RangeData {
  Eigen::Vector3f origin;
  PointCloud returns;
  PointCloud misses;
};

struct OdometryData {
  common::Time time;
  transform::Rigid2d pose;
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

RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform);
RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
