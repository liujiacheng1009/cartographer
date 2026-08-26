/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_COST_HELPERS_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_COST_HELPERS_H_

#include <array>

#include "cartographer/foundation/transform.h"
#include "cartographer/foundation/math.h"

namespace cartographer {
namespace mapping {
namespace optimization {

// 'start' and 'end' have the format [x, y, yaw].
template <typename T>
static std::array<T, 3> ComputeUnscaledError(
    const transform::Rigid2d& relative_pose, const T* const start,
    const T* const end) {
  const T cos_theta_i = cos(start[2]);
  const T sin_theta_i = sin(start[2]);
  const T delta_x = end[0] - start[0];
  const T delta_y = end[1] - start[1];
  const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y,
                  -sin_theta_i * delta_x + cos_theta_i * delta_y,
                  end[2] - start[2]};
  return {{T(relative_pose.translation().x()) - h[0],
           T(relative_pose.translation().y()) - h[1],
           common::NormalizeAngleDifference(
               T(transform::Yaw(relative_pose)) - h[2])}};
}

template <typename T>
std::array<T, 3> ScaleError(const std::array<T, 3>& error,
                            const double translation_weight,
                            const double rotation_weight) {
  return {{error[0] * translation_weight, error[1] * translation_weight,
           error[2] * rotation_weight}};
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_COST_HELPERS_H_
