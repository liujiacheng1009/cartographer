// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#include "cartographer/slam/options.h"

#include "cartographer/core/voxel_filter.h"
#include "cartographer/slam/ceres_scan_matcher_2d.h"
#include "cartographer/slam/motion_filter.h"
#include "cartographer/slam/pose_extrapolator_interface.h"
#include "cartographer/slam/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/slam/submap_2d.h"

namespace cartographer {
namespace common {

mapping::CeresSolverOptions CreateCeresSolverOptionsConfig(
    ParameterDictionary* parameter_dictionary) {
  mapping::CeresSolverOptions config;
  config.set_use_nonmonotonic_steps(
      parameter_dictionary->GetBool("use_nonmonotonic_steps"));
  config.set_max_num_iterations(
      parameter_dictionary->GetNonNegativeInt("max_num_iterations"));
  config.set_num_threads(parameter_dictionary->GetNonNegativeInt("num_threads"));
  CHECK_GT(config.max_num_iterations(), 0);
  CHECK_GT(config.num_threads(), 0);
  return config;
}

ceres::Solver::Options CreateCeresSolverOptions(
    const mapping::CeresSolverOptions& config) {
  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = config.use_nonmonotonic_steps();
  options.max_num_iterations = config.max_num_iterations();
  options.num_threads = config.num_threads();
  return options;
}

}  // namespace common

namespace mapping {

namespace scan_matching {

RealTimeCorrelativeScanMatcherOptions
CreateRealTimeCorrelativeScanMatcherOptions(
    common::ParameterDictionary* const parameter_dictionary) {
  RealTimeCorrelativeScanMatcherOptions options;
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  options.set_translation_delta_cost_weight(
      parameter_dictionary->GetDouble("translation_delta_cost_weight"));
  options.set_rotation_delta_cost_weight(
      parameter_dictionary->GetDouble("rotation_delta_cost_weight"));
  CHECK_GE(options.translation_delta_cost_weight(), 0.);
  CHECK_GE(options.rotation_delta_cost_weight(), 0.);
  return options;
}

}  // namespace scan_matching

LocalTrajectoryBuilderOptions2D CreateLocalTrajectoryBuilderOptions2D(
    common::ParameterDictionary* const parameter_dictionary) {
  LocalTrajectoryBuilderOptions2D options;
  options.set_min_range(parameter_dictionary->GetDouble("min_range"));
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));
  options.set_min_z(parameter_dictionary->GetDouble("min_z"));
  options.set_max_z(parameter_dictionary->GetDouble("max_z"));
  options.set_missing_data_ray_length(
      parameter_dictionary->GetDouble("missing_data_ray_length"));
  options.set_num_accumulated_range_data(
      parameter_dictionary->GetInt("num_accumulated_range_data"));
  options.set_voxel_filter_size(
      parameter_dictionary->GetDouble("voxel_filter_size"));
  options.set_use_online_correlative_scan_matching(
      parameter_dictionary->GetBool("use_online_correlative_scan_matching"));
  *options.mutable_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary->GetDictionary("adaptive_voxel_filter").get());
  *options.mutable_loop_closure_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary
              ->GetDictionary("loop_closure_adaptive_voxel_filter")
              .get());
  *options.mutable_real_time_correlative_scan_matcher_options() =
      scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
          parameter_dictionary
              ->GetDictionary("real_time_correlative_scan_matcher")
              .get());
  *options.mutable_ceres_scan_matcher_options() =
      scan_matching::CreateCeresScanMatcherOptions2D(
          parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
  *options.mutable_motion_filter_options() = CreateMotionFilterOptions(
      parameter_dictionary->GetDictionary("motion_filter").get());
  *options.mutable_pose_extrapolator_options() = CreatePoseExtrapolatorOptions(
      parameter_dictionary->GetDictionary("pose_extrapolator").get());
  *options.mutable_submaps_options() = CreateSubmapsOptions2D(
      parameter_dictionary->GetDictionary("submaps").get());
  options.set_use_imu_data(parameter_dictionary->GetBool("use_imu_data"));
  return options;
}

namespace optimization {

OptimizationProblemOptions CreateOptimizationProblemOptions(
    common::ParameterDictionary* const parameter_dictionary) {
  OptimizationProblemOptions options;
  options.set_huber_scale(parameter_dictionary->GetDouble("huber_scale"));
  options.set_odometry_translation_weight(
      parameter_dictionary->GetDouble("odometry_translation_weight"));
  options.set_odometry_rotation_weight(
      parameter_dictionary->GetDouble("odometry_rotation_weight"));
  options.set_local_slam_pose_translation_weight(
      parameter_dictionary->GetDouble("local_slam_pose_translation_weight"));
  options.set_local_slam_pose_rotation_weight(
      parameter_dictionary->GetDouble("local_slam_pose_rotation_weight"));
  options.set_fixed_frame_pose_translation_weight(
      parameter_dictionary->GetDouble("fixed_frame_pose_translation_weight"));
  options.set_fixed_frame_pose_rotation_weight(
      parameter_dictionary->GetDouble("fixed_frame_pose_rotation_weight"));
  options.set_fixed_frame_pose_use_tolerant_loss(
      parameter_dictionary->GetBool("fixed_frame_pose_use_tolerant_loss"));
  options.set_fixed_frame_pose_tolerant_loss_param_a(
      parameter_dictionary->GetDouble("fixed_frame_pose_tolerant_loss_param_a"));
  options.set_fixed_frame_pose_tolerant_loss_param_b(
      parameter_dictionary->GetDouble("fixed_frame_pose_tolerant_loss_param_b"));
  options.set_log_solver_summary(
      parameter_dictionary->GetBool("log_solver_summary"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsConfig(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
