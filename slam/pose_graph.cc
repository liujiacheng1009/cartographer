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

#include "cartographer/slam/pose_graph.h"

#include "cartographer/slam/constraint_builder.h"
#include "cartographer/slam/options.h"
#include "cartographer/core/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

void PopulateOverlappingSubmapsTrimmerOptions2D(
    PoseGraphOptions* const pose_graph_options,
    common::ParameterDictionary* const parameter_dictionary) {
  constexpr char kDictionaryKey[] = "overlapping_submaps_trimmer_2d";
  if (!parameter_dictionary->HasKey(kDictionaryKey)) return;

  auto options_dictionary = parameter_dictionary->GetDictionary(kDictionaryKey);
  auto* options = pose_graph_options->mutable_overlapping_submaps_trimmer_2d();
  options->set_fresh_submaps_count(
      options_dictionary->GetInt("fresh_submaps_count"));
  options->set_min_covered_area(
      options_dictionary->GetDouble("min_covered_area"));
  options->set_min_added_submaps_count(
      options_dictionary->GetInt("min_added_submaps_count"));
}

PoseGraphOptions CreatePoseGraphOptions(
    common::ParameterDictionary* const parameter_dictionary) {
  PoseGraphOptions options;
  options.set_optimize_every_n_nodes(
      parameter_dictionary->GetInt("optimize_every_n_nodes"));
  *options.mutable_constraint_builder_options() =
      constraints::CreateConstraintBuilderOptions(
          parameter_dictionary->GetDictionary("constraint_builder").get());
  options.set_matcher_translation_weight(
      parameter_dictionary->GetDouble("matcher_translation_weight"));
  options.set_matcher_rotation_weight(
      parameter_dictionary->GetDouble("matcher_rotation_weight"));
  *options.mutable_optimization_problem_options() =
      optimization::CreateOptimizationProblemOptions(
          parameter_dictionary->GetDictionary("optimization_problem").get());
  options.set_max_num_final_iterations(
      parameter_dictionary->GetNonNegativeInt("max_num_final_iterations"));
  CHECK_GT(options.max_num_final_iterations(), 0);
  options.set_global_sampling_ratio(
      parameter_dictionary->GetDouble("global_sampling_ratio"));
  options.set_log_residual_histograms(
      parameter_dictionary->GetBool("log_residual_histograms"));
  options.set_global_constraint_search_after_n_seconds(
      parameter_dictionary->GetDouble(
          "global_constraint_search_after_n_seconds"));
  PopulateOverlappingSubmapsTrimmerOptions2D(&options, parameter_dictionary);
  return options;
}

}  // namespace mapping
}  // namespace cartographer
