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

#include "cartographer/slam/submap_2d.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/core/port.h"
#include "cartographer/slam/probability_grid_range_data_inserter_2d.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

SubmapsOptions2D CreateSubmapsOptions2D(
    common::ParameterDictionary* const parameter_dictionary) {
  SubmapsOptions2D options;
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  *options.mutable_grid_options_2d() = CreateGridOptions2D(
      parameter_dictionary->GetDictionary("grid_options_2d").get());
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(
          parameter_dictionary->GetDictionary("range_data_inserter").get());

  CHECK_GT(options.num_range_data(), 0);
  return options;
}

Submap2D::Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid,
                   ValueConversionTables* conversion_tables)
    : Submap(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))),
      conversion_tables_(conversion_tables) {
  grid_ = std::move(grid);
}

Submap2D::Submap2D(const transform::Rigid3d& local_pose, int num_range_data,
                   bool finished, std::unique_ptr<Grid2D> grid,
                   ValueConversionTables* conversion_tables)
    : Submap(local_pose),
      grid_(std::move(grid)),
      conversion_tables_(conversion_tables) {
  set_num_range_data(num_range_data);
  set_insertion_finished(finished);
}

void Submap2D::ToSubmapTextureResponse(
    const transform::Rigid3d&,
    SubmapTextureResponse* const response) const {
  if (!grid_) return;
  response->submap_version = num_range_data();
  response->textures.emplace_back();
  SubmapTexture* const texture = &response->textures.back();
  grid()->DrawToSubmapTexture(texture, local_pose());
}

void Submap2D::InsertRangeData(
    const sensor::RangeData& range_data,
    const ProbabilityGridRangeDataInserter2D* range_data_inserter) {
  CHECK(grid_);
  CHECK(!insertion_finished());
  range_data_inserter->Insert(range_data,
                              static_cast<ProbabilityGrid*>(grid_.get()));
  set_num_range_data(num_range_data() + 1);
}

void Submap2D::Finish() {
  CHECK(grid_);
  CHECK(!insertion_finished());
  grid_ = grid_->ComputeCroppedGrid();
  set_insertion_finished(true);
}

ActiveSubmaps2D::ActiveSubmaps2D(const SubmapsOptions2D& options)
    : options_(options),
      range_data_inserter_(
          absl::make_unique<ProbabilityGridRangeDataInserter2D>(
              options.range_data_inserter_options()
                  .probability_grid_range_data_inserter_options_2d())) {}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::submaps() const {
  return std::vector<std::shared_ptr<const Submap2D>>(submaps_.begin(),
                                                      submaps_.end());
}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::InsertRangeData(
    const sensor::RangeData& range_data) {
  if (submaps_.empty() ||
      submaps_.back()->num_range_data() == options_.num_range_data()) {
    AddSubmap(range_data.origin.head<2>());
  }
  for (auto& submap : submaps_) {
    submap->InsertRangeData(range_data, range_data_inserter_.get());
  }
  if (submaps_.front()->num_range_data() == 2 * options_.num_range_data()) {
    submaps_.front()->Finish();
  }
  return submaps();
}

std::unique_ptr<ProbabilityGrid> ActiveSubmaps2D::CreateGrid(
    const Eigen::Vector2f& origin) {
  constexpr int kInitialSubmapSize = 100;
  float resolution = options_.grid_options_2d().resolution();
  return absl::make_unique<ProbabilityGrid>(
      MapLimits(resolution,
                origin.cast<double>() + 0.5 * kInitialSubmapSize * resolution *
                                            Eigen::Vector2d::Ones(),
                CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
      &conversion_tables_);
}

void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f& origin) {
  if (submaps_.size() >= 2) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    CHECK(submaps_.front()->insertion_finished());
    submaps_.erase(submaps_.begin());
  }
  submaps_.push_back(absl::make_unique<Submap2D>(
      origin,
      CreateGrid(origin),
      &conversion_tables_));
}

}  // namespace mapping
}  // namespace cartographer
