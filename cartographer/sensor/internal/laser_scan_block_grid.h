/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_LASER_SCAN_BLOCK_GRID_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_LASER_SCAN_BLOCK_GRID_H_

#include <map>
#include <utility>
#include <vector>

#include "Eigen/Core"

namespace cartographer {
namespace sensor {
namespace internal {

// Sparse XY block storage used while encoding a planar laser scan.
template <typename ValueType>
class LaserScanBlockGrid {
 public:
  using BlockIndex = std::pair<int, int>;
  struct BlockIndexLess {
    bool operator()(const BlockIndex& lhs, const BlockIndex& rhs) const {
      return lhs.second < rhs.second ||
             (lhs.second == rhs.second && lhs.first < rhs.first);
    }
  };
  using Blocks =
      std::map<BlockIndex, std::vector<ValueType>, BlockIndexLess>;

  std::vector<ValueType>* MutableBlock(const Eigen::Array2i& index) {
    return &blocks_[{index.x(), index.y()}];
  }

  const Blocks& blocks() const { return blocks_; }

 private:
  Blocks blocks_;
};

}  // namespace internal
}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_LASER_SCAN_BLOCK_GRID_H_
