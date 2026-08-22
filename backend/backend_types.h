#ifndef CARTOGRAPHER_BACKEND_TYPES_H_
#define CARTOGRAPHER_BACKEND_TYPES_H_

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "absl/types/optional.h"
#include "cartographer/foundation/geometry.h"
#include "cartographer/foundation/geometry.h"
#include "cartographer/backend/id.h"
#include "cartographer/mapping/submaps.h"

namespace cartographer {
namespace mapping {

struct Constraint {
  struct Pose {
    transform::Rigid3d zbar_ij;
    double translation_weight;
    double rotation_weight;
  };
  SubmapId submap_id;
  NodeId node_id;
  Pose pose;
  enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
};

struct SubmapPose {
  int version;
  transform::Rigid3d pose;
};

struct SubmapData {
  std::shared_ptr<const Submap> submap;
  transform::Rigid3d pose;
};

struct TrajectoryData {
  double gravity_constant = 9.8;
  std::array<double, 4> imu_calibration{{1., 0., 0., 0.}};
};

enum class TrajectoryState { ACTIVE, FINISHED, FROZEN, DELETED };

using GlobalSlamOptimizationCallback =
    std::function<void(const std::map<int, SubmapId>&,
                       const std::map<int, NodeId>&)>;

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_BACKEND_TYPES_H_
