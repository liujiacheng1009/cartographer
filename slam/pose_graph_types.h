#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_TYPES_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_TYPES_H_

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "absl/types/optional.h"
#include "cartographer/core/rigid_transform.h"
#include "cartographer/core/time.h"
#include "cartographer/slam/id.h"
#include "cartographer/slam/submaps.h"

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

struct LandmarkNode {
  struct LandmarkObservation {
    int trajectory_id;
    common::Time time;
    transform::Rigid3d landmark_to_tracking_transform;
    double translation_weight;
    double rotation_weight;
  };
  std::vector<LandmarkObservation> landmark_observations;
  absl::optional<transform::Rigid3d> global_landmark_pose;
  bool frozen = false;
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
  absl::optional<transform::Rigid3d> fixed_frame_origin_in_map;
};

enum class TrajectoryState { ACTIVE, FINISHED, FROZEN, DELETED };

using GlobalSlamOptimizationCallback =
    std::function<void(const std::map<int, SubmapId>&,
                       const std::map<int, NodeId>&)>;

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_TYPES_H_
