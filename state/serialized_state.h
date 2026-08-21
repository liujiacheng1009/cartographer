#ifndef CARTOGRAPHER_STATE_SERIALIZED_STATE_H_
#define CARTOGRAPHER_STATE_SERIALIZED_STATE_H_

#include <map>
#include <string>
#include <vector>

#include "cartographer/slam/id.h"
#include "cartographer/slam/pose_graph_interface.h"
#include "cartographer/slam/trajectory_node.h"
#include "cartographer/slam/xy_index.h"

namespace cartographer {
namespace io {

struct SerializedGrid2D {
  double resolution;
  Eigen::Vector2d max;
  mapping::CellLimits cell_limits;
  std::vector<uint16> cells;
  Eigen::AlignedBox2i known_cells_box;
  float min_correspondence_cost;
  float max_correspondence_cost;
};

struct SerializedSubmap2D {
  mapping::SubmapId id{0, 0};
  transform::Rigid3d global_pose;
  transform::Rigid3d local_pose;
  int num_range_data;
  bool finished;
  SerializedGrid2D grid;
};

struct SerializedNode {
  mapping::NodeId id{0, 0};
  transform::Rigid3d global_pose;
  mapping::TrajectoryNode::Data data;
};

struct SerializedState {
  std::vector<int> trajectory_ids;
  std::vector<SerializedSubmap2D> submaps;
  std::vector<SerializedNode> nodes;
  std::vector<mapping::PoseGraphInterface::Constraint> constraints;
  std::map<int, mapping::PoseGraphInterface::TrajectoryData> trajectory_data;
  std::map<std::string, transform::Rigid3d> landmark_poses;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_STATE_SERIALIZED_STATE_H_
