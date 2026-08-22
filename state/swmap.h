#ifndef CARTOGRAPHER_STATE_SWMAP_H_
#define CARTOGRAPHER_STATE_SWMAP_H_

#include <map>
#include <string>
#include <vector>

#include "cartographer/slam/id.h"
#include "cartographer/slam/pose_graph_types.h"
#include "cartographer/slam/trajectory_node.h"
#include "cartographer/slam/grid_2d.h"

namespace cartographer {
namespace mapping {
class PoseGraph;
}
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
  std::vector<mapping::Constraint> constraints;
  std::map<int, mapping::TrajectoryData> trajectory_data;
};

bool WriteSwMap(const std::string& filename,
                const mapping::PoseGraph& pose_graph,
                bool include_unfinished_submaps);
SerializedState ReadSwMap(const std::string& filename);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_STATE_SWMAP_H_
