#ifndef CARTOGRAPHER_STATE_MAP_DATABASE_H_
#define CARTOGRAPHER_STATE_MAP_DATABASE_H_

#include <string>

#include "cartographer/state/serialized_state.h"

namespace cartographer {
namespace mapping {
class PoseGraph;
}
namespace io {

bool WriteSwMap(const std::string& filename,
                const mapping::PoseGraph& pose_graph,
                bool include_unfinished_submaps);
SerializedState ReadSwMap(const std::string& filename);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_STATE_MAP_DATABASE_H_
