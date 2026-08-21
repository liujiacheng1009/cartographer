#include "cartographer/core/range_data.h"

namespace cartographer {
namespace sensor {

RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform) {
  return RangeData{transform * range_data.origin,
                   TransformPointCloud(range_data.returns, transform),
                   TransformPointCloud(range_data.misses, transform)};
}

RangeData CropRangeData(const RangeData& range_data, const float min_z,
                        const float max_z) {
  return RangeData{range_data.origin,
                   CropPointCloud(range_data.returns, min_z, max_z),
                   CropPointCloud(range_data.misses, min_z, max_z)};
}

}  // namespace sensor
}  // namespace cartographer
