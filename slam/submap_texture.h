#ifndef CARTOGRAPHER_SLAM_SUBMAP_TEXTURE_H_
#define CARTOGRAPHER_SLAM_SUBMAP_TEXTURE_H_

#include <string>
#include <vector>

#include "cartographer/core/rigid_transform.h"

namespace cartographer {
namespace mapping {

struct SubmapTexture {
  std::string cells;
  int width = 0;
  int height = 0;
  double resolution = 0.;
  transform::Rigid3d slice_pose;
};

struct SubmapTextureResponse {
  int submap_version = 0;
  std::vector<SubmapTexture> textures;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_SLAM_SUBMAP_TEXTURE_H_
