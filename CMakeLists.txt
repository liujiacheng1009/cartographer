# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.2)

project(cartographer)

get_filename_component(CARTOGRAPHER_SOURCE_PARENT
  "${PROJECT_SOURCE_DIR}/.." ABSOLUTE)

set(CARTOGRAPHER_MAJOR_VERSION 1)
set(CARTOGRAPHER_MINOR_VERSION 0)
set(CARTOGRAPHER_PATCH_VERSION 0)
set(CARTOGRAPHER_VERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION}.${CARTOGRAPHER_PATCH_VERSION})
set(CARTOGRAPHER_SOVERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION})
option(BUILD_PROMETHEUS "build Prometheus monitoring support" false)

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_cartographer_project()

find_package(absl REQUIRED)
set(BOOST_COMPONENTS iostreams)
if(WIN32)
  list(APPEND BOOST_COMPONENTS zlib)
  set(Boost_USE_STATIC_LIBS FALSE)
endif()
find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(SQLite3 REQUIRED)
if(${BUILD_PROMETHEUS})
  find_package( ZLIB REQUIRED )
endif()

include(FindPkgConfig)
if (NOT WIN32)
  PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)
else()
  find_library(CAIRO_LIBRARIES cairo)
endif()

# Install catkin package.xml
install(FILES package.xml DESTINATION share/cartographer)

install(DIRECTORY cmake DESTINATION share/cartographer/)

file(GLOB_RECURSE ALL_LIBRARY_HDRS
  "core/*.h" "slam/*.h" "state/*.h")
set(ALL_LIBRARY_SRCS
  core/collator.cc
  core/fixed_ratio_sampler.cc
  core/histogram.cc
  core/metrics.cc
  core/ordered_multi_queue.cc
  core/parameter_dictionary.cc
  core/point_cloud.cc
  core/range_data.cc
  core/rigid_transform.cc
  core/task.cc
  core/thread_pool.cc
  core/time.cc
  core/timestamped_transform.cc
  core/trajectory_collator.cc
  core/voxel_filter.cc
  slam/ceres_pose.cc
  slam/ceres_scan_matcher_2d.cc
  slam/collated_trajectory_builder.cc
  slam/connected_components.cc
  slam/constraint_builder_2d.cc
  slam/constraint_builder.cc
  slam/correlative_scan_matcher_2d.cc
  slam/eigen_quaterniond_from_two_vectors.cc
  slam/fast_correlative_scan_matcher_2d.cc
  slam/global_trajectory_builder.cc
  slam/grid_2d.cc
  slam/imu_tracker.cc
  slam/local_trajectory_builder_2d.cc
  slam/map_builder.cc
  slam/map_builder_interface.cc
  slam/motion_filter.cc
  slam/occupied_space_cost_function_2d.cc
  slam/optimization_problem_2d.cc
  slam/options.cc
  slam/overlapping_submaps_trimmer_2d.cc
  slam/pose_extrapolator.cc
  slam/pose_extrapolator_interface.cc
  slam/pose_graph.cc
  slam/pose_graph_2d.cc
  slam/pose_graph_trimmer.cc
  slam/probability_grid.cc
  slam/probability_grid_range_data_inserter_2d.cc
  slam/probability_values.cc
  slam/range_data_collator.cc
  slam/range_data_inserter_interface.cc
  slam/ray_to_pixel_mask.cc
  slam/real_time_correlative_scan_matcher_2d.cc
  slam/spa_cost_function_2d.cc
  slam/submap_2d.cc
  slam/trajectory_builder_interface.cc
  slam/trajectory_connectivity_state.cc
  slam/value_conversion_tables.cc
  state/color.cc
  state/file_writer.cc
  state/image.cc
  state/swmap.cc
  state/submap_painter.cc
)

# Remove dotfiles/-folders that could potentially pollute the build.
file(GLOB_RECURSE ALL_DOTFILES ".*/*")
if (ALL_DOTFILES)
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_DOTFILES})
endif()
set(INSTALL_SOURCE_HDRS ${ALL_LIBRARY_HDRS})

add_library(${PROJECT_NAME} STATIC ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})

configure_file(
  ${PROJECT_SOURCE_DIR}/core/config.h.cmake
  ${PROJECT_BINARY_DIR}/cartographer/core/config.h)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${CERES_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${CERES_LIBRARIES})

target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

if (WIN32)
  find_package(glog REQUIRED)
  set(GLOG_LIBRARY glog::glog)
else()
  set(GLOG_LIBRARY glog)
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIBRARY})
target_link_libraries(${PROJECT_NAME} PUBLIC gflags)
if(WIN32)
  # Needed to fix conflict with MSVC's error macro.
  target_compile_definitions(${PROJECT_NAME} PUBLIC -DGLOG_NO_ABBREVIATED_SEVERITIES)
endif()
if(MSVC)
  # Needed for VS 2017 5.8
  target_compile_definitions(${PROJECT_NAME} PUBLIC -D_ENABLE_EXTENDED_ALIGNED_STORAGE -D_USE_MATH_DEFINES)
endif()

if("${CAIRO_INCLUDE_DIRS}")
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    "${CAIRO_INCLUDE_DIRS}")
endif()
target_link_libraries(${PROJECT_NAME} PUBLIC ${CAIRO_LIBRARIES})

target_link_libraries(${PROJECT_NAME} PUBLIC sqlite3
  absl::algorithm
  absl::base
  absl::debugging
  absl::flat_hash_map
  absl::memory
  absl::meta
  absl::numeric
  absl::str_format
  absl::strings
  absl::synchronization
  absl::time
  absl::utility 
)
if (NOT WIN32)
  target_link_libraries(${PROJECT_NAME} PUBLIC pthread)
endif()
if(${BUILD_PROMETHEUS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${ZLIB_LIBRARIES})
  target_link_libraries(${PROJECT_NAME} PUBLIC prometheus-cpp-core)
  target_link_libraries(${PROJECT_NAME} PUBLIC prometheus-cpp-pull)
  target_compile_definitions(${PROJECT_NAME} PUBLIC USE_PROMETHEUS=1)
endif()

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CARTOGRAPHER_SOURCE_PARENT}>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT CartographerExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

foreach(HDR ${INSTALL_SOURCE_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION include/cartographer/${DIR}
  )
endforeach()

set(CARTOGRAPHER_CMAKE_DIR share/cartographer/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  cartographer-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/cartographer/cartographer-config.cmake
  PATH_VARS CARTOGRAPHER_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/cartographer
)

install(
  EXPORT CartographerExport
  DESTINATION share/cartographer/cmake/
  FILE CartographerTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/cmake/cartographer/cartographer-config.cmake
  DESTINATION share/cartographer/
)
