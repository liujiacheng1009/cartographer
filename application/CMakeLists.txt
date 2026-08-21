cmake_minimum_required(VERSION 3.16)
project(cartographer_bag_application LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(cartographer REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Eigen3 REQUIRED)
find_package(gflags REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(cartographer_bag_runner bag_runner.cc)
target_link_libraries(cartographer_bag_runner PRIVATE
  cartographer_offline_core
  gflags
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  ${sensor_msgs_TARGETS}
  tf2::tf2
  ${tf2_msgs_TARGETS}
  tf2_ros::tf2_ros
)

install(TARGETS cartographer_bag_runner RUNTIME DESTINATION bin)
