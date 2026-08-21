// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#include <sys/resource.h>
#include <time.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <string>

#include "cartographer/core/parameter_dictionary.h"
#include "cartographer/core/time.h"
#include "cartographer/core/transform.h"
#include "cartographer/slam/map_builder.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"

DEFINE_string(offline_configuration_directory, "", "Configuration directory");
DEFINE_string(configuration_basenames, "", "Single YAML configuration basename");
DEFINE_string(bag_filenames, "", "Single ROS 2 bag directory");
DEFINE_string(offline_load_state_filename, "", "Frozen .swmap input");
DEFINE_bool(offline_load_frozen_state, true, "Load map as frozen");
DEFINE_string(offline_save_state_filename, "", ".swmap output");
DEFINE_string(trajectory_filename, "", "Optimized trajectory CSV output");

namespace {
namespace carto = cartographer;

carto::common::Time FromRos(const builtin_interfaces::msg::Time& stamp) {
  const int64_t nanoseconds = static_cast<int64_t>(stamp.sec) * 1000000000ll +
                              stamp.nanosec;
  return carto::common::FromUniversal(
      carto::common::kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll +
      (nanoseconds + 50) / 100);
}

double ToUnixSeconds(carto::common::Time time) {
  return (carto::common::ToUniversal(time) -
          carto::common::kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) *
         1e-7;
}

carto::transform::Rigid3d ToRigid3d(
    const geometry_msgs::msg::Transform& transform) {
  return {{transform.translation.x, transform.translation.y,
           transform.translation.z},
          {transform.rotation.w, transform.rotation.x, transform.rotation.y,
           transform.rotation.z}};
}

carto::transform::Rigid3d ToRigid3d(const geometry_msgs::msg::Pose& pose) {
  return {{pose.position.x, pose.position.y, pose.position.z},
          {pose.orientation.w, pose.orientation.x, pose.orientation.y,
           pose.orientation.z}};
}

std::string NormalizeFrame(std::string frame) {
  if (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
  return frame;
}

template <typename Message>
Message Deserialize(const rosbag2_storage::SerializedBagMessage& message) {
  rclcpp::SerializedMessage serialized(*message.serialized_data);
  Message result;
  rclcpp::Serialization<Message>().deserialize_message(&serialized, &result);
  return result;
}

std::map<std::string, std::string> TopicTypes(rosbag2_cpp::Reader* reader) {
  std::map<std::string, std::string> result;
  for (const auto& topic : reader->get_all_topics_and_types()) {
    result.emplace(topic.name, topic.type);
  }
  return result;
}

void PreloadTransforms(const std::string& bag, tf2_ros::Buffer* buffer) {
  rosbag2_cpp::Reader reader;
  reader.open(bag);
  const auto types = TopicTypes(&reader);
  while (reader.has_next()) {
    const auto message = reader.read_next();
    if (message->topic_name != "/tf" &&
        message->topic_name != "/tf_static") {
      continue;
    }
    CHECK_EQ(types.at(message->topic_name), "tf2_msgs/msg/TFMessage");
    const auto tf = Deserialize<tf2_msgs::msg::TFMessage>(*message);
    for (auto transform : tf.transforms) {
      transform.header.frame_id = NormalizeFrame(transform.header.frame_id);
      transform.child_frame_id = NormalizeFrame(transform.child_frame_id);
      buffer->setTransform(transform, "bag",
                           message->topic_name == "/tf_static");
    }
  }
}

carto::sensor::TimedPointCloudData ConvertScan(
    const sensor_msgs::msg::LaserScan& scan, const std::string& tracking_frame,
    const tf2_ros::Buffer& buffer) {
  carto::sensor::TimedPointCloud points;
  points.reserve(scan.ranges.size());
  float angle = scan.angle_min;
  for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment) {
    const float range = scan.ranges[i];
    if (std::isfinite(range) && range >= scan.range_min &&
        range <= scan.range_max) {
      points.push_back({{range * std::cos(angle), range * std::sin(angle), 0.f},
                        static_cast<float>(i) * scan.time_increment});
    }
  }
  auto time = FromRos(scan.header.stamp);
  if (!points.empty()) {
    const float duration = points.back().time;
    time += carto::common::FromSeconds(duration);
    for (auto& point : points) point.time -= duration;
  }
  const auto transform = buffer.lookupTransform(
      tracking_frame, NormalizeFrame(scan.header.frame_id),
      rclcpp::Time(scan.header.stamp));
  const auto sensor_to_tracking = ToRigid3d(transform.transform).cast<float>();
  return {time, sensor_to_tracking.translation(),
          carto::sensor::TransformTimedPointCloud(points, sensor_to_tracking),
          {}};
}

carto::sensor::OdometryData ConvertOdometry(
    const nav_msgs::msg::Odometry& odometry, const std::string& tracking_frame,
    const tf2_ros::Buffer& buffer) {
  const auto transform = buffer.lookupTransform(
      tracking_frame, NormalizeFrame(odometry.child_frame_id),
      rclcpp::Time(odometry.header.stamp));
  return {FromRos(odometry.header.stamp),
          ToRigid3d(odometry.pose.pose) *
              ToRigid3d(transform.transform).inverse()};
}

void ValidateBagOnlyConfig(
    carto::common::ParameterDictionary* dictionary,
    const carto::mapping::TrajectoryBuilderOptions& trajectory_options) {
  if (dictionary->HasKey("num_laser_scans"))
    CHECK_EQ(dictionary->GetNonNegativeInt("num_laser_scans"), 1);
  if (dictionary->HasKey("num_multi_echo_laser_scans"))
    CHECK_EQ(dictionary->GetNonNegativeInt("num_multi_echo_laser_scans"), 0)
      << "MultiEchoLaserScan is not supported by the bag-only application.";
  if (dictionary->HasKey("num_point_clouds"))
    CHECK_EQ(dictionary->GetNonNegativeInt("num_point_clouds"), 0)
      << "PointCloud2 is not supported by the bag-only application.";
  if (dictionary->HasKey("use_nav_sat"))
    CHECK(!dictionary->GetBool("use_nav_sat"))
      << "GPS is not supported by the bag-only application.";
  if (dictionary->HasKey("use_landmarks"))
    CHECK(!dictionary->GetBool("use_landmarks"))
      << "Landmarks are not supported by the bag-only application.";
  CHECK(!trajectory_options.trajectory_builder_2d_options().use_imu_data())
      << "IMU topics are not supported by the bag-only application.";
}

void WriteTrajectory(const std::string& filename, int trajectory_id,
                     carto::mapping::PoseGraphInterface* pose_graph) {
  if (filename.empty()) return;
  std::ofstream output(filename);
  CHECK(output.good()) << "Cannot open trajectory output '" << filename << "'.";
  output << std::setprecision(17) << "timestamp,x,y,theta\n";
  for (const auto& item : pose_graph->GetTrajectoryNodes()) {
    if (item.id.trajectory_id != trajectory_id) continue;
    const auto& pose = item.data.global_pose;
    const auto& q = pose.rotation();
    const double yaw = std::atan2(2. * (q.w() * q.z() + q.x() * q.y()),
                                  1. - 2. * (q.y() * q.y() + q.z() * q.z()));
    output << ToUnixSeconds(item.data.time()) << ',' << pose.translation().x()
           << ',' << pose.translation().y() << ',' << yaw << '\n';
  }
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  CHECK(!FLAGS_offline_configuration_directory.empty());
  CHECK(!FLAGS_configuration_basenames.empty());
  CHECK(!FLAGS_bag_filenames.empty());
  CHECK_EQ(FLAGS_bag_filenames.find(','), std::string::npos)
      << "Only one bag is supported.";

  auto dictionary = carto::common::ParameterDictionary::LoadFile(
      FLAGS_offline_configuration_directory + "/" +
      FLAGS_configuration_basenames);
  const std::string tracking_frame =
      NormalizeFrame(dictionary->GetString("tracking_frame"));
  const auto map_options = carto::mapping::CreateMapBuilderOptions(
      dictionary->GetDictionary("map_builder").get());
  const auto trajectory_options = carto::mapping::CreateTrajectoryBuilderOptions(
      dictionary->GetDictionary("trajectory_builder").get());
  ValidateBagOnlyConfig(dictionary.get(), trajectory_options);

  auto map_builder = carto::mapping::CreateMapBuilder(map_options);
  if (!FLAGS_offline_load_state_filename.empty()) {
    map_builder->LoadStateFromFile(FLAGS_offline_load_state_filename,
                                   FLAGS_offline_load_frozen_state);
  }
  const std::set<carto::mapping::TrajectoryBuilderInterface::SensorId> sensors = {
      {carto::mapping::TrajectoryBuilderInterface::SensorId::SensorType::RANGE,
       "scan"},
      {carto::mapping::TrajectoryBuilderInterface::SensorId::SensorType::ODOMETRY,
       "odom"}};
  const int trajectory_id = map_builder->AddTrajectoryBuilder(
      sensors, trajectory_options, nullptr);
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'";

  auto node = rclcpp::Node::make_shared("cartographer_bag_runner");
  tf2_ros::Buffer tf_buffer(node->get_clock(), tf2::durationFromSec(3600.), node);
  tf_buffer.setUsingDedicatedThread(true);
  PreloadTransforms(FLAGS_bag_filenames, &tf_buffer);

  const auto started = std::chrono::steady_clock::now();
  rosbag2_cpp::Reader reader;
  reader.open(FLAGS_bag_filenames);
  const auto types = TopicTypes(&reader);
  auto* trajectory = map_builder->GetTrajectoryBuilder(trajectory_id);
  while (reader.has_next()) {
    const auto message = reader.read_next();
    if (message->topic_name == "/scan") {
      CHECK_EQ(types.at("/scan"), "sensor_msgs/msg/LaserScan");
      trajectory->AddSensorData(
          "scan", ConvertScan(Deserialize<sensor_msgs::msg::LaserScan>(*message),
                              tracking_frame, tf_buffer));
    } else if (message->topic_name == "/odom") {
      CHECK_EQ(types.at("/odom"), "nav_msgs/msg/Odometry");
      trajectory->AddSensorData(
          "odom", ConvertOdometry(Deserialize<nav_msgs::msg::Odometry>(*message),
                                  tracking_frame, tf_buffer));
    }
  }
  map_builder->FinishTrajectory(trajectory_id);
  map_builder->pose_graph()->RunFinalOptimization();
  const double wall_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
          .count();
  LOG(INFO) << "Elapsed wall clock time: " << wall_seconds << " s";
  timespec cpu = {};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu);
  LOG(INFO) << "Elapsed CPU time: " << cpu.tv_sec + 1e-9 * cpu.tv_nsec << " s";
  rusage usage = {};
  CHECK_EQ(getrusage(RUSAGE_SELF, &usage), 0);
  LOG(INFO) << "Peak memory usage: " << usage.ru_maxrss << " KiB";
  WriteTrajectory(FLAGS_trajectory_filename, trajectory_id,
                  map_builder->pose_graph());
  if (!FLAGS_offline_save_state_filename.empty()) {
    CHECK(map_builder->SerializeStateToFile(
        true, FLAGS_offline_save_state_filename));
  }
  rclcpp::shutdown();
  return 0;
}
