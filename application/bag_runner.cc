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

#include "cartographer/application/config.h"
#include "cartographer/foundation/geometry.h"
#include "cartographer/foundation/transform.h"
#include "cartographer/application/slam_system.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "yaml-cpp/yaml.h"

DEFINE_string(offline_configuration_directory, "", "Configuration directory");
DEFINE_string(configuration_basenames, "", "Single YAML configuration basename");
DEFINE_string(bag_filenames, "", "Single ROS 2 bag directory");
DEFINE_string(calibration_filename, "", "Sensor calibration YAML");
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

struct Calibration {
  std::string tracking_frame;
  std::string lidar_topic;
  std::string lidar_frame;
  double lidar_time_offset_seconds;
  carto::transform::Rigid3d tracking_from_lidar;
  std::string odometry_topic;
  std::string odometry_reference_frame;
  std::string odometry_child_frame;
  double odometry_time_offset_seconds;
  carto::transform::Rigid3d tracking_from_odometry_child;
};

carto::transform::Rigid3d ReadRigidTransform(const YAML::Node& node,
                                             const std::string& name) {
  CHECK(node.IsSequence() && node.size() == 4)
      << name << " must be a 4x4 matrix.";
  Eigen::Matrix4d matrix;
  for (int row = 0; row != 4; ++row) {
    CHECK(node[row].IsSequence() && node[row].size() == 4)
        << name << " must be a 4x4 matrix.";
    for (int column = 0; column != 4; ++column) {
      matrix(row, column) = node[row][column].as<double>();
      CHECK(std::isfinite(matrix(row, column))) << name << " is not finite.";
    }
  }
  CHECK(matrix.row(3).isApprox(Eigen::RowVector4d(0., 0., 0., 1.), 1e-9))
      << name << " must have homogeneous last row [0, 0, 0, 1].";
  const Eigen::Matrix3d rotation = matrix.topLeftCorner<3, 3>();
  CHECK((rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(),
                                                    1e-6))
      << name << " rotation is not orthonormal.";
  CHECK_LE(std::abs(rotation.determinant() - 1.), 1e-6)
      << name << " rotation determinant must be +1.";
  return {matrix.topRightCorner<3, 1>(), Eigen::Quaterniond(rotation)};
}

Calibration LoadCalibration(const std::string& filename) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(filename);
  } catch (const std::exception& error) {
    LOG(FATAL) << "Failed to load calibration '" << filename
               << "': " << error.what();
  }
  CHECK_EQ(root["schema_version"].as<int>(), 1);
  CHECK_EQ(root["convention"].as<std::string>(), "T_parent_child");
  CHECK_EQ(root["units"]["translation"].as<std::string>(), "meter");
  CHECK_EQ(root["units"]["rotation"].as<std::string>(), "unitless");
  const auto lidar = root["lidar"];
  const auto odometry = root["odometry"];
  CHECK_EQ(lidar["message_type"].as<std::string>(),
           "sensor_msgs/msg/LaserScan");
  CHECK_EQ(odometry["message_type"].as<std::string>(),
           "nav_msgs/msg/Odometry");
  CHECK_EQ(odometry["pose_convention"].as<std::string>(),
           "T_reference_child");
  return {
      NormalizeFrame(root["tracking_frame"].as<std::string>()),
      lidar["topic"].as<std::string>(),
      NormalizeFrame(lidar["frame"].as<std::string>()),
      lidar["time_offset_seconds"].as<double>(),
      ReadRigidTransform(lidar["T_tracking_sensor"], "T_tracking_sensor"),
      odometry["topic"].as<std::string>(),
      NormalizeFrame(odometry["reference_frame"].as<std::string>()),
      NormalizeFrame(odometry["child_frame"].as<std::string>()),
      odometry["time_offset_seconds"].as<double>(),
      ReadRigidTransform(odometry["T_tracking_child"], "T_tracking_child")};
}

carto::sensor::TimedPointCloudData ConvertScan(
    const sensor_msgs::msg::LaserScan& scan,
    const Calibration& calibration) {
  CHECK_EQ(NormalizeFrame(scan.header.frame_id), calibration.lidar_frame);
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
  time += carto::common::FromSeconds(calibration.lidar_time_offset_seconds);
  if (!points.empty()) {
    const float duration = points.back().time;
    time += carto::common::FromSeconds(duration);
    for (auto& point : points) point.time -= duration;
  }
  const auto sensor_to_tracking = calibration.tracking_from_lidar.cast<float>();
  return {time, sensor_to_tracking.translation(),
          carto::sensor::TransformTimedPointCloud(points, sensor_to_tracking),
          {}};
}

carto::sensor::OdometryData ConvertOdometry(
    const nav_msgs::msg::Odometry& odometry,
    const Calibration& calibration) {
  CHECK_EQ(NormalizeFrame(odometry.header.frame_id),
           calibration.odometry_reference_frame);
  CHECK_EQ(NormalizeFrame(odometry.child_frame_id),
           calibration.odometry_child_frame);
  return {FromRos(odometry.header.stamp) +
              carto::common::FromSeconds(
                  calibration.odometry_time_offset_seconds),
          ToRigid3d(odometry.pose.pose) *
              calibration.tracking_from_odometry_child.inverse()};
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
}

void WriteTrajectory(const std::string& filename, int trajectory_id,
                     carto::mapping::TrajectoryBackend2D* backend) {
  if (filename.empty()) return;
  std::ofstream output(filename);
  CHECK(output.good()) << "Cannot open trajectory output '" << filename << "'.";
  output << std::setprecision(17) << "timestamp,x,y,theta\n";
  for (const auto& item : backend->GetTrajectoryNodes()) {
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
  CHECK(!FLAGS_calibration_filename.empty());
  CHECK_EQ(FLAGS_bag_filenames.find(','), std::string::npos)
      << "Only one bag is supported.";

  auto dictionary = carto::common::ParameterDictionary::LoadFile(
      FLAGS_offline_configuration_directory + "/" +
      FLAGS_configuration_basenames);
  const std::string tracking_frame =
      NormalizeFrame(dictionary->GetString("tracking_frame"));
  const Calibration calibration = LoadCalibration(FLAGS_calibration_filename);
  CHECK_EQ(calibration.tracking_frame, tracking_frame);
  CHECK_EQ(calibration.lidar_topic, "/scan");
  CHECK_EQ(calibration.odometry_topic, "/odom");
  const auto map_options = carto::mapping::CreateSlamSystemOptions(
      dictionary->GetDictionary("map_builder").get());
  const auto trajectory_options = carto::mapping::CreateTrajectoryBuilderOptions(
      dictionary->GetDictionary("trajectory_builder").get());
  ValidateBagOnlyConfig(dictionary.get(), trajectory_options);

  auto map_builder = carto::mapping::CreateSlamSystem(map_options);
  if (!FLAGS_offline_load_state_filename.empty()) {
    map_builder->LoadStateFromFile(FLAGS_offline_load_state_filename,
                                   FLAGS_offline_load_frozen_state);
  }
  const std::set<carto::mapping::SlamSystem::SensorId> sensors = {
      {carto::mapping::SlamSystem::SensorId::SensorType::RANGE,
       "scan"},
      {carto::mapping::SlamSystem::SensorId::SensorType::ODOMETRY,
       "odom"}};
  const int trajectory_id = map_builder->AddTrajectoryBuilder(
      sensors, trajectory_options, nullptr);
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'";

  const auto started = std::chrono::steady_clock::now();
  rosbag2_cpp::Reader reader;
  reader.open(FLAGS_bag_filenames);
  const auto types = TopicTypes(&reader);
  auto* trajectory = map_builder->GetTrajectoryBuilder(trajectory_id);
  while (reader.has_next()) {
    const auto message = reader.read_next();
    if (message->topic_name == calibration.lidar_topic) {
      CHECK_EQ(types.at("/scan"), "sensor_msgs/msg/LaserScan");
      trajectory->AddSensorData(
          "scan", ConvertScan(Deserialize<sensor_msgs::msg::LaserScan>(*message),
                              calibration));
    } else if (message->topic_name == calibration.odometry_topic) {
      CHECK_EQ(types.at("/odom"), "nav_msgs/msg/Odometry");
      trajectory->AddSensorData(
          "odom", ConvertOdometry(Deserialize<nav_msgs::msg::Odometry>(*message),
                                  calibration));
    }
  }
  map_builder->FinishTrajectory(trajectory_id);
  map_builder->backend()->RunFinalOptimization();
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
                  map_builder->backend());
  if (!FLAGS_offline_save_state_filename.empty()) {
    CHECK(map_builder->SerializeStateToFile(
        true, FLAGS_offline_save_state_filename));
  }
  rclcpp::shutdown();
  return 0;
}
