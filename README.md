# Localization2D

Localization2D 是一个面向 ROS 2 `rosbag2` 的离线二维激光 SLAM 与冻结地图定位
实现。它源自 Cartographer，但已收敛为一个严格的 2D 工程：以单个 `LaserScan` 和
轮式 `Odometry` 为输入，输出后端最终优化后的轨迹 CSV 与可复用的 `.swmap` 地图状态。

它不是上游 Cartographer SDK，也不兼容其 ROS 1、Lua 配置或 3D 教程。

## 能力与边界

- 2D 子地图、局部 scan matching、非局部约束、闭环和 Ceres 位姿图优化。
- 新图建图，以及加载冻结 `.swmap` 后的独立复访定位。
- 固定外参与传感器时间偏移由 bag 旁的 `calibration.yaml` 显式描述并严格校验。
- 使用 Eigen、Sophus 1.22.10、Ceres、Abseil 和 ROS 2 Humble；构建目标为 C++20。
- 仅接收一个 `sensor_msgs/msg/LaserScan` 和一个 `nav_msgs/msg/Odometry` 话题；不读取
  `/tf`、`/tf_static`、IMU、GPS、PointCloud2、MultiEchoLaserScan 或 landmark。

## 输入与输出

```text
rosbag2 + calibration.yaml
        │
        ├── LaserScan ──┐
        └── Odometry ───┼──> Local SLAM → loop closure / PGO
                        │
                        └──> trajectory.csv + map.swmap
```

`trajectory.csv` 格式为 `timestamp,x,y,theta`，其中位姿是 final optimization 完成后的
全局结果。标定文件必须与 bag 放在一起，包含 topic、frame、刚体外参和各传感器的
`time_offset_seconds`。

## 构建与运行

本仓库通常作为 SweepNav 2D 的子模块，保留在
`third_party/cartographer` 路径下；源码使用 `cartographer/...` 头文件前缀，因此不要仅因
远端仓库改名而重命名这个目录。

在 SweepNav 2D 根目录初始化并构建：

```bash
git submodule update --init --recursive
docker compose build cartographer
```

容器中生成的程序为 `cartographer_bag_runner`。完整的建图、冻结地图定位和命令行示例见
[入门教程](doc/GETTING_STARTED.md)。本机构建需要预先安装 ROS 2 Humble、Ceres、Eigen、
Sophus 1.22.10、Abseil、gflags、glog、yaml-cpp、SQLite3 和 Zlib。

## 文档与验证

- [文档首页](doc/README.md)：按使用、算法开发和维护三条路径组织。
- [标定文件规范](doc/CALIBRATION_SCHEMA.md)：外参与时间偏移的字段语义。
- [配置参考](doc/CONFIGURATION_REFERENCE.md)：严格 YAML 参数和调参边界。
- [开发与回归指南](doc/DEVELOPMENT_GUIDE.md)：分级回归、真实数据集与轨迹 SHA256 门禁。

结构重构不能仅比较精度阈值；必须对六个基线会话比较最终轨迹的 SHA256。算法或标定语义
变化时，还必须记录 ATE、AOE、RPE 和 Coverage，并解释差异来源。

## 许可证

本项目继承并遵守 [Apache License 2.0](LICENSE)。上游版权与许可声明保留在源码中。
