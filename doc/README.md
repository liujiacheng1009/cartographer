# SweepNav Cartographer 2D 文档

本目录采用 OpenCV 文档常用的分层方式：先从文档首页进入模块页，再按任务阅读教程、
算法文章和接口参考。目标是让使用者先完成一个可运行流程，再按需深入实现细节。

> 当前产品是面向 ROS 2 bag 的严格 2D 激光 SLAM，不是上游 Cartographer SDK。
> 支持单个 `LaserScan`、轮式里程计、离线建图和冻结地图定位；不读取 `/tf`，也不接收
> IMU、GPS、landmark、fixed-frame pose、MultiEchoLaserScan 或 PointCloud2。

## 快速入口

| 我想做什么 | 从这里开始 |
|---|---|
| 第一次运行 bag 建图 | [入门教程](GETTING_STARTED.md) |
| 理解代码如何分层 | [模块与源码索引](MODULES.md) |
| 理解 scan/odom 如何进入前后端 | [Bag 输入架构](BAG_PIPELINE_ARCHITECTURE.md) |
| 编写或检查标定文件 | [标定文件规范](CALIBRATION_SCHEMA.md) |
| 调整 YAML 参数 | [配置参考](CONFIGURATION_REFERENCE.md) |
| 查找主要 C++ 类型和调用顺序 | [C++ API 概览](API_REFERENCE.md) |
| 修改代码并做完整验证 | [开发与回归指南](DEVELOPMENT_GUIDE.md) |

## 文档类型

### 教程与指南

- [入门教程](GETTING_STARTED.md)：从 bag、标定和配置到轨迹与 `.swmap` 输出。
- [开发与回归指南](DEVELOPMENT_GUIDE.md)：构建、测试、轨迹哈希和提交要求。

### 模块页

- [模块与源码索引](MODULES.md)：application、frontend、mapping、backend、
  scan_matching、serialization、foundation 的职责、依赖和关键类型。

### 算法与架构文章

- [Bag 输入架构](BAG_PIPELINE_ARCHITECTURE.md)：时间语义、数据排序、局部 SLAM、
  node-submap 约束、回环、PGO、线程关系和结束屏障。

### 规范与参考

- [标定文件规范](CALIBRATION_SCHEMA.md)：固定外参、frame、topic、时间偏移与校验规则。
- [配置参考](CONFIGURATION_REFERENCE.md)：完整 YAML 层次、参数作用和调参风险。
- [C++ API 概览](API_REFERENCE.md)：稳定入口、值类型、生命周期与线程约束。

## 推荐阅读路径

```text
使用者
  README → GETTING_STARTED → CALIBRATION_SCHEMA → CONFIGURATION_REFERENCE

算法开发者
  README → MODULES → BAG_PIPELINE_ARCHITECTURE → API_REFERENCE

维护者
  README → MODULES → DEVELOPMENT_GUIDE → API_REFERENCE
```

## 当前稳定入口

- 可执行程序：`cartographer_bag_runner`
- 系统装配：`mapping::SlamSystem`
- 单一前端：`mapping::Frontend2D`
- 单一后端：`mapping::TrajectoryBackend2D`
- 配置样例：`config/iilabs3d_offline.yaml`
- 标定样例：每个 rosbag2 目录中的 `calibration.yaml`
- 状态格式：`.swmap` schema v4（仅保存 `x/y/yaw` 平面位姿）

## 文档维护原则

1. 文档中的命令必须能从仓库根目录执行。
2. 教程引用真实配置和真实入口，不保留已删除的上游 API。
3. 参数参考以 `application/slam_options.h` 和 `application/slam_options.cc` 为准。
4. 输入格式以 `application/bag_runner.cc` 的严格校验为准。
5. 架构图必须同时标明所有权、非拥有依赖和线程边界。
6. 结构重构不得只记录精度 PASS；必须比较确定性的轨迹 SHA256。

## 版本范围

本文档描述仓库当前的 C++20 bag-only 实现。上游 Cartographer、ROS 1 或
`cartographer_ros` 的教程与接口并不自动适用于本工程。
