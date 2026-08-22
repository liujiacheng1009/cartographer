# 使用 rosbag2 完成第一次 2D 建图

## 目标

本教程从一个包含 `LaserScan` 和 `Odometry` 的 rosbag2 出发，生成优化轨迹 CSV 和
`.swmap` 地图状态，并说明每个输入为什么必要。

## 前置条件

- ROS 2 Humble rosbag2 数据目录；
- 一个单线或等价平面激光雷达话题；
- 一个轮式里程计话题；
- 与 bag 放在固定相对位置的 `calibration.yaml`；
- 仓库根目录可使用 Docker Compose。

当前程序不会读取 `/tf` 或 `/tf_static`。固定外参必须写入标定文件，动态运动只来自
scan matching 与 odometry。

## 1. 检查数据目录

推荐布局：

```text
dataset/
├── sequence_2d/
│   ├── metadata.yaml
│   ├── sequence_2d_0.db3
│   └── calibration.yaml
└── ground_truth.tum          # 可选，仅由 benchmark 使用
```

`metadata.yaml` 与 `.db3` 是标准 rosbag2 文件。`calibration.yaml` 的完整字段见
[标定文件规范](CALIBRATION_SCHEMA.md)。程序会验证消息类型、topic、frame、矩阵正交性
和齐次矩阵最后一行，错误时直接终止，不做静默猜测。

## 2. 检查 SLAM 配置

基线配置位于：

```text
third_party/cartographer/config/iilabs3d_offline.yaml
```

至少确认以下参数符合传感器：

- `tracking_frame` 与标定文件一致；
- `min_range`、`max_range` 不超出雷达可靠量程；
- `grid_options_2d.resolution` 满足地图精度和内存要求；
- `num_background_threads` 与目标 CPU 预算匹配；
- `num_range_data` 与期望 submap 尺寸匹配。

配置解析是严格的：未知字段、字段重复读取、字段未消费、类型错误或缺失必需字段都会
失败。完整说明见[配置参考](CONFIGURATION_REFERENCE.md)。

## 3. 构建 Release 镜像

从主仓库根目录执行：

```bash
docker compose build cartographer
```

镜像使用 C++20 和 Release 配置构建 `cartographer_bag_runner`。源码发生变化后必须重建，
否则 benchmark 可能误用旧镜像。

## 4. 运行建图

runner 的等价命令结构如下：

```bash
cartographer_bag_runner \
  --offline_configuration_directory=/path/to/config \
  --configuration_basenames=iilabs3d_offline.yaml \
  --bag_filenames=/path/to/sequence_2d \
  --calibration_filename=/path/to/sequence_2d/calibration.yaml \
  --trajectory_filename=/tmp/trajectory.csv \
  --offline_save_state_filename=/tmp/map.swmap
```

每个 flag 都是单值；当前不支持逗号分隔的多 bag、多配置或多雷达。轨迹 CSV 格式是：

```text
timestamp,x,y,theta
```

其中 timestamp 为 Unix 秒，`x/y` 单位为米，`theta` 单位为弧度；位姿是后端最终优化后
的 global pose。

## 5. 理解运行过程

```text
YAML + calibration
        │
rosbag2 ├─ LaserScan → 标定外参 → TimedPointCloudData ┐
        └─ Odometry → child 外参 → OdometryData       ├→ DataDispatcher
                                                        ↓
Frontend2D → LocalSlam2D → node/submap → TrajectoryBackend2D
                                      → loop closure → PGO
                                      → CSV / .swmap
```

bag 只读一遍。每帧 scan 内的点带相对时间，最后一个有效点的偏移为零；scan 的数据时刻
是最后一个有效点时刻。`DataDispatcher` 再按数据时刻合并 scan 和 odom，保证前端不会
看到倒序输入。

## 6. 运行冻结地图定位

在已有 `.swmap` 上定位时增加：

```bash
--offline_load_state_filename=/tmp/map.swmap \
--offline_load_frozen_state=true
```

加载轨迹会被冻结，新 bag 创建新的活动轨迹。历史 submap 可以参与约束搜索，但不会再被
插入或优化为活动地图内容。结束时仍可输出包含旧地图和新轨迹的状态。

## 7. 判断结果是否有效

最低检查项：

- runner 正常退出；
- CSV 除表头外存在节点；
- 时间严格递增；
- 所有数值有限；
- `.swmap` 非空且可再次加载；
- 日志中 final optimization 完成。

算法回归不能只看地图图片。仓库 benchmark 使用外部 OptiTrack 真值计算 Coverage、ATE、
AOE 和 RPE，并另外对轨迹文件做 SHA256 对照。

## 常见失败

| 症状 | 常见原因 | 处理 |
|---|---|---|
| frame mismatch | bag frame 与标定字段不同 | 去掉前导 `/` 后逐字核对 |
| transform is not orthonormal | 外参矩阵含缩放或录入误差 | 重新导出刚体变换 |
| unsupported topic/type | 输入不是 LaserScan/Odometry | 先离线转换数据集 |
| 时间倒序 | 单个传感器时间戳回退 | 修复数据，而非在 dispatcher 中排序掩盖 |
| 轨迹漂移 | 外参方向写反或 odom pose 约定错误 | 检查 `T_parent_child` 方向 |
| CPU 占用高 | 后台线程和 Ceres 线程叠加 | 参考配置页的并发章节 |

## 下一步

- 数据路径细节：[Bag 输入架构](BAG_PIPELINE_ARCHITECTURE.md)
- 参数含义：[配置参考](CONFIGURATION_REFERENCE.md)
- 代码入口：[模块与源码索引](MODULES.md)

