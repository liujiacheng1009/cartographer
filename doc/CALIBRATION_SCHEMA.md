# Bag 传感器标定文件规范

## 文件位置

规范化数据包目录必须同时包含：

```text
native_input/
├── calibration.yaml
├── metadata.yaml
└── native_input.db3
```

bag 只保存随时间变化的 `/scan` 和 `/odom` 观测。固定外参、frame 约定和时间偏移
属于数据集元数据，只能写入 `calibration.yaml`，不得通过 `/tf` 或 `/tf_static`
重复发布。

## Schema v1

```yaml
# T_parent_child maps coordinates from child into parent.
schema_version: 1
convention: T_parent_child
units:
  translation: meter
  rotation: unitless
tracking_frame: eve/base_footprint

lidar:
  topic: /scan
  message_type: sensor_msgs/msg/LaserScan
  frame: eve/laser
  time_offset_seconds: 0
  T_tracking_sensor:
    - [1, 0, 0, 0.12]
    - [0, 1, 0, 0]
    - [0, 0, 1, 0.3174]
    - [0, 0, 0, 1]

odometry:
  topic: /odom
  message_type: nav_msgs/msg/Odometry
  reference_frame: eve/odom
  child_frame: eve/base_footprint
  pose_convention: T_reference_child
  time_offset_seconds: 0
  T_tracking_child:
    - [1, 0, 0, 0]
    - [0, 1, 0, 0]
    - [0, 0, 1, 0]
    - [0, 0, 0, 1]
```

## 坐标约定

采用与 OpenVINS 外参文件相近的 4×4 齐次矩阵表达，但显式命名变换方向：

```text
p_parent = T_parent_child × p_child
```

因此：

- `T_tracking_sensor` 把 lidar frame 中的点转换到 tracking frame；
- `T_tracking_child` 描述 odometry child frame 到 tracking frame 的固定关系；
- odometry 消息 pose 按 `T_reference_child` 解释；
- runner 输出 tracking pose 时计算
  `T_reference_tracking = T_reference_child × inverse(T_tracking_child)`。

矩阵旋转部分必须正交且行列式为 `+1`，平移单位固定为米，最后一行必须是
`[0, 0, 0, 1]`。四元数只存在于源 bag 的标定提取阶段，不写入终态文件，从而避免
`xyzw`/`wxyz` 顺序歧义。

## 时间约定

`time_offset_seconds` 的符号定义为：

```text
t_corrected = t_message + time_offset_seconds
```

当前 IILABS3D 没有独立的时钟偏移标定，因此 lidar 和 odometry 均为 `0`。字段仍是
必填项，便于以后接入硬件同步误差已知的数据集，而不改变 schema。

## 生成与校验

`tools/cartographer_native_worker.py` 在构造 benchmark 的 `native_input` 时读取源 bag
中的一次性静态标定，解析完整 frame 链并生成该文件。生成后，终态 bag 会删除
`/tf` 和 `/tf_static`。

runner 启动时会校验：

- schema、坐标约定、单位和消息类型；
- 配置的 tracking frame 与标定一致；
- bag 消息 topic/frame 与标定一致；
- 4×4 矩阵有限、末行合法、旋转正交且没有镜像反射。

标定不匹配属于数据包构造错误，runner 应立即失败，不做隐式 frame 猜测或回退到 TF。
