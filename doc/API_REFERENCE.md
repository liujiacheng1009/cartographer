# C++ API 概览

## 范围与稳定性

本工程构建一个 bag-only 可执行程序，不安装通用 SDK 头文件。因此这里的“API”是内部稳定
边界和维护索引，不承诺上游 Cartographer 式 ABI 兼容。优先从 `SlamSystem`、
`Frontend2D` 和 `TrajectoryBackend2D` 三个边界理解代码。

## `mapping::SlamSystem`

头文件：`application/slam_system.h`

职责：创建和拥有整套运行时对象，管理轨迹生命周期、地图加载/保存和后端访问。

| 方法 | 语义 | 调用约束 |
|---|---|---|
| `AddTrajectoryBuilder(...)` | 注册预期传感器并创建活动前端 | 输入前调用 |
| `FinishTrajectory(id)` | 关闭 dispatcher 队列并结束后端轨迹 | 每条活动轨迹一次 |
| `LoadStateFromFile(...)` | 加载 `.swmap`，可冻结旧轨迹 | 新轨迹创建前 |
| `SerializeStateToFile(...)` | 写出后端一致快照 | final optimization 后 |
| `backend()` | 获取非拥有后端指针 | 生命周期不超过 SlamSystem |

典型顺序：

```cpp
auto system = mapping::CreateSlamSystem(system_options);
const int id = system->AddTrajectoryBuilder(sensor_ids, trajectory_options,
                                             local_result_callback);
// 按时间提交 scan 和 odometry。
system->FinishTrajectory(id);
system->backend()->RunFinalOptimization();
system->SerializeStateToFile(true, output_path);
```

## `sensor::DataDispatcher`

头文件：`frontend/data_dispatcher.h`

输入变体 `SensorData` 当前只有：

- `TimedPointCloudData`
- `OdometryData`

dispatcher 根据 trajectory 和 sensor ID 建队列。只有所有尚未结束的预期队列都具有可比较
队首时才释放最早数据；`FinishTrajectory()` 将队列标为完成并排空。调用方必须保证每个
传感器自身时间非递减，dispatcher 只做跨传感器归并，不修复源数据乱序。

## `mapping::Frontend2D`

头文件：`frontend/frontend_2d.h`

公开重载：

```cpp
void AddSensorData(const std::string&, const sensor::TimedPointCloudData&);
void AddSensorData(const std::string&, const sensor::OdometryData&);
```

scan 经 dispatcher 回调进入 `LocalSlam2D`。产生 insertion result 时，前端把 node、插入
submap 和 odometry 交给后端。`SensorId::SensorType` 只有 `RANGE`、`ODOMETRY`。

## `mapping::LocalSlam2D`

头文件：`frontend/local_slam_2d.h`

核心阶段：

1. 累积并裁剪 range data；
2. 使用 `PoseExtrapolator` 将每个点变换到统一时刻；
3. voxel/adaptive voxel filter；
4. 可选实时相关匹配产生粗初值；
5. Ceres scan matcher 精化；
6. motion filter 判断是否生成 node；
7. 插入活动 submap。

局部结果是实时估计，不包含未来回环带来的全局修正。

## `mapping::TrajectoryBackend2D`

头文件：`backend/trajectory_backend_2d.h`

写接口：`AddNode`、`AddOdometryData`、`FinishTrajectory`、`FreezeTrajectory`、
`AddTrimmer`、`RunFinalOptimization`。

查询接口：`GetTrajectoryNodes`、`GetAllSubmapData`、`GetAllSubmapPoses`、
`GetTrajectoryStates`、`GetOdometryData`、`constraints`。

后端可以同时保存多条 trajectory，但每个局部 submap 不是一条独立 trajectory：

```text
trajectory_id
├── SubmapId{trajectory_id, submap_index}
└── NodeId{trajectory_id, node_index}
```

普通建图只有一条活动 trajectory；冻结地图定位通常包含一条或多条加载轨迹，再新增一条
活动定位轨迹。`InitialTrajectoryPoseState` 用 `to_trajectory_id`、`relative_pose` 和
`time` 表达新轨迹相对参考轨迹的初始坐标关系。

线程约束通过 Abseil lock annotations 标注。公开写操作通常把 work item 放入后端队列；
约束搜索在 `TaskExecutor` 工作线程执行，图状态修改和 PGO 由后端同步边界串行化。

内部 `SubmapState` 的两个值描述约束搜索资格：

- `kNoConstraintSearch`：submap 仍在插入数据；允许确定的 intra-submap 插入约束，禁止
  额外回环搜索；
- `kFinished`：栅格固定，可以构建 fast-correlative 索引并验证 inter-submap 候选。

`kFinished` 不代表索引、约束和 PGO 已全部完成，只代表 submap 内容不会继续变化。

## `constraints::ConstraintEngine2D`

头文件：`backend/constraint_engine_2d.h`

为每个 finished submap 延迟建立 fast-correlative 索引，并对 node-submap pair 进行局部或
全局约束匹配。粗匹配分数低于 `min_score`/`global_localization_min_score` 时拒绝；通过后
由 Ceres matcher 精化相对位姿并生成加权约束。
`MaybeAddConstraint()` 的距离筛选、每 submap 固定比例采样和异步完成屏障详见
[Bag 输入架构 3.3 节](BAG_PIPELINE_ARCHITECTURE.md#33-maybeaddconstraint-的候选筛选与任务依赖)。

## `optimization::PoseOptimizer2D`

头文件：`backend/pose_optimizer_2d.h`

优化变量是 node pose 和 submap pose。残差来源包括局部/回环 node-submap 约束、连续
node 的局部 SLAM 关系和 odometry。冻结轨迹的变量保持常量，用于已知地图定位。

## 核心值类型

| 类型 | 头文件 | 关键语义 |
|---|---|---|
| `common::Time` | `foundation/time.h` | 100 ns universal ticks |
| `transform::Rigid2/3` | `foundation/geometry.h` | `T_parent_child` 风格刚体变换 |
| `TimedRangefinderPoint` | `foundation/sensor_data.h` | 点坐标加相对 scan 结束时刻 |
| `TimedPointCloudData` | 同上 | scan 批次，点时间通常非正 |
| `OdometryData` | 同上 | reference 到 tracking frame 的 pose |
| `RangeData` | 同上 | origin、returns、misses |
| `NodeId` / `SubmapId` | `backend/id.h` | `(trajectory_id, index)` 强类型 ID |
| `Constraint` | `backend/backend_types.h` | node-submap 相对位姿及权重 |

## 错误处理

输入与内部不变量主要使用 glog `CHECK`。这意味着配置、标定、frame 或 schema 错误属于
不可恢复的批处理失败，进程非零退出；调用者应保留 stderr 并把失败视为数据质量问题，
而不是重试同一输入。
