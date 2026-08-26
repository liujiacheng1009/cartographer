# 模块与源码索引

## 概览

与 OpenCV 的 module page 类似，本页先描述每个模块的目的，再列出算法、主要类型、依赖
和相关教程。目录按运行时所有权划分，深度不超过两层。

```text
application ─┬→ frontend ─→ mapping ─→ scan_matching
             ├→ backend  ─→ mapping ─→ scan_matching
             └→ serialization

frontend/backend/mapping/scan_matching/serialization ─→ foundation
```

依赖应大体从上向下。`foundation` 不得反向依赖前后端；`frontend` 通过窄接口向
`backend` 提交 node 和 odometry，不拥有后端。

## application：应用装配与数据边界

**目的**：解析配置和命令行，读取 rosbag2/标定，装配并结束完整 SLAM 生命周期。

主要文件：

- `bag_runner.cc`：唯一离线可执行入口；消息反序列化、输入校验、数据转换和文件输出。
- `slam_system.{h,cc}`：拥有 dispatcher、前端数组、后端和任务执行器。
- `slam_options.{h,cc}`：强类型配置结构及 YAML 到 options 的转换。
- `config.{h,cc}`：严格 `ParameterDictionary`。

主要类型：`SlamSystem`、`SlamSystemOptions`、`TrajectoryBuilderOptions`、
`ParameterDictionary`。

## frontend：实时局部轨迹估计

**目的**：按时间接收 scan/odom，完成扫描期间运动补偿、局部匹配、submap 插入和 node
生成。该模块是单一公开前端，不再有 collated/global/local 三层公开 builder。

主要组件：

- `DataDispatcher`：每个 `(trajectory_id, sensor_id)` 一个队列，做全局时间归并。
- `Frontend2D`：输入边界和前后端桥接。
- `LocalSlam2D`：滤波、预测、scan matching、运动过滤和 submap 插入。
- `PoseExtrapolator`：由历史 pose 与 odometry 估计 scan 内各点姿态。
- `MotionFilter`：按时间、平移、旋转门限抑制冗余 node。

线程模型：bag 回放线程调用前端；前端算法本身按输入顺序同步执行。

## mapping：地图数据结构

**目的**：保存概率栅格与活动 submap，并执行 range insertion。

主要类型：

- `ProbabilityGrid`：概率栅格存储和查询；
- `ProbabilityGridRangeDataInserter2D`：hit/miss 更新；
- `Submap2D`：单个局部地图；
- `ActiveSubmaps2D`：维护重叠的活动 submap。

一个 scan 可以同时插入两个活动 submap；旧 submap 达到配置容量后 finished，随后可建立
回环搜索索引。

层级关系为 `Trajectory → Submap → Node`：trajectory 表示一次建图或定位会话，而不是
单个局部地图。普通建图通常只有一条 trajectory；已知地图定位会同时存在冻结的历史
trajectory 和新的活动 trajectory。

## scan_matching：局部匹配与回环搜索

**目的**：求 scan 相对栅格的最优二维位姿。

算法：

- `CeresScanMatcher2D`：连续优化，占据概率残差加平移/旋转先验；
- `FastCorrelativeScanMatcher2D`：后端 branch-and-bound 回环候选搜索；
- `PrecomputationGridStack2D`：为 finished submap 建立多分辨率搜索索引。

前端和后端 matcher 不能互换：前者围绕 odom/外推初值做局部定位，后者在更大窗口内
验证历史 node-submap 候选。

### FastCorrelativeScanMatcher2D 的分支定界搜索

后端的快速相关匹配把每个离散候选表示为“一个旋转后的 scan + 一个栅格平移偏移”。若逐一在
原始概率栅格上评分，候选数随平移窗口、角度采样和栅格分辨率相乘增长。分支定界不改变要找的
最优离散位姿，而是通过可信的分数上界跳过不可能胜出的区域。

对每个 finished submap，`PrecomputationGridStack2D` 在构造 matcher 时建立
`branch_and_bound_depth` 层预计算栅格。第 `d` 层每个单元保存原栅格中
`2^d × 2^d` 区域的最大占据匹配分数；实现使用两次滑动窗口最大值，因而构建每层索引是线性的。
对一个候选，`ScoreCandidates()` 将所有 scan 点查询到的这些最大值取平均。这个值是其覆盖区域
内任一更细平移候选的上界：用最大值替代实际单元值只能抬高分数，不会低估。因此若上界不超过
当前最佳分数（初始为调用方传入的 `min_score`），该候选及全部子节点都可安全剪枝。

搜索流程与源码一一对应：

```text
Match()/MatchFullSubmap()
  → GenerateLowestResolutionCandidates()：以 2^max_depth 的步长枚举旋转/平移根候选
  → ScoreCandidates(最粗预计算栅格)，按上界从高到低排序
  → BranchAndBound()
       ├─ 上界 <= 当前最佳：停止该分支
       └─ 否则分成至多 4 个 (x, y) 子块，使用下一层更细预计算栅格重新评分
            → 深度 0：返回该叶子；其分数是原始单元分辨率的实际离散匹配分数
```

`BranchAndBound()` 始终将已找到的最佳叶子分数回传为下一分支的阈值；候选已按上界降序排列，
所以高分分支优先收紧阈值，后续剪枝更多。`branch_and_bound_depth` 越大，根候选越稀疏、索引和
递归层数越多；过小会减少剪枝，过大则增加预计算和分裂开销。它只影响搜索开销和离散搜索结构，
不替代 `min_score` 的约束质量门限；通过该粗搜索后，候选仍会交给 Ceres 做连续位姿精化。

## backend：约束图与全局优化

**目的**：保存 node/submap 图，在单后端 worker 上搜索约束并执行 pose graph optimization，管理
轨迹状态、裁剪和全局查询。

主要组件：

- `TrajectoryBackend2D`：唯一公开后端和状态所有者；
- `ConstraintEngine2D`：同步索引缓存和候选 scan matching；
- `PoseOptimizer2D`：Ceres PGO；
- `TrajectoryBackend2D` 私有 FIFO worker：让后端串行工作异步于前端；
- trimmer：删除不再需要的 node/submap 数据。

约束的基本形式始终是 **node ↔ submap**，不是 submap ↔ submap。所谓“submap 回环”是
通过历史 node 与非插入 submap 的约束将两段轨迹关联起来。多个不同 node-submap pair
按枚举顺序串行筛选，PGO 在约束批次完成后继续串行修改全局状态。

`SubmapState::kNoConstraintSearch` 表示活动 submap 的栅格仍会变化，不能作为额外回环
搜索目标，但它仍拥有当前 node 的 intra-submap 插入约束；`kFinished` 表示栅格固定，
可以建立快速匹配索引并搜索 inter-submap 约束。从 `.swmap` 恢复的历史 submap 按
finished 状态参与定位约束搜索。
当前定位模式仍会枚举活动轨迹自身的 finished submap；关闭这类历史回环、
保留少量滚动局部 submap 的减负方案见
[Bag 输入架构 9.3 节](BAG_PIPELINE_ARCHITECTURE.md#93-已知地图定位的候选约束减负项)。

## serialization：状态持久化

**目的**：把后端状态保存为 SQLite + zlib 的 `.swmap`，并恢复 frozen 轨迹。

当前 schema 为 v4。版本不匹配会明确拒绝，不做隐式字段补全。轨迹 CSV 是方便评测的
派生输出，不代替 `.swmap`。

## foundation：无所有者基础能力

**目的**：提供不依赖 SLAM 工作流的值类型和小型工具。

- `transform.h`、`utils.h`：SE(2)、点云变换和角度工具；
- `time.h`：统一时间尺度；
- `sensor_data.*`：点、点云、RangeData、TimedPointCloudData、OdometryData；
- `voxel_filter.*`：固定和自适应体素滤波；
- `sampling.*`：确定比例采样。

## 端到端所有权

```text
SlamSystem
├── owns DataDispatcher
├── owns TrajectoryBackend2D（含单后端 worker）
│   ├── owns ConstraintEngine2D
│   └── owns PoseOptimizer2D
└── owns Frontend2D[]
    ├── owns LocalSlam2D
    ├── borrows DataDispatcher
    └── borrows TrajectoryBackend2D
```

结束轨迹后必须等待后端 FIFO fence 并执行 final optimization，才能安全序列化和析构
仍由后端 worker 使用的 node、submap 与 matcher。

## 相关文档

- 使用教程：[GETTING_STARTED.md](GETTING_STARTED.md)
- 数据及时序：[BAG_PIPELINE_ARCHITECTURE.md](BAG_PIPELINE_ARCHITECTURE.md)
- 主要接口：[API_REFERENCE.md](API_REFERENCE.md)
