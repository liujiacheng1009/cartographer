# 开发、验证与文档贡献指南

## 修改前先确定边界

| 改动类型 | 应修改的模块 |
|---|---|
| bag/topic/frame/标定转换 | `application/` |
| 输入排序、运动补偿、局部匹配 | `frontend/` |
| 栅格、submap、range insertion | `mapping/` |
| 回环、约束、PGO、轨迹状态 | `backend/` |
| matcher 数学实现 | `scan_matching/` |
| `.swmap` schema | `serialization/` |
| 无 SLAM 所有者的值类型 | `foundation/` |

不要创建含义泛化的 `util`、`common2` 或新的单实现 interface。新抽象至少应具有第二个
实现、独立测试替身，或明确隔离第三方依赖的价值。

## 构建

```bash
docker compose build cartographer
```

容器内使用 Release、C++20 和 `cmake --build -j2`。本机缺 ROS/Abseil/Ceres 开发包时，
不要用不完整的本机构建结果替代容器构建。

## 分级验证

### 纯文档改动

- 检查相对链接；
- 检查命令和文件名仍存在；
- `git diff --check`。

### 不改变算法的结构重构

```bash
docker compose build cartographer
PYTHONPATH=. .venv/bin/python tools/run_slam_benchmark.py --tier smoke --backend cartographer-native
PYTHONPATH=. .venv/bin/python tools/run_slam_benchmark.py --tier nightly --backend cartographer-native
PYTHONPATH=. .venv/bin/python tools/run_slam_benchmark.py --tier weekly --backend cartographer-native
PYTHONPATH=. .venv/bin/python tools/run_known_map_localization_benchmark.py --tier smoke
PYTHONPATH=. .venv/bin/pytest -q tests
```

要求 smoke 1/1、nightly 3/3、weekly 2/2、known-map 1/1、Python 69/69，无跳过。

### 算法或 schema 改动

除上述测试外，还应：

- 定位第一个发生变化的 node；
- 说明变化来自输入、局部匹配、约束还是 PGO；
- 对 schema 增加旧版本拒绝或迁移测试；
- 记录 ATE/AOE/RPE 和两个 full case 的 Coverage；
- 只有确认是预期功能变化时才更新基线。

## 轨迹确定性门禁

结构重构必须比较 `trajectory.jsonl` 的 SHA256，PASS 门限不足以证明轨迹相同。当前六个
基线依次覆盖四个切片和两个完整 bag：

```text
iilabs3d-slippage-start
iilabs3d-slippage-middle-a
iilabs3d-slippage-middle-b
iilabs3d-slippage-end
iilabs3d-slippage-full
iilabs3d-ramp-full
```

若 hash 不同，先比较节点数、末节点时间，再找首个不同节点。不要直接用新的宽松精度
PASS 覆盖确定性差异。

## 并发改动检查

后端使用一个私有 FIFO worker，constraint matcher、索引构建和 PGO 按顺序执行；PGO 的
Ceres `num_threads` 只控制求解器内部并行。修改时检查：

- PGO 内部线程是否造成过度订阅；
- 耗时 matcher/PGO 是否在持有全局状态锁时执行；
- final optimization 前 FIFO fence 是否已执行；
- submap/matcher 生命周期是否覆盖后端 worker；
- work item 枚举和约束汇总顺序是否改变确定性输出。

## 文档格式

每篇页面应包含适用范围，并尽量按以下结构编写：

1. 目标或模块目的；
2. 前置条件；
3. 原理或算法；
4. 可复制命令/代码；
5. 输入输出与错误；
6. 相关源码；
7. 下一步阅读。

模块页链接算法文章、教程和 API；教程中的示例必须与生产入口一致。新增页面后同步更新
`doc/README.md`，避免形成没有入口的孤立文档。

## 提交顺序

建议把代码重构、行为变化、文档和外层子模块指针分开提交。Cartographer 子模块先提交并
推送，再更新主仓库指针；否则主仓库会引用远端不存在的 commit。
