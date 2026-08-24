# `uw_slam` 代码逻辑与新人快速上手指南

> 适用范围：`8df083b` 及 2026-08-22 当前工作树。工作树中的 P1 配置校验和有限
> plumb-bob 去畸变原语已重新构建、测试；尚未提交的事实不等同于发布基线。

## 一句话概括

`uw_slam` 是一个以 Protobuf 规范化消息模型为中心、算法与 ROS2/HoloOcean 解耦的水下声光
融合 SLAM 平台。目前处于“模块骨架 + 可运行端到端链路”阶段，还不是统一的实时产品系统。

## 整体逻辑结构

从代码依赖关系看，上层依赖下层：

```text
apps
       ↓
application（用例编排）
       ↓
runtime + evaluation + adapters（ROS2 隔离在 adapters/ros2/）
       + frontends + factor_builders + estimation + mapping
       ↓
core (sensor_models + measurement_api)
       ↓
domain
       ↓
schemas/proto
```

从运行时数据流看：

```text
仿真 / ROS2 / 合成数据
        ↓
Observation 原始观测
        ↓
Frontend
        ↓
MeasurementEvidence / HypothesisSet
        ├─→ FactorBuilder → ResidualBlock → 位姿图优化
        └─→ 声光关联 → 深度融合 → MapEvidence
                                      ↓
                         Submap / 指标 / 轨迹 / Manifest
```

顶层 [`CMakeLists.txt`](../CMakeLists.txt) 与集中式 `cmake/Libraries.cmake` 基本就是
完整的模块依赖清单。

| 目录 | 核心职责 |
|---|---|
| `schemas/proto/` | 跨语言规范化消息模型的唯一事实源：观测、量测、因子、状态、地图、标定 |
| `include/domain`、`src/domain` | Protobuf 的类型安全包装、数据校验、`MakeEvidence` 辅助函数 |
| `include/sensor_models`、`src/sensor_models` | `Pose3`、相机模型、有限去畸变、声呐 beam、坐标投影 |
| `include/measurement_api` | C++ 进程内 Frontend、FactorBuilder、ResidualBlock、Provider 抽象接口，不是 Protobuf 消息定义 |
| `include/frontends`、`src/frontends` | CFAR 声呐检测、立体深度、声光关联和概率融合 |
| `include/factor_builders`、`src/factor_builders` | 相对位姿、深度、声呐距离残差 |
| `include/estimation`、`src/estimation` | 位姿图和 Eigen 实现的 Gauss-Newton/LM |
| `include/mapping`、`src/mapping` | 局部地图证据管理、融合深度到点云的转换 |
| `include/runtime`、`src/runtime` | MCAP、配置、同步、队列、状态机、RunManifest 等 runtime 支持原语；尚无已组合的在线调度器 |
| `include/adapters`、`src/adapters`、`adapters/ros2` | HoloOcean、ROS2、SVIn 等外部系统边界 |
| `include/evaluation`、`src/evaluation` | ATE、深度/融合和点云地图质量指标（尚无 RPE） |
| `include/application`、`src/application` | 跨算法、runtime 与评测层的用例编排；当前包含离线回放管线 |
| `apps` | 参数解析和进程入口；调用 `application` 服务或单一用途的共享原语 |
| `tests` | 消息格式与接口一致性测试（`contracts/`）、按层单元测试、确定性回放（`integration/`） |
| `external_repos` | 只读参考代码，不是本系统运行主体 |

最重要的设计规则是：算法层不能知道 ROS2、HoloOcean 或 vendor 类型；外部数据进入
算法层前，必须先转换成 Protobuf 规范化消息类型。

`schemas/proto/` 只定义可跨语言传递的消息类型；`include/measurement_api` 则定义
C++ 进程内接口。后者的 `ResidualBlock` 是求解器接口，不属于 Protobuf 的唯一事实源。

理解这一规则可以从以下接口开始：

- [`include/domain/domain.hpp`](../include/domain/domain.hpp)：
  Protobuf 类型安全包装、量测结果构造和校验。
- [`include/measurement_api/frontend.hpp`](../include/measurement_api/frontend.hpp)：
  声呐和光学前端接口。
- [`include/measurement_api/factor_builder.hpp`](../include/measurement_api/factor_builder.hpp)：
  从量测结果构造残差块的接口。
- [`include/measurement_api/residual_block.hpp`](../include/measurement_api/residual_block.hpp)：
  求解器使用的残差和雅可比接口。

## 核心数据概念

理解代码前，先区分以下核心消息类型与进程内接口：

| 概念 | 含义 |
|---|---|
| `Observation` / `SonarFrame` / `ImageFrame` | 传感器直接产生的原始观测 |
| `MeasurementEvidence` | 前端从原始观测中提取出的量测结果，带来源、噪声建议和算法版本 |
| `HypothesisSet` | 一个观测对应的多个候选及歧义信息 |
| `FactorCandidate` | 建议使用哪种残差模型和信息权重 |
| `ResidualBlock` | `include/measurement_api` 定义的进程内接口，交给求解器计算残差和雅可比 |
| `StateSnapshot` | 某一版本的估计状态 |
| `MapEvidence` | 绑定到 keyframe/局部坐标系的局部地图数据 |

除 `ResidualBlock` 外，表中的核心消息类型的唯一事实源位于
[`schemas/proto/uw/domain/`](../schemas/proto/uw/domain/)；`ResidualBlock` 及其同类进程内
接口位于 [`include/measurement_api/`](../include/measurement_api/)。

## 当前三条真实执行链

### 1. 声呐位姿图回放主链

CLI 入口是 [`apps/replay_demo.cpp`](../apps/replay_demo.cpp)，实际用例编排位于
[`src/application/replay_pipeline.cpp`](../src/application/replay_pipeline.cpp)：

1. `synth_bag_gen` 生成圆弧轨迹、相对位姿、深度、声呐帧和 ground truth，写入
   统一 MCAP 录制格式。
2. `replay_demo` 加载四层 YAML 配置。
3. `McapEventSource` 顺序扫描一次 bag（按 `logTime` 排序），把消息拆成
   `CanonicalEvent` 经 `PumpEvents` 分发进 `ReplayInputAccumulator`
   （`include/application/replay_input_accumulator.hpp`）——关键帧身份来自
   wire 里的 `ObservationId`/`source_observations`，不是时间反推；这一步产出
   下面第 4 步开始要用的 `ReplayInputData`。
4. 从 `ReplayInputData` 中取出相对位姿量测结果，建立 keyframe 和初始里程计
   轨迹——具体怎么拿到这份量测结果取决于历史字段名 `estimator_mode`（见下），
   不是选择估计求解器。
5. 原始 `SonarFrame` 经 `CFAR → 极坐标转换 → DBSCAN`，得到声呐候选。
6. `SubmapManager`（按 keyframe 索引的局部地图数据存储，不是完整的 submap 生命周期
   管理器）做最近邻地标关联，再生成声呐距离因子。
7. 深度量测结果生成绝对 Z 方向因子。
8. 相对位姿、声呐距离、深度残差一起进入 `PoseGraphProblem`。
9. Gauss-Newton/LM 优化所有非固定 keyframe。
10. 优化结果写入 `StateStore`，更新地图位姿，并计算 ATE。
11. 输出 TUM 轨迹和 RunManifest。

`estimator_mode` 是保留兼容性的历史字段名，决定第 4 步怎么拿到相对位姿证据，而不是
选择求解器；两条路径最终使用同一个 `GaussNewtonSolver`。`frontends.landmark_detector`
还会选择 blob/Harris 检测器。sonar/optical frontend 当前各只有一个实现；`map_backend`
是预留的地图实现选择字段，目前只支持 `submap_point_cloud_v1`，未知标识符会 fail-fast：

`stereo_landmark_vo` 仅在已加载的 rig 含相机时使用；否则 `replay_demo` 会像
`black_box_vio` 一样回退读取 `/evidence/relative_pose`。配置校验不会拒绝这个无相机的组合。

- `black_box_vio`（默认，`configs/experiment/synthetic_smoke.yaml`）：直接读取
  `synth_bag_gen` 写进 bag 的 ground-truth+noise 相对位姿证据，是占位的
  "black-box VIO" 桩，不是真实的视觉里程计。
- `stereo_landmark_vo`（`configs/experiment/synthetic_smoke_vo.yaml`）：改由
  [`StereoLandmarkVoFrontend`](../include/frontends/stereo_landmark_vo_frontend.hpp)
  从左右相机帧实时计算——角点/blob 检测 + NCC 匹配 + RANSAC 刚体拟合出帧间相对
  位姿，是真实的视觉里程计（仍不融合 IMU，是 VO 不是 VIO）。

两条路径在合成场景下收敛的 ATE 量级相近（约 0.06m），验证方法见
[测试与验证指南](./testing-and-verification-guide-2026-08-20.md)。

对应的主调用链是：

```text
synth_bag_gen
 → 统一 MCAP 录制格式
 → McapEventSource + PumpEvents → ReplayInputAccumulator
 → replay_demo
 → SonarCfarFrontend
 → RelativePose/SonarRange/Depth FactorBuilder
 → PoseGraphProblem
 → GaussNewtonSolver
 → StateStore + SubmapManager
 → ATE + trajectory + RunManifest
```

这仍是一条批处理验证链，不是在线消息循环——但输入阶段已经不是"按 topic 多次
扫描 MCAP"了：`McapEventSource` 只顺序扫描 bag 一次，按 `logTime` 排序把消息
拆成 `CanonicalEvent`，`PumpEvents` 分发进 `PipelineInputPort`
（`replay_demo` 用 `ReplayInputAccumulator` 实现）。这条 `EventSource`/
`PipelineInputPort` 接口本身与来源无关——MCAP 回放和未来的供应商 SDK live
source 都能喂给同一个 `PipelineInputPort`，但**有界调度、Start/Stop/Drain
生命周期、真正的在线消息循环仍未实现**（下一实施包起点：供应商 SDK
`EventSource` + runtime hardening），不要把"输入主链已统一"误读成"在线
模式已经存在"。

### 2. 声光深度融合链

入口是
[`apps/acoustic_optic_scenario_matrix.cpp`](../apps/acoustic_optic_scenario_matrix.cpp)：

```text
9 类合成退化场景
 → capture-time 同步
 → Stereo SAD 视差和光学深度先验
 → Sonar CFAR 候选
 → 声呐弧投影到相机
 → range/bearing 几何门控
 → 标量 posterior depth 优化
 → 方差改善与 innovation gate
 → 深度/误融合率/延迟指标
```

融合采用“无法证明一致就不融合”的策略：整张深度图默认保留光学结果，只有通过
几何关联、方差改善和残差门限的像素才升级为声光融合结果。核心实现见
[`acoustic_optic_depth_fusion_frontend.cpp`](../src/frontends/acoustic_optic_depth_fusion_frontend.cpp)。

`AcousticOpticDepthFusionFrontend` 是声光融合模块；它位于 `frontends` 路径只是保留的
历史命名，不表示它只是单模态前端。主要组件包括：

| 阶段 | 实现 |
|---|---|
| 时间同步 | `include/runtime/acoustic_optic_synchronizer.hpp` |
| 光学深度 | `include/frontends/stereo_optical_depth_frontend.hpp` |
| 声呐检测 | `include/frontends/sonar_cfar_frontend.hpp` |
| 跨模态关联 | `include/frontends/acoustic_optic_associator.hpp` |
| 概率深度融合 | `include/frontends/acoustic_optic_depth_fusion_frontend.hpp` |
| 深度和误融合评测 | `include/evaluation/depth_metrics.hpp`、`include/evaluation/fusion_metrics.hpp` |
| 点云地图评测原语 | `include/evaluation/map_metrics.hpp`（Chamfer/completeness/outlier/F-score；暴力最近邻，尚未接回放） |

最新的
[`acoustic_optic_map_bridge`](../include/mapping/acoustic_optic_map_bridge.hpp)
会进一步把融合深度转换成 `base_link` 局部点云。局部证据不会提前烘焙到世界坐标，
因此位姿图修正后，`SubmapManager` 可以使用最新 keyframe 位姿重新计算世界点。

`replay_demo` 在带相机 rig 下会并行运行这条声光链，并把融合点云交给
`SubmapManager`；它仍不把稠密深度作为位姿图因子，所以不会改变定位优化。九场景矩阵
的 CTest 同时检查确定性与最低有效覆盖：除明确设计为 fail-closed/回退的故障场景外，
0 accepted 会使测试失败；质量收益和墙钟延迟 gate 仍是 opt-in。

### 3. 真实 HoloOcean 离线 VO 链

`adapters/holoocean/uw_holoocean_adapter/record_session.py` 已在原生 Windows
HoloOcean 2.3.0 上生成过统一 MCAP 录制格式。`configs/experiment/real_holoocean_vo.yaml`
选择真实相机 rig、Harris 角点和 `stereo_landmark_vo`，由 `replay_demo --align-ate`
消费录制的 RGB 双目图像：

```text
真实 HoloOcean 双目 + GT + depth
 → 统一 MCAP 录制格式
 → RGB8 转 MONO8
 → Harris + NCC/行门控/分数间隔 + RANSAC
 → 49 条 relative-pose + 50 条 depth factor
 → 位姿图 + 对齐 ATE
```

审计样本约 76 MB、50 个 keyframe，不含声呐/IMU/DVL；对齐 ATE RMSE 为
`0.5596 m`，求解器 30 次迭代后 `stalled`，稠密地图为空。因此它证明了“真实录制能
进入离线 VO”，没有证明实时闭环、真实声光融合或生产精度。

## 配置、适配器与外部系统

### 配置层次

配置按以下顺序叠加：

```text
defaults → rig → scenario → experiment → 显式 CLI 参数
```

- `defaults/`：求解器、因子信息权重、warmup 等平台默认值。
- `rig/`：相机/声呐内外参、时间偏移和噪声模型。
- `scenario/`：轨迹、场景退化、故障和随机 seed。
- `experiment/`：算法选择、地图后端和输出策略。

解析入口是 [`src/runtime/config.cpp`](../src/runtime/config.cpp)，字段说明见
[`configs/README.md`](../configs/README.md)。

### 外部接入

- `adapters/holoocean`：Python HoloOcean 网关、坐标转换和统一 MCAP 录制格式写入。
  Python 写出的 bag 可以直接由 C++ `replay_demo` 读取；`record_session.py` 是
  `synth_bag_gen` 的真实会话对应物，把一次真实 HoloOcean 录制转换成同样的
  统一 MCAP 录制格式。项目已在原生 Windows 仿真器上录制并离线回放过一份双目 bag；当前
  Linux 开发机没有 HoloOcean/UE5，实时可靠性仍未自动回归。
- `adapters/ros2`：ROS2 传输边界，是唯一允许出现 ROS2 头文件的地方。当前 HoloOcean
  声呐节点能够订阅并转换消息，但尚未驱动完整声呐前端和估计链。
- `include/adapters`、`src/adapters`（文档见 `adapters/svin_bridge.md`、
  `adapters/holoocean_ros_bridge.md`）：把 SVIn、HoloOcean ROS bridge 等外部语义
  转换成平台 Provider，与 ROS2 传输层解耦，可在无 ROS2 环境下单测。
- `external_repos`：只读参考和移植来源，不应直接修改。

## 新人 60–90 分钟上手路径

### 第一步：运行完整验证

```bash
tools/verify_pipeline.sh --out-dir /tmp/uw_slam_onboarding
cat /tmp/uw_slam_onboarding/summary.txt
```

2026-08-22 当前工作树实跑为 136/136 CTest、35/35 Python，默认回放 ATE RMSE
`0.0665821 m`；数字会随模块增加而变化，以本次 `summary.txt` 为准。

除测试结果外，重点查看：

- `synthetic.mcap`：管线输入。
- `demo_trajectory.tum`：估计结果。
- `demo_run_manifest.json`：本次运行的环境和配置记录。

### 第二步：建立目录地图

用约 10 分钟阅读：

1. [根 README 的架构部分](../README.md#架构)。
2. [顶层 CMake](../CMakeLists.txt)。
3. [文档中心](./README.md)。

### 第三步：理解核心消息与接口

优先阅读以下六类 Proto：

```text
observation.proto   原始数据
measurement.proto   算法提取的量测
hypothesis.proto    多候选及歧义
factor.proto        待构建因子
state.proto         估计状态
map.proto           局部地图证据
```

对应的进程内 C++ 接口位于 `include/measurement_api/`，不是 Protobuf 消息：

- `frontend.hpp`
- `factor_builder.hpp`
- `residual_block.hpp`
- `providers.hpp`

### 第四步：追踪一条完整主链

顺序阅读 `replay_demo::main()`，先理解对象怎样组合，不必立刻钻进所有数学实现。

之后追踪以下纵向切片：

```text
SonarFrame
 → SonarCfarFrontend
 → SonarRangeFactorBuilder
 → SonarRangeResidual
 → PoseGraphProblem
 → GaussNewtonSolver
```

每看一个实现，立即查看同目录测试。测试通常比长注释更快说明输入、边界和预期输出。

### 第五步：阅读声光链

```text
scenario_matrix/main.cpp
 → StereoOpticalDepthFrontend
 → AcousticOpticAssociator
 → AcousticOpticDepthFusionFrontend
 → AcousticOpticMapBridge
```

阅读每个模块时，只回答四个问题：

1. 输入和输出分别是什么规范化消息类型与进程内接口？
2. 数据当前处于哪个坐标系？
3. 不确定度和门限由谁决定？
4. 哪个测试证明它的行为？

## 按任务快速定位

| 任务 | 首先查看 | 对应验证 |
|---|---|---|
| 修改核心消息字段 | `schemas/proto/`、`include/domain` | `contract.*` tests |
| 修改声呐检测 | `include/frontends/sonar_cfar_frontend.hpp` | `unit.frontends.SonarCfarFrontend*` |
| 修改相对位姿视觉里程计（VO） | `include/frontends/stereo_landmark_vo_frontend.hpp` | `unit.frontends.StereoLandmarkVoFrontend*`，端到端见 `configs/experiment/synthetic_smoke_vo.yaml` |
| 修改相机去畸变 | `include/sensor_models/camera_rectifier.hpp` | `unit.core.PlumbBobDistortionTest.*`、`unit.core.UndistortImageTest.*`；尚未接入回放 |
| 修改因子数学 | `include/factor_builders/`、`src/factor_builders/` | 残差和雅可比测试（`unit.factor_builders.*`） |
| 修改求解器 | `include/estimation`、`src/estimation` | `unit.estimation.*` |
| 修改声光融合 | associator、depth fusion（均在 `include/frontends`、`src/frontends`） | 对应 `unit.frontends.*`、scenario matrix |
| 修改地图输出 | map bridge、submap manager（`include/mapping`、`src/mapping`） | `unit.mapping.*` |
| 修改地图指标 | `include/evaluation/map_metrics.hpp`、`src/evaluation/map_metrics.cpp` | `unit.evaluation.MapMetrics.*`；当前只适合小点集 |
| 修改配置或回放 | `include/runtime`、`src/runtime`、`include/application`、`src/application`、`apps` | `unit.runtime.Config.*`、`unit.application.*`、`integration.*`；未知选择必须启动失败 |
| 修改 RunManifest provenance | `include/runtime/run_manifest.hpp`、`src/application/replay_pipeline.cpp` | 端到端检查 `<out>_run_manifest.json` |
| 接入真实设备 | `include/adapters`、`src/adapters`、`adapters/ros2` | `unit.adapters.*`，加端到端实机验证 |

## 当前容易误解的边界

- `frontends.landmark_detector` 会真正选择检测器；上文的 `estimator_mode` 和
  `map_backend` 说明也不代表已有动态插件或第二后端。
- 位姿图只优化 keyframe 位姿，不联合优化地标。
- 声呐消费者主要使用每帧 top-1 候选。
- ROS2 HoloOcean 节点目前只完成传输和格式转换，尚未驱动完整算法链。
- `McapEventSource`/`PipelineInputPort`/`PumpEvents`（`include/runtime/event_source.hpp`、
  `include/application/pipeline_input_port.hpp`）统一了 MCAP 回放的输入读取方式，
  并且这套接口本身与来源无关；但这只是"输入主链"这一层——供应商 SDK 的
  `EventSource` 实现、有界队列/背压调度、Start/Stop/Drain 生命周期、真正的
  在线消息循环都还不存在，不要把它读成"在线/实时模式已经接通"。
- 声光组件和场景矩阵已经能够端到端运行，`replay_demo` 也会生成融合局部地图数据；
  但稠密声光结果不参与位姿图优化。
- `camera_rectifier` 是已测试但未接线的有限 plumb-bob 去畸变原语，只适用于当前平行
  双目假设，不是通用离轴极线校正；直接用于现有真实 bag 会降低纹理和 VO 跟踪率。
- `map_metrics` 已定义点云 Chamfer/completeness/outlier/F-score，但还是
  `O(NM)` 暴力最近邻的小点集原语，没有接入 replay、真实 reference map 或质量门禁。
- 根 README 的测试数量可能落后于当前构建；测试数量和状态应以 `ctest --test-dir build -N`
  以及实际测试输出为准。
- 遇到文档与代码状态的信息冲突时，应按以下顺序判断：

```text
源码 / 测试 / Proto
 → 当前代码库参考和组件文档
 → 根 README
 → 长期架构文档
 → 历史 Pipeline 文档
```

## 推荐的新人介绍方式

给新成员介绍本仓库时，建议准备以下三项内容：

1. 本文中的一张分层图和一张数据流图。
2. 现场运行一次 `synth_bag_gen → replay_demo`。
3. 共同追踪一条从 Protobuf 输入到测试断言的纵向调用链。

这样通常能让新人在一小时左右具备定位代码、判断模块边界和确定修改验证范围的能力。
