---
title: uw_slam 代码库参考文档
type: codebase-reference
status: current
scope: "what the code does today, file-by-file/type-by-type — not a design proposal"
updated: 2026-08-22
verified_commit: 8df083b
verified_worktree: "2026-08-22 P1 config validation and camera_rectifier changes"
---

# uw_slam 代码库参考文档

本文是 `uw_slam` 的代码级参考文档：基于 commit `8df083b` 及 2026-08-22 当前工作树
逐层、逐目录、逐类型地记录实际存在的类型、函数、字段、参数和数据流。工作树中的
P1 配置校验与 `camera_rectifier` 已用干净构建验证，但尚未提交，不能当作发布基线。
本文的权威范围是“当前代码做什么”，不是目标架构或未来计划。

这份文档和仓库里已有的三份文档分工不同，互不重复：

| 文档 | 性质 | 回答的问题 |
|---|---|---|
| [`docs/README.md`](./README.md) | 文档路由 | 遇到具体任务应该先读哪份文档、冲突时以谁为准 |
| [`README.md`](../README.md) | 项目门面 | 这是什么、怎么编译、怎么跑 demo |
| [`acoustic-optic-slam-platform-architecture-2026-08-17.md`](./acoustic-optic-slam-platform-architecture-2026-08-17.md) | 长期架构设计（已批准） | 系统**应该**长成什么样、为什么这么设计 |
| [`holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md`](./holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md) | 演进中/历史工程方案 | 第一阶段 baseline 如何设计，以及为什么演变成当前架构 |
| 本文 | 代码参考 | 代码**现在**长什么样：真实类型、真实函数、真实数据怎么流动 |

架构文档描述的是目标状态，很多设计尚未实现或只实现了一部分；本文只记录"读一遍
代码之后能确认的事实"，凡是设计与实现有出入的地方，都会明确标出"文档说 X，代码
实际是 Y"。

## 常用任务入口

- 查 Protobuf 与核心消息类型：[第 4 节](#4-跨语言规范化消息模型schemasproto)。
- 查 core 接口和传感器模型：[第 5 节](#5-core-层)。
- 查前端、因子、求解器和地图：[第 6 节](#6-algorithms-层)。
- 查完整 Demo 数据流：[第 10 节](#10-端到端运行时序)。
- 查配置加载与覆盖关系：[第 11 节](#11-配置系统-configs)。
- 查测试、构建和工具：[第 12–14 节](#12-测试体系-tests)。
- 查当前实现边界：[第 15 节](#15-已知边界)。

## 目录

1. [现状速览](#1-现状速览)
2. [架构总览：分层与依赖方向](#2-架构总览分层与依赖方向)
3. [目录结构地图](#3-目录结构地图)
4. [跨语言规范化消息模型：schemas/proto/](#4-跨语言规范化消息模型schemasproto)
5. [core/ 层](#5-core-层)
6. [algorithms/ 层](#6-algorithms-层)
7. [runtime/ 层](#7-runtime-层)
8. [adapters/ 层](#8-adapters-层)
9. [apps/ 与 evaluation/](#9-apps-与-evaluation)
10. [端到端运行时序：把 synth_bag_gen 和 replay_demo 串起来](#10-端到端运行时序)
11. [配置系统 configs/](#11-配置系统-configs)
12. [测试体系 tests/](#12-测试体系-tests)
13. [构建系统](#13-构建系统)
14. [工具链 tools/](#14-工具链-tools)
15. [已知边界](#15-已知边界)

---

## 1. 现状速览

骨架 + 每层至少一条真实可跑的端到端链路。2026-08-22 干净构建实跑 136/136 个 CTest、
35/35 个 Python 测试通过；默认 `synth_bag_gen → replay_demo` 在 6 次迭代收敛，ATE
RMSE `0.0665821 m`（12 个匹配位姿）。`estimator_mode` 是为兼容保留的历史字段名，只选择
相对位姿量测结果来源，不选择求解器：默认 `black_box_vio` 读取 bag 里
`synth_bag_gen` 写入的 ground-truth+noise 黑盒量测结果，`stereo_landmark_vo` 则由
`StereoLandmarkVoFrontend` 从左右相机帧实时计算立体路标视觉里程计量测（见 6.13 节和
[9.2 节](#92-appsreplay_demo--端到端主流程)）。后者还在一份真实 HoloOcean 双目 bag
上完成了离线回放；两条路径最终都进入同一个 `GaussNewtonSolver`。该样本的求解器
`stalled`、对齐 ATE RMSE `0.5596 m`，因此只能证明真实
数据链可达，不能作为生产精度声明。不是空骨架，也不是生产系统，具体缺口见第 15 节。

---

## 2. 架构总览：分层与依赖方向

依赖只允许单向：

```
core → {algorithms, runtime, evaluation, adapters} → application → apps
```

`include/`、`src/` 下任何生产代码不允许出现 ROS/HoloOcean/第三方 vendor 头，也不允许
再用旧的 `uw/...` 手写头路径，由 `tools/lint/check_layer_dependencies.py`（
`tools/lint/check_no_ros_in_core.sh` 是它的兼容入口，[第 14 节](#14-工具链-tools)）
静态检查这条不变量。
跨语言（C++/Python）规范化消息模型的唯一事实源是 `schemas/proto/`；
`include/measurement_api/` 定义算法的 C++ 进程内接口。统一 MCAP 录制格式也用于回放
（未压缩，因为 C++ 构建关掉了 zstd/lz4 后端，保证 Python 写的 bag 能被 C++ 直接读）。

> **2026-08-21 布局重构**：C++ 源码从"每个细粒度实现一个 package"（`core/domain/`、
> `algorithms/frontends/sonar_cfar_frontend/` 这类多层嵌套目录，各自带独立
> `CMakeLists.txt`/`include/src/test`）迁移为共享 `include/<role>/`、`src/<role>/`
> 根 + 按架构层合并的 CMake target。真实 target 名不再带 `uw_` 前缀，统一通过
> `uw::<name>` alias 引用（例如 `uw::domain`、`uw::frontends`）；本节及下文若干处
> 仍保留 `uw_xxx` 这种旧前缀写法，是当时（迁移前）的构建产物名，与当前
> `cmake/Libraries.cmake` 里的真实 target 名不再一致，读到类似写法时以
> `cmake/Libraries.cmake`/`cmake/Tests.cmake` 的当前内容为准。下表已更新为当前
> 布局。

| 层 | 源码目录 | CMake target（`uw::` alias） | 依赖 | 作用 |
|---|---|---|---|---|
| `schemas/` | `schemas/proto/` | `domain_proto`（生成） | 无 | 核心消息定义（protobuf），C++/Python 绑定的共同来源 |
| domain | `include/domain`、`src/domain` | `domain` | `domain_proto` | 生成类型的 C++ 人体工学层（Stamp 转换、oneof payload 访问器） |
| core（sensor_models + measurement_api） | `include/sensor_models`、`include/measurement_api`、`src/sensor_models` | `core` | `domain`, Eigen3 | `Pose3`、相机/去畸变、声呐 beam 几何、`SonarFrontend`/`FactorBuilder`/`ResidualBlock`/`*Provider` 抽象接口 |
| frontends（合并全部前端实现，含 CFAR、立体深度、立体 VO、声光关联/融合） | `include/frontends`、`src/frontends` | `frontends` | `core` | 声呐与光学前端 |
| factor_builders（合并三种残差/因子构建） | `include/factor_builders`、`src/factor_builders` | `factor_builders` | `core` | 三种残差 + 雅可比 |
| estimation | `include/estimation`、`src/estimation` | `estimation` | `core`, Eigen3 | Gauss-Newton/LM 求解器、`PoseGraphProblem`、`StateStore` |
| mapping（合并 submap_manager + acoustic_optic_map_bridge） | `include/mapping`、`src/mapping` | `mapping` | `core` | 按 keyframe 存储的地图证据管理 |
| runtime | `include/runtime`、`src/runtime` | `runtime` | `core`, `mcap_impl`, `protobuf`, `yaml-cpp`, Eigen3 | 状态机、四车道队列原语、分层配置加载、`RunManifest`、MCAP 读写封装 |
| adapters/holoocean（Python，独立包） | `adapters/holoocean/` | Python `uw_holoocean_adapter` | protobuf, mcap, numpy | 直连 HoloOcean Python API |
| adapters/ros2（可选，`UW_BUILD_ROS2`） | `adapters/ros2/include`、`adapters/ros2/src` | `ros2_adapters`（INTERFACE）+ `holoocean_sonar_bridge_node`（可执行） | ROS2 Jazzy, `holoocean_interfaces`, `adapters` | ROS2 话题 → `SonarFrame` 的传输层桥接 |
| adapters（合并 svin_bridge + holoocean_ros_bridge 两个无 ROS provider） | `include/adapters`、`src/adapters`（文档：`adapters/svin_bridge.md`、`adapters/holoocean_ros_bridge.md`） | `adapters` | `core` | 两个具体 provider 实现（第三个 baseline 现在是独立执行边界，见 `baselines/`） |
| evaluation | `include/evaluation`、`src/evaluation` | `evaluation` | `core` | ATE、深度、融合和点云地图指标（没有 RPE） |
| application | `include/application`、`src/application` | `application` | 算法、runtime、evaluation | 跨层用例编排；当前包含离线回放管线 |
| apps | `apps/synth_bag_gen.cpp`, `apps/replay_demo.cpp` 等 | 各自独立可执行文件 | `application` 或单一用途所需层 | 参数解析与进程入口 |

---

## 3. 目录结构地图

```
schemas/proto/uw/domain/     12 个 .proto 文件，跨语言规范化消息模型唯一事实源
include/                     手写公共头文件，按角色分区（物理 uw/ 层已去掉，C++ namespace 不变）
  domain/                    domain.hpp：Stamp 助手、oneof payload 访问器模板
  sensor_models/             Pose3、声呐 beam、PinholeCamera/StereoGeometry、camera_rectifier
  measurement_api/           Frontend/FactorBuilder/ResidualBlock/Provider 抽象（纯头文件）
  frontends/                 sonar_cfar_frontend / stereo_optical_depth_frontend /
                              stereo_landmark_vo_frontend（+ 其内部用到的 harris_corner_detector /
                              landmark_blob_detector / patch_matcher / rigid_transform_fit） /
                              acoustic_optic_associator /
                              acoustic_optic_depth_fusion_frontend 等全部前端头文件
  factor_builders/           sonar_range_residual（移植自 SVIn，雅可比独立重导）、
                              relative_pose_residual（原生）、depth_residual（原生）
  estimation/                gauss_newton_solver / pose_graph_problem / state_store
  mapping/                   submap_manager（按 keyframe 存储 MapEvidence）、
                              acoustic_optic_map_bridge（声光 plan 6）
  runtime/
    state_machines.hpp       三个正交的滞回状态机
    bounded_queue.hpp        四车道有界队列原语
    config.hpp                defaults→rig→scenario→experiment 分层配置类型
    run_manifest.hpp          RunManifest（一次运行的不可变记录）
    mcap_io.hpp                MCAP 读写的 protobuf 封装
    acoustic_optic_synchronizer.hpp  纯函数：capture-time 声光配对/拒绝（声光 plan 3）
  evaluation/                trajectory_metrics / depth_metrics / fusion_metrics / map_metrics
  adapters/                  svin_bridge_local_odometry_provider、
                              holoocean_ros_bridge_sonar_frame_provider（均无 ROS2 依赖）
  application/               replay_pipeline 等跨层用例接口
src/                         对应 include/ 分区的实现（.cpp），结构镜像 include/
apps/
  synth_bag_gen.cpp           合成带真值的 MCAP bag（位姿图/sonar/depth 路径）
  synth_stereo_gen.cpp        合成单帧立体图像对 + GT 深度（声光 plan 2，独立于上者）
  optical_baseline_eval.cpp   跑 StereoOpticalDepthFrontend，用 depth_metrics 打分
  acoustic_optic_scenario_matrix.cpp / acoustic_optic_scenarios.{cpp,hpp}
                               声光 plan 1-4 全组件真实接线 + 9 场景矩阵（声光 plan 5）
  replay_demo.cpp              仅解析参数并调用 application/replay_pipeline
adapters/
  holoocean/                  Python 包 uw_holoocean_adapter，直连 HoloOcean，未随本次重构改动
  ros2/                       UW_BUILD_ROS2 开关保护的 ROS2 节点；include/adapters/、src/
                               （物理隔离边界，唯一允许出现 ROS2 头文件的地方）
  svin_bridge.md               SVIn provider 说明文档（原 third_party/svin_bridge/README.md）
  holoocean_ros_bridge.md       HoloOcean ROS provider 说明文档（原 third_party/.../README.md）
  datasets/                    纯 stub，只有 README
baselines/
  sonar_camera_reconstruction/ 纯 stub 外部基线，脚本体是 TODO+exit 1（原
                                adapters/third_party/sonar_camera_reconstruction_baseline/）
configs/                       defaults/rig/scenario/experiment 四层 YAML
tests/
  core/、frontends/、factor_builders/、estimation/、mapping/、runtime/、
  evaluation/、adapters/       按架构层分组的单元测试源码
  contracts/                   protobuf round-trip 消息格式与接口一致性测试（原 l0_contracts/）
  integration/
    determinism_test.sh                两次跑 replay_demo 逐字节比对
    optical_baseline_smoke_test.sh     synth_stereo_gen + optical_baseline_eval 端到端阈值门禁
    acoustic_optic_scenario_matrix_determinism_test.sh
                                        两次跑矩阵逐字节比对（延迟除外）+ 最低有效覆盖 gate
  lint/                        check_layer_dependencies_test.py
tools/
  lint/check_no_ros_in_core.sh       依赖不变量检查（兼容入口）
  lint/check_layer_dependencies.py   实际实现：include/src 层间依赖 + ROS/vendor 隔离检查
  codegen/gen_py.sh                  生成 Python protobuf 绑定
  setup_dev_env.sh                   apt→conda-forge 回退安装脚本
cmake/
  Dependencies.cmake                 选项 + Eigen/Protobuf/MCAP/yaml-cpp/GTest/ROS2 依赖发现
  Libraries.cmake                    全部生产 library、alias、source list、link graph
  Applications.cmake                 全部 executable target
  Tests.cmake                        全部测试 executable、CTest discovery、labels
  UwProtobuf.cmake                   生成 domain_proto
  UwMcap.cmake                       FetchContent_Populate 拉取 MCAP header-only SDK
```

本地 C++ 源码目录不再各自持有 `CMakeLists.txt`；只有仓库根 `CMakeLists.txt` 和上面
`cmake/` 下的几个集中式文件负责整棵 target 图（见
[第 13 节](#13-构建系统)）。

---

## 4. 跨语言规范化消息模型：schemas/proto/

`package uw.domain;`，proto3。11 个文件，导入关系：`time.proto`/`ids.proto` 是叶子；
`observation.proto` 依赖两者；`sonar.proto` 依赖 `observation.proto`；
`measurement.proto` 依赖 `ids.proto`+`calibration.proto`；`factor.proto` 依赖
`ids.proto`+`time.proto`；`state.proto` 依赖三者；`map.proto` 依赖 `ids.proto`；
`hypothesis.proto` 依赖 `measurement.proto`；`health.proto` 依赖 `time.proto`。

### `time.proto`
- `enum ClockDomain`：`UNSPECIFIED/SIMULATION/SYSTEM_MONOTONIC/SENSOR_HARDWARE`
- `message Stamp { int64 seconds = 1; int32 nanos = 2; }`，故意不用
  `google.protobuf.Timestamp`（避免引入 well-known-types 依赖），字段布局照抄它。

### `ids.proto` —— 强类型 ID
每个 ID 都是独立的单字段 message，而不是裸 `string`/`uint64`：
`SensorId`、`FrameId`、`SequenceId`、`ObservationId`、`EvidenceId`、`KeyframeId`、
`StateId`、`SubmapId`、`CalibrationVersion`、`ModelVersion`、`StateVersion`。
protoc 因此为每一个生成独立的 C++/Python 类，类型系统直接阻止"把 SensorId 传去
需要 StateId 的地方"。这不是 C++ 侧手写的 phantom type/strong typedef，强类型
完全来自 protobuf 的 wrapper-message 模式，`include/domain/` 里没有任何手写的
`KeyframeId` 类。

### `observation.proto`
`ObservationHeader`：`observation_id` `sensor_id` `sequence_id` `capture_time`
`receive_time`（Stamp，capture/receive 分离，见第 8.1 节 `time_utils.py`）
`clock_domain` `sensor_frame` `calibration_version`
`validity`（嵌套 enum：`OK/DEGRADED/REJECTED`）`provenance`（string，core 不解析）。
每条原始观测、量测和量测结果消息都携带一个，只在 adapter 边界产生一次，下游不
重新推导。

### `sonar.proto`
`SonarFrame`：`header` `intensity_tensor`（bytes，行主序 `[num_ranges,num_beams]`）
`num_ranges` `num_beams` `encoding`（`UINT8_GRAY`）`range_bins`（repeated float）
`azimuth_angles`（repeated float，必须严格递增，由
`uw::domain::IsAzimuthAscending()` 校验）`min_range`/`max_range`/`range_resolution`
`horizontal_fov` `elevation_aperture`（永不被折叠成单点估计）`gain_metadata`
`sound_speed_assumption`。字段设计参照 `sonar_camera_reconstruction` 的
`OculusPing`/`OculusFire`。

### `image.proto`
`ImageFrame`：规范化相机原始观测。`header`（`ObservationHeader`，与
`SonarFrame` 共用同一套 capture/receive time、frame、calibration version、provenance
语义）`width` `height` `row_stride_bytes` `encoding`（嵌套 enum：`MONO8/RGB8/BGR8`）
`pixel_data`（bytes）`is_rectified` `exposure_seconds`。每个物理相机各自发出自己的
`ImageFrame`；左右目配对由 runtime 按 capture time 和 rig 配置重建，不通过 topic 名
隐式推断。`is_rectified` 是 producer 的声明：合成生成器写 `true`，真实录制保留 raw
语义。当前 stereo frontends 仍假定输入满足极线几何而不检查该 flag；
`camera_rectifier` 能生成有限去畸变结果，但尚未接进 `replay_demo`。

### `measurement.proto` —— 带物理语义的 typed payload
- `SonarRangeBearing`：`range_m` `bearing_rad` `range_sigma_m` `bearing_sigma_rad`
  `sonar_frame`。故意不含 elevation，2D 前视声呐 ping 本来就观测不到。
- `RelativePoseMeasurement`：`from_keyframe` `to_keyframe` `relative_pose`
  （语义 `from_T_to`）`covariance_6x6_row_major`（36 个 double，顺序
  `[tx,ty,tz,rx,ry,rz]`）。
- `PressureDepthMeasurement`：`depth_m` 是正向下的水深量，world/body frame 是 Z-up，
  因此位姿消费者用 `pose_z = -depth_m`；`sigma_m` 与该量同单位。
- `VisualTrackMeasurement`/`SonarRegistrationMeasurement`/`ImuPreintegrationMeasurement`：
  占位消息，暂无对应的 factor_builder 消费。
- `OpticalDepthPriorMeasurement`/`FusedDepthMeasurement`：已落地 wire contract、C++ validation
  和 C++/Python round-trip tests。`OpticalDepthPriorMeasurement` 由
  `StereoOpticalDepthFrontend`（6.7）真正产出；`FusedDepthMeasurement` 由
  `AcousticOpticDepthFusionFrontend::Fuse`（6.9）真正产出；它是融合模块，保留在
  `frontends` 路径只是历史命名。带相机 rig 的
  `replay_demo` 和场景矩阵都会调用。两者的 `depth_m` 都是相机 optical frame
  z-forward 距离，不是 `PressureDepthMeasurement` 的世界水深。
- `StereoDepthMeasurement`：保留的早期占位 payload，新代码不再以它作为通用 optical
  frontend 输出。
- 带来源、有效域和不确定度描述的量测结果（`MeasurementEvidence`）：`evidence_id` `source_observations`（repeated）
  `estimated_noise_scale`（**只是前端建议值，绝不是最终 information**）
  `quality_features`（map）`observable_subspace` `valid_domain`
  `algorithm_version` `model_version`，然后一个覆盖上述 9 种 payload 的 `oneof`。

### `factor.proto`
`FactorCandidate`：`associated_state_ids` `measurement_type` `residual_model`
（决定哪个 FactorBuilder 消费它）`proposed_noise` `observable_subspace`
`robust_policy_hint`（`NONE/HUBER/CAUCHY`）`evidence_ids` `valid_from`/`valid_to`。
前端只能提议 candidate，只有 typed FactorBuilder 才真正构建残差，前端不能直接
注入权重。

### `state.proto`
`StateSnapshot`：`state_id` `state_version` `capture_timestamp` `pose_wb`
`velocity_w_mps` `imu_bias`（6：gyro+accel）`marginal_uncertainty_row_major`
`tracking_status`（`TRACKING/DEGRADED/LOST/RELOCALIZING/RECOVERING`）
`calibration_version` `contributing_measurements`。单一权威 `StateStore`
（single-writer/multi-reader，见 [6.5](#65-includeestimation--gauss-newtonlm-求解器)），真值永不进入这个消息。

### `map.proto`
保存在局部坐标系中的局部地图数据（`MapEvidence`）：`evidence_id` `keyframe_id` `state_version` `local_frame`
`representation_type`（`POINT_CLOUD/OCCUPANCY/TSDF/SURFEL/SEMANTIC_MASK`）
`geometry_or_occupancy`（bytes；POINT_CLOUD 时是紧凑小端 float32 xyz 三元组）
`uncertainty` `source_observations` `reintegration_policy`（`TRANSFORM_ONLY`/
`FULL_REFUSE`）。这是对 `sonar_camera_reconstruction` `merge.py` 的刻意反模式：
保留局部坐标 + state_version 引用，而不是插入时就转换并固定进一个会过期的
全局帧。同样的原则贯穿 `submap_manager`（见 [6.6](#66-includemappingsubmap_managerhpp)）。

### `health.proto`
`HealthReport`：`component_id` `status`（`HEALTHY/SUSPECT/UNAVAILABLE/RECOVERING`）
`reason_code` `input_valid_rate` `queue_depth` `latency_p50/p95/p99_ms`
`residual_mean/stddev` `dropped_frame_count` `valid_domain_rate`
`out_of_distribution_rate` `last_recovery_time`。每个模块统一发布这个消息。

### `hypothesis.proto`
`HypothesisSet`：`candidates`（repeated `MeasurementEvidence`）
`calibrated_likelihoods`（与 candidates 等长同序）`rejected_candidates`
`ambiguity_reason` `out_of_distribution`。存在的意义是不让 FLS elevation 歧义/
多路径/误关联过早被折叠成一个点，但 v1 算法只消费 top-1
（`uw::domain::TopCandidate<T>()`）。

### `calibration.proto`
`Transform3D { matrix_row_major: repeated double[16] }`（4x4 齐次矩阵，行主序）。
`FrameEdge { parent_frame, child_frame, transform }`（语义 `parent_T_child`，
v1 故意不带逐条不确定度）。`ImuNoiseModel`/`CameraIntrinsics`/`SonarBeamModel`
（含 `sonar_enabled`，对应 SVIn 的 `isSonarUsed`）/`DepthSensorModel`（含
`depth_enabled`，对应 `isDepthUsed`）。`RigCalibrationSnapshot`：
`calibration_version` `frame_tree`（repeated FrameEdge）`cameras` `imu_noise`
`sonar_beam_models` `depth_models` `time_offset_seconds`（map）`notes`，唯一
标定事实源，ROS 静态 TF / 各工具自己的 YAML 都应该从它派生，不应该另外维护
（`adapters/svin_bridge.md` 单向从它生成一次性的 SVIn yaml，绝不反向）。

---

## 5. core/ 层

### 5.1 `include/domain` —— `domain`（`uw::domain`）

`include/domain/domain.hpp` + `src/domain/domain.cpp`，是生成类型之上的一层薄薄的
C++ 人体工学封装，不是第二套 schema。

```cpp
Stamp ToStamp(std::chrono::system_clock::time_point);
std::chrono::system_clock::time_point ToTimePoint(const Stamp&);
double ToSeconds(const Stamp&);
Stamp FromSeconds(double);

bool IsAzimuthAscending(const SonarFrame&);   // sonar.proto 不变量的实现

// b2c19e1 新增：RGB8/BGR8 -> MONO8（ITU-R BT.601 亮度加权 0.299R+0.587G+0.114B，
// 逐像素取整），失败（校验不过、非 RGB8/BGR8/MONO8 编码）时返回 std::nullopt；
// 输入已是 MONO8 时原样返回（no-op）。存在的原因：真实 HoloOcean 相机是 RGB8
// （camera_conversion.py），但 StereoLandmarkVoFrontend（6.13 节）/
// StereoOpticalDepthFrontend（6.7 节）硬性要求 MONO8 输入，这个转换在消费点
// （application/replay_pipeline）而不是录制点（record_session.py）做，颜色信息不在采集时就丢弃。
std::optional<ImageFrame> ConvertToMono8(const ImageFrame& frame);
```

`PayloadTraits<T>` 模板是 `MeasurementEvidence` oneof 的类型安全访问点：没有通用
定义，每个 payload 类型必须通过 `UW_DOMAIN_DEFINE_PAYLOAD_TRAITS(Type, field)` 宏
显式注册（生成 `Has()`/`Get()`/`Set()`，映射到 protobuf oneof 的
`has_field()`/`field()`/`mutable_field()`）。7 种 payload 类型都这样注册。基于它的
自由函数：

```cpp
template <typename T> bool HasPayload(const MeasurementEvidence&);
template <typename T> const T& GetPayload(const MeasurementEvidence&);
template <typename T> void SetPayload(MeasurementEvidence&, T);
template <typename T>
MeasurementEvidence MakeEvidence(EvidenceId, std::vector<ObservationId> sources,
                                  T payload, double noise_scale, std::string algo_version);
template <typename T> std::optional<T> TopCandidate(const HypothesisSet&);  // 只取 top-1
```

> **文档-代码出入**：`measurement.proto` 的注释声称这套访问器住在
> `include/measurement_api/measurement_evidence.hpp`，
> 该文件不存在。实际实现在 `domain`（`uw::domain`）目标里的 `domain.hpp`。

### 5.2 `include/sensor_models` —— 现已并入 `core`（`uw::core`）

只依赖 `domain`（`uw::domain`） + `Eigen3::Eigen`。

`Pose3`（`geometry.hpp`/`.cpp`）：平移 + 四元数 xyzw，故意不引入 Sophus/manif，
只做 compose/inverse/apply：

```cpp
struct Pose3 {
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();

  static Pose3 Identity();
  Pose3 operator*(const Pose3& rhs) const;   // 复合：先 this 后 rhs（在 this 的局部系下）
  Pose3 Inverse() const;
  Eigen::Vector3d Apply(const Eigen::Vector3d& point_local) const;  // 变换一个点

  std::array<double, 7> ToParameterBlock() const;        // [tx,ty,tz,qx,qy,qz,qw]
  static Pose3 FromParameterBlock(const double* seven);

  uw::domain::Transform3D ToProto() const;
  static Pose3 FromProto(const uw::domain::Transform3D&);  // 元素数 != 16 时静默返回 Identity
};
```
无 equality/interpolation 方法。7 参数布局 `[tx,ty,tz,qx,qy,qz,qw]` 照抄
SVIn/OKVIS 的参数块惯例。

声呐 beam 几何（`sonar_beam_model.hpp`/`.cpp`）：只有两个自由函数，纯几何，
不含噪声/beam pattern（那部分在 `sonar_cfar_frontend`）：

```cpp
Eigen::Vector3d SonarRangeBearingToPlanePoint(double range_m, double bearing_rad);
// 返回 (r*cos(bearing), r*sin(bearing), 0) —— 2D FLS 实际能观测到的唯一东西

std::vector<Eigen::Vector3d> ExpandElevationFan(double range_m, double bearing_rad,
                                                 double elevation_aperture_rad,
                                                 int num_elevation_samples);
// 把一个 range-bearing 回波沿竖直孔径展开成候选 3D 点扇形；
// 明确标注"仅用于建图密集证据生成，绝不能喂给位姿因子"（移植自
// sonar_camera_reconstruction 的 get_extended_coordinates）
```

相机模型（`camera_model.hpp`/`.cpp`）提供 `PinholeCamera`、`StereoGeometry` 和
`SonarArcProjector` 使用的 optical-frame 几何。`StereoGeometry::Resolve()` 只接受
朝向一致、基线可按当前约定解析的平行双目，不是通用 stereo calibration。

2026-08-22 工作树新增 `camera_rectifier.hpp`/`.cpp`，已经编进 `core` 并通过 9 个
`PlumbBobDistortionTest`/`UndistortImageTest`：

```cpp
struct PlumbBobDistortion {
  double k1, k2, p1, p2, k3;
  bool IsIdentity() const;
  static std::optional<PlumbBobDistortion> FromIntrinsics(const CameraIntrinsics&);
};
Eigen::Vector2d ApplyPlumbBobDistortion(const PlumbBobDistortion&, const Eigen::Vector2d&);
std::optional<ImageFrame> UndistortImage(const ImageFrame&, const CameraIntrinsics&);
```

它接受 plumb-bob 0/4/5 个系数以及 MONO8/RGB8/BGR8，按同一 K 反向 remap、双线性采样，
非零畸变时输出 `is_rectified=true`；identity 畸变原样返回，保留原 flag。它只在平行
rig 前提下完成镜头去畸变，不处理任意相机相对旋转。`replay_demo` 尚未调用它：对现有
真实 bag 的实测 warp 几何正确，但纹理 Laplacian variance 下降约 43%，VO 跟踪从
50/50 降到 8/50，需要先联合重调 Harris/matcher。

### 5.3 `include/measurement_api` —— 纯头文件，现已并入 `core`（`uw::core`）

`frontend.hpp`：
```cpp
class SonarFrontend {
 public:
  virtual ~SonarFrontend() = default;
  virtual uw::domain::HypothesisSet ProcessSonarFrame(const uw::domain::SonarFrame&) = 0;
  virtual uw::domain::HealthReport Health() const = 0;
};
```
注意：没有通用的 `Frontend<T>` 模板，设计上刻意不搞一个模板套所有模态
（声呐/视觉/立体视觉的输入输出物理上不同，架构文档 7.4 节）。`SonarFrontend`
输出永远是 `HypothesisSet`，从不折叠成单一 6DoF 位姿。

`frontend.hpp` 还定义了独立的 optical 契约（不是 `SonarFrontend` 的泛化）：
```cpp
struct CameraFrameBundle {
  uw::domain::ImageFrame primary;
  std::optional<uw::domain::ImageFrame> secondary;
};

class OpticalDepthFrontend {
 public:
  virtual ~OpticalDepthFrontend() = default;
  virtual std::optional<uw::domain::MeasurementEvidence> Process(
      const CameraFrameBundle&, const uw::domain::RigCalibrationSnapshot&) = 0;
  virtual uw::domain::HealthReport Health() const = 0;
};
```
`CameraFrameBundle` 是进程内值类型，不是新的录包消息——统一 MCAP 录制格式只保留独立
`ImageFrame`，配对由 runtime 按 capture time 和 rig 配置重建。L0 contract test 里仍保留
一个 fake stereo、一个 fake monocular metric 实现，用来验证接口没有写死双目；
`include/frontends/stereo_optical_depth_frontend.hpp/` 现在提供了真正的
`StereoOpticalDepthFrontend`（见 6.7 节）；带相机 rig 的 `replay_demo` 会在并行声光
pass 中构造并调用它，但其输出不会成为位姿图因子。

`factor_builder.hpp`：
```cpp
struct FactorBuildContext {
  std::vector<Eigen::Vector3d> nearby_points_W;   // 可选的空间上下文
};

class FactorBuilder {
 public:
  virtual ~FactorBuilder() = default;
  virtual bool CanBuild(const uw::domain::FactorCandidate&) const = 0;
  virtual std::unique_ptr<ResidualBlock> Build(const uw::domain::FactorCandidate&,
                                                const uw::domain::MeasurementEvidence&,
                                                const FactorBuildContext&) const = 0;
};
```
`CanBuild` 靠 `FactorCandidate.residual_model` 匹配。执行"FactorBuilder 拥有数学
模型"这条架构不变量，前端永远不能直接注入权重。

`residual_block.hpp`（Ceres 风格契约）：
```cpp
class ResidualBlock {
 public:
  virtual ~ResidualBlock() = default;
  virtual int ResidualDim() const = 0;
  virtual std::vector<int> ParameterBlockSizes() const = 0;
  virtual bool Evaluate(const std::vector<const double*>& parameters, double* residuals,
                        std::vector<double*>* jacobians) const = 0;
};
```
这是刻意留的替换口子：未来把手写 Gauss-Newton 换成 Ceres/GTSAM 时，
`factor_builders` 不需要动（架构文档第 20 节，明确延后决策，CLAUDE.md 提醒不要
顺手去重构它）。注意：仓库里没有任何抽象 `Solver` 基类，`GaussNewtonSolver`
是具体的非虚类；"可替换"只是靠 `factor_builders` 只依赖 `ResidualBlock` 这一层
间接性来保证的，还没有真正的 `Solver` 接口。

`providers.hpp`（面向 adapters 的非阻塞轮询接口）：
```cpp
class LocalOdometryProvider { virtual std::optional<MeasurementEvidence> PollRelativePose() = 0; ... };
class MapObservationProvider { virtual std::vector<MapEvidence> PollMapEvidence() = 0; ... };
class SonarFrameProvider    { virtual std::optional<SonarFrame> PollSonarFrame() = 0; ... };
class CameraFrameProvider   { virtual std::optional<ImageFrame> PollImageFrame() = 0; ... };
```
全部非阻塞/轮询式，方便调度器车道在不阻塞 adapter 线程的情况下抽干队列。具体实现
只存在于 `adapters/`，`core/`/`algorithms/` 只知道接口。`CameraFrameProvider` 目前只有
L0 contract test 里的 fake 实现，没有真正的相机 adapter。

---

## 6. algorithms/ 层

### 6.1 `include/frontends/sonar_cfar_frontend.hpp`

三个文件：`cfar_detector.{hpp,cpp}`、`dbscan.{hpp,cpp}`、
`sonar_cfar_frontend.{hpp,cpp}`，现在合并进 `frontends`（`uw::frontends`）target。
测试用的是构造出来的合成数据，不是 golden fixture 文件（旧的空 `fixtures/`
目录已随本次布局重构一起清理）。

CFAR 检测器（移植自 `sonar_camera_reconstruction` 的 `CFAR.py`+`cfar.cpp`）：
```cpp
enum class CfarVariant { kCA, kSOCA, kGOCA, kOS };
struct CfarParams {
  int num_training_cells = 20;   // Ntc，须为偶数
  int num_guard_cells = 4;       // Ngc，须为偶数
  double probability_false_alarm = 1e-3;  // Pfa
  int rank = -1;                 // OS-CFAR rank，-1 表示用 Ntc/2
};
```
阈值因子在构造函数里一次性为四种变体求解：CA 是闭式解
`Ntc * (Pfa^(-1/Ntc) - 1)`；SOCA/GOCA/OS 通过自实现的二分法（`SolveDecreasingRoot`）
在基于 log-gamma 的 `GosocaCore` 函数上求根，替代了上游的
`scipy.optimize.root` 多起点搜索。`Detect()` 沿 range 轴逐 beam 滑窗，边界行
（首尾 `train_hs+guard_hs` 行）恒为 0；CA 对称求训练窗和，SOCA/GOCA 分前后段取
min/max，OS 用 `nth_element` 取排序统计量。

DBSCAN（`dbscan.hpp/cpp`）：明确是原创重实现，不是移植，教科书版
Ester et al. 1996，O(n²) 邻域查询，输入 `std::vector<Eigen::Vector2d>`。选择自实现
是因为上游 `cluster_scanline` 包了一层 sklearn，本仓库不想引入这个依赖。

`SonarCfarFrontend`（实现 `SonarFrontend`）：
```cpp
struct SonarCfarFrontendParams {
  CfarParams cfar;
  CfarVariant cfar_variant = CfarVariant::kSOCA;   // 匹配上游实际默认用法
  uint8_t detector_threshold = 0;
  double dbscan_eps_m = 0.2;         // 匹配上游 DBSCAN(eps=0.2, ...)
  int dbscan_min_samples = 2;
  double default_range_sigma_m = 0.05;
  double default_bearing_sigma_rad = 0.01;
};
```
`ProcessSonarFrame` 流程：① `!IsAzimuthAscending` 直接拒绝该帧，标记
`out_of_distribution` ② `intensity_tensor` 字节 → `Eigen::MatrixXf` ③ CFAR 检测
④ 每个 beam 列取最近命中且 `intensity > detector_threshold` 的 range bin 作为
首次接触点 ⑤ 转成局部笛卡尔坐标 `(r·cosθ, r·sinθ)` ⑥ DBSCAN 聚类 ⑦ 每个簇算
平均 range/bearing，打包成 `SonarRangeBearing`，通过 `MakeEvidence<>()` 包装，
`noise_scale = 1/簇大小`，模型标签 `"sonar_cfar_frontend_v1"`，likelihood=簇大小，
按 likelihood 降序放入 `candidates`；多于一簇时设置
`ambiguity_reason = "multiple DBSCAN clusters detected in this ping"`；
噪声点（`label==-1`）进 `rejected_candidates`。

输出始终停留在声呐局部极坐标系（range,bearing），从不重映射到笛卡尔图像网格，
更不会转换并固定进世界/`map` 坐标系。这是相对上游 `imaging_sonar.py`/`merge.py` 的
刻意偏离（见 `NOTICE`）。

### 6.2 `include/factor_builders/sonar_range_factor_builder.hpp` —— 全仓库数学核心

残差移植自 SVIn/OKVIS 的 `SonarError`：
```
mean = average(landmark_subset_W)          // 固定、不参与优化的 3D 点
delta = T_WS.translation - mean
range_corrected = ||delta||
residual = sqrt_information * (range_m - range_corrected)
```
参数块 `[0]` 是 7 维位姿。`landmark_subset_W` 为空，或
`range_corrected < 1e-9`（位姿与地标均值重合，退化）时 `Evaluate` 返回 `false`。

**雅可比是独立重新推导的**（不是从上游抄的）：
```
d(range_corrected)/d(translation) = delta / ||delta||
d(residual)/d(translation)        = -sqrt_information * delta / range_corrected
d(residual)/d(rotation)           = 0   （精确为零，不是近似）
```
头文件注释明确解释了为什么必须重推：上游通过一个近似方向（经过声呐 beam 的
range/bearing 端点而非地标均值，且没有 `residual = measured - computed` 隐含的
符号翻转）计算，这不等于它自己那个残差公式的真实导数，直接照抄会引入错误。

有限差分测试（`sonar_range_residual_test.cpp`）：对位姿的 3 个平移分量各做
`±1e-6` 中心差分，与解析雅可比逐列比对（容差 `1e-5`），并且精确断言（`EXPECT_EQ`
而非 `NEAR`）朝向列恒为 0。

Builder：`kResidualModel = "sonar_range_v1"`；要求量测结果携带
`SonarRangeBearing` 且 `context.nearby_points_W` 非空；
`sqrt_information = proposed_noise > 0 ? proposed_noise : 1.0`，builder 信任
已经被上游 cap 过的噪声值，不再自行判断（架构文档 8.4 节）。v1 限制：估计器
还不联合优化 3D 地标（图变量只有 keyframe 位姿），所以 `nearby_points_W` 目前由
外部提供（合成回放里来自 scenario 配置），不是来自实时地标库。这是
`submap_manager` 未来自然的扩展点。

### 6.3 `include/factor_builders/relative_pose_factor_builder.hpp`（原生，非移植）

6D 残差，两个 7 维位姿参数块 `[T_WBi, T_WBj]`：
```
predicted = T_WBi.Inverse() * T_WBj
translation_error = predicted.translation - measured.translation
rotation_error = measured_q.conjugate() * predicted.rotation
rotation_residual = 2 * rotation_error.vec()          // 小角度四元数误差近似
if rotation_error.w() < 0: rotation_residual = -rotation_residual   // 修正双覆盖符号翻转
residual = [sqrt_info_t * translation_error; sqrt_info_r * rotation_residual]
```
雅可比用内部中心有限差分计算（不是解析式），对两个 7 参数块各扰动
`±1e-6`。头文件明确称这是"刻意的 v1 简化（构造即正确）"，等真正上 Ceres/GTSAM
后再换成闭式的最小 SO3 雅可比。Builder（`kResidualModel = "relative_pose_v1"`）
对平移/旋转两块用同一个各向同性的标量 sqrt-information，注释关联到一个真实的
SVIn 审计发现（架构文档 22.4 节）：SVIn 的 `nav_msgs/Odometry` 没有可用的位姿
协方差，`LocalOdometryProvider` 包装层只能自己估一个噪声尺度，所以用单标量是
"诚实的 v1 选择"而不是伪造各向异性协方差。

### 6.4 `include/factor_builders/depth_factor_builder.hpp`（原生，非移植）

1D 残差，单个 7 维位姿参数块。世界系 Z 朝上，测得的深度（正值=水面以下）对应
位姿 Z 的负值：
```
residual = sqrt_information * (measured_depth_m + translation.z())
```
雅可比只有 tz 分量非零（`= sqrt_information`），线性关系，精确计算不需要有限差分。
**这就是 CLAUDE.md 里"z 轴 anchor bug"提到的那个因子**：一旦图里有深度因子，z 就
不再是 gauge freedom，固定/anchor keyframe 必须给自己真实的深度衍生 z，而不能
想当然地钉在 `Pose3::Identity()` 的 z=0（`application/replay_pipeline` 的处理见
[第 9.2 节](#92-appsreplay_demo--端到端主流程)）。

### 6.5 `include/estimation` —— Gauss-Newton/LM 求解器

`GaussNewtonOptions`/`GaussNewtonSummary` 被放在 namespace scope（不是嵌套在
`GaussNewtonSolver` 类里），专门规避 CLAUDE.md 记录的那个 GCC bug：嵌套聚合类型
作为同一个类里另一个方法的 `const T& = {}` 默认参数会编译失败：
```cpp
struct GaussNewtonOptions {
  int max_iterations = 30;
  double initial_lambda = 1e-3;
  double lambda_up_factor = 5.0;
  double lambda_down_factor = 3.0;
  int max_inner_retries = 8;
  double cost_change_tolerance = 1e-12;
};
struct GaussNewtonSummary {
  int iterations = 0;
  double initial_cost = 0.0;
  double final_cost = 0.0;
  bool converged = false;
};
// GaussNewtonSolver::Solve(PoseGraphProblem&, const GaussNewtonOptions& = GaussNewtonOptions{});
```

算法：稠密 Eigen 实现的 Levenberg-Marquardt，直接在每个 keyframe 的原始 7 参数
块上操作，不是严格的 6-DOF 切空间/流形更新，每步接受后就地重归一化四元数
（`RenormalizeQuaternion`）。这是文档化的刻意 v1 简化（架构文档第 20 节）。

主循环（最多 `max_iterations=30` 次外层迭代）：
1. 线性化：遍历所有残差块绑定，累积稠密法方程 `JtJ`/`Jtr`（只对自由/非固定
   keyframe 的列有贡献，每个 7×7 分块由 `free_index` 索引）。
2. 内层阻尼重试（最多 `max_inner_retries=8` 次）：
   `damped(i,i) += lambda * max(damped(i,i), 1e-12)`（Marquardt 式对角缩放）；
   `delta = damped.ldlt().solve(-jtr)`，稠密 LDLT（Cholesky）线性求解，
   有意为 v1 问题规模（个位数到几百个 keyframe）设计；应用 delta 并重归一化
   四元数；重新算 cost；`trial_cost <= cost_at_linearization` 则接受
   （`lambda /= 3`），否则回滚 + `lambda *= 5` 重试。
3. 若内层重试全部失败：跳出外层循环，诚实报告"没收敛"（`converged=false`）。
4. 收敛判据：`|cost_at_linearization - current_cost| < 1e-12` → `converged=true`。

`PoseGraphProblem`：图变量只有 keyframe 位姿，不联合优化 3D 地标。
`AddKeyframe(id, initial_pose, fixed=false)`、`AddResidualBlock(block, involved_keyframes)`
（顺序必须匹配 `ResidualBlockSizes()`，未知 id 抛 `std::out_of_range`）、
`SetKeyframePose`/`GetKeyframePose`。`GaussNewtonSolver` 通过
`friend class GaussNewtonSolver` 直接访问 `PoseGraphProblem` 的私有 map，避免拷贝
参数数组。

`StateStore`：单写者/多读者的版本化快照环形缓冲（`std::deque<StateSnapshot>`，
默认容量 256），`Commit()` 分配单调递增的 `next_version_`。

集成测试 `ThreeKeyframeChainConvergesToTruth`：3-keyframe 链（`kf0` 固定于原点，
`kf1`/`kf2` 自由且初值有扰动），两个真实 `RelativePoseResidual` + 一个真实
`DepthResidual`，求解后 `final_cost < 1e-6`、平移误差 `< 1e-3`，证明
"FactorBuilder → ResidualBlock → PoseGraphProblem → 求解器" 这条链确实能拼起来
跑，而且用的是真实残差类型，不是 test double。

### 6.6 `include/mapping/submap_manager.hpp`

目录名叫"submap"，但实现粒度其实是按 keyframe，没有距离/重叠/帧数触发的
"新建 submap" 逻辑。数据结构是
`std::unordered_map<keyframe_id, KeyframeMapState{pose_WB, evidence, stale}>`。

设计原则（对应架构 7.8/9/21 节）：`MapEvidence` 始终保存在局部坐标系并引用
源观测，插入时绝不转换并固定进全局位姿。这是对 `sonar_camera_reconstruction`
`merge.py` 的刻意反模式（呼应 [4](#4-跨语言规范化消息模型schemasproto) 的 `map.proto`
注释）。世界系坐标是按需从 keyframe 当前已知位姿现算的。

- `AddMapEvidence(evidence)`：追加到对应 keyframe 的 evidence 列表。
- `UpdateKeyframePose(id, new_pose_WB)`：keyframe 位姿变化时调用（例如位姿图优化
  之后）。若该 keyframe 任一条 evidence 的 `reintegration_policy() ==
  FULL_REFUSE`，标记该 keyframe `stale=true`；`TRANSFORM_ONLY` 的 evidence 不受
  影响，因为 `WorldPointsForKeyframe` 每次调用都会用*当前*位姿重新变换，位姿
  修正会自动传播，不需要重跑前端。
- `WorldPointsForKeyframe(id)`：目前只解码 `POINT_CLOUD` 表示（其余类型返回空，
  "v1 未实现"），把 `geometry_or_occupancy` 重新解释为紧凑 `float[3]` 三元组，
  逐点应用 `pose_WB.Apply(local)`。

### 6.7 `include/frontends/stereo_optical_depth_frontend.hpp`（声光 plan 2：optical baseline）

实现 `include/measurement_api/frontend.hpp` 的 `OpticalDepthFrontend`，产出 plan 1
新增的 `OpticalDepthPriorMeasurement`（`scale_status=METRIC`, `producer_type="stereo"`）。
原创实现，不移植第三方——本仓库没有 OpenCV/vendor 图像依赖，延续 `dbscan.hpp` 的先例。

- `include/sensor_models/camera_model.hpp`（`PinholeCamera`/`StereoGeometry`）：
  `PinholeCamera::FromIntrinsics` 从 `CameraIntrinsics.k_matrix_row_major` 读 fx/fy/cx/cy，
  忽略 distortion（v1 假设像素已去畸变）。`StereoGeometry::Resolve` 要求 rig 里两台相机的
  `frame_tree` 边旋转部分完全相同（`isApprox`，1e-9），只允许纯平移基线——对应
  `configs/rig/example_auv.yaml` 的实际布局；`valid=false` 而不是对不满足这个假设的
  外参静默给出错误的深度（一般任意朝向的立体校正在这个阶段刻意不实现）。
- `include/frontends/stereo_optical_depth_frontend.hpp/block_matcher.hpp`（`BlockMatcher`）：
  固定窗口 SAD 逐像素视差搜索，`right(u, v)` 在 `left` 里搜 `(u-d, v)`，`d` 取
  `[min_disparity, max_disparity]` 里 SAD 最小的一个；`min_disparity` 默认 1（视差 0 意味着
  无穷远，深度换算会除零）。迭代顺序固定（无 `unordered_map`/多线程），可复现。
- `stereo_optical_depth_frontend.hpp`（`StereoOpticalDepthFrontend`）：`bundle.secondary`
  缺失、`StereoGeometry::Resolve` 失败、或两张图 encoding/width/height 不一致都直接
  `std::nullopt`（拒绝整个 bundle，不猜测）。有效像素：`depth_m = fx * baseline / disparity_px`；
  `variance_m2 = (depth_m^2 / (fx * baseline) * disparity_sigma_px)^2`（标准逐像素视差
  不确定度传播，`disparity_sigma_px` 默认 0.5，是假设的固定值，不是标定出来的）。无效像素
  `depth_m=0, variance_m2=0`，匹配 plan 1 `ValidateOpticalDepthPrior` 对无效像素"没有语义"
  的约定。
- `evaluation/depth_metrics.hpp`（`ComputeDepthMetrics`）：只比较两个 grid 都标记为 valid 的
  像素，`valid_coverage_fraction` 相对 GT-valid 像素数定义（不是全图）；v1 限制（同
  `ComputeAte` 的写法一样明确写出）：不做 sonar-covered/视觉退化区域拆分，那需要场景 mask
  和声光关联，属于后续 plan。
- `apps/synth_stereo_gen.cpp`、`apps/optical_baseline_eval.cpp`：独立于
  `apps/synth_bag_gen.cpp`/`apps/replay_demo` 的两个新二进制，生成单帧合成立体图像对 +
  GT 深度（复用 `OpticalDepthPriorMeasurement` 当 GT 载体，`producer_type="ground_truth"`），
  跑真正的 `StereoOpticalDepthFrontend` 并用 `ComputeDepthMetrics` 打分。在真实 rig 尺寸
  （640x480，`fx=420`，`baseline=0.12m`）上实测 `rmse_m=0`、`coverage≈0.93`——场景是无噪声的
  单一深度平面，这证明的是几何管线本身正确，不是真实纹理下的匹配鲁棒性（那是 plan 5 的
  场景矩阵要验证的）。`uw_l2_optical_baseline_smoke_test` 把这套流程接成 CI 门禁
  （`--max-rmse-m 0.05 --min-coverage 0.9`）。最初它独立于位姿图回放；当前带相机 rig
  的 `synth_bag_gen`/`replay_demo` 也会构造它用于并行声光融合，但不会把稠密深度加成
  新的位姿图因子。`frontends.optical` 目前只有这一种被配置校验接受的实现。

### 6.8 `include/frontends/acoustic_optic_associator.hpp`（声光 plan 3：cross-modal geometry）

只做几何关联审计，**不做** posterior depth 优化——`AcousticOpticAssociationRecord` 的
`posterior_depth_m`/`posterior_variance_m2` 始终留 0，`reason` 从不设成
`POSTERIOR_INVALID`/`VARIANCE_NOT_IMPROVED`/`CROSS_MODAL_CONFLICT`（后两者依赖 posterior
残差，属于 plan 4 `AcousticOpticDepthFusionFrontend` 的范围）。继承仓库既有的 v1 规则
（`hypothesis.proto` 文档化的限制）：每次 `Associate()` 只消费 `HypothesisSet` 的 top-1
候选，最多产出一条 record。

- `include/sensor_models/camera_model.hpp` 新增 `OpticalFromBodyRotation()`：本平台
  `frame_tree`（`camera_*_link`/`sonar_link`/`base_link`）都是 body convention（x 前、y 左、
  z 上，跟声呐自己的局部系一致），而 `PinholeCamera::Project`/`Unproject` 是标准 optical
  convention（z 前、x 右、y 下）。plan 2 从没碰到这个问题——`StereoOpticalDepthFrontend`
  只用 `baseline_m`/`fx` 的标量，从没把 rig 的 `Pose3` 和 `Project` 接到一起。这个固定旋转
  （硬件安装常数，不是标定值）是 plan 3 第一次需要把两者接起来时补上的。
- `include/sensor_models/sonar_arc_projector.hpp`（`ProjectSonarArcToCamera`/
  `UnprojectPixelToSonarRangeBearing`）：前者采样理想弧
  `p_S(phi)=rho[cos(phi)cos(theta),cos(phi)sin(theta),sin(phi)]`（架构文档 8.1 节），经
  `camera_T_sonar`（body convention）→`OpticalFromBodyRotation()`→`PinholeCamera::Project`
  投到像素，只保留 optical-frame 深度为正且落在图像内的采样；后者是反方向（像素+
  `depth_m` 反投影→sonar frame→range/bearing，elevation 主动丢弃）。**`depth_m` 的语义是
  camera optical-frame 的 z（跟 `OpticalDepthPriorMeasurement.depth_m` 完全一致），不是到
  相机的欧氏距离**——单元测试踩过这个坑：boresight（bearing=0）时两者数值相同掩盖了这个
  区别，换成非零 bearing 才会暴露（0.05 rad 的测试差了 0.005m）。
- `runtime/acoustic_optic_synchronizer.hpp`（`SynchronizeAcousticOptic`）：纯函数，不是
  状态机/队列消费者。用 `t_reference = t_sensor_capture + time_offset_seconds[sensor_id]`
  （plan 1 的符号约定）分别修正 primary/secondary image 和 sonar 的 capture_time，
  pairwise 最大差超过 `max_time_delta_s` 就整体拒绝（`nullopt`），不做任何外推。
  `time_offset_seconds` 缺某个 sensor_id 时默认 0 偏移（v1 简化，写在函数注释里，没有
  RunManifest/health 审计）。
- `acoustic_optic_associator.hpp`（`AcousticOpticAssociator::Associate`）：先查
  `optical_evidence` 的 `scale_status`——非 `METRIC` 直接 `REJECTED`/`SCALE`；再用
  `sonar_arc_projector` 把该 sonar 假设的理想弧投到相机图像，对每个落在图内且
  `valid_mask` 有效的像素，用它的 `depth_m` 反投影回 sonar frame 算预测 range/bearing，
  和检测本身的 range/bearing 做残差 gate（`range_gate_m`/`bearing_gate_rad`），通过的按
  归一化残差平方和打分；**同一个像素被多个弧采样命中时会先去重**（保留最优分数）再判
  ambiguity margin——这是单元测试才发现的坑：`elevation_aperture_rad=0` 时全部
  `arc_samples` 采样会退化成同一个点，去重前会被误判成"多个互相竞争的候选"而错误标成
  `AMBIGUOUS`。几何最优两项仍落在 `ambiguity_margin` 内时，再比较两者深度差与
  `depth_agreement_sigma * sqrt(var_a + var_b)`：深度一致说明只是同一点的冗余弧采样，
  继续接受最优项；深度不一致才保留真正的 `AMBIGUOUS`。`candidate_pixel_indices`/
  `best_score`/`second_best_score`/`prior_depth_m`/`prior_variance_m2` 都是这一层就能
  算出来的几何量。
- plan 5 场景矩阵和带相机 rig 的 `replay_demo` 现已实际调用这三个组件；plan 3/4 的
  “只交付组件”是历史实施阶段，不再是当前接线状态。

### 6.9 `include/frontends/acoustic_optic_depth_fusion_frontend.hpp`（声光 plan 4：probabilistic fusion）

第一次真正产出 `FusedDepthMeasurement`（wire 量测结果，不只是进程内类型）。"不能证明一致，
就不融合"（架构文档第 9 节）：`Fuse()` 只要光学量测结果有有效的
`OpticalDepthPriorMeasurement` payload，就一定返回一个完整分辨率的 `FusedDepthMeasurement`
——**每个像素默认 `DEPTH_CONTRIBUTION_OPTICAL_ONLY`**（optical prior 原样透传），最多有
**一个**像素（plan 3 top-1 声呐假设选中的那个，且几何关联 `ACCEPTED`）可能被升级成
`DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC`——升级条件是 posterior 优化收敛、方差相对 prior 有
实质改善、且残差通过 innovation gate；任何一步没过，那个像素照样保持 optical prior 原值，
不是部分应用的"半融合"结果。`HypothesisSet` 为空（声呐掉线）时优雅降级成全图
optical-only、`associations` 为空——这是文档化的正常行为，不是错误路径
（架构文档第 10 节场景 8 sonar_dropout）。只有光学量测结果完全没有
`OpticalDepthPriorMeasurement` payload 时才返回 `std::nullopt`（没有可以融合的东西）。

- `posterior_depth_optimizer.hpp`（`OptimizePosteriorDepth`）：对 plan 3 选中的那个像素，
  优化标量 depth `d`：`min_d (d-d_o)²/σ_d² + (range(d)-ρ)²/σ_ρ² + (bearing(d)-θ)²/σ_θ²`，
  `range(d)`/`bearing(d)` 直接复用 plan 3 的 `UnprojectPixelToSonarRangeBearing`——这一层
  没有新增任何几何原语，只是绕着已有函数加了个标量优化器。v1 用**朴素平方残差**
  （Gaussian loss，不是 Huber/Cauchy，留作后续增强）和**确定性、有界的黄金分割搜索**
  （`d ∈ [d_o - k·σ_d, d_o + k·σ_d]`），不是无约束 Gauss-Newton——保证不会发散，代价是
  假设该区间内代价函数近似单峰（跟 `GaussNewtonSolver` 自己写明的 v1 局限性同一个精神）。
  posterior variance 用 Laplace 近似（`2/f''(d*)`，`f''` 用中心差分数值估计）。三种情况
  返回 `valid=false`：`prior_variance_m2`/`sonar_range_sigma_m`/`sonar_bearing_sigma_rad`
  任一 `<=0`，或最优点/代价非有限。
- `acoustic_optic_depth_fusion_frontend.hpp`（`AcousticOpticDepthFusionFrontend::Fuse`）：
  内部持有一个 `AcousticOpticAssociator`（plan 3）。只有 plan 3 判定 `ACCEPTED` 的候选才会
  被送进 posterior 优化；优化结果按顺序检查——非 finite → `REJECTED`/`POSTERIOR_INVALID`；
  方差没有按配置比例改善 → `REJECTED`/`VARIANCE_NOT_IMPROVED`；range/bearing 残差超过
  `innovation_gate_sigma` 倍 sonar sigma → `CONFLICT`/`CROSS_MODAL_CONFLICT`——这两个
  reason 正是 plan 3 明确留白、说"依赖 posterior 残差、属于 plan 4"的那两个。全部通过才
  写回 `posterior_depth_m`/`posterior_variance_m2` 并把该像素的 `contribution_mask` 设成
  `ACOUSTIC_OPTIC`。单元测试里用一个 boresight 退化配置（`range(d)=d`、`bearing(d)=0`
  恒成立）把整个 cost function 收敛成闭式加权最小二乘，可以直接断言优化器数值上收敛到
  手算的精确解，而不只是"往对的方向挪动了"。
- 三个模块都被 plan 5 场景矩阵调用；带相机 rig 的 `replay_demo` 也会运行同一套
  optical/sonar/fusion pass 并把点云局部地图数据交给 `SubmapManager`。后者是按 keyframe
  索引的局部地图数据存储，不是完整的 submap 生命周期管理器。位姿图 loop 仍不消费稠密
  深度，因此定位因子集合不会因这条并行 pass 改变。

### 6.10 `apps/acoustic_optic_scenario_matrix.cpp`（声光 plan 5：simulation/replay/evaluation）

第一次把 plan 1-4 的真实组件接成一条完整流水线跑通：`SynchronizeAcousticOptic` →
`StereoOpticalDepthFrontend` → `SonarCfarFrontend`（**不是新组件，是这个系列开始之前
就已存在的实现**）→ `AcousticOpticDepthFusionFrontend::Fuse`，跑架构文档第 10 节的
9 场景矩阵，每个场景默认 20 次独立种子 trial。不经过 MCAP（详见该 app 源文件头部注释
的取舍说明）；"三路消融"落地为 2 个条件（optical-only vs fused）× 2 个 region 切片
（全图、sonar 投影覆盖区，后者直接复用 `AcousticOpticAssociationRecord.
candidate_pixel_indices`，不是新的管线）——第三个切片（视觉退化区）被跳过，因为本
plan 的退化场景是整张图均匀退化，一个"局部退化区" mask 会退化成跟全图切片完全一样，
没有独立信息量。

**两个真实 bug，是靠实跑（不是单测）才暴露的，值得记录避免以后重踩：**

1. **GT 深度和实际烘焙进立体图像对的视差不自洽**：早期版本手写了一个"看起来合理"的
   GT 深度（6.0m），但实际用来生成图像对的视差是从 `fx*baseline/GT深度` 四舍五入到
   最近整数像素再反推回去的——四舍五入前后的深度不相等，造成全图恒定 ~0.3m 的
   系统性 RMSE，长得像一个流水线 bug，实际是场景构造的自洽性问题。修法：GT 深度必须
   由"四舍五入后的整数视差"反推，而不是反过来，这个原则跟 plan 2 的
   `synth_stereo_gen` 一致。
2. **立体图像对合成时，"这个像素属于目标 patch 还是背景"的判定，用错了参考系**：
   最初实现里 `RIGHT(u,v) = LEFT_texture(u + disparity_at(u,v))`，`disparity_at`
   直接读 RIGHT 自己的像素坐标 `(u,v)` 来判定 patch 归属。这看起来对称、无害，实际上
   因为视差本身会把 LEFT/RIGHT 的坐标错开，导致"能被干净恢复出目标视差的安全区域"
   在 LEFT（也就是深度网格实际索引的参考系）里被整体平移了 `target_disparity_px`
   个像素——连 patch 正中心都落在污染区里，block matcher 稳定恢复出一个两个视差之间
   的错误折中值。表现为：候选像素的 optical 深度既不等于目标深度也不等于背景深度，
   sonar 残差因此巨大，几何关联全部 `NO_CANDIDATE`。修法（`MakeStereoPair`）：改成
   标准的"背景铺满整张 RIGHT 图，再把目标 patch（从 LEFT 对应位置取内容、按视差平移）
   贴上去覆盖背景"——RIGHT 因此完全由自己的坐标决定内容来源，不再依赖"用哪个视差"
   这个尚待判定的量来判定自己的坐标属于哪个区域。

**P0 复核与关联器修复（`8df083b`）**：上面两个合成器 bug 修复后，矩阵进一步暴露
`clean_textured`/`elevation_stress` 的并列候选会被 100% 判为 `AMBIGUOUS`。根因不是
必须保留的物理歧义：近 boresight 时 bearing 与 elevation 无关、range 只有二阶变化，
同一平面 patch 上多个弧采样点会几何打平，但可能只是同一深度的冗余估计。
`AcousticOpticAssociator` 现在先比较前两名 `depth_m`：在
`depth_agreement_sigma=3.0` 倍联合标准差内一致就接受最优项，只有深度也明显不一致才
保留 `AMBIGUOUS`。两个回归测试分别锁定同意/不同意路径；固定 seed、20 trial 下
`clean_textured` 与 `elevation_stress` 均从 0/20 恢复到 20/20 accepted。

九场景的 gate 语义必须分开理解：`time_offset_fault`、`extrinsic_perturbation`、
`sonar_dropout`、`optical_invalid_region` 刻意构造为同步拒绝、几何 fail-closed 或
光学回退，0 accepted 是预期结果并被最低覆盖 gate 排除；其余五个有效场景必须至少
产生一个 accepted。`tests/integration/acoustic_optic_scenario_matrix_determinism_test.sh`
以 `--seed 4242 --trials-per-scenario 8` 运行两次，比较去掉真实墙钟
`p95_latency_ms` 后的输出，并保留第一次矩阵进程的退出码；coverage gate 非零会让
CTest 失败。`--min-fusion-improvement-fraction` 已实现但仍 opt-in，校准后的质量收益、
NLL 和真实调度器 P95 延迟门仍是后续工作。

### 6.11 `include/mapping/acoustic_optic_map_bridge.hpp`（声光 plan 6：局部地图数据交接，系列收尾）

**2026-08-22 更新**：这个文件现在有两个函数，不是一个——见本节末尾新增的
`FuseDepthIntoSurfels` 小节（P3 roadmap item 2「visual-only 和 sonar-grounded
两条局部几何路径」）。下面这几段描述的仍是原有的 `BuildMapEvidenceFromFusedDepth`，
**未被这次改动触碰**（`git log` 上是纯新增，不是修改）。

只有一个函数：`BuildMapEvidenceFromFusedDepth`。把 plan 4 的 `FusedDepthMeasurement`
转成 `MapEvidence`（`POINT_CLOUD` 表示），喂给 `include/mapping/submap_manager.hpp`——
**这个模块是声光系列开始之前就已经存在的**，本 plan 一行都没改它
（`git diff --stat -- include/mapping/submap_manager.hpp` 是空的），只是新增了第二个
`MapEvidence` 生产者。

坐标系链路（复用已有的三段几何，没有新增任何投影原语）：
`像素+深度 --PinholeCamera::Unproject--> optical frame --OpticalFromBodyRotation()ᵀ-->
camera body frame --camera_pose.Apply()--> base_link frame`——**存的是 base_link 系，不是
camera-optical 系，也不是 world 系**。这是刻意的：`SubmapManager::WorldPointsForKeyframe`
是用 `pose_WB.Apply(local)` 把本地点变到世界系，`pose_WB` 语义是"keyframe 的 base_link
→world 位姿"，所以 local 点必须先落在 base_link 系，`WorldPointsForKeyframe` 才能直接复用、
不用改一行代码。`reintegration_policy` 设成 `TRANSFORM_ONLY`——相机外参当作固定值（跟
plan 2-4 的既有 v1 范围一致），位姿修正只需要移动，不需要拿 `source_observations` 重新
跑一遍前端。

值得指出的对比：`src/application/replay_pipeline.cpp` 的声呐 landmark 插入代码用的是
另一条路——直接把点存成 `local_frame="world"`，keyframe pose 钉死成 identity，本质是绕开
"local 点要落在哪个参考系"这个问题的权宜写法（对应 README 里记录的 z=0 anchor 那类 v1
限制）。本 plan **没有改动、也没有替换** `replay_demo` 这段代码——只是新增了一条按照
`map.proto` 自己文档注释里写的原则（"local_frame + state_version，等 StateStore 修正
时才重新变换"）实现的、真正意义上"对"的路径，还没有接进 `replay_demo` 使用。

单测（`tests/mapping/acoustic_optic_map_bridge_test.cpp`，编译进 `mapping_tests`）里最后一个用例直接实例化真正的
`SubmapManager`（不是 mock），先设一次 keyframe pose 验证世界系坐标，再设第二次
*不同* 的 pose、**不重新 `AddMapEvidence`**，验证 `WorldPointsForKeyframe` 立刻反映新
位姿——这正是架构文档第 16 节"融合局部地图数据可在 state 更新后重新变换，不被前端
固化到 world frame"这条完成条件的直接证明。

**声光系列六个 plan 到这里全部完成**：contracts/calibration → optical baseline →
cross-modal geometry → probabilistic fusion → simulation/evaluation → 局部地图数据交接。
六个 plan 交付的是一套经过单测和端到端场景矩阵验证过的、可复用的组件集合。见 6.12 节——
回放管线现在会在加载了带相机的 rig 时真正构造并调用这些组件，但这是一次
独立的、后续的集成工作（见下），不是六个 plan 本身自带的。

#### `FuseDepthIntoSurfels`（P3 roadmap item 2，2026-08-22）

第二个函数，同文件、同 CMake target（`mapping`/`mapping_tests`），复用同一份
`FindCamera`/`FindEdgePose` 匿名命名空间辅助函数，跟 `BuildMapEvidenceFromFusedDepth`
并列存在，**不是替换**——两者都还在，签名和行为都没变。

**决策：一条统一路径，不是两个独立入口。** roadmap 说的"visual-only 和
sonar-grounded 两条局部几何路径"，直接对应 `measurement.proto` 里
`FusedDepthMeasurement.contribution_mask` 已经在用的 `DepthContribution` 枚举
（`DEPTH_CONTRIBUTION_OPTICAL_ONLY` / `DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC`）——这不是
新发明的分类，是 `src/frontends/acoustic_optic_depth_fusion_frontend.cpp`（plan 4）
早就在写、但 `BuildMapEvidenceFromFusedDepth` 一直没用上的字段。验证过一个关键前提：
只有当 posterior（声呐修正后）方差比 optical prior 方差**证明性地**更好（差距超过
`min_variance_improvement_fraction`）时，像素才会被标成 `ACOUSTIC_OPTIC`（见该文件
87-90 行的 reject 分支）——也就是说，"sonar-grounded"像素在数据模型层面就保证比
"visual-only"像素置信度更高，不需要额外判断。`FuseDepthIntoSurfels` 把
`confidence = 1/variance_m2`（`Surfel::confidence` 文档注释里写好的约定）直接喂给
`include/mapping/surfel_map.hpp`（P3 D8）的置信度加权合并——两条路径的区别，落到代码
里就是同一个像素携带的 confidence 数值不同，`SurfelMap::MergeInto` 已有的加权平均逻辑
自动让声呐修正过的观测在合并时占主导，不需要在这个新函数里加任何 if/else 分支区分
两条路径。

**法向量估计**：`FusedDepthMeasurement` 是按 `width x height` 行主序排列的规则网格
（不是无序点云），所以每个像素的右邻居 `(u+1,v)` 和下邻居 `(u,v+1)` 若也有效，就能
反投影三个点、取切向量叉乘得到一个真实的局部法向量——`tangent_down.cross(tangent_right)`
这个叉乘顺序对着摄像机方向的正面平面会给出朝向摄像机的法向（optical 系里 -Z），跟本
仓库大多数地方一样，只是"面向传感器"的第一版约定，没有做多视角一致性的符号归一化。
只有同时具备右、下邻居的像素才会调用 `SurfelMap::AddPointWithNormal`；其余像素仍走
`AddPoint`（法向未知）。

**真实数据验证**（不只是手搭的单测 fixture）：临时在
`apps/acoustic_optic_scenario_matrix.cpp` 里加了一个只触发一次的探针（验证完已经
`git checkout` 撤销，不是永久改动），喂真实 `clean_textured` 场景第一条 trial 产出的
`FusedDepthMeasurement`：285322 个像素喂入，合并成 34662 个 surfel，其中 34624 个
（99.9%）成功估计出法向量，confidence 取值范围 `[0.0016, 522.5]`——全部是有限数值，
没有 inf/nan/负数。这也顺带实测验证了 `SurfelMap` 头文件里早就写明的暴力 O(n) 扩展性
限制是真的：探针最初写成跨整个 9 场景矩阵累积进同一个 `SurfelMap`，直接让整个
`acoustic_optic_scenario_matrix` 二进制从平时的约 38s 变成 75s+ 还没跑完（被手动
kill），改成只触发一次之后才在正常时间内跑完。

**跟 D8 的关系**：D8 自己的 scope 边界写得很清楚——`SurfelMap` 要接进真实
pipeline（比如 `replay_demo`/`SubmapManager`）之前，空间索引是硬前提，不是可以往后
拖的优化项。这次的改动同样没有碰这个边界：`FuseDepthIntoSurfels` 证明了"给定真实
`FusedDepthMeasurement`，能不能算出正确的 confidence 加权和法向量"这个问题，
**没有**让 `SurfelMap` 变成 `MapEvidence`/`replay_demo` 的第四个证据源；那仍然需要
先解决 O(n) 扩展性问题，属于后续工作。

**单测**（`tests/mapping/acoustic_optic_map_bridge_test.cpp` 新增 3 个 case）：
(1) 无 `FusedDepthMeasurement` payload 时返回 0，不崩溃；(2) 一个 2x2 正面平面 patch，
手算出预期法向量 `(-1,0,0)`（推导过程写在测试注释里），跟代码算出来的比对；
(3) 两个相距 0.02m（在默认 0.05m 合并半径内）、但 variance 差 100 倍的观测点，验证
合并后的 confidence 恰好是两者之和、位置明显偏向高置信度（sonar-grounded）那一侧，
不是简单平均。

#### `SurfelMap` 的 pose correction reintegration（P3 roadmap item 4，D11，2026-08-22）

**要解决的真实架构张力**：`SubmapManager`（点云那条路径）的 reintegration 几乎是免费
的，因为它从不跨 keyframe 融合证据——`MapEvidence` 按 keyframe 存局部坐标系原始点，
`WorldPointsForKeyframe` 每次调用都用**当前**位姿重新变换，位姿变了只是变换矩阵变了，
局部点本身从来没被改写过。`SurfelMap`（D8/D9）完全不同：它的核心价值就是**跨
keyframe** 的置信度加权融合——`MergeInto` 维护的是一个运行加权平均，一个 `Surfel`
当前的 `position_W`/`normal_W`/`confidence` 是若干个 keyframe 观测混合之后的结果，
且不记录是谁贡献了什么、贡献时的原始值是多少。如果某个 keyframe 的位姿后来被位姿图
修正了，没法简单"重新变换"一个已经跟别的 keyframe 混合过的 surfel——那次混合是在
**旧位姿**下算出来的，而且（这是决策的关键）`MergeInto` 只保留归一化后的
`normal_W`，不保留归一化前的加权和，所以就算想做"减去旧贡献、按新位姿重新加"的
增量式回退（retract-and-redo），法向量这一半在数学上都不是无损可逆的。

**决策：局部观测账本 + 按需整体重建，不是增量式 retract。** 新增
`SurfelMap::AddKeyframeObservation(WithNormal)(keyframe_id, point_local, ...,
local_to_world)`：每次调用既立刻按当前 `local_to_world` 融合进 `surfels_`（跟
`AddPoint`/`AddPointWithNormal` 一样便宜），又把这条原始局部观测存进一个按
`keyframe_id` 分组的账本（`keyframe_records_`）。`ReintegrateKeyframe(keyframe_id,
new_local_to_world)` 更新该 keyframe 记录的位姿，然后**清空 `surfels_`、用账本里
每个 keyframe 各自当前的位姿把所有观测重新跑一遍融合**——不是增量回退，是精确重算。
代价是 O(账本里全部观测数)，不是 `AddPoint` 那种 O(1) 摊还；这个代价是刻意接受的：
`SurfelMap` 本来就还没接入真实 pipeline（D8/D9 都反复确认这一点，见下），O(n) 暴力
最近邻本身就还没解决扩展性问题，在"还没解决扩展性之前，先保证正确性"这个前提下，
精确重算比增量回退更简单、不会跨多次 retract/redo 累积浮点误差，权衡是合理的。

**刻意不做的事：没有配一个 `StaleKeyframes()` 式的"脏标记"查询。** 本仓库自己的
点云路径（`SubmapManager::StaleKeyframes()`）已经有一个这样的机制，但 P1 workstream
B5（audit 工具那轮）验证过一个事实：**除了它自己的单测，仓库里没有任何代码调用
`StaleKeyframes()`**——一个"检测到 stale、但没人消费"的机制不解决任何实际问题。
`ReintegrateKeyframe` 反过来是"位姿修正落地的那一刻就地重算"，正确性由调用约定
保证，不依赖"以后某个东西会去轮询一个标记"这种从没被验证过的假设。

**顺手修的一个真实 bug，不是事后补充**：写第一版 `ReintegrateKeyframe` 时，
`RebuildFromKeyframeRecords()` 无条件 `surfels_.clear()` 再只按账本重建——这会把
通过 `AddPoint`/`AddPointWithNormal`（不挂靠任何 keyframe）加进来的 surfel 在第一次
调用任意一次 `ReintegrateKeyframe` 时**直接销毁**，跟头文件本来准备写的"未挂靠点不受
影响"矛盾。修法：`AddPoint`/`AddPointWithNormal` 内部也把观测记进账本，用一个保留
的、真实 keyframe_id 永远不会撞上的空字符串键（`kUnattributedKeyframeId`，
`surfel_map.cpp` 匿名命名空间），身份是 identity 位姿——这样任何一次重建都会把它们
原样重放回去，不会丢。`SurfelMap.PlainAddPointSurfelsSurviveReintegrationOf
AnUnrelatedKeyframe` 这个单测的注释里写明了这个 bug 和修法，不是事后补的说明，是
写测试时真实发现、真实修的。`NumTrackedKeyframes()` 特意排除这个保留键，语义上只数
"真的通过 `AddKeyframeObservation` 挂靠过的 keyframe"。

**D9 的接入**：`FuseDepthIntoSurfels` 签名加了 `keyframe_id` 参数（此前
`SurfelMap`/`FuseDepthIntoSurfels` 都没有任何真实调用方，只有自己的单测用它——改签名
不影响任何已落地的 pipeline 代码），内部从算 world-frame 点改成算 base_link-frame
点（跟 `BuildMapEvidenceFromFusedDepth` 用的是同一个"local"约定），再调用
`AddKeyframeObservation(WithNormal)` 而不是原来的 `AddPoint(WithNormal)`——这样
`FuseDepthIntoSurfels` 自己文档里写过的那句"SurfelMap has no deferred-reintegration
concept yet"就不再成立了。

**验证（真实跑出来的，不是推算）**：
- `SurfelMap` 层新增 4 个单测：两个 keyframe 观测同一物理点先合并、纠正其中一个的
  位姿后按新位姿正确分裂成两个 surfel（`ReintegratingAKeyframeAfterPoseCorrection
  RefusesItsObservationsAtTheNewPose`）；位姿修正后仍在合并半径内、验证融合后位置
  按新权重正确更新（`ReintegratingAKeyframeThatStillMergesUpdatesTheFusedPosition
  Correctly`）；未挂靠点在别的 keyframe 重整合时不受影响（上面提到的那个 bug
  回归测试）；对没记录过的 `keyframe_id` 调用 `ReintegrateKeyframe` 是空操作
  （`ReintegrateKeyframeIsNoOpForAnUntrackedKeyframeId`）。
- `FuseDepthIntoSurfels` 层新增 1 个衔接测试
  （`ReintegratingAKeyframeAfterFusionCorrectlyRefusesItsContribution`）：两次真实
  `FuseDepthIntoSurfels` 调用（同一像素、不同 keyframe_id、初始位姿相距 2cm）先合并
  成 1 个 surfel、confidence 正确累加到 2.0，再对其中一个 keyframe 做 2m 量级的真实
  位姿修正、调用 `ReintegrateKeyframe` 后正确分裂成 2 个 surfel。
- 已有的 D8/D9 单测（`AddPoint`/`AddPointWithNormal`/`ConsumesSubmapManager
  WorldPointsForKeyframeOutput` 等 17 个、`FuseDepthIntoSurfels` 原有 3 个）全部
  不改行为、全部继续通过——只是三处调用点加了一个 `keyframe_id` 实参。
- `cmake --build`：干净。`ctest --test-dir build --output-on-failure`：**165/165**
  （D11 开始前是 160/160——5 个新 case：`SurfelMap` 4 个 + `FuseDepthIntoSurfels`
  1 个）。`tools/lint/check_no_ros_in_core.sh`：OK。

**跟 D8/D9 的关系，没有越界**：`SurfelMap`/`FuseDepthIntoSurfels` 仍然没有接入
`apps/replay_demo`/`MapEvidence`/`SubmapManager` 成为真实 pipeline 的证据源——那仍然
需要先解决 D8 自己文档写明的 O(n) 暴力最近邻扩展性问题，这次的改动没有碰这个边界，
也没有试图绕过去。

#### 异常点抑制与自由空间/遮挡处理（P3 roadmap item 3，D10，2026-08-22）

roadmap 这一条"uncertainty-aware 融合、自由空间/遮挡处理和异常点抑制"里，
"uncertainty-aware 融合"那一半 D9 已经做了（confidence 加权合并）；D10 补的是剩下
两半。

**异常点抑制：一个统计门限，直接复用仓库已有的 sigma-multiple 门限惯例。**
`SurfelMapParams` 新增 `outlier_gate_sigma`（默认 3.0），跟
`AcousticOpticAssociatorParams::depth_agreement_sigma`、
`AcousticOpticDepthFusionParams::innovation_gate_sigma` 用的是同一套约定和默认值——
`FindNearest` 在 `merge_distance_m` 内找到候选后，还要再过一道门：新观测和既有 surfel
的位置差平方是否超过`（1/existing.confidence + 1/新观测confidence）* sigma²`（跟
`acoustic_optic_associator.cpp` 里 `depth_agreement_sigma` 的平方比较公式完全一样，只是
从标量深度换成了 3D 距离）。**决策：门限没过不是丢弃观测，是让它单独成为一个新
surfel**，不是简单拒绝——这保留了信息（可能是真的第二个表面，或者一个移动物体，不只是
传感器噪声），跟简单丢弃相比更保守，也更符合仓库一贯"宁可保留两个假设，不强行平均出一个
可能错的结果"的风格。新增 `NumOutliersRejected()` 诊断计数器（风格上跟
`BoundedQueue::DroppedCount()` 一致）。

**验证过一件事，而不是假设它成立**：`DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC` 像素（更高
confidence、更低 variance）跟一个已有的、置信度较低的 `OPTICAL_ONLY` surfel 冲突时，
到底该走"融合并让声呐修正的观测主导"（D9 已验证的行为）还是"判成异常点、拆成两个
surfel"？推导下来：**同一套统计门限自动做出了正确区分，不需要额外的 if/else 按
contribution 类型分支**——因为门限用的是两者的*组合*方差，既有 surfel 自己越不确定，
组合方差就越大，门限就越松，一个适度的差异会落在门限内正常合并（D9 那种"高置信度
观测主导"场景）；只有当差异大到连组合不确定性都盖不住时，才会被判成真正的冲突。这跟
D9 自己"一条统一路径，不用按来源分支"的思路是同一个洞察的延续。

**自由空间/遮挡处理：`SurfelMap::CarveFreeSpace(ray_origin_W, ray_end_W)`，范围有意
限定在光学路径。** 一个观测点意味着从传感器到这个点的整条视线上都没有遮挡物——任何
已有的、真正落在传感器和这个新观测点之间（不是恰好在新观测点自己这里，也不是超出新
观测点更远）、且垂直距离在 `free_space_corridor_radius_m`（默认等于
`merge_distance_m`）内的 surfel，都被这条视线证伪了。策略：**不是直接删除，是每次
碰撞把 confidence 乘以 `free_space_confidence_decay`（默认 0.5），跌破
`free_space_removal_confidence_threshold`（默认 0.01）才真正移除**——单次视线本身也是
有噪声的证据，跟这个类一贯"靠多次观测累积、不靠单次判定"的风格一致。**范围决策**：只
处理相机/光学深度这条几何（`FuseDepthIntoSurfels` 调用点，见下），没有覆盖声呐——本
仓库稀疏声呐 landmark 走的是完全独立的另一条路（`SubmapManager::QueryNearestPoint`，
`src/application/replay_pipeline.cpp` 声呐那段代码驱动），根本不喂给 `SurfelMap`，没有
现成的接入点可以扩展，所以没做，不是漏掉。

**写测试时抓到、修在合并前的一个真实 bug**：`FuseDepthIntoSurfels` 对每个像素先调用
`AddKeyframeObservation(WithNormal)` 再调用 `CarveFreeSpace`，用的是*同一个*观测点。
如果这个点跟附近已有 surfel 合并、confidence 加权平均把位置拉到离像素自己的精确反投影
点差了几毫米，`CarveFreeSpace` 原始实现里"t 参数是否 <1"这个判断可能因为浮点误差把这个
刚合并出来的 surfel 自己判成"挡在视线中间"，当场把自己碳化掉。修法：加了一道跟 t 参数
无关的直接保护——任何在 `ray_end_W`（终点本身，不是投影点）`free_space_corridor_radius_m`
范围内的 surfel，一律不参与碳化，不管 t 算出来是多少。`SurfelMap.CarveFreeSpaceDoesNot
CarveASurfelNearButNotExactlyAtTheEndpoint` 这个单测用手算的 t=0.995（应该被原始 t<1
判断误伤，但被新保护挡住）精确复现了这个 bug 和修法。

**接入点**：`FuseDepthIntoSurfels` 里每个像素融合完之后紧跟着调用一次
`surfels.CarveFreeSpace(camera_origin_W, pose_WB.Apply(point_base_link))`——`camera_
origin_W` 只在循环外算一次（相机位置对同一帧所有像素是常量）。

**验证（真实跑出来的，不是推算）**：
- `SurfelMap` 层新增 7 个单测：异常点门限内接受（0.3m 差、门限约 0.4243m）/门限外拒绝
  拆成两个 surfel（0.5m 差）各一个，都在测试注释里手算了门限的具体数值；`CarveFreeSpace`
  的衰减到移除（两次碰撞：1.0→0.5→0.25，配合自定义移除阈值 0.4 精确复现）、走廊外不受
  影响、终点处/终点之外不受影响、终点附近但 t<1 不被误伤（上面那个 bug 的回归测试）、
  以及碳化效果不会在 `ReintegrateKeyframe` 重建后保留（碳化只改 `surfels_`，不记进
  `keyframe_records_` 账本，这是刻意的、写进了头文件的已知边界，不是遗漏）各一个。
- `cmake --build`：干净，无新增警告。`ctest --test-dir build --output-on-failure`：
  **172/172**（D10 开始前是 165/165——7 个新 case，全部在 `SurfelMap` 这一层；
  `FuseDepthIntoSurfels` 的 4 个已有单测不变，因为新增的 `CarveFreeSpace` 调用对它们
  用到的小规模、宽松间距的 fixture 没有产生足够近的伴随 surfel 去触发碳化）。
  `tools/lint/check_no_ros_in_core.sh`：OK。真实 `synth_bag_gen`+`replay_demo` 跑一遍，
  ATE 和迭代次数跟基线完全一致（0.0665821m，6 次迭代）——`SurfelMap` 仍未接入这条
  pipeline，这次改动不可能影响它。

**跟 D8/D9/D11 的关系，没有越界**：`SurfelMap`/`FuseDepthIntoSurfels` 仍然没有接入
`apps/replay_demo`/`MapEvidence`/`SubmapManager` 成为真实 pipeline 的证据源——同一个
O(n) 扩展性前提没有被这次改动碰过。异常点抑制和自由空间碳化都只用单测验证正确性，没有
像 D9 那样额外跑一次真实场景数据的探针——已有的 hand-derived 单测已经把两个新机制的
判定边界钉得很精确，真实数据能验证的主要是"规模够不够用"，而规模问题本身就是 D8 那个
还没解决的前提，不是这次工作范围内的事。

### 6.12 回放管线接入声光融合（rig-gated，位姿图本身不受影响）

`replay_demo`/`synth_bag_gen` 现在会在 `--experiment` 加载的 rig 含相机时，真正构造并跑
`StereoOpticalDepthFrontend` → `SonarCfarFrontend`（复用已有实例，不是新建）→
`AcousticOpticDepthFusionFrontend::Fuse` → `BuildMapEvidenceFromFusedDepth`，产出结果存进
`submap_manager` 的**第三个** `MapEvidence` bucket（跟既有的 `"landmarks"` bucket 并列，
按 keyframe 单独存）。**没有 `--experiment` 时这两个 app 的行为逐字节不变**——新代码全部
包在 `if (rig.has_value())` 里，`uw_l2_replay_determinism_test`（不传 `--experiment`）在
改动前后都能过，这是这次改动的硬性回归红线。

**明确没做的事**：稠密深度**没有**变成位姿图的新 factor 类型——`PoseGraphProblem`/
`GaussNewtonSolver`/轨迹 ATE 完全不受影响，声光输出只是并行存进 submap，不参与位姿估计。
把稠密深度接成 factor 是一个量级更大、需要新残差模型和信息量标定的工作，不在这次改动
范围内，也不应该被理解成"顺手就能做"的后续小任务。

**下面两个数字是 `f285e0d` 时的历史复核记录，不是当前验收基线**。保留它们是为了
解释声光集成刚接线时的行为；当前正确性由 6.10 节的场景矩阵和第 12 节的测试门禁判断：

1. `configs/experiment/synthetic_smoke.yaml`（既有场景，逐字节未改）：
   `acoustic-optic: 12 keyframes with camera data, 0 accepted, 0 ambiguous, 0 conflict,
   12 rejected, 3418897 map evidence points added`，`ATE: rmse=0.0666m`。0 accepted 不是
   bug——直接算过：这个场景的三个目标在整条轨迹上没有一帧的方位角落在相机窄视场（半 FOV
   ≈0.65 rad）内，只在声呐的宽视场（半 FOV 3.0 rad）里，真实几何决定的，不调整既有
   scenario 去凑一个"看起来更好"的数字。即便如此，342 万个稠密立体点仍然被正确地当
   `OPTICAL_ONLY` 贡献存进了 submap——这本身就是这次集成的真实产出，不是"什么都没发生"。
2. 新增、独立于上面那个的 `configs/experiment/acoustic_optic_demo.yaml`
   （`configs/scenario/acoustic_optic_demo.yaml` 只有一个目标，放在 kf0 相机正前方，
   两个视场都能看到）：`0 accepted, 0 ambiguous, 0 conflict, 10 rejected`（另外 2 个
   keyframe 目标连声呐视场都出了，走 `synth_bag_gen` 既有的"frame written background-only"
   告警路径，不是新代码的问题）。`ATE: rmse=0.178m`。

   > **数字为什么跟本节最初写下时不一样**：26c8b26 刚接线时这两个场景分别记录的是
   > `rmse=0.213m`/`3424176` 个地图点 和 `3 ambiguous`/`rmse=0.061m`；本次复核实测
   > 变成了 `rmse=0.0666m`/`3418897` 和 `0 ambiguous`/`rmse=0.178m`。根因不在声光融合
   > 本身，而是 b2c19e1（6.13 节）
   > 改了 `synth_bag_gen.cpp` 的 `BuildVisualLandmarks`（"按 keyframe 而不是散布在整条
   > 轨迹上放置视觉路标"），而这个函数消费的是跟 `relative_pose_noise`/`sonar_range_noise`/
   > `sonar_bearing_noise` 同一个种子为 42 的 `std::mt19937_64 rng`、且在它们之前调用——
   > draw 序列一变，后面所有噪声采样全部跟着偏移，即使这两个 experiment/scenario yaml
   > 文件本身字节未改。`acoustic_optic_demo.yaml` 场景下 `ambiguous` 从 3 降到 0 意味着
   > 本节原先"两次独立复现同一个 DBSCAN 多候选现象"的说法不再被当前代码路径支持，
   > 此处如实更正而不是保留一个不再成立的结论；plan 5 场景矩阵（6.10 节）本身的
   > `clean_textured`/`elevation_stress` 结论不受影响，因为 `acoustic_optic_scenario_matrix`
   > 是独立 trial，不经过 `synth_bag_gen` 这条共享 RNG 流。

### 6.13 `include/frontends/stereo_landmark_vo_frontend.hpp`（声光系列之外：真实相对位姿 VO，b2c19e1）

commit `b2c19e1` 新增，独立于第 6.1–6.12 节的声光 plan 1–6 系列，也不移植自任何
external repo（原创实现，无需 NOTICE 条目）。目的：把 `synth_bag_gen` 写进 bag 的
ground-truth+noise "black-box VIO" 相对位姿证据换成从左右相机帧真正算出来的相对
位姿，供回放管线在 `estimator_mode: stereo_landmark_vo` 时消费（见 9.2 节）。
五个新文件，全部合并进既有的
`frontends`（`uw::frontends`）target，测试合并进既有的 `frontends` GTest
executable（`tests/frontends/{harris_corner_detector,landmark_blob_detector,
patch_matcher,rigid_transform_fit,stereo_landmark_vo_frontend}_test.cpp`）：

- `include/frontends/landmark_blob_detector.hpp`（`LandmarkBlobDetector`）：固定阈值
  （`intensity_threshold=140`）连通域检测——高于阈值的像素做 4-连通 flood fill，每个
  连通域归约成质心 + 固定尺寸外观 patch（`patch_half_size=6`，从原始未阈值化图像采样）。
  `min_blob_pixels=4`/`max_blob_pixels=400` 分别滤掉单像素噪声和大片饱和区域。原创
  实现，是为 `synth_bag_gen.cpp` 的 `BuildVisualLandmarks` 合成高亮方块场景调的参数，
  默认检测器（`frontends.landmark_detector` 未设置或设为 `bright_blob` 时使用）。
- `include/frontends/harris_corner_detector.hpp`（`HarrisCornerDetector`）：Sobel 梯度
  → 窗口化结构张量（`window_radius=2`）→ `R = det(M) - k*trace(M)^2`（`k=0.04`）→
  相对阈值（`quality_level=0.01`，乘以本图最强响应，不是绝对量级，因为 Harris 响应
  单位是梯度的 4 次方，不同曝光/内容下没有固定意义的绝对刻度）→ 非极大值抑制
  （`nms_radius=5` 邻域内只留最强响应）→ 按响应强度取前 `max_corners=60` 个。输出复用
  `LandmarkBlobDetector` 同一个 `LandmarkBlob` 类型（`pixel_count` 对点特征无意义，
  恒为 1，只是为了跟 `PatchMatcher`/`StereoLandmarkVoFrontend` 共用一套下游类型）。
  是给真实相机画面用的检测器——没有理由假设真实场景里存在孤立高亮色块，通过
  `frontends.landmark_detector: harris_corner` 选择。原创实现，同 `dbscan.hpp`/
  `landmark_blob_detector.hpp` 一样的先例（见 NOTICE）。
- `include/frontends/patch_matcher.hpp`（`PatchMatcher`）：在两组 `LandmarkBlob` 的
  外观 patch 之间做贪心最优匹配，用归一化互相关（NCC，`min_ncc_score=0.6` 阈值）——
  纯外观匹配，不看位置。确定性：所有候选对按固定 `(a-index, b-index)` 顺序打分一次，
  然后反复取当前剩余候选里分数最高的一对（打平按 index 顺序），直到没有候选
  ≥ `min_ncc_score` 或一侧耗尽为止，不依赖 hash/map 迭代顺序。`StereoLandmarkVoFrontend`
  用同一个类做两种匹配：立体（左右目）和时序（上一帧左目 vs 当前帧左目）。**注意跟
  6.7 节 `block_matcher.hpp` 的 `BlockMatcher` 区分**：`BlockMatcher` 是逐像素固定窗口
  SAD 稠密视差搜索（`StereoOpticalDepthFrontend` 用），`PatchMatcher` 是离散路标之间的
  NCC 匹配（`StereoLandmarkVoFrontend` 用）——两个名字相近但是两套独立实现，`block_matcher`
  预先于 b2c19e1 就存在，不是这次新增的。
- `include/frontends/rigid_transform_fit.hpp`（`FitRigidTransform`/
  `FitRigidTransformRansac`）：`FitRigidTransform` 是闭式 Kabsch/Procrustes SVD 解，
  求 `T` 使 `Σ||b[i] - T.Apply(a[i])||²` 最小，至少需要 3 个点，SVD 不收敛或点数不足时
  返回 `std::nullopt` 而不是给一个数值垃圾的变换。`FitRigidTransformRansac` 是它的
  RANSAC 鲁棒化版本：反复从对应点里随机采样 3 点拟合候选变换，用
  `inlier_threshold_m=0.3` 统计每个候选能解释多少全体对应点，保留最优候选的 inlier
  集合后再对整个 inlier 集合做一次 `FitRigidTransform` 精修（标准 RANSAC 流程）；
  `max_iterations=200`，最优候选 inlier 数不足 `min_inliers=3` 时整体返回
  `std::nullopt`；恰好 3 个点时直接退化成 `FitRigidTransform`（没什么可鲁棒化的）。
  要求调用方传入一个显式播种、构造后不再重新播种的 `std::mt19937_64&`（CLAUDE.md 的
  RNG 纪律/确定性回放测试的直接要求），`StereoLandmarkVoFrontend` 在构造时用
  `params.rng_seed`（默认 12345）播种一个自己的实例专用 RNG，正是为此。
- `include/frontends/stereo_landmark_vo_frontend.hpp`
  （`StereoLandmarkVoFrontend : VisualOdometryFrontend`）：有状态（跨 `Process()` 调用
  保存上一帧三角化出的路标：3D 点 + 外观 patch），流程：① 用 `params_.detector_kind`
  选定的检测器（默认 `bright_blob`）分别检测左右图路标 ② `PatchMatcher`（`stereo_matcher`
  参数）做左右目匹配，视差 `disparity = left.centroid_u - right.centroid_u`，
  `disparity < min_disparity_px`（默认 1.0，跟 `BlockMatcher` 同一个约定：视差 0 意味着
  无穷远）的匹配丢弃，用跟 `StereoOpticalDepthFrontend` 一样的公式
  `depth_m = fx·baseline/disparity` 反投影出相机系 3D 点 ③ 首帧（没有"上一帧"）直接
  返回 `std::nullopt` ④ 非首帧：`PatchMatcher`（`temporal_matcher` 参数）把当前帧路标
  和上一帧路标再做一次外观匹配（真实前端没有任何外部给的路标 id，只能靠外观重新
  关联），少于 `min_landmarks_for_pose`（默认 3）对匹配则放弃这一帧 ⑤
  `FitRigidTransformRansac(current, previous, ransac, rng_)` 拟合两组三角化点之间的
  刚体变换，失败（RANSAC 内点不足/SVD 不收敛）则放弃 ⑥ 相机光学系变换转体坐标系
  变换（见下方"踩过的坑"）⑦ 包装成 `RelativePoseMeasurement`
  （`from_keyframe`=上一帧 id，`to_keyframe`=当前帧 `ImageFrame.header.observation_id`），
  经 `MakeEvidence(..., "stereo_landmark_vo_frontend_v1")` 返回。硬性前提：两张图必须都是
  `MONO8`（`ConvertToMono8`，见第 8.1 节新增内容）、尺寸一致，`bundle.secondary` 缺失或
  `StereoGeometry::Resolve` 失败（同 6.7 节的纯平移基线假设）直接拒绝整个 bundle。
  `Health()`：`frames_processed_>0 && frames_rejected_==frames_processed_` 时报告
  `STATUS_SUSPECT`，否则 `STATUS_HEALTHY`。

**camera-optical vs. body 坐标系混淆 bug（实跑 demo 才发现，单元测试测不出来）**：
`FitRigidTransformRansac` 返回的变换是在左相机的**光学系**（`PinholeCamera::Project`/
`Unproject` 的约定）里算出来的，但 `RelativePoseMeasurement.relative_pose`（
`RelativePoseFactorBuilder`、`PoseGraphProblem` 的 keyframe 位姿、`synth_bag_gen` 的
真值生成器）在管线其余各处全部按**body 系**语义消费。`src/frontends/
stereo_landmark_vo_frontend.cpp`（约 150–171 行）用 rig 的 camera→body 外参对光学系
变换做共轭：`body_T_camera_optical * cam_from_T_cam_to * body_T_camera_optical⁻¹`
（外参在两个 keyframe 上是同一个固定标定值，所以这个共轭在数学上是精确的，不是近似）。
漏掉这一步是真实踩过的坑：本模块自己的单元测试测不出来（测试直接在"相机系"里构造
合成点，从不构造 body/optical 不一致的场景），平移量级看起来是对的（约 1m/step，
匹配真实每步位移），但几乎全部落在光学系 z 轴（前向）而不是载具实际运动的 body
x/y 平面——一个纯旋转误差，量级上不可见，一旦复合进位姿图就是灾难性的：修复前
ATE 卡在 6.67m 不收敛，修复后收敛到 0.061m（跟 `black_box_vio` 同量级）。

**已验证的运行结果**：`configs/experiment/synthetic_smoke_vo.yaml`（跟
`synthetic_smoke.yaml` 基本一样，关键差别是 `estimator_mode: stereo_landmark_vo`，
见第 11 节）本次复核实跑：`stereo_landmark_vo_frontend: computed relative-pose evidence from camera
frames`，`added 10 relative-pose factors, 11 keyframes`（比 `black_box_vio` 路径少
一个——首帧没有"上一帧"可比，`Process()` 对 kf0 恒返回 `std::nullopt`，`kf0` 因此
从未通过这条路径被 `AddKeyframe` 过第二次，其余 11 个 keyframe 各产生一条相对位姿
证据），`solver: 7 iterations, cost 65.9557 -> 9.3534 (converged)`，
`ATE: rmse=0.060835m mean=0.0543098m max=0.0925716m`。

---

## 7. runtime/ 层

这里目前提供的是 runtime 支持原语，不是已经组合好的在线调度器；`replay_demo` 仍是离线批处理。

### 7.1 状态机 `state_machines.hpp`

架构文档 12.1 节的三个正交状态机（系统跟踪状态、单模态健康状态、建图节流
状态）用同一个滞回（hysteresis）模板实现：

```cpp
enum class SystemState { kBoot, kWarmup, kInitializing, kTracking, kDegraded,
                          kLost, kRelocalizing, kRecovering };
enum class ModalityHealthState { kHealthy, kSuspect, kUnavailable, kRecovering };
enum class MappingState { kFull, kThrottled, kKeyframeOnly, kPaused };

template <typename StateEnum>
class HysteresisStateMachine {
 public:
  HysteresisStateMachine(StateEnum initial, std::chrono::milliseconds min_hold);
  StateEnum state() const;
  const std::string& reason_code() const;
  bool Request(StateEnum new_state, std::string reason_code, bool force = false);
  // 距上次接受的转换不足 min_hold 时是 no-op（返回 false），除非 force=true
  // （只留给硬故障，比如致命传感器丢失，不能等 hold timer）
};
using SystemStateMachine = HysteresisStateMachine<SystemState>;
using ModalityHealthStateMachine = HysteresisStateMachine<ModalityHealthState>;
using MappingStateMachine = HysteresisStateMachine<MappingState>;
```

`min_hold` 存在的目的是"避免权重在阈值附近震荡"（架构 12.1 节原话）。建图状态
永远不能直接决定定位状态这条规则，是靠三个类之间没有共享可变状态来保证的，不是
靠运行时检查，纯粹的类型/所有权层面的隔离。

### 7.2 四车道有界队列原语 `bounded_queue.hpp`

架构第 13 节："ROS 回调/adapter 只做有界队列入队，调度顺序由调度器决定。"

```cpp
enum class OverflowPolicy {
  kDropOldest,   // 相机：保留最新帧
  kDropNewest,   // 很少适用，留着仅为完整性
  kReject,       // IMU：绝不能被静默丢弃——调用方必须自己处理失败/背压
};

template <typename T>
class BoundedQueue {
 public:
  BoundedQueue(std::size_t capacity, OverflowPolicy policy);
  bool Push(T item);           // 只有 kReject 策略下才可能返回 false
  std::optional<T> TryPop();
  std::size_t Size() const;
  uint64_t DroppedCount() const;
};

enum class Lane {
  kLocalization,  // 最高优先级：IMU、局部估计器、StateStore、TF
  kCorrection,    // 声呐配准、图优化、重定位
  kMapping,       // 立体/声呐积分、submap、mesh
  kEvidence,      // 录制、指标、可视化、模型日志——最低优先级
};
```

`configs/defaults/platform.yaml` 的 `runtime.lanes` 就是给这四条车道各自配置
`priority`/`queue_capacity`/`overflow_policy`（见
[第 11 节](#11-配置系统-configs)）。每条车道的 overflow 策略不同，IMU 不能随便
丢，相机/建图可以丢旧/低价值项，所以策略是构造参数而不是硬编码。

> **当前实现边界**：`BoundedQueue`/`Lane` 本身只是队列原语和
> 车道枚举，没有一个把四条车道real-time 调度、优先级抢占串起来的"调度器"类。
> `src/application/replay_pipeline.cpp`/`apps/synth_bag_gen.cpp` 目前都是单线程直接遍历 MCAP
> 消息，并不实际实例化 `BoundedQueue`/驱动四车道调度。这部分是运行时原语已经
> 就位、但尚未组成在线调度器的一层。相对地，`estimator_mode` 和
> `frontends.landmark_detector` 会真实派发；sonar/optical frontend 与 `map_backend`
> 是经校验的单值字段。

### 7.3 分层配置加载 `config.hpp` / `config.cpp`

对应架构 14.2 节 `defaults → rig → scenario → experiment` 四层。用 yaml-cpp
解析成类型化 struct（`rig` 层例外，见下）：

```cpp
struct SqrtInformationDefaults { double relative_pose=20.0, sonar_range=15.0, depth=20.0; };
struct PlatformDefaultsConfig {
  std::string solver = "gauss_newton_v1";
  int max_iterations = 30;
  double initial_lambda = 1e-3;
  SqrtInformationDefaults default_sqrt_information;
  double warmup_seconds = 0.0;   // 见下方"预热窗口"
  bool require_converged = true;
  double max_ate_rmse_m = -1.0;
  int min_matched_ate_poses = 0;
  bool require_nonempty_map = false;
};
struct ScenarioConfig {
  uint64_t seed = 42; int num_keyframes = 12;
  double radius_m = 8.0, arc_radians = 1.4, depth_m = 12.0;
  ScenarioNoiseConfig noise;
  std::vector<Eigen::Vector3d> sonar_targets_world;
};
struct ExperimentConfig {
  PlatformDefaultsConfig defaults;
  uw::domain::RigCalibrationSnapshot rig;   // 直接是 protobuf 消息，不是平行 struct
  ScenarioConfig scenario;
  std::string sonar_frontend = "sonar_cfar_frontend_v1";
  std::string optical_frontend = "stereo_depth_frontend_v1";
  std::string landmark_detector = "bright_blob";
  std::string estimator_mode = "black_box_vio";
  std::string map_backend = "submap_point_cloud_v1";
  bool write_run_manifest = true;
};

PlatformDefaultsConfig LoadPlatformDefaultsConfig(const std::string& path);
uw::domain::RigCalibrationSnapshot LoadRigConfig(const std::string& path);
ScenarioConfig LoadScenarioConfig(const std::string& path);
ExperimentConfig LoadExperimentConfig(const std::string& path);
std::optional<std::string> ValidateExperimentConfigSelections(const ExperimentConfig& config);
```

`rig` 层直接解析进 `RigCalibrationSnapshot` protobuf 消息（不是另建一套
struct），`LoadRigConfig` 里逐字段 `snapshot.mutable_xxx()->set_yyy(...)`，
保证"标定长什么样"只有一处定义。

路径解析（`configs/experiment/*.yaml` 里容易出错的地方）：`LoadExperimentConfig`
的真实实现：
```cpp
ExperimentConfig LoadExperimentConfig(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  // experiment 文件位于 configs/experiment/*.yaml，其中 defaults/rig/scenario
  // 写的是 "defaults/x.yaml"/"rig/y.yaml"/"scenario/z.yaml" —— 也就是相对
  // configs/（四个层目录的共同父目录），而不是相对 configs/experiment/ 自己。
  // 从 experiment 文件往上走两级才能到 configs/。
  const std::string base_dir =
      std::filesystem::path(path).parent_path().parent_path().string();
  ...
  if (root["defaults"])
    config.defaults = LoadPlatformDefaultsConfig(ResolveRelative(base_dir, root["defaults"].as<std::string>()));
  if (root["rig"])
    config.rig = LoadRigConfig(ResolveRelative(base_dir, root["rig"].as<std::string>()));
  if (root["scenario"])
    config.scenario = LoadScenarioConfig(ResolveRelative(base_dir, root["scenario"].as<std::string>()));
  ...
}
```
`ResolveRelative(base_dir, maybe_relative)`：若目标路径本身是绝对路径就原样返回，
否则拼到 `base_dir` 后面。这正是第一次实现时漏算的一层 `parent_path()`，
CLAUDE.md/README 都提到这个问题，这里是对应的确切代码。

`replay_demo` 加载 experiment 后立即调用 `ValidateExperimentConfigSelections()`：
sonar/optical frontend 与 `map_backend` 当前各只有一个实现标识符，未知值会启动失败；
`map_backend` 是预留的地图实现选择字段，目前唯一支持的值是
`submap_point_cloud_v1`。`estimator_mode`（`black_box_vio`/`stereo_landmark_vo`）和
`landmark_detector`（`bright_blob`/`harris_corner`）会真正选择代码路径。fail-fast
只解决“静默忽略错误配置”，不表示已经有多个 frontend/map backend 可动态切换。

`warmup_seconds`（`PlatformDefaultsConfig`）：一次运行最开始 N 秒内的
keyframe 只接受相对位姿（dead-reckoning）因子，不接受声呐 range/深度这类
"绝对参考"因子（0=禁用，即不做区分）。这个设计借鉴自一个姊妹 ROS2
SVIn+HoloOcean 部署（`workfiles_02` 的 `merge_node`），VIO 的 IMU bias 还没
收敛前不能信绝对修正。本仓库没有在线 IMU 滤波器，所以类比实现是：warmup 窗口
内的 keyframe 仍然留在位姿图里（仍会通过相对位姿因子被航位推算、仍会被
求解器优化），只是跳过声呐/深度因子的构建；`kf0` 不论 `warmup_seconds` 取值
如何，始终是固定 anchor，换成 warmup 窗口之后的某个 keyframe 做 anchor
需要知道它真实的 x/y/yaw，而这在图里根本不可观测，只有 `kf0` 因为
`synth_bag_gen` 把它放在世界系原点这个构造事实才能用 `Pose3::Identity()`
（具体应用见 [9.2 节](#92-appsreplay_demo--端到端主流程)）。

### 7.4 `RunManifest`

架构 14.2 节：每次运行产出一个不可变 RunManifest；动态参数变化必须变成带时间戳
的事件，绝不能静默覆盖一份已写出的 manifest，调用方应把它当 write-once 对待。

```cpp
struct RunManifest {
  std::string run_id, git_commit, config_hash, calibration_hash, model_hash;
  std::string dataset_or_scenario, simulator, os_info, cpu_info, gpu_info;
  uint64_t seed = 0;
  std::string start_time_iso8601;
  std::string end_time_iso8601;   // 运行结束前为空
  std::string ToJson() const;     // 手写的极简 JSON 序列化，不引入 JSON 依赖
};
```
`ToJson()` 是手写字符串拼接，注释说明：假定字段值不含未转义的引号/控制字符
（对 git hash/config hash/run id 这类字段成立），为这个小而完全可控的字段集写
通用转义器是不必要的复杂度。`src/application/replay_pipeline.cpp` 里 `run_id` 具体是
`replay_demo_<unix秒>`，`simulator` 写死为
`"synthetic (apps/synth_bag_gen.cpp)"`（见 9.2 节）。
P0 后调用方会填 `UW_GIT_COMMIT`、experiment 文件的 FNV-1a hash、序列化 rig 的
FNV-1a hash、bag 路径、OS、CPU、CPU-only GPU 说明、scenario seed 和 UTC 起止时间。
`model_hash` 仍为空；hash 不是加密摘要；`simulator` 即使回放真实 HoloOcean bag 也仍
写成 synthetic，因此完整 dataset/simulator/dependency provenance 尚未闭环。

### 7.5 MCAP I/O 封装 `mcap_io.hpp`

对 MCAP C++ SDK 的一层薄薄的、感知 protobuf 的封装，让调用方
（`synth_bag_gen`/`replay_demo`）不用直接碰 `mcap::McapWriter`/`McapReader` 的
schema/channel 记账，只需按 topic 读写 typed protobuf 消息：

```cpp
std::string BuildFileDescriptorSet(const google::protobuf::Descriptor* descriptor);
// 序列化 .proto 依赖的传递闭包成 FileDescriptorSet —— 这就是让 MCAP 的
// "protobuf" 编码 schema 自描述（能在 Foxglove Studio 之类工具里直接查看），
// 而不只是一个裸类型名标签的关键。

class McapProtobufWriter {
 public:
  bool Open(const std::string& path);
  void Close();
  template <typename T>
  bool WriteMessage(const std::string& topic, uint64_t log_time_ns, const T& message);
  // 内部：EnsureChannel() 按需注册 schema/channel；sequence 按 channel 自增
};

template <typename T>
bool ReadMcapMessages(const std::string& path, const std::string& topic,
                      const std::function<void(uint64_t log_time_ns, const T&)>& callback);
// 按文件顺序把 topic 上的每条消息反序列化成 T 并回调；解析失败的消息静默跳过
// （topic 被跨 schema 版本复用时会发生，v1 不当硬错误，但读评测关键数据的
// 调用方应该自己核对消息计数）
```

Python 侧（`adapters/holoocean/uw_holoocean_adapter/canonical_writer.py`）用同样的
算法（遍历 `message.DESCRIPTOR.file.dependencies` 构建 `FileDescriptorSet`）镜像
实现了 `CanonicalMcapWriter`，并且同样强制 `CompressionType.NONE`，因为 C++
构建禁用了 zstd/lz4（`MCAP_COMPRESSION_NO_ZSTD/LZ4`），压缩过的 Python bag 在
C++ 侧读不出来。这保证了 Python 写的 bag 能被 `replay_demo` 零转换直接消费。

`cmake/UwMcap.cmake`（见 [13 节](#13-构建系统)）里唯一 `#define MCAP_IMPLEMENTATION`
的翻译单元是 `cmake/mcap_impl.cpp`，任何调用 `mcap::McapWriter`/`McapReader`
的目标必须链接 `mcap_impl`（不是仅 `mcap`）。

---

## 8. adapters/ 层

### 8.1 `adapters/holoocean` —— Python 包 `uw_holoocean_adapter`

直连 HoloOcean Python API，取代 `ocean_t` 的脚本集合。

- `coordinates.py`：`Pose` dataclass（`translation:(3,)`,
  `quaternion_xyzw:(4,)`），有 `compose`/`inverse`/`apply`/`to_matrix4`/
  `identity()`，与 C++ 侧 `Pose3` 对齐。修复了 `ocean_t/src/svin2_pipeline.py`
  里 `CoordTransformer._SE3_to_pose` 的一个真实 bug：欧拉角万向锁分支
  （`cos(pitch) <= 1e-6` 时强制把 yaw 设为 0）。`matrix_to_quaternion` 用
  Shepperd 方法，穿过万向锁数值稳定。坐标约定：HoloOcean/UE 是左手 Z-up、单位
  cm；本仓库 body/world 系是右手 Z-up，两者关系 `T_ue_to_auv = diag(1,-1,-1)`，
  和 `ocean_t` 的约定相同，只是重新实现、去掉了欧拉角分支。
- `holoocean_driver.py`：`HoloOceanSession`，`ocean_t/src/main.py` 的替代。
  惰性/带保护地导入 `holoocean` 包（缺失时抛出清晰的 `RuntimeError`，它是可选
  依赖，`pyproject.toml` 的 `holoocean` extra）。持有一个用显式 `seed` 一次性
  播种的 `numpy.random.Generator`（不做全局 `np.random.seed()` 中途重新播种，
  这是修复 `ocean_t` 一个确定性 bug 的直接产物：`svin2_pipeline.py` 每帧不带
  参数调 `np.random.seed()`，破坏了 L2 回放确定性）。`apply_randomization()`
  是 `NotImplementedError`，明确未完成，等真机 HoloOcean 环境再补。
  **更新（`b2c19e1`）**：本机（本仓库所在的开发沙箱）依然没有 HoloOcean/Unreal
  安装；但项目里另一台机器（同事 pengb 的原生 Windows，WSL2 因缺少 Vulkan
  光线追踪支持没法渲染）已经真实装了 HoloOcean 2.3.0 并跑过。那次真实运行（一部分
  通过一个一次性脚本、一部分通过 `record_session.py` 的首次真实录制尝试）发现并
  修掉了 `HoloOceanSession` 里 4 个真实 bug（`step()` 误把 `env.tick(action)` 当成
  应用动作的调用，应该是 `env.step(action)`；`__init__` 从没调用 `env.reset()`，
  HoloOcean 要求首次 tick/step 前必须 reset；`step()` 读了不存在的公开属性
  `env.ticks_per_sec`，只能读私有的 `env._ticks_per_sec`；`close()` 调用了不存在的
  `env.close()`，`HoloOceanEnvironment` 只通过上下文管理器协议
  `__exit__(None,None,None)` 暴露清理逻辑）。随后 `record_session.py` 已在该原生
  Windows 环境跑出一份约 78 MB 的真实 bag，所以“从未端到端跑过”已经不成立；但
  模块文档字符串仍保守标为 "fixed against known issues, not yet proven"，更准确的
  当前解释是“单次录制已成功，尚无可重复的真实仿真自动回归，长期可靠性未证明”。
  本仓库所在 Linux 开发机仍没有 HoloOcean 环境，无法现场重跑这一段。
- `camera_conversion.py`（`b2c19e1` 新增）：`holoocean_camera_to_image_frame()`，把
  HoloOcean `RGBCamera` 的一次读数转成规范化 `uw.domain.ImageFrame`。通道顺序
  是实测确认的，不是照抄 HoloOcean 官方文档——文档写的是 RGBA，但对着真实 HoloOcean
  2.3.0（`OpenWater-HoveringCamera` 场景，原生 Windows）抓一帧、把前三通道原样看
  和反转后看对比：不反转是一片看起来不对的琥珀/橙色，反转后是物理正确的蓝色水下
  场景（沙地、珊瑚、沉船）——所以实际运行时是 BGR(A)，转换时丢弃可能存在的 alpha
  通道，反转前三通道，输出 `IMAGE_ENCODING_RGB8`。
- `state_conversion.py`（`b2c19e1` 新增）：`pose_sensor_to_state_snapshot()`
  （`PoseSensor` 读数 → `/gt/state` 的 `StateSnapshot`，明确是仿真真值不是估计）、
  `depth_sensor_to_evidence()`（`DepthSensor` 读数 → `/evidence/depth` 的
  `MeasurementEvidence`，跟 `synth_bag_gen` 一样的 positive-down 深度约定，
  `depth_m` 由下游取负号消费，例如 `replay_demo` 的 `kf0_z = -depth_m`）。
- `record_session.py`（`b2c19e1` 新增）：`apps/synth_bag_gen.cpp` 的真实传感器
  对应物，把 `HoloOceanSession` + `camera_conversion` + `state_conversion` +
  `CanonicalMcapWriter` 接成一个能跑的录制入口（`python -m
  uw_holoocean_adapter.record_session --out bag.mcap`）。只在相机传感器实际发布的
  tick 上写一个 keyframe（相机以自己配置的 Hz 运行，比仿真 tick 率慢），非相机 tick
  照常 step 但不产出任何 bag 消息——跟 `synth_bag_gen` 把每条消息都挂到一个 keyframe
  上而不是挂到裸 tick 上是同一个约定。`record_frames()` 是可测的核心（吃手写的
  `RawSensorFrame` 序列，不需要真实 HoloOcean 安装），`record_session()` 包一层真实
  `HoloOceanSession` 供 CLI 用。按 MEMORY 记录，已经在真机上录制过一次 78MB 的真实
  bag（真实相机帧转换正常，`HarrisCornerDetector` 能在真实画面上找到足够多的真实
  角点）——这次复核未重新验证这份 bag，只核对了转换代码本身。
- `canonical_writer.py`：见 [7.5 节](#75-mcap-io-封装-mcap_iohpp)。
- `scenario_randomization.py`：类型化、可采样的 `ScenarioRandomization`
  dataclass（嵌套 `VisualDegradation`/`SonarDegradation`/
  `TimingAndCalibrationDegradation`），取代 `ocean_t` 的 `water_control_panel.py`
  GUI（原来只有 2 个滑条/4 个硬编码预设，没法程序化驱动）。预设：
  `PRESET_CLEAR`/`PRESET_TURBID`/`PRESET_DEEP`/`PRESET_CRITICAL_DEGRADED`。
  `sample_uniform_sweep(rng, low, high)` 对每个数值字段递归均匀采样，同样接受
  显式 `rng`，与 driver 一样不用全局种子。
- `time_utils.py`：唯一生产 `Stamp`/clock-domain 值的地方，修复了另一个
  `ocean_t` 审计发现（`main.py` 和 `svin2_pipeline.py` 用两种互不一致的方式算
  时间戳，没有 capture/receive 区分）。`wall_clock_seconds()` 明确只用于
  receive time，从不用于估计。

依赖（`pyproject.toml`）：`protobuf>=4.21`、`mcap>=1.0`、`numpy>=1.24`，`pytest`
是 dev extra，`holoocean` 是可选 extra（保证没装 HoloOcean 的机器也能装/测其余
部分）。本次复核实跑
`cd adapters/holoocean && pip install -e ".[dev]" && pytest tests -q`：35 个测试
全过，覆盖 `coordinates`/`canonical_writer`/`scenario_randomization`/
`camera_conversion`/`state_conversion`/`record_session`（后三个是 `b2c19e1` 新增，
`record_session` 的测试跑的是不需要真实 HoloOcean 安装的 `record_frames()`）——
`adapters/holoocean/README.md` 不再把旧的 "9/9 passing" 当作当前数字。
`HoloOceanSession` 本身仍然没有一个针对它自己（而不是它调用的
转换函数）的自动化测试，因为需要真实仿真环境；见上面 `holoocean_driver.py` 条目。

### 8.2 `adapters/ros2` —— `UW_BUILD_ROS2` 开关保护

仓库里唯一允许出现 ROS2 头的地方。两个目标：

| 目标 | 状态 |
|---|---|
| `ros2_adapters (uw::ros2_adapters)`（`ros2_svin_odometry_bridge.hpp`） | 整个类体全部注释掉，一份文档化骨架，从未真正编译过 |
| `holoocean_sonar_bridge_node`（`ros2_holoocean_sonar_bridge.hpp` + `holoocean_sonar_bridge_main.cpp`） | 真实代码，已在本机对 ROS2 Jazzy + `holoocean_interfaces` 编译链接成功、启动无报错，但从未接过真实 `holoocean_main` 数据流（需要 UE5/GPU，本机未做） |

`Ros2HoloOceanSonarBridge : public rclcpp::Node` 订阅参数 `input_topic`（默认
`/holoocean/auv0/ImagingSonar`，`SensorDataQoS`），消息类型
`holoocean_interfaces::msg::ImagingSonar`。每条消息回调里调用
`provider_.PushImagingSonar(msg.timestamp, msg.bins_range, msg.bins_azimuth,
msg.image_range, observation_id)`（合成的自增 `"holoocean_sonar_N"` observation
id），除此之外什么都不做（注释："这里没有估计/信号处理"）。`main()` 硬编码
`HoloOceanImagingSonarParams{horizontal_fov_rad=2.0943951(120°), min_range_m=0.5,
max_range_m=30.0}`（注释说这个值"匹配 holoocean_bridge 的默认场景 FOV"）。

编译门槛：`find_package(holoocean_interfaces REQUIRED)`，它不在公共 ROS2 包
索引里，必须从 `external_repos/holoocean-ros/holoocean_interfaces` colcon
build 到独立的 `~/ros2_ws`（symlink 而非拷贝），把它的 `install/` 加进
`CMAKE_PREFIX_PATH`。这个节点被明确称为"纯传输层"，下游（`SonarFrontend::
ProcessSonarFrame`）完全没有接线，`sonar_cfar_frontend` 目前只在
`replay_demo` 的 bag 回放路径里被用到，跟这条 ROS2 路径完全不相交。

### 8.3 无 ROS provider 适配器（`include/adapters`、`src/adapters`，合并进 `adapters`/`uw::adapters` target）

统一模式：一个不依赖 ROS2/ROS1 的 C++ 类实现某个 `include/measurement_api` provider
接口，带一个显式 `Push*()` 注入点（不需要任何传输层就能单测），外面再套一层
真实的（或骨架的）ROS2 订阅接线。文档见 `adapters/svin_bridge.md`、
`adapters/holoocean_ros_bridge.md`（原 `adapters/third_party/{svin_bridge,
holoocean_ros_bridge}/README.md`，这次布局重构随实现一起搬到了 `adapters/` 根，
不再各自持有独立目录）；`sonar_camera_reconstruction` baseline 是独立的执行边界，
本节末尾单独说明，已搬到 `baselines/sonar_camera_reconstruction/`。

`holoocean_ros_bridge`（真实、已测试）：
`HoloOceanRosBridgeSonarFrameProvider : SonarFrameProvider`。`PushImagingSonar()`
把一个展平的行主序 `[num_ranges,num_beams]`、值域 `[0,1]` 的 float32 强度图转成
`uw::domain::SonarFrame`，从 `HoloOceanImagingSonarParams`（fov/min/max）填充
`range_bins`/`azimuth_angles`（因为 `ImagingSonar.msg` 两者都不携带），镜像每一行
（`num_beams-1-c`）修正 HoloOcean 的列序与本平台升序方位角约定的差异
（`IsAzimuthAscending`），量化/裁剪到 `uint8`。尺寸不匹配或维度为 0 的畸形帧被
静默丢弃并计数。镜像翻转和字段映射细节（用 `image_range` 而非
`image_azimuth`；FOV/range 来自场景 JSON 而非消息本身）是读一个同事独立维护、
未 vendor 进本仓库的 `holoocean_bridge` 包（对 `sonar_oculus/OculusPing` 做同样
转换）反推出来的。

`svin_bridge`（真实、已测试注入点，里程计侧未对真实 SVIn 验证）：
`SvinBridgeLocalOdometryProvider : LocalOdometryProvider`，包装 SVIn 的
`okvis_odometry`（`nav_msgs/Odometry`）为黑盒相对位姿证据，自己估一个协方差代理
（因为 SVIn 不提供真实位姿协方差，架构 22.4 节审计发现）。只有注入点单测跑过；
ROS2 订阅侧（`ros2_svin_odometry_bridge.hpp`）完全注释，从未编译。

`baselines/sonar_camera_reconstruction/`（原 `adapters/third_party/
sonar_camera_reconstruction_baseline/`，纯 stub，诚实标注为不可用）：意图是
跑*未修改*的上游 `sonar_camera_reconstruction` ROS1 包作为外部对比 baseline，
从不链接进 `include`/`src`。两个未解决的阻塞点：① 上游依赖未 vendor 的
`bruce_slam`，原仓库本身编不过；② 上游是 ROS1 Noetic，本机既没有 ROS1 也没有
`bruce_slam`。`run_baseline.sh` 是真实脚本骨架（有 `--bag`/`--out` 参数解析），
但函数体就是三行 `echo "TODO: ..."` 加一行 `exit 1`，不做任何真实工作。

### 8.4 `adapters/datasets/`

纯 stub，只有一个 `README.md`，说明意图：把公开数据集（EuRoC 风格、SVIn 自己的
公开数据集、RUSSO/Tank/SonarSweep）转换成 `adapters/holoocean` 和
`apps/synth_bag_gen.cpp` 产出的统一 MCAP 录制格式/protobuf schema，让
`replay_demo` 不关心 bag 来源。"本轮未实现：还没有转换/测试过任何公开数据集"。

---

## 9. apps/ 与 evaluation/

### 9.1 `apps/synth_bag_gen.cpp` —— 合成数据生成

CLI：`--experiment <yaml>`（第一遍解析，通过 `ApplyScenarioConfig` 叠加
`ScenarioConfig`）→ `--out`/`--num-keyframes`/`--seed`（第二遍解析，覆盖
experiment 里的值，和 `replay_demo` 一样"CLI 参数最后生效"）。

`ScenarioOptions` 默认值：`num_keyframes=12, radius_m=8.0, arc_radians=1.4,
depth_m=12.0, relative_pose_noise_m=0.02, sonar_range_noise_m=0.03,
sonar_bearing_noise_rad=0.01, seed=42`；`sonar_targets_world` 默认空 →
`BuildSonarTargets` 退回到 3 个硬编码的类海底点。

真值轨迹（`BuildGroundTruthTrajectory`）：一段圆弧，`t∈[0,1]` 按
`num_keyframes` 插值，`theta = t*arc_radians`；位置
`= (radius·sinθ, radius·(1-cosθ), -depth_m)`（深度恒定，平面圆弧），朝向
`= AngleAxis(theta, UnitZ())`（只有 yaw，与弧线相切）。固定 5Hz 间隔：
`t_ns = i * 200_000_000`（每 keyframe 0.2s）。

噪声模型：`std::mt19937_64 rng(seed)`，三个独立的
`std::normal_distribution<double>`：`pose_noise(0, relative_pose_noise_m)`、
`range_noise(0, sonar_range_noise_m)`、`bearing_noise(0, sonar_bearing_noise_rad)`，
由 seed 完全确定，不做全局 RNG 重新播种（与 Python adapter 同样的纪律）。

相对位姿证据（`/evidence/relative_pose`，每个 `i>0`）：
`true_relative = trajectory[i-1].Inverse() * trajectory[i]`；给平移 x/y/z 各自
独立加 `N(0, relative_pose_noise_m)`（旋转不加噪，噪声只作用于平移）。

声呐（`/raw/sonar_frame`，每个在 12m 范围内的目标各产生一个合成 ping，
写的是真实像素而不是预先算好写死的证据）：算 `local = trajectory[i].Inverse().Apply(target)`，
`range = |local|`（超过 12.0 跳过），`bearing = atan2(local.y, local.x)`，
range/bearing 各加噪声后调 `BuildSyntheticSonarFrame`：固定传感器几何
`num_ranges=600, num_beams=300, min_range=0, max_range=15m, horizontal_fov=6.0 rad`
（故意偏宽/不真实，只是用来练 `sonar_cfar_frontend`，不是标定过的设备模型），
背景强度 `5`，在量化后的 `(row,col)` bin 上画一个 3 列宽的强度 `200` 光斑（够宽
让 DBSCAN 的 `min_samples=2` 能聚出簇，单像素聚不成簇）。

深度（`/evidence/depth`，每个 keyframe）：
`PressureDepthMeasurement{depth_m = -pose.z, sigma_m = 0.05}`（`sigma_m` 只是
声明值，代码没有真的按它采样噪声）。

`/scenario/sonar_targets` 一次性写成 `MapEvidence`（`POINT_CLOUD`，紧凑 float32
xyz）。只依赖 `uw::domain`/`uw::core`/`uw::runtime`，不依赖
`uw::estimation`，纯数据合成。

相机（`/raw/camera/left`、`/raw/camera/right`，`uw.domain.ImageFrame`，`MONO8`）：
只在 `--experiment` 加载的 rig 带相机时才写（`config.rig.cameras_size() > 0`），
每个 keyframe 一对，`BuildStereoPair` 用真实的每 keyframe 相机几何把
`BuildVisualLandmarks`（`b2c19e1` 新增，`rig` 加载时才调用）撒出的视觉路标画成
固定 640×480 灰度图上的亮块（背景 0，路标位置一个基于路标 id 的 hash 图案），
喂给 6.13 节 `StereoLandmarkVoFrontend`/`LandmarkBlobDetector`。**`BuildVisualLandmarks`
用的是跟 `relative_pose_noise`/`sonar_range_noise`/`sonar_bearing_noise` 同一个种子
为 42 的 `std::mt19937_64 rng`，且在它们之前被调用**——`b2c19e1` 把这个函数从"路标
散布在整条轨迹上"改成"按 keyframe 各自锚定"（为了让路标密度在窄视场时序匹配下
沿全程都够用），这改变了它消费的随机数抽取次数，从而连带偏移了后面所有噪声
采样——即使 experiment/scenario yaml 文件本身字节未改，同一个 seed 现在也会生成
一份不同的 bag（6.12 节末尾的数字更正就是这个偏移的直接后果）。没有 `--experiment`
或 rig 不带相机时，`BuildVisualLandmarks` 完全不被调用，相机相关代码路径不执行，
这是 `tests/integration/determinism_test.sh`（不传 `--experiment`）能保持逐字节
不变的前提。

### 9.2 `application/replay_pipeline` + `apps/replay_demo` —— 端到端主流程

CLI：`--bag <path>`（必填）、`--experiment <yaml>`（可选）、
`--out <prefix>`（默认 `/tmp/replay_demo`）、`--max-iterations N`
（CLI 覆盖 experiment，experiment 覆盖内建默认，三层覆盖链），以及真实数据评测用的
`--align-ate`（拟合无尺度刚体对齐；默认关闭以保持合成基准数字不变）。

`apps/replay_demo.cpp` 的 `main()` 只解析参数；实际流程由
`src/application/replay_pipeline.cpp` 的 `RunReplayPipeline()` 执行：

1. 加载配置：给了 `--experiment` 就 `LoadExperimentConfig` →
   `ValidateExperimentConfigSelections`；未知算法标识符立即退出。随后拿到
   `PlatformDefaultsConfig`（求解器 max_iterations/initial_lambda、三种因子的
   sqrt-information 常数、`warmup_seconds`、`write_run_manifest`）。
2. 建立在线声呐路标存储：实例化 `SubmapManager`，用固定 Identity pose 创建
   `"landmarks"` bucket。`replay_demo` 不读取 `/scenario/sonar_targets` 做数据关联；
   路标从实际 CFAR 检测和当前航位推算位姿在线发现。
3. 预热窗口：`warmup_keyframes = ceil(warmup_seconds / 0.2s)`
   （0.2s 是 `synth_bag_gen` 固定的 5Hz keyframe 间隔）；这些 keyframe 只获得
   相对位姿（航位推算）因子，被排除在声呐 range/深度这类"绝对参考"因子之外，
   对应"VIO bias 收敛前不融合绝对修正"这条工程经验。
4. `kf0` anchor 的 z：扫 `/evidence/depth`，取第一条
   `source_observations(0) == "kf0"` 的 `PressureDepthMeasurement`，
   `kf0_z = -depth_m`。**这就是 CLAUDE.md"已经踩过的坑"里那个 z 轴 anchor
   bug 的修复代码**，`kf0` 固定位姿的平移/旋转其余部分是
   `Pose3::Identity()`，但 z 用它自己真实的深度证据种下，而不是留在 0，因为一旦
   图里有深度因子，z 就不再是 gauge freedom。
5. `PoseGraphProblem problem`；`AddKeyframe("kf0", kf0_pose, fixed=true)`。
6. 相对位姿一遍（`b2c19e1` 起真的按 `estimator_mode` 分支，见下方"文件头注释里
   明确列出的 v1 限制"段落的更正）：
   - `estimator_mode == "black_box_vio"`（默认，或没传 `--experiment`）：读
     `/evidence/relative_pose`；若 `from` keyframe 已存在，航位推算出 `to` 的初始猜测
     `problem.GetKeyframePose(from) * measured_relative`，`AddKeyframe(to, guess)`，用
     `RelativePoseFactorBuilder::Build(...)`（`proposed_noise = relative_pose_sqrt_info`）
     构建残差块，绑定 `{from, to}`。
   - `estimator_mode == "stereo_landmark_vo"` 且 `rig.has_value()`（两个条件都要满足，
     否则回退到上面 `black_box_vio` 的分支）：改读 `/raw/camera/left,right`，按
     `capture_time` 换算出 keyframe id（复用跟下面第 14 步声光 pass 相同的
     `keyframe_id_for_time` lambda），按 `kf0..kfN` 顺序（不是 bag 流顺序，因为前端
     跨调用有状态）依次喂进 6.13 节的 `StereoLandmarkVoFrontend::Process()`，两张图先经
     `uw::domain::ConvertToMono8`（`synth_bag_gen` 写的已经是 MONO8，这里是 no-op；真实
     HoloOcean 录制是 RGB8，这里才是真正需要转换的地方）。后续 `AddKeyframe`/`Build`/
     `AddResidualBlock` 跟 `black_box_vio` 分支完全一样，只是量测结果来自前端实时计算
     而不是 bag 里预存的量测结果。`landmark_detector` 字段（`config.landmark_detector`，
     yaml 里 `frontends.landmark_detector`，默认 `bright_blob`）只在这个分支下被消费，
     选 `StereoLandmarkVoFrontendParams::detector_kind`。`camera_rectifier` 当前没有接进
     这里，真实帧仍以 distorted RGB→MONO8 结果进入 VO。
7. 声呐一遍：配置 `SonarCfarFrontend`
   （`num_training_cells=16, num_guard_cells=4, pfa=1e-2,
   detector_threshold=50`，与 `sonar_cfar_frontend_test` 的合成 fixture 参数
   一致）。读 `/raw/sonar_frame`，跳过图里不存在或在预热窗口内的 keyframe 对应
   的帧，调用 `sonar_frontend.ProcessSonarFrame(frame)`（真实跑一遍
   CFAR+极坐标+DBSCAN，不是预算好的证据）→ `HypothesisSet`，只用
   `candidates(0)`（top-1，按 `hypothesis.proto` 的 v1 规则）。数据关联：用当前
   dead-reckoned pose 把 range/bearing 检测投到世界系，调用
   `SubmapManager::QueryNearestPoint(predicted_point_W, 1.5m)`；命中则复用稳定路标，
   未命中则把该预测点作为新 `MapEvidence` 插入 `"landmarks"` bucket。随后用
   `FactorBuildContext{nearby_points_W = {landmark_W}}` 构建
   `SonarRangeFactorBuilder` 残差块。这是真实在线查询，但仍没有联合路标优化。该 pass
   还用 `steady_clock` 记录每个声呐帧（包括早退）的批处理 CPU 耗时并打印 nearest-rank
   P95；它不是 live capture-to-pose latency，也没有门限。
8. 深度一遍：读 `/evidence/depth`，跳过预热窗口，构建
   `DepthFactorBuilder`（`proposed_noise = depth_sqrt_info`）。
9. 求解：`GaussNewtonSolver::Solve(problem, {max_iterations,
   initial_lambda})`，打印迭代次数/初始与最终 cost/是否收敛。
10. 状态/地图接线：遍历 `problem.KeyframeOrder()`，逐个提交
    `StateSnapshot` 到 `StateStore`，调用
    `submap_manager.UpdateKeyframePose(kf_id, pose)`，把
    `{timestamp_s = i*0.2, pose}` 追加进 `estimated_trajectory`。
11. 若 rig 带相机，按 keyframe 同步双目/声呐，运行
    `StereoOpticalDepthFrontend → SonarCfarFrontend → AcousticOpticDepthFusionFrontend`
    并经 `AcousticOpticMapBridge` 把融合点云局部地图数据交给 `SubmapManager`。这是并行地图
    pass，不向 `PoseGraphProblem` 新增稠密深度因子。
12. 真值：读 `/gt/state`（`StateSnapshot`）进 `ground_truth_trajectory`，
    时间戳取自 `capture_timestamp`。
13. 评测：`uw::evaluation::ComputeAte(estimated, ground_truth, 0.05, align_ate)`，打印
    rmse/mean/max/匹配数；`--align-ate` 仅拟合 rotation+translation，不估 scale。
14. 输出：写 `<out_prefix>_trajectory.tum`（TUM 格式：
    `timestamp tx ty tz qx qy qz qw`），除非配置里 `write_run_manifest=false`，
    否则再写 `<out_prefix>_run_manifest.json`
    （`run_id = replay_demo_<unix秒>`，`dataset_or_scenario = bag路径`，
    `simulator = "synthetic (apps/synth_bag_gen.cpp)"`，并填 git/config/calibration/platform/
    seed/time 字段）。随后检查 `require_converged`、ATE 匹配数/RMSE、非空地图等 P0 gate；
    即使 gate 失败也先保留产物，再以退出码 2 报错。

文件头注释里明确列出的 v1 限制：没有真实的可靠性调度器（sqrt-information
常数是固定值，不是标定出来的）；路标来自在线 submap 查询但不会作为变量联合优化，
首次发现时还要用当前 pose 和零 elevation 初始化；只消费 top-1 声呐假设；分层配置
驱动求解器/噪声参数，`estimator_mode` 和 `landmark_detector` 真的驱动上面第 6 步；
sonar/optical frontend 和 map backend 仍写死为各自唯一实现，但配置校验会拒绝其他
标识符，不会“读取后照常运行”。参见 [configs/README.md](../configs/README.md)。

链接关系（见 `cmake/Libraries.cmake` 和 `cmake/Applications.cmake`）：
`replay_demo` 只链接 `uw::application`；`uw::application` 再组合 `uw::runtime`、
`uw::estimation`、`uw::evaluation`、`uw::factor_builders`、`uw::mapping` 和
`uw::frontends`，避免可执行入口直接持有整条算法依赖图。

### 9.3 `evaluation/` —— 轨迹、深度、融合与地图指标

轨迹侧只实现了 ATE，仓库里没有任何 RPE 代码（grep 过
`Rpe/RPE/relative_pose_error` 均无命中）。深度/融合指标见 6.7 节；地图侧新增
`MapMetricsResult`/`ComputeMapMetrics()`，计算双向最近邻平均距离之和（本仓库的
Chamfer 定义）、completeness、outlier ratio 和 F-score。

```cpp
struct TrajectoryPose { double timestamp_s; Pose3 pose_WB; };
struct AteResult { double rmse_m, mean_m, max_m; int num_matched_poses; };
AteResult ComputeAte(const std::vector<TrajectoryPose>& estimated,
                     const std::vector<TrajectoryPose>& ground_truth,
                     double max_time_diff_s = 0.05,
                     bool align_before_scoring = false);
```
实现：对每个估计位姿，按 `|时间戳差|` 线性扫描最近邻匹配真值（v1 对小规模合成
场景够用），超过 `max_time_diff_s` 未匹配则跳过。每次匹配的误差只是平移
欧氏距离（`(est.translation - gt.translation).norm()`），完全不计算旋转
误差。累积 `rmse_m = sqrt(Σerr²/matched)`、`mean_m`、`max_m`。

`align_before_scoring=true` 时，会先用至少 3 对匹配平移点做 Kabsch/Umeyama SVD，
拟合把估计轨迹映到真值的单一 rotation+translation，再计算上述平移误差；不估 scale，
退化或匹配不足时回退到未对齐结果。合成场景默认关闭以保持既有数字，真实 HoloOcean
回放通过 `replay_demo --align-ate` 启用。仍未实现 RPE、旋转误差和 Sim3 尺度对齐。

`ComputeMapMetrics(estimated, reference, distance_threshold_m)` 当前用暴力
`O(|estimated|*|reference|)` 最近邻，只在小点集单测中验证；空输入采用显式非 NaN
约定。它尚未接入 `replay_demo`，也没有地图 reference 数据入口或质量 gate。现有回放
可产生数百万局部地图数据点，正式接线前必须先引入 KD-tree/octree 等空间索引。

---

## 10. 端到端运行时序

这是把 [9.1](#91-appssynth_bag_gencpp--合成数据生成) 和
[9.2](#92-appsreplay_demo--端到端主流程) 串成一条时间线，是日常 CI 使用的合成闭环。
此外，真实 HoloOcean 录制 bag 已经跑通同一个离线回放入口；两者的成熟度边界见本节
末尾和第 15 节：

```
synth_bag_gen --experiment configs/experiment/synthetic_smoke.yaml --out synthetic.mcap
  │
  ├─ LoadExperimentConfig → ApplyScenarioConfig（seed/num_keyframes/radius/...）
  ├─ BuildGroundTruthTrajectory  （圆弧真值，5Hz keyframe）
  ├─ 逐 keyframe：
  │    ├─ 加噪声的 RelativePoseMeasurement → /evidence/relative_pose
  │    ├─ 对每个 12m 内的 sonar target：渲染真实声呐强度图（光斑）→ /raw/sonar_frame
  │    ├─ PressureDepthMeasurement（depth = -z）→ /evidence/depth
  │    └─ 真值 StateSnapshot → /gt/state
  └─ 一次性 MapEvidence（sonar targets 点云）→ /scenario/sonar_targets
       写入 synthetic.mcap（McapProtobufWriter，未压缩）

replay_demo --bag synthetic.mcap --experiment configs/experiment/synthetic_smoke.yaml --out demo
  │
  ├─ LoadExperimentConfig（同一份 experiment yaml，同一套 defaults/rig/scenario）
  ├─ kf0 anchor：从 /evidence/depth 找 kf0 自己的深度，种 kf0 的 z
  ├─ PoseGraphProblem：AddKeyframe("kf0", fixed=true)
  ├─ 相对位姿一遍：estimator_mode=black_box_vio（默认）时 /evidence/relative_pose →
  │                航位推算初值 → RelativePoseFactorBuilder；estimator_mode=
  │                stereo_landmark_vo 且 rig 带相机时改成 /raw/camera/left,right →
  │                StereoLandmarkVoFrontend（6.13 节，检测+匹配+RANSAC Kabsch 拟合）→
  │                同一个 RelativePoseFactorBuilder（见 configs/experiment/
  │                synthetic_smoke_vo.yaml）
  ├─ 声呐一遍：/raw/sonar_frame → SonarCfarFrontend::ProcessSonarFrame（真跑 CFAR+DBSCAN）
  │              → top-1 假设 → SubmapManager 查询/发现路标 → SonarRangeFactorBuilder
  ├─ 深度一遍：/evidence/depth → DepthFactorBuilder
  ├─ GaussNewtonSolver::Solve（LM，稠密 LDLT，≤30 次迭代）
  ├─ 逐 keyframe：StateStore::Commit + submap_manager.UpdateKeyframePose
  ├─ /gt/state → ground_truth_trajectory
  ├─ ComputeAte(estimated, ground_truth)  → rmse/mean/max
  └─ 写 demo_trajectory.tum（TUM 格式）+ demo_run_manifest.json（RunManifest）
```

当前默认 `estimator_mode=black_box_vio` 的验证流水线（seed=42）实测 6 次迭代收敛，
ATE RMSE `0.0665821 m`、mean `0.0562694 m`、max `0.109553 m`，匹配 12 个位姿。
不同 seed 的数字会波动，这些观测值不是硬编码验收阈值。早期约 3 cm 的结果依赖直接
使用真值路标；改为在线路标发现后，声呐
缺少 elevation 且路标不参与联合优化，初始化误差会分摊到 x/y 估计。`estimator_mode=
stereo_landmark_vo`（`configs/experiment/synthetic_smoke_vo.yaml`，见 6.13 节）
实测 7 次迭代收敛，`ATE rmse=0.060835m`——量级上不比 `black_box_vio` 差，尽管相对
位姿证据是真算出来的、不是从 bag 里读预先造好的证据。

真实数据路径也已实际执行：`configs/experiment/real_holoocean_vo.yaml` 回放一份原生
Windows HoloOcean 2.3.0 录制、约 78 MB、50 个 keyframe 的双目+深度+真值 bag，得到
49 条 VO 相对位姿、50 个深度 factor；对齐后 ATE RMSE `0.5596 m`。该 bag 不含
sonar/IMU/DVL，因此 sonar factor 和稠密地图输出为 0；求解器达到 30 次迭代后状态为
`stalled`。这证明真实离线数据入口和 VO 路径可运行，不等于实时闭环或生产可用。
`tests/integration/determinism_test.sh` 就是把这整条流程跑两遍、diff
`_trajectory.tum`，验证其中没有藏着全局可变随机状态（见
[第 12 节](#12-测试体系-tests)）——它不传 `--experiment`，所以只覆盖
`estimator_mode=black_box_vio` 这条默认路径，`stereo_landmark_vo` 分支目前没有
专门的确定性回归测试。

---

## 11. 配置系统 configs/

四层，每层一个 YAML 文件，`--experiment <path>` 驱动 `LoadExperimentConfig` 一次
性加载全部（详细的路径解析机制见 [7.3 节](#73-分层配置加载-confighpp--configcpp)）。

`configs/defaults/platform.yaml`（平台级默认值，不含任何具体机体/场景信息）：
```yaml
estimation:
  solver: gauss_newton_v1
  max_iterations: 30
  initial_lambda: 1.0e-3
  warmup_seconds: 0.0
reliability:
  default_sqrt_information:
    relative_pose: 20.0
    sonar_range: 15.0
    depth: 20.0
runtime:
  lanes:
    localization: { priority: highest, queue_capacity: 64, overflow_policy: reject }
    correction:   { priority: high,    queue_capacity: 32, overflow_policy: drop_oldest }
    mapping:      { priority: medium,  queue_capacity: 16, overflow_policy: drop_oldest }
    evidence:     { priority: low,     queue_capacity: 256, overflow_policy: drop_oldest }
gates:
  require_converged: true
  max_ate_rmse_m: -1.0
  min_matched_ate_poses: 0
  require_nonempty_map: false
```
（`runtime.lanes` 描述的是 [7.2 节](#72-四车道有界队列原语-bounded_queuehpp)
`BoundedQueue`/`Lane` 该怎么配置；目前没有任何 app 真正读取这一段去实例化队列，
文件里写着，代码里还没接线消费。）

`configs/rig/example_auv.yaml`（标定唯一事实源，对应 `RigCalibrationSnapshot`；两个 app
都消费相机/声呐外参、内参、时间偏移等与当前路径有关的字段，IMU 噪声等仍未接线）：
```yaml
calibration_version: "example_auv_v2"
frame_tree:
  - parent_frame: base_link
    child_frame: sonar_link
    transform_row_major: [1,0,0,0.1, 0,1,0,0, 0,0,1,-0.05, 0,0,0,1]
imu_noise:
  sigma_gyro_c: 1.6968e-4
  ...
sonar_beam_models:
  - sensor_id: sonar0
    horizontal_fov_rad: 2.09
    elevation_aperture_rad: 0.19
depth_models:
  - sensor_id: depth0
    noise_sigma_m: 0.05
```

`configs/scenario/synthetic_smoke.yaml`（"跑什么数据"，完整接入
`synth_bag_gen`）：
```yaml
seed: 42
num_keyframes: 12
radius_m: 8.0
arc_radians: 1.4
depth_m: 12.0
noise:
  relative_pose_noise_m: 0.02
  sonar_range_noise_m: 0.03
  sonar_bearing_noise_rad: 0.01
sonar_targets_world:
  - [2.0, 3.0, -13.0]
  - [6.0, 6.0, -11.5]
  - [-1.0, 8.0, -12.5]
```

`configs/experiment/synthetic_smoke.yaml`（"怎么跑"；`rig`/`scenario`/
`defaults` 三个 key 相对 `configs/` 而非本文件目录，见 7.3 节）：
```yaml
rig: rig/example_auv.yaml
scenario: scenario/synthetic_smoke.yaml
defaults: defaults/platform.yaml
frontends:
  sonar: sonar_cfar_frontend_v1
factor_builders:
  - relative_pose_v1
  - sonar_range_v1
  - depth_v1
estimator_mode: black_box_vio
map_backend: submap_point_cloud_v1
output:
  trajectory_format: tum
  write_run_manifest: true
```

`configs/experiment/synthetic_smoke_vo.yaml`（`b2c19e1` 新增，见 6.13 节）：跟上面
`synthetic_smoke.yaml` 基本一样（`rig`/`scenario`/`defaults`/`factor_builders`/
`map_backend`/`output` 全同），差别只有两处：多了一行 `frontends.optical:
stereo_depth_frontend_v1`（目前是唯一被校验接受的实现，rig 是否带相机决定声光 pass），
以及把 `estimator_mode` 换成 `stereo_landmark_vo`，用来触发回放管线里
真正会分支的那条路径。

`configs/experiment/real_holoocean_vo.yaml` 选择
`rig/example_auv_real_camera.yaml`、`harris_corner`、`stereo_landmark_vo`，用于已有真实
双目 bag 的离线回放。该样本产出 49 条相对位姿和 50 条深度因子；`--align-ate` 后
RMSE `0.5596 m`，求解器仍 `stalled`，所以它是链路证据而不是通过的生产 benchmark。

四层消费程度总结（`configs/README.md` 已有，这里复述一遍方便对照代码）：`defaults`
完整接入 `replay_demo`；`rig` 在两个 app 里已经消费——加载了带相机的 rig 时驱动
6.12 节的声光融合分支（以及 6.13 节 `stereo_landmark_vo` 分支，如果 `estimator_mode`
也选了它），没有相机（或没传 `--experiment`）时两个 app 行为逐字节不变；`scenario`
完整接入 `synth_bag_gen`；`experiment` 的每个算法选择都先经
`ValidateExperimentConfigSelections`。`frontends.sonar`/`frontends.optical`/
`map_backend` 当前仍各只有一个实现，rig 是否带相机决定声光流水线是否运行；未知名称
会启动失败而不是静默忽略。回放管线会按 `estimator_mode` 分支（
`stereo_landmark_vo` 需要
`rig` 带相机，用 6.13 节的 `StereoLandmarkVoFrontend` 从相机帧实时算相对位姿，
见 `configs/experiment/synthetic_smoke_vo.yaml`）；同一分支下
`frontends.landmark_detector`（`bright_blob`/`harris_corner`）也真的会被消费，选
`StereoLandmarkVoFrontend` 内部用哪个 landmark 检测器，但只在 `estimator_mode ==
stereo_landmark_vo` 时才读取。

---

## 12. 测试体系 tests/

三层，对应架构文档验收面设计，现在全部集中到顶层 `tests/`，按架构层组织成
独立的 GTest executable（`cmake/Tests.cmake` 用 `gtest_discover_tests()` 展开成
单个 case，CTest 名字前缀 `unit.<layer>.`/`contract.`/`integration.`）：

### 消息格式与接口一致性测试 —— `tests/contracts/domain_contract_test.cpp` 与 `tests/contracts/measurement_api_contract_test.cpp`

`cmake/Tests.cmake` 将这两个源文件编译为同一个 `contract_tests` target，并链接
`uw::domain`、`uw::core` 和 gtest。`domain_contract_test.cpp` 覆盖 Protobuf round-trip
与消息校验；`measurement_api_contract_test.cpp` 覆盖 `CameraFrameProvider` 和
`OpticalDepthFrontend` 的接口边界：
- `ObservationHeaderRoundTrips`：序列化/解析一个 `ObservationHeader`，字段相等
  性在 round-trip 后保持。
- `SonarFrameAscendingAzimuthAccepted`/`...Rejected`：分别用升序/非升序方位角
  数组测 `IsAzimuthAscending`。
- `MeasurementEvidencePayloadRoundTripsThroughOneof`：构造一个
  `SonarRangeBearing`，经 `MakeEvidence<>()` 包进 `MeasurementEvidence`，
  round-trip 后 `HasPayload<SonarRangeBearing>` 为真、
  `HasPayload<PressureDepthMeasurement>` 为假。
- `measurement_api_contract_test.cpp`：用 fake `CameraFrameProvider` 和
  `OpticalDepthFrontend` 验证轮询、单目/双目输入边界及其产生的规范化量测结果。
  `measurement_api` 还定义 `FactorBuilder` 与 `ResidualBlock` 等进程内接口，但本测试
  不宣称覆盖它们的行为。

### 集成/回放测试 —— `tests/integration/determinism_test.sh`

对真实二进制（不是 mock）跑两遍：
```bash
"$SYNTH_BAG_GEN" --out "$WORKDIR/scenario.mcap" --seed 7
"$REPLAY_DEMO" --bag "$WORKDIR/scenario.mcap" --out "$WORKDIR/run1" >/dev/null
"$REPLAY_DEMO" --bag "$WORKDIR/scenario.mcap" --out "$WORKDIR/run2" >/dev/null
diff -q "$WORKDIR/run1_trajectory.tum" "$WORKDIR/run2_trajectory.tum"
```
"逐字节一致"的意思是：同一个 bag/config/seed，跑两次 `replay_demo`，两份
`_trajectory.tum` 必须 diff 干净。这是"没有藏着全局可变随机状态"的直接验证，
求解器、RNG 使用、任何 map/hash 迭代顺序里的不确定性一旦泄漏进轨迹输出，这个
测试就会挂，这也是 CLAUDE.md 强调"不要用全局 `np.random.seed()`"这条纪律的实际
把关机制。

### 按层单元测试（`tests/{core,frontends,factor_builders,estimation,mapping,runtime,evaluation,adapters}/`）

已在第 5/6/7 节列出，汇总一份速查表；CTest 用例名带 `unit.<layer>.` 前缀，例如
`unit.factor_builders.SonarRangeResidual...`：

| 层 | 测试焦点 |
|---|---|
| `factor_builders`（sonar_range） | 有限差分验证解析雅可比；朝向列精确为 0 |
| `factor_builders`（relative_pose） | 残差在真值处为零；有限差分雅可比非零 |
| `factor_builders`（depth） | 残差公式与雅可比模式 |
| `estimation` | 三 keyframe 链收敛到真值（用真实 factor_builders，不是 test double） |
| `frontends`（sonar_cfar_frontend） | 合成声呐图上的 CFAR+DBSCAN（构造数据，非 fixture 文件） |
| `frontends`（harris_corner_detector/landmark_blob_detector/patch_matcher/rigid_transform_fit/stereo_landmark_vo_frontend，6.13 节） | 手造图像/点集：检测器的 NMS/阈值行为、`PatchMatcher` 的确定性贪心匹配、`FitRigidTransformRansac` 对已知刚体变换+离群点的恢复、`StereoLandmarkVoFrontend` 端到端的证据产出（不包含真实相机图像） |
| `mapping`（submap_manager） | `TRANSFORM_ONLY` vs `FULL_REFUSE` 的 stale 行为 |
| `core`（camera_rectifier） | plumb-bob 畸变/去畸变、边界采样、MONO8/RGB8/BGR8 与 0/4/5 系数校验 |
| `runtime`（config） | 真实 experiment 逐字段断言；支持项通过、未知 frontend/backend/estimator/detector fail-fast |
| `runtime`（mcap_io） | protobuf 消息经 MCAP 写入/读回的 round-trip |
| `evaluation` | ATE 零误差/已知偏移/可选对齐；深度和融合指标；点云地图完美重叠、非对称、无重叠和空输入约定 |

```bash
ctest --test-dir build --output-on-failure   # C++/脚本：当前 136 个（实跑为准）
(cd adapters/holoocean && pytest -q)          # Python：当前 35 个
tools/lint/check_no_ros_in_core.sh            # 依赖不变量（兼容入口）
```

---

## 13. 构建系统

顶层 `CMakeLists.txt`：C++17，默认 `RelWithDebInfo`，
`CMAKE_EXPORT_COMPILE_COMMANDS ON`，构建产物统一到 `build/bin/`（可执行文件）和
`build/lib/`（库）。构建开关包括 `UW_BUILD_ROS2`（默认 OFF，需要 sourced ROS2
环境）、`UW_BUILD_TESTS`（默认 ON）、`UW_SANITIZER`（`OFF`/`address`/`thread`）和
`UW_COVERAGE`（默认 OFF，gcc/gcov `--coverage`）。根文件本身只做项目级设置和四个集中式
`include()`，不直接声明任何 target：

```cmake
include(cmake/Dependencies.cmake)   # 选项 + Eigen/Protobuf/MCAP/yaml-cpp/GTest/ROS2 依赖发现
include(cmake/Libraries.cmake)      # 全部生产 library + uw::alias + link graph
include(cmake/Applications.cmake)   # 全部 executable target
if(UW_BUILD_TESTS)
  include(cmake/Tests.cmake)        # 全部测试 executable + CTest discovery + labels
endif()
```

本地 C++ 源码目录（`include/`、`src/`、`apps/`、`tests/`）下不再各自持有
`CMakeLists.txt`；源文件列表在 `cmake/Libraries.cmake`/`cmake/Applications.cmake`/
`cmake/Tests.cmake` 里显式写出（不用递归 glob），target 名不带 `uw_` 前缀，业务
代码之间只通过 `uw::<name>` alias 互相 `target_link_libraries()`。生产依赖图：

```text
domain_proto
     ↑
uw::domain
     ↑
uw::core（sensor_models + measurement_api）
     ↑
├── uw::frontends       （合并全部前端实现）
├── uw::factor_builders  （合并三种残差/因子构建）
├── uw::estimation
├── uw::mapping          （合并 submap_manager + acoustic_optic_map_bridge）
├── uw::runtime
├── uw::evaluation
└── uw::adapters         （合并 svin_bridge + holoocean_ros_bridge 两个无 ROS provider）
     ↑
uw::application         （组合算法、runtime 与 evaluation）
     ↑
apps                    （参数解析与进程入口）

uw::adapters
     ↑
uw::ros2_adapters       # 仅 UW_BUILD_ROS2=ON，物理隔离在 adapters/ros2/
```

`cmake/UwProtobuf.cmake`：`find_package(Protobuf REQUIRED)` +
`find_package(absl CONFIG REQUIRED)`。glob（`CONFIGURE_DEPENDS`，新增 .proto
自动生效）`schemas/proto/uw/domain/*.proto`，`protobuf_generate(LANGUAGE cpp ...
PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}/generated)`，构建 `domain_proto`
STATIC 库（迁移前叫 `uw_domain_proto`，现在去掉了前缀）。显式链接一整组 absl
组件（`flat_hash_map/hash/strings/status/statusor/synchronization/time/base/log/cord`，以及
coverage 链接顺序暴露出的 `log_internal_check_op`），
绕开 `protobuf::libprotobuf` 的 `INTERFACE_LINK_LIBRARIES` 在
conda-forge 工具链多层静态库传递时不可靠的问题（"DSO missing from command
line"，CLAUDE.md 记录的那个已知问题）。生成代码的警告靠 `COMPILE_OPTIONS "-w"` 抑制。

`cmake/UwMcap.cmake`：MCAP C++ SDK 是 header-only、单 TU 实现模式
（`#ifdef MCAP_IMPLEMENTATION`），没有自己的 CMakeLists.txt（上游假定用
Bazel/Conan/vendoring），所以用 `FetchContent_Declare` + `FetchContent_Populate`
（不是 `FetchContent_MakeAvailable`，因为没有子目录可 `add_subdirectory`），配
`cmake_policy(SET CMP0169 OLD)` 保住这种经典用法在新版 CMake 上还能跑。手动搭
两个目标：`mcap`（INTERFACE，含 `MCAP_COMPRESSION_NO_ZSTD`/`_NO_LZ4` 编译宏，
v1 写不压缩的 chunk，避免引入 zstd/lz4）和 `mcap_impl`（STATIC，编译
`cmake/mcap_impl.cpp`，唯一 `#define MCAP_IMPLEMENTATION` 的翻译单元，注释警告
绝不能在别的文件重复定义）。

---

## 14. 工具链 tools/

`tools/lint/check_no_ros_in_core.sh`：现在只是一个兼容入口，实际转发给
`tools/lint/check_layer_dependencies.py`（`exec python3 tools/lint/
check_layer_dependencies.py "$ROOT"`）。真正的检查逻辑解析 `include/`、`src/`、
`adapters/ros2/`、`apps/` 下的 `.hpp/.cpp/.h/.cc`，按物理目录（`include/<role>/`、
`src/<role>/`）判断每个文件属于哪个架构层，再校验：
- 每层只允许 include 自己 + 依赖图里在它下游的层（domain → core →
  frontends/factor_builders/estimation/mapping/runtime/evaluation/adapters →
  application → apps，规则见 `tools/lint/check_layer_dependencies.py` 里的 `ALLOWED` 表）；
- 旧的手写 `uw/...` include 路径一律判定失败（迁移后应统一用去掉 `uw/` 层的路径，
  例如 `"sensor_models/geometry.hpp"` 而不是 `"uw/sensor_models/geometry.hpp"`；
  生成的 `uw/domain/*.pb.h` protobuf 头是唯一例外）；
- ROS/HoloOcean/vendor 头（`rclcpp/`、`ros/`、`rmw/`、`nav_msgs/`、`sensor_msgs/`、
  `geometry_msgs/`、`holoocean_interfaces/`、`okvis/`、`sonar_oculus`）只允许出现
  在 `adapters/ros2/` 下。

单元测试在 `tests/lint/check_layer_dependencies_test.py`，CTest 里注册为
`lint.layer_dependency_unit`/`lint.layer_dependencies` 两条（label `lint`）。

`tools/codegen/gen_py.sh`：唯一的 codegen 脚本，`protoc -I ...
--python_out=...` 把 `schemas/proto/uw/domain/*.proto` 重新生成到
`adapters/holoocean/uw_holoocean_adapter/schema_pb2/`。生成的 `*_pb2.py` 是
gitignored 的，这是开发环境搭建的便捷步骤，不是检查进版本控制的产物，保持
`.proto` 单一事实源。`protoc` 不在 PATH 上时快速失败并给出清晰提示。

`tools/setup_dev_env.sh`：两段回退。① `try_apt()`：
`timeout 60 sudo apt-get update -qq && timeout 300 sudo apt-get install -y -qq
protobuf-compiler libprotobuf-dev libeigen3-dev libgtest-dev cmake
build-essential`，60s 超时专门用来探测卡住/被限速的镜像（对应 CLAUDE.md 记录的
沙箱环境 HTTP(80) apt 镜像卡死、HTTPS(443) 正常的问题）。② 只有 apt 失败才走
`try_conda()`：建/复用一个 `uw_slam_build` conda-forge 环境
（`eigen libprotobuf protobuf gtest cmake`），打印出确切的
`PATH`/`cmake -DCMAKE_PREFIX_PATH` 调用方式，这正是本仓库在沙箱开发环境里实际
被编译/测试所走的那条路径。

`tools/run_quality_checks.sh`：把额外质量检查放在独立 build 目录，支持
`sanitizer`（ASan+UBSan）、`coverage`（gcov 源码行摘要）、`static-analysis`
（cppcheck，报告但不设失败阈值）和 `all`。`.github/workflows/ci.yml` 已增加并行的
sanitizer 与 quality job。`UW_SANITIZER=thread` 仍可手动选择，但当前 conda-forge
protobuf/gtest 是未插桩动态库，会产生已知假阳性，且沙箱还需关闭 ASLR，所以 CI
刻意不跑 TSan；coverage/static-analysis 目前也只报告，不设覆盖率/告警门槛。

---

## 15. 已知边界

（与 README.md「已知边界」一致，这里从代码事实的角度复述一遍）

- `adapters/ros2/holoocean_sonar_bridge_node` 真实编译+启动验证过，但没有接过
  真实 `holoocean_main` 进程（需要 UE5+Epic EULA），也没有接到
  `SonarFrontend`/`replay_demo` 下游，它是纯传输层。`ros2_adapters (uw::ros2_adapters)` 是全
  注释骨架，从未编译。
- `--experiment` 完整接入 `defaults`/`scenario`；`rig` 在两个 app 里被消费，加载带
  相机的 rig 会打开 6.12 节的声光融合 pass，并可支持 6.13 节的
  `stereo_landmark_vo` 分支。`ValidateExperimentConfigSelections()` 会在回放开始前
  拒绝未知 sonar/optical frontend、map backend、estimator 或 landmark detector；
  但 sonar/optical/map 当前分别只有一个可用实现，验证通过不代表这些字段已经拥有
  多实现动态派发。`estimator_mode` 和 `frontends.landmark_detector` 会真实驱动分支：选
  `stereo_landmark_vo` 且 rig 带相机时把相对位姿量测结果来源从 bag 里的
  `/evidence/relative_pose` 换成 `StereoLandmarkVoFrontend` 实时计算的结果；
  `frontends.landmark_detector` 只在这个分支下被消费，选择具体路标检测器。rig 中
  `imu_noise`、`sonar_beam_models` 的部分细节仍未被这两个 app 消费。
- `StereoLandmarkVoFrontend`（6.13 节）验证方式跟仓库其余部分不太一样：单元测试
  用手造的合成点/图像，`configs/experiment/synthetic_smoke_vo.yaml` 在
  `synth_bag_gen` 的合成场景上实测收敛（ATE 0.061m），但 `determinism_test.sh` 不传
  `--experiment`，所以这条分支没有专门的双跑 diff 确定性回归测试。真实 HoloOcean
  bag 已跑过完整 `replay_demo`：50 帧均进入输入，产出 49 条相对位姿，对齐 ATE RMSE
  `0.5596 m`，但求解器在 30 次迭代处 `stalled`，且该录制没有 sonar/IMU/DVL；因此
  应表述为“真实离线路径已跑通、质量与多传感器闭环仍未达标”。
- `CameraRectifier` 已有 plumb-bob same-K 去畸变原语和单元测试，但尚未接入
  `replay_demo`。在上述真实 bag 上离线试验时，几何校正会使当前 VO 默认参数的跟踪从
  50/50 降到 8/50，说明 warp 本身可用而前端需要随校正后的影像重新调参；它也不是
  支持任意离轴双目 rig 的通用 rectifier。
- `adapters/holoocean` 的 `HoloOceanSession`（`holoocean_driver.py`）本身仍然没有
  被完整驱动过一次并证明可靠——`b2c19e1` 修的 4 个 bug 是真实运行中发现的，但模块
  自己的文档字符串明确写"fixed against known issues, not yet proven"；本仓库
  开发沙箱机器没有 HoloOcean/Unreal 安装，这条路径只在同事的原生 Windows 机器上
  跑过。
- `runtime/bounded_queue.hpp`/`state_machines.hpp` 的原语已经实现，但没有任何
  app 真正实例化四车道队列或驱动状态机转换，这层运行时基础设施是"已搭、未接线"。
- 求解器是 Eigen 手写 LM，不是 Ceres/GTSAM（架构第 20 节延后决策），且直接在原始
  7 参数块上做加法更新+事后重归一化，不是严格的流形更新。
- 位姿图只优化 keyframe 变量，不联合优化路标点；`nearby_points_W` 由
  `SubmapManager::QueryNearestPoint()` 在线查询或发现。新路标用当前 dead-reckoned pose
  和零 elevation 初始化，后续只复用固定位置，不会被图优化精化。
- `include/mapping/submap_manager.hpp` 尽管叫"submap"，实现粒度是按 keyframe，
  没有距离/重叠/帧数触发的 submap 边界逻辑。
- 轨迹评测只有 ATE（平移 RMSE/mean/max），没有 RPE 或旋转误差。它可选做
  Kabsch/Umeyama 的 SE3 rotation+translation 对齐（不估 scale）；没有 Sim3 尺度对齐，
  少于 3 对或退化匹配会回退到未对齐结果。点云地图指标虽已有 API/单测，但采用暴力
  最近邻，尚未接回放、reference 数据或门禁，不能当作地图评测闭环已经完成。
- `sonar_camera_reconstruction_baseline` 是纯 stub（`run_baseline.sh` 函数体是
  TODO+`exit 1`），因为上游依赖未 vendor 的 `bruce_slam` 且需要 ROS1 Noetic。
