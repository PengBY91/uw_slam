# uw_slam 两条主线代码走读

> 按当前代码事实走读两条端到端主线的代码块与主要逻辑，供"想真正读懂这条链"的人使用。
> 核对于 commit `80d4464` + 当前工作树，2026-08-26。文中 `file:line` 行号以该版本为准，后续会漂移——以函数名/文件为准，行号只作辅助定位。

## 文档定位

- 根 [README](../README.md) 回答"项目是什么、怎么跑起来"；本文回答"这条链在代码里到底怎么走"。
- 与[新人上手指南](./uw-slam-newcomer-guide.md)（按调用链的入门串讲，核对至 2026-08-22）互补：本文覆盖其后新增的实时闭环/在线辅助能力，且对每个阶段给出更细的机制说明（为什么这么设计、踩过什么坑）。
- 与[代码库参考](./uw-slam-codebase-reference-2026-08-18.md)（按类型/模块的事实清单）互补：本文按**执行顺序**组织，参考文档按**模块**组织。

## 目录

- [0. 两条主线共享的地基](#0-两条主线共享的地基)
- [主线一：离线 SLAM 管线](#主线一离线-slam-管线)
- [主线二：ROV 在线驾驶辅助（实时闭环）](#主线二rov-在线驾驶辅助实时闭环)
- [两条主线的关系](#两条主线的关系)

---

## 0. 两条主线共享的地基

两条主线不是两套代码，而是**同一套消息模型和同一个事件入口**的两种事件来源（回放 vs 实时）。

### 0.1 规范消息模型（Protobuf）

`schemas/proto/uw/domain/*.proto` 是 C++/Python 跨语言消息的唯一事实源：

| 文件 | 内容 |
|---|---|
| `observation.proto`/`image.proto`/`sonar.proto` | 观测头（`ObservationHeader`：sensor_id / sensor_frame / observation_id / capture_time / calibration_version）、`ImageFrame`、`SonarFrame` |
| `measurement.proto` | `MeasurementEvidence` + 各 payload（`RelativePoseMeasurement`、`PressureDepthMeasurement`、`OpticalDepthPriorMeasurement`、`FusedDepthMeasurement`、`SonarRangeBearing`…） |
| `factor.proto`/`hypothesis.proto` | `FactorCandidate`（含 `robust_policy_hint`）、`HypothesisSet`（前端输出） |
| `state.proto` | `StateSnapshot`（估计状态）、`VehicleState`（车辆状态输入） |
| `map.proto` | `MapEvidence`（局部点云/surfel 载荷，`representation_type` + `reintegration_policy`） |
| `health.proto` | `HealthReport`（HEALTHY/SUSPECT/UNAVAILABLE/RECOVERING + reason_code） |
| `target.proto` | `TargetDetection` / `TargetTrack` / `TargetTrackSet` / `OperatorAssistState`（主线二输出契约） |
| `calibration.proto` | `RigCalibrationSnapshot`（rig 配置层的解析目标） |
| `ids.proto`/`time.proto`/`vehicle.proto`/`imu.proto`/`dvl.proto` | 标识、时间戳、`ImuSample`、`DvlSample` |

C++ 侧统一经 `include/domain/domain.hpp` 使用；Python 侧由 `tools/codegen/gen_py.sh` 生成 `schema_pb2`（不入库）。

两条铁律写在字段层面：位姿一律 `Pose3`（平移 + xyzw 四元数，禁欧拉角）；`PressureDepthMeasurement.depth_m` 正向下（world Z-up，位姿 z = `-depth_m`），光学 `depth_m` 是 optical frame 正向前——同名不同义，不可混用。

### 0.2 规范 topic 词表与统一事件契约

`include/runtime/canonical_topics.hpp` 定义全部规范 topic 及其消息类型与角色：

| Topic | 消息 | 角色 |
|---|---|---|
| `/raw/camera/left`、`/raw/camera/right` | `ImageFrame` | 算法输入 |
| `/raw/sonar_frame` | `SonarFrame` | 算法输入 |
| `/raw/imu`、`/raw/dvl`、`/raw/vehicle_state` | `ImuSample` / `DvlSample` / `VehicleState` | 算法输入 |
| `/evidence/depth`、`/evidence/relative_pose` | `MeasurementEvidence` | 算法输入 |
| `/health` | `HealthReport` | 算法输入 |
| `/evidence/map` | `MapEvidence` | 算法输入 |
| `/gt/state` | `StateSnapshot` | **仅评测支路**（`CanonicalTopicRole::kReferenceOnly`） |

`runtime/canonical_event.hpp` 把"topic + payload 变体 + capture/receive 时戳 + 序号"捆成 `CanonicalEvent`；`runtime/event_source.hpp` 定义与来源无关的 `EventSource` 契约；`application/event_pump.hpp` 的 `PumpEvents(source, port)` 从任一 EventSource 取事件、按 topic 分发到 `application/pipeline_input_port.hpp` 的 `PipelineInputPort::OnXxx(...)`。

一致性由 `tests/integration/event_source_parity_test.cpp` 把关：同一批事件经 MCAP 与内存两种 EventSource 注入，应用侧观察到的顺序必须完全一致。

### 0.3 分层依赖（lint 强制）

```
domain → core → {frontends, factor_builders, estimation, mapping,
                 runtime, evaluation, adapters, opencv_adapters} → application → apps
```

- ROS2 只在 `adapters/ros2/`（lint 角色 `ros2`）；OpenCV 只在 `adapters/opencv/`（角色 `opencv_adapters`，源码住在 `adapters/opencv/{include,src}`）；Ceres 只在 `adapters/ceres/`；nanoflann 只在 `adapters/spatial_index/`。
- `estimation`/`mapping` 层见到的求解器/空间索引是纯虚接口（`Solver`、`SurfelSpatialIndex`），第三方实现在 adapters 层、由 `application` 注入。
- 检查入口：`tools/lint/check_no_ros_in_core.sh`（实际实现 `tools/lint/check_layer_dependencies.py`）。

### 0.4 算法接口抽象

`include/measurement_api/`：`frontend.hpp`（声呐/光学/立体前端接口）、`factor_builder.hpp`（`FactorCandidate` → `ResidualBlock`）、`residual_block.hpp`、`target_frontend.hpp`（`VisualAssistFrontend`：左图 + 可选深度先验 + 内参 → 目标 + 路径偏移 + 健康）。

---

## 主线一：离线 SLAM 管线

### 数据流总览

```
数据生产                     统一入口                  编排（RunReplayPipeline）                       输出
synth_bag_gen ──MCAP──┐                          ┌→ ⑤相对位姿(VO/桩) → ⑥回环 ┐
adapters/datasets ────┼→ McapEventSource ──────→ │ ⑦声呐+数据关联 → ⑧深度    ├→ ⑨求解 → ⑩状态/地图
adapters/holoocean ───┘   PumpEvents              └→ ⑪声光融合(并行,不入图)  ┘        ↓
                          ReplayInputAccumulator                                     ⑫ATE → ⑬TUM+Manifest+gate
```

### 1. 数据生产端

**`apps/synth_bag_gen.cpp`**（约 530 行）——按 scenario YAML（`configs/scenario/*.yaml`：轨迹弧段半径/弧度、深度、各噪声 σ、seed）生成规范化 MCAP。核心字段：`num_keyframes=12`、`radius_m=8`、`arc_radians=1.4`、`depth_m=12`、`seed=42`（`synthetic_smoke` 默认）。

写入 topic（见文件头注释）：

| Topic | 内容 | 要点 |
|---|---|---|
| `/gt/state` | 每 keyframe 真值 | 仅供 ATE/评测支路 |
| `/evidence/relative_pose` | ground-truth+noise 的 `RelativePoseMeasurement` | 这就是 `black_box_vio` 桩的实体 |
| `/raw/sonar_frame` | `runtime/synthetic_sonar.hpp` + `sensor_models/sonar_beam_model.hpp` 渲染的成像声呐帧 | **不是**预算好的检测——真 CFAR 前端在回放侧跑 |
| `/evidence/depth` | `PressureDepthMeasurement` | 正向下 |
| `/scenario/sonar_targets` | `MapEvidence`（世界系目标点） | 供审计/复现 |
| `/raw/camera/left,right` | `BuildStereoPair` 合成双目对 | 仅 rig 带相机时写；landmark 是**按 id 唯一的哈希图案**（给 bright_blob 检测器调的），每 keyframe 复用同一世界系 landmark 云 |

真实数据的等价入口（同格式 bag，回放侧不区分来源）：

- `adapters/holoocean/uw_holoocean_adapter/record_session.py`：HoloOcean 录制。相机 keyframe 只在双目同 tick 出现时成立；声呐/IMU/DVL 按各自频率独立写，`observation_id` 键在各自的 tick 序号上（不是相机 keyframe 计数）。
- `adapters/datasets/uw_dataset_adapter/euroc_converter.py`：EuRoC MH_01（ROS1 bag → 规范 MCAP）。自研容错 rosbag1 读取器 + 逐帧去畸变（原始畸变帧在这种重复结构场景产不出相对位姿因子，docstring 记录了诊断）；GT/IMU 刻意不转换。

### 2. 入口

`apps/replay_demo.cpp`（45 行）：只做参数解析（`--bag`/`--experiment`/`--out`/`--max-iterations`/`--align-ate` 等），调 `uw::application::RunReplayPipeline`。所有逻辑都在 application 层——这是"apps 只做入口"规则的样板。

### 3. 编排核心：`RunReplayPipeline`（`src/application/replay_pipeline.cpp:258`）

#### ① 配置加载与校验（`:292-309`）

`LoadExperimentConfig`（`src/runtime/config.cpp`）完成 `defaults→rig→scenario→experiment` 四层合并（路径相对 `configs/` 解析）；`ValidateExperimentConfigSelections` 对未知选择器 fail-fast。取出：`defaults`（求解/前端/回环/在线辅助/车道参数）、rig（**仅当带相机**才进入 `std::optional`）、`estimator_mode`、`landmark_detector`、scenario seed、config/calibration hash。CLI 覆盖最后生效（`:309`）。

真正驱动分支的三个选择器：`estimator_mode`（`black_box_vio` 默认 / `stereo_landmark_vo`）、`frontends.landmark_detector`（`bright_blob` 默认 / `harris_corner`）、`estimation.solver`（`gauss_newton_v1` 默认 / `ceres_v1`，后者需 `-DUW_BUILD_CERES_SOLVER=ON` 编译，选了没编译直接退出）。

#### ② 统一事件扫描（`:311-333`）

一次 `McapEventSource` 顺序扫全 bag（按 logTime）→ `PumpEvents` → `ReplayInputAccumulator`（`include/application/replay_input_accumulator.hpp`，`PipelineInputPort` 的回放实现）。要点：

- 未知 topic / schema 不匹配 / payload 解析失败 → 计入 `EventSourceReport`，打 warning，不静默丢；
- 身份校验：空 `observation_id`、`(sensor_id, observation_id)` 重复、evidence 引用不存在的 source observation → `ReplayInputDiagnostics` 报错、整场非零退出；
- 产出扁平 `ReplayInputData`：`images` / `sonar_frames` / `evidence` / `reference_states`（+每条 evidence 的 log_time_ns）。

#### ③ 双目校正（一次，`:335-359`）

`opencv_adapters::StereoRectificationContext::Create(rig, params)`（`adapters/opencv/`）：支持任意 plumb-bob 畸变、不同内参、非平行/非水平离轴 rig；产出 rectified 图 + 带**新 `calibration_version`** 的 derived `RigCalibrationSnapshot`。`Create` 失败 = 该 rig 根本无法校正，立即整场失败（下游所有相机 pass 会静默产出零证据，早死更诚实）。

`get_rectified()`（`:510`）按 keyframe 惰性缓存（`ConvertToMono8` + `Process`），VO pass 与声光融合 pass 共享，不重复重采样。

> 已知边界（见根 README「真实数据」节）：真实 HoloOcean 机体的非 y 轴基线会让 `cv::stereoRectify` 施加大旋转、把主点搬离图像中心，VO 在这种图上尚未调通。

#### ④ warmup 窗口 + kf0 锚点（`:361-423`)

- `warmup_seconds`（默认 0）换算成 keyframe 数（`kKeyframeIntervalS = 0.2`，对应 synth_bag_gen 的 5 Hz——这是本文件**唯一**还假设 kf 命名/间隔的地方，其余身份全来自 wire 的 observation_id）。窗口内 keyframe 仍进图、仍吃相对位姿（死推算），但**跳过声呐/深度两类"绝对参考"因子**——批处理位姿图版的"VIO 偏置收敛前别信绝对修正"。
- kf0 是 fixed anchor：x/y/yaw 对"相对位姿 + range-only"图是真 gauge freedom，取 `Identity()`；**z 不是**——有深度因子后 z 有绝对参考，所以 kf0 的 z 用它自己的深度证据（`-depth_m`）设初值（`:408-418`），而不是钉在 0。历史上钉 0 导致约束冲突、30 迭代不收敛、ATE 4.6 m（实跑 demo 发现，单元测试全绿）。

#### ⑤ 相对位姿证据——二选一分支（`:573-676`）

- **`stereo_landmark_vo`**（`:574-653`）：按序遍历 `ordered_camera_keyframe_ids`（bag 首现顺序，非数值假设）；`StereoLandmarkVoFrontend::Process(bundle, DerivedRig())` 在 rectified 对上跑 检测（blob/Harris 双模）→ 立体匹配 → 时序匹配 → RANSAC 刚体拟合，产出 `RelativePoseMeasurement` 证据（附带数值 SE(3) 协方差）+ 逐帧 VO 健康。关键细节：
  - 前端有状态（跨调用比较 landmark），必须按序喂；
  - `harris_corner`（真实图像）路径会收紧匹配参数（`max_row_diff_px`、`min_score_margin`，`:611-613`）——真实水下纹理重复，纯外观匹配曾把 ATE 打到 587 m；
  - 新 keyframe 的初值 = 死推算 `problem.GetKeyframePose(from) * measured_relative`（`:639-640`）；
  - `RelativePoseFactorBuilder::Build` 用真实 6×6 协方差白化残差，上限拆平移/旋转两路（`platform.yaml` 的 `reliability.default_sqrt_information`）。
- **默认 `black_box_vio`**（`:655-675`）：直接遍历 bag 里 `/evidence/relative_pose` 桩证据，同样死推算 + 建因子。

两分支最终产出同一种 `RelativePoseMeasurement`，喂同一个求解器——`estimator_mode` 切换的是**证据来源**，不是求解器。

#### ⑥ 回环闭合——独立的第二 pass（`:680-737`）

前置：rig + `stereo_landmark_vo` + `defaults.loop_closure.enabled`（默认关，`defaults/platform_loop_closure.yaml` 是开启版）。**必须在 ⑤ 完全结束后跑**（不交错）：此时 `problem.GetKeyframePose()` 还是**求解前的死推算位姿**——正是位姿邻近检索需要的"插入时刻位置"。`LoopClosureFrontend::Process(bundle, rig, kf_id, 当前位姿)`：候选检索（半径内 + keyframe 序号间隔）→ 当前帧立体三角化 → 与归档候选匹配 → RANSAC → 产出 `RelativePoseMeasurement` 回环边；进图时挂 `RobustPolicy::kHuber`（`:729-730`）。

v1 边界（有意保守）：位姿邻近而非外观检索（无 DBoW2）；固定 Harris 检测器，与合成高亮图案外观假设不匹配——放宽搜索半径在合成场景上反而让 ATE 从 1.26 m 恶化到 9.24 m（错误匹配多到 Huber 压不住），完整对照记录在 `configs/experiment/synthetic_loop_closure_vo_enabled.yaml` 头注释。

#### ⑦ 声呐 pass——每个 experiment 都跑（`:739-848`）

逐 `/raw/sonar_frame`：

1. 跳过 warmup keyframe / 图中不存在的 keyframe；
2. `SonarCfarFrontend::ProcessSonarFrame`（CFAR → 极坐标 → DBSCAN），v1 规则只取 top-1 候选（`hypothesis.proto` 文件头）；
3. **数据关联**（在线路标发现）：用当前死推算位姿把检测投到世界系 → `SubmapManager::QueryNearestPoint(predicted_W, 1.5m)`——命中则复用已存路标位置（比单次噪声检测稳），未命中则作为新路标插入（`SubmapManager` 在这里被当**在线路标库**用，"landmarks" bucket）；
4. `SonarRangeFactorBuilder::Build`（`context.nearby_points_W = {landmark_W}`）→ 单 keyframe 的 range-only 因子。

声呐无 elevation（0.0 填充，`:799`）；不联合优化路标 ⇒ 首次观测的 elevation 误差会摊到 x/y——这就是合成 demo ATE ~0.06-0.07 m（而非 ~3 cm）的主要来源。逐帧记录处理延迟并报 P95（`:743-769`，RAII 计时器保证拒绝路径也被计时）。

#### ⑧ 深度 pass（`:850-868`）

`/evidence/depth` 的 `PressureDepthMeasurement` → `DepthFactorBuilder` → 每 keyframe 一元因子（z 绝对参考），warmup 跳过。

#### ⑨ 求解（`:870-900`）

`PoseGraphProblem` 只含 keyframe 顶点 + 相对位姿/声呐距离/深度（/回环）边。`estimation.solver` 分支：

- `gauss_newton_v1`：`GaussNewtonSolver`（Eigen 手写 GN + LM 阻尼；`huber_delta` 只作用于 kHuber 标记的回环边）；
- `ceres_v1`：`adapters/ceres` 的 `CeresPoseGraphSolver`（同一 `PoseGraphProblem` 接口）；二进制没编译 Ceres 时**报错退出**，不静默回退。

默认不换：`tools/bench/solver_benchmark.sh`（smoke 12 kf / stress 1000 kf / 可选真实 bag）的对比数据是"要不要换默认"这个延后决策的关闭条件。

#### ⑩ 状态提交 + 轨迹（`:902-948`）

按 `KeyframeOrder` 逐 keyframe：`DecideTrackingStatus`（求解收敛 + **该帧处理时刻**的 VO 健康——非回溯性，VO 后来退化不改早先帧的状态）→ `BuildStateSnapshot` → `StateStore::Commit`；`submap_manager.UpdateKeyframePose` 写回优化后位姿；组装估计轨迹。时间戳三级优先（`.emplace` 的固定顺序即优先级）：左相机 capture_time > `/gt/state` 自身时间戳（仅计时，不入位姿）> evidence 的 log_time_ns。

#### ⑪ 声光融合 pass——与位姿图完全并行（`:950-1081`）

rig 带相机才跑。逐 keyframe：

```
SynchronizeAcousticOptic（时间同步判定）
  → StereoOpticalDepthFrontend（rectified 对上的块匹配稠密深度）
  → SonarCfarFrontend（复用 ⑦ 的同一实例，出声呐假设）
  → AcousticOpticDepthFusionFrontend::Fuse（弧投影 + 跨模态关联 + 后验深度优化）
  → BuildMapEvidenceFromFusedDepth（mapping 层：逐像素反投影到 base_link，
     按 DepthContribution 权重 → 第三类 MapEvidence bucket 存 submap/surfel）
```

关联结果按 `ACCEPTED / AMBIGUOUS / CONFLICT / REJECTED` 计数；地图点按 `contribution_mask` 分 optical-only / acoustic-optic 两类计数。**本 pass 永不触碰 `PoseGraphProblem`/求解器/ATE**（`:956-957` 注释明示）——稠密深度不是新因子类型，这是"位姿图不消费光学深度"边界的实现处。`AMBIGUOUS` 的判定有深度一致性门（前两名深度也要不满足合并方差门才算歧义），不要删。

#### ⑫ 评测（`:1083-1086`）

`ComputeAte`（`evaluation/trajectory_metrics.hpp`）对 `/gt/state` 算 RMSE/mean/max（可选对齐）；无 GT 的 bag（EuRoC）报 0 matched，属预期。

#### ⑬ 输出 + gate（`:1088-1139`）

- TUM 轨迹 `<out>_trajectory.tum`；
- 不可变 `RunManifest`（git commit、config/calibration hash、**derived calibration hash**、seed、起止时间、OS/CPU）；
- `EvaluateReplayGates`：`require_converged` 默认开；`max_ate_rmse_m` / `min_matched_ate_poses` / `require_nonempty_map` / `min_acoustic_optic_accepted` / `min_acoustic_optic_map_points` 按 experiment 的 `gates:` 段 opt-in。失败返回**退出码 2**，但输出文件总在 gate 前写完——失败后仍有东西可查。

### 4. 各层支撑代码速查

| 阶段 | 头（`include/`） | 实现（`src/` 或 adapters） |
|---|---|---|
| 消息模型 | `domain/domain.hpp` | 生成自 `schemas/proto/` |
| 算法接口 | `measurement_api/{frontend,factor_builder,residual_block}.hpp` | — |
| 声呐前端 | `frontends/{cfar_detector,dbscan,sonar_cfar_frontend}.hpp` | 同名 |
| 立体 VO | `frontends/{stereo_landmark_vo_frontend,landmark_blob_detector,harris_corner_detector,patch_matcher,block_matcher,rigid_transform_fit,camera_body_conjugation}.hpp` | 同名 |
| 声光融合 | `frontends/{acoustic_optic_associator,posterior_depth_optimizer,acoustic_optic_depth_fusion_frontend}.hpp` + `sensor_models/sonar_arc_projector.hpp` | 同名 |
| 回环 | `frontends/loop_closure_frontend.hpp` | 同名 |
| 因子 | `factor_builders/{relative_pose,sonar_range,depth}_*.hpp` | 同名（sonar 残差移植自 SVIn、雅可比独立重推，见 `NOTICE`） |
| 求解 | `estimation/{pose_graph_problem,gauss_newton_solver,state_store}.hpp` | 同名 + `adapters/ceres/` |
| 地图 | `mapping/{submap_manager,acoustic_optic_map_bridge,surfel_map}.hpp` | 同名 + `adapters/spatial_index/`（nanoflann 索引注入） |
| 运行时 | `runtime/{config,mcap_io,mcap_event_source,event_source,canonical_event,canonical_topics,run_manifest,acoustic_optic_synchronizer,synthetic_sonar}.hpp` | 同名 |
| 评测 | `evaluation/{trajectory_metrics,depth_metrics,fusion_metrics,map_metrics}.hpp` | 同名 |
| 双目校正 | `adapters/opencv/include/opencv_adapters/stereo_rectifier.hpp` | `adapters/opencv/src/` |

### 5. 验证方式

- 单测：`tests/{contracts,core,frontends,factor_builders,estimation,mapping,runtime,evaluation,adapters}/`（CTest 标签 `contract.*` / `unit.<layer>.*`）；
- 确定性回放：`tests/integration/determinism_test.sh`（同 bag/config/seed 两次运行逐字节一致）；
- 声光九场景矩阵：`acoustic_optic_scenario_matrix` app + `tests/integration/acoustic_optic_scenario_matrix_determinism_test.sh`（含最低有效覆盖 gate；预期拒绝场景是刻意的 fail-closed 语义，不是回归）；
- **实跑 demo**：CLAUDE.md 明确要求——z 轴 anchor、外参共轭方向两个关键 bug 都是单测全绿、实跑才发现的。

---

## 主线二：ROV 在线驾驶辅助（实时闭环）

### 闭环总览：四个进程 + 一组话题

```
┌─ 进程1（Python）────────┐   /holoocean/auv0/{LeftCamera,RightCamera,   ┌─ 进程2（C++）───────────┐
│ realtime_ros_session     │→  ImagingSonar,VehicleState} + /clock       │ holoocean_realtime_node │
│  HoloOcean/UE5 每 tick:  │                                           │  ROS2→规范化事件→算法    │
│  step() → 转换 → 发布    │←  /uw/pilot/thrusters（飞手命令回注）      │  → /uw/hmi/{status,overlay}│
└──────┬───────────────────┘                                           └──────┬──────────────────┘
       │ /uw/sim/ground_truth（只有进程4能订阅）                                │
       ▼                                                                    ▼
┌─ 进程4（Python）────────┐                                           ┌─ 进程3（Python）────────┐
│ task_scorer             │←  /uw/hmi/status                          │ scripted_pilot          │
│  评分 + RunReport + gate │                                           │  订阅 status → 推进器命令│
└─────────────────────────┘                                           └─────────────────────────┘
```

进程 0 是 `realtime_gate`（`realtime_gate.py` 的 `run_gate`，`:190`）：manifest 组合先 fail-fast 校验，四进程依次拉起，任一意外退出即失败，`ProcessGroup.stop()` 在 `try/finally` 里保证全部拆除。

**当前边界**：本机（Linux 开发机）无 HoloOcean/UE5、无 rclpy——需要它们的路径（`HoloOceanSession`、`RealtimeRosSession::main`、`realtime_gate` 的真实监督运行）写完但未实测。本机可验证的是 C++ 侧两个冒烟 app（有 CTest 集成测试）和 Python 侧全部单元测试。需求逐条追溯：`docs/traceability/rov-realtime-closed-loop.csv`（`tools/lint/check_realtime_traceability.py` 校验规格↔CSV 完整性，本身是个测试）。

### 1. Python 仿真侧（`adapters/holoocean/uw_holoocean_adapter/`）

| 模块 | 主要逻辑 |
|---|---|
| `holoocean_driver.py` | `HoloOceanSession`：封装真实 HoloOcean `make()/step()`（lazy import + 守护，无 HoloOcean 的机器上其余模块照常可测） |
| `scenario_manifest.py` | 加载/校验版本化场景+任务 manifest（`scenarios/blue_rov_aid_sv1213_base.json` + `aquaculture_search.yaml`/`structure_inspection.yaml`）成类型化 `RealtimeScenarioManifest`。fail-fast 项：重复/错配传感器、缺声呐标定字段、未知 prop 材质、任务目标缺视觉或声学属性，以及红线"**`algorithm_topics` 不得包含真值 topic**"；仓库自有 key（`uw_metadata`/`algorithm_topics`）在传给 `holoocean.make()` 前剥掉 |
| `scenario_randomization.py` + `sensor_perturbation.py` | 确定性多轴随机化 API（浑浊度/亮度/运动模糊/粒子/过曝/双目失配/声呐盲区/假回波/距离偏差），施加到真实图像/声呐阵列；RNG 由调用方持有，绝不新建/重置 |
| `pilot_command_model.py` | 推进器命令整形：死区 → 饱和到 `ActuatorModelSpec.limit` → 一阶惯性趋近（`time_constant_s`）。实时网关的 `/uw/pilot/thrusters` 与 `scripted_pilot` 共用 |
| `fault_injector.py` | `build_fault_schedule(seed, profile, duration)` 预采样每 topic 的丢帧/重复/有界乱序/中断事件堆（min-heap）；`apply()` 运行时出栈，**从不 sleep 仿真循环**；`apply_thruster_fault()` 对已整形命令单通道降效 |
| `realtime_ros_session.py` | 主循环（`:372-379`）：按 manifest tick 率 `session.tick()` → `build_realtime_messages()`（`:67`，每个传感器按自己的频率独立出消息，`/clock` 每 tick 发）→ 发布 + `spin_once`；订阅 `/uw/pilot/thrusters` 把整形后命令送进 HoloOcean。`HoloOceanSession.tick()` 内还施加随机化与故障 |
| `ros_message_conversion.py` | `build_topic_map()`（`:66`）是**话题契约唯一出处**：四个 `algorithm_inputs`（左右相机/声呐/车辆状态）+ 独立的 `pilot_camera` + `/clock` + `scoring_truth`。`VehicleState` odometry 只由 `VehicleOrientation`+`IMUSensor`+`DepthSensor` 组合并加噪，**永不用真值 `PoseSensor`** |
| `scripted_pilot.py` | 有界测试飞手（非产品自主）：只订阅 `/uw/hmi/status`；guidance 无效或 >500 ms 陈旧 → 全零推力；活动航迹 source 只剩 SONAR（视觉丢失）→ 0.4× 增益保守声呐搜索；4 推进器水平布局的简单增益控制（偏航 60 单位/rad、前进 8 单位/m） |
| `task_scorer.py` | **全仓库唯一允许消费 `/uw/sim/ground_truth` 的模块**（`observe_truth`；`observe_assist` 永不见真值）：precision/recall、误报/分钟、bearing/range/横向偏移 P95、track 有效率、任务成功率（对 `TaskSpec.success_conditions`）、完成时间、降级完成 |
| `run_report.py` | `RunReport`（代码/场景/任务/配置/标定 hash、seed、host、HoloOcean/UE 版本、传感器率、RTF、result-age/state-age 分位数、队列统计、资源采样、健康/故障时间线、评分）+ 四档 `GateSpec`：result-age/state-age P95 = minimum 350/150、nominal 与 disturbed 250/100、overload 500/200 ms，外加 deadline-miss 率上限（1%/5%）、RSS 增长上限（256 MiB）、CPU/GPU 余量下限（20%），逐条有边界测试 |
| `async_diagnostic_recorder.py` | 有界、drop-oldest、非阻塞的诊断 tap：`try_submit` 永不等待 sink/工作线程；sink 阻塞只降级自身统计，不影响主循环 |
| `realtime_gate.py` | 进程监督（见上）；`--profile/--task/--scenario/--seed/--seeds/--soak-duration-s/--gateway-binary/--out-dir` |

### 2. C++ ROS2 网关（传输边界，三段式接缝）

**为什么是三段**：`ros2` lint 角色只许依赖 `{adapters, measurement_api, sensor_models, domain, domain_proto}`，但网关需要拥有整个算法栈（runtime/application/opencv_adapters/frontends）。解法是依赖倒置接缝：

1. **`adapters/ros2/src/holoocean_realtime_node.cpp`**（ROS TU）：真 `rclcpp::Node`。订阅五个话题（四个算法输入 + `PilotCamera`，`:147-158`，QoS depth 1——只要最新）；回调只做 ROS 消息 → `RawHolo*` 中间结构 → `sink_->OnXxx(...)`；发布 `/uw/hmi/overlay`（`sensor_msgs/Image`）与 `/uw/hmi/status`（`std_msgs/String`，JSON）。`main` = init/spin/shutdown。**永不订阅 `/uw/sim/ground_truth` 和 `/uw/pilot/thrusters`**（后者归进程 1）。
2. **`include/adapters/holoocean_live_conversion.hpp`**：可移植的 `RawHolo* → uw::domain` 转换，无 ROS 头，可单测（BGR(A)→RGB 修正等在这里做）。
3. **`include/adapters/holoocean_realtime_sink.hpp`**（纯虚 `HoloOceanRealtimeSink`/`HoloOceanRealtimeOutput` 声明）+ **`src/application/holoocean_realtime_sink.cpp`**（实现，application 角色，可自由依赖全栈）。

`holoocean_realtime_sink.cpp` 是整个闭环的 C++ 装配（`OnlineAssistRealtimeSink`，`:189`）：

- **构造即就位**（`:191-232`）：`LiveEventSource`（默认四车道配置）、`OpenCvVisualAssistFrontend`、`SonarCfarFrontend`（与主线一同一实现）、`RealtimeAssistOutputSink`（内嵌 `OperatorOverlayRenderer`，replace-latest 缓存 pilot 图/声呐帧）、`OnlineAssistPipeline`（`dense_depth_provider = nullptr`，稠密默认关）、`ForwardingPort`（`PipelineInputPort` → pipeline 直转）；
- **泵线程**（`:207-231`）：`PumpEvents(source_, *port_)` 驱动整条链；try/catch 兜底并 `std::cerr` 记日志 + `Close()`（修过"泵线程静默死亡"的洞，提交 `ed95979`）；
- **ROS 回调 → 入队**（`Submit`，`:261`）：`source_.Submit({topic, monotonic_ns, ++seq, payload})`；`kClosed`（关闭中）外的一切状态交给 `LiveEventSource` 自己的统计；
- **`OnPilotCamera`**（`:245`）：**不入队**，只 `SetLatestPilotImage` 给叠加渲染器——呈现通道与算法通道分离的硬要求；
- **发布路径**（`RealtimeAssistOutputSink::Publish`，`:128`）：每次 pipeline 出 `OperatorAssistState`，用最新 pilot 图 + 最新声呐帧渲染叠加图（畸形声呐帧只省略副面板），连同 JSON status 交回 ROS 节点发布。

### 3. 运行时实时入口（`include/runtime/`）

**`live_event_source.hpp`** —— `EventSource` 的实时实现：

- 四车道（localization / correction / mapping / evidence），每车道一条 `bounded_queue.hpp` 的 `BoundedQueue`，容量与溢出策略独立（`reject` / `drop_oldest`，配在 `defaults/platform.yaml` 的 `runtime.lanes:`，默认 64/32/16/256）；
- 入队**语义校验**：未知 topic、按传感器追踪的序号跳变/回退、时间倒退 → 对应 `LiveSubmitStatus::k*Rejected` 并计数（`kAccepted/kAcceptedAfterDroppingOldest/kDroppedNewest/kOverflowRejected/kSemanticRejected/kDuplicateOrOutOfOrderRejected/kReferenceRejected/kClosed`）；
- **出队后才打 receive 时戳**（提交 `1c44753`：入队线程的时间不代表处理时间）；
- 每车道 128 样本 `rolling_latency.hpp` 滚动分位数 + 容量违规/序号跳变计数 → `LiveSourceStats`（gate 的 result-age 与队列健康就吃这里）；
- 调度按车道优先级轮转（`schedule_cursor_`），deadline 感知（提交 `04c2ec5`）。

**`acoustic_optic_buffer.hpp`**：异步立体对 + 声呐 + 车辆状态的配对缓存——只有稠密深度需要"三样齐"才触发；生命周期/陈旧性有专门加固（提交 `5c2b8c4`/`7461960`）。

### 4. 应用编排核心：`OnlineAssistPipeline`（`src/application/online_assist_pipeline.cpp`，全部逻辑在 `Impl`）

设计原则（头注释明示）：**视觉与声呐目标检测相互独立**——声呐掉线不停视觉跟踪，反之亦然（降级运行是目标，不是故障）；稠密深度是唯一走完整同步 bundle + 墙钟预算的路径。

#### 输入分发（`:120-174`）

| 入口 | 逻辑 |
|---|---|
| `OnImageFrame`（`:120`） | 喂 buffer 配对；若是 rig 首相机（左）→ `RunVisualDetection`；bundle 齐则 `HandleBundle`；总是 `PublishNow` |
| `OnSonarFrame`（`:131`） | 喂 buffer；`RunSonarDetection`（声呐驱动关联批）；`PublishNow` |
| `OnVehicleState`（`:140`） | 记 liveness 时戳（首状态传感器）；喂 buffer；`PublishNow` |
| `OnHealthReport`（`:151`） | 按**我方收到时刻**（`now_()`）记外部健康——防上报方时钟漂移；`PublishNow` |
| `OnImuSample`/`OnDvlSample`/`OnMeasurementEvidence`/`OnReferenceState`/`OnMapEvidence`（`:165-169`） | **接受但永不进算法**（不阻塞在线循环；真值类事件在入口层就被拒） |
| `UpdateRig`（`:176`） | 标定热更新：版本变了 → 重建 associator/tracker、清 pending、置 `recovering_`、计数 `calibration_reset_count` |

#### 视觉检测（`RunVisualDetection`，`:195`）

`visual_frontend_->Process(左图, 可选稠密深度先验, 内参)` → `TargetDetection` 列表 + 路径横向偏移（含 σ）+ 健康。两个关键决策：

- **替换式**存 `pending_visual_`（不累积）：累积会让同目标的二次检测挤进同一关联批，而 tracker 每批每航迹至多配一条检测——第二条会炸出重复航迹；
- **等声呐再关联**：视觉检测先挂在 `pending_visual_`，等下一次声呐到达一起配对（避免视觉抢先 flush 后 observation_id 被 tracker 记忆、毒化后续配对）；仅当 `SonarRecentlyLive()` 为假（声呐真掉线）才立即单独 flush——保证声呐掉线时纯视觉航迹也能及时出来。

#### 声呐检测（`RunSonarDetection`，`:239`）

`SonarCfarFrontend::ProcessSonarFrame`（与主线一同码）→ `SonarTargetExtractor::Extract`（多目标）→ 替换式存 `pending_sonar_` → **立即 `FlushAssociation`**（声呐是正常状态下的关联批驱动者）。

#### 关联批（`FlushAssociation`，`:259`）

`TargetAssociator::Associate(视觉批, 声呐批, rig)` → `TargetMeasurement` 列表（见前端层）→ `TargetTracker::Update(批, now)` **整批原子**（任一 observation_id 已被接受则整批拒绝——所以 flush 后无条件清空两个 pending，成败都清）；`recovering_` 期间要等出现 CONFIRMED 航迹才解除。

#### 稠密深度——可选、预算内（`HandleBundle`，`:279`）

前置：`dense.enabled`（**默认 false**）+ 无在飞任务 + bundle 图已 rectified。`DenseDepthProvider::RunBounded`（默认实现 `StereoBlockMatchDenseDepthProvider` 包 `StereoOpticalDepthFrontend`，**事后**墙钟检查超预算即弃——块匹配不可抢占）；结果作为后续视觉帧的深度先验（`DenseCurrentlyFresh`，`:323`，新鲜度窗口 = `modality_stale_after_s`）。质量拒绝/超时/未启用/无 provider 统一表现为 `dense_deadline_missed`——操作员词汇只区分"有没有"，不区分"为什么没有"。

#### 降级判定（`ComputeDegradation`，`:339`）——固定优先级链

```
recovering_
  → RECOVERING / "recovering"（guidance 无效）
双模态都死 → UNAVAILABLE / "all_assist_unavailable"
车辆状态陈旧 → UNAVAILABLE / "vehicle_state_stale"
声呐死     → SUSPECT / "sonar_unavailable"      （guidance 仍有效！）
视觉死     → SUSPECT / "visual_unavailable"      （guidance 仍有效！）
前端自报降级（视觉优先提升为系统级 reason，声呐仍在 sensor_health 里可见）
稠密超期   → SUSPECT / "dense_deadline_missed"
否则       → HEALTHY
```

要点：单模态掉线只是 SUSPECT 且 `guidance_valid=true`——**降级运行本来就是这套系统的设计目标**。liveness 判定用 capture_time 距今（`modality_stale_after_s` 默认 1 s；车辆状态 `vehicle_state_stale_after_s` 0.5 s）。

#### 发布（`PublishNow`，`:380`）

组装 `OperatorAssistState`（`target.proto`）：

- 当前航迹集 `tracker().ToProtoSet(wall_s)`（陈旧航迹不进集合）；
- 路径横向偏移 + σ：**gate 在视觉 liveness 上**——相机掉线后不再无限重发旧值；
- `system_health`（上面的降级决策）+ `sensor_health`（视觉/声呐前端健康 + 未过期的外部上报，过期即丢，不永久重发）；
- `data_age_ms`（三路输入最新 capture_time 距今）；
- `guidance_valid` + `degradation_reason`。

→ `sink_->Publish(state)`（`AssistOutputSink` 接口；实时侧是 replace-latest + 渲染，冒烟侧是 `latest_assist_sink.hpp`）。`OnlineAssistPipelineDiagnostics` 记录发布数/关联拒绝数/稠密尝试与超时数/标定重置数。

### 5. 前端算法层

| 组件 | 位置 | 逻辑 |
|---|---|---|
| `VisualAssistFrontend` 接口 | `measurement_api/target_frontend.hpp` | `Process(左 rectified 图, 可选深度先验, 内参) → {targets, path_lateral_offset_m, path_offset_sigma_m, health}`；不暴露任何 cv:: 类型 |
| `OpenCvVisualAssistFrontend` | `adapters/opencv/include/adapters/opencv_visual_assist_frontend.hpp` | HSV 色相/饱和度/亮度阈值的目标检测（养殖区色相带，最小连通面积）+ Canny/Hough 结构线聚类与路径偏移估计（参考距离 + σ）；亮度/对比度/Laplacian 方差/纹理支撑度等 **fail-closed 质量门**（弱结构/协方差溢出直接报降级，不产半吊子结果，提交 `2485594`/`6a5455c`） |
| `OperatorOverlayRenderer` | `adapters/opencv/include/adapters/operator_overlay_renderer.hpp` | 无头渲染：pilot 主画面 + 声呐极坐标副面板（畸形帧只省略面板）+ 逐航迹标注 + 非健康横幅 + 偏移箭头；永不 imshow/waitKey |
| `SonarTargetExtractor` | `frontends/sonar_target_extractor.hpp` | CFAR 假设 → 多目标 `TargetDetection`（配置驱动、参数校验、确定性排序保证溯源稳定，提交 `b70f2e1`） |
| `TargetAssociator` | `frontends/target_associator.hpp` | 跨模态配对成 `TargetMeasurement`（bearing ± range + 2×2 协方差，时间经 rig `time_offset` 校正）。门链：输入有效性 → 标定版本一致 → 坐标系可解析 → 时间差 → 类别相容 → 方差上限 → bearing/range/联合 Mahalanobis → 运动连续性 → 配对代价；每条拒绝带 `{metric, value, threshold}` 结构化诊断（不靠 NaN 传达语义）。几何与预测有专门加固（提交 `34e7783`） |
| `TargetTracker` | `frontends/target_tracker.hpp` | 每目标状态 `[bearing, ḃ, range, ṙ]` + 4×4 协方差；Mahalanobis 门控关联、2 hit 确认、3 miss 降级、`stale_after_s` 陈旧、≤`max_prediction_dt_s`（0.5 s）预测填补、同目标航迹合并（bearing/range 双阈）；bearing-only 航迹**不伪造 range**（proto3 显式 presence）；整批原子更新（重复 observation_id 整批拒绝） |
| `TargetFusionComponents` | `frontends/target_fusion_components.hpp` | associator + tracker 捆成一个可注入单元，标定变更时整体重建 |

### 6. 不依赖仿真器的验证路径

- **`apps/live_ingress_smoke.cpp`**：混合速率（默认相机 20 / 声呐 10 / 状态 50 Hz）合成事件打进 `LiveEventSource`，验证 submit 状态、队列统计、延迟分位数、`--inject-stall-ms` 背压下的行为；CTest：`tests/integration/live_ingress_smoke_test.sh`。
- **`apps/online_assist_smoke.cpp`**：**真实前端 + 真实 pipeline** 的实时切片——同样 20/10/50 Hz，真立体图（合成纹理）+ 真声呐渲染（`runtime/synthetic_sonar.hpp`）+ 真车辆状态，经 `LiveEventSource → PumpEvents → OnlineAssistPipeline`；`--drop-visual-at-s` / `--drop-sonar-at-s` 打降级路径。断言（见测试脚本）：`fused_tracks>0`、`truth_delivered=0`（真值绝不进算法侧）、`stale_normal_tracks=0`、`queue_capacity_violations=0`、`result_age_p95_ms<250`。
- Python 侧：`adapters/holoocean/tests/` 22 个文件覆盖全部可移植逻辑（含 gate 预算逐条边界测试、故障调度确定性、topic 契约、脚本飞手等）。

### 7. 配置、场景与 gate

- 场景/任务 manifest：`adapters/holoocean/scenarios/`（`blue_rov_aid_sv1213_base.json` + 两个任务 yaml）；
- gate profile：`configs/experiment/rov_realtime_{minimum,nominal,disturbed,overload}.yaml`——**是 Python `realtime_gate` 的 profile 文件，不是 C++ `--experiment` 配置**（nominal 20/10/50 Hz；disturbed 开故障；overload 加负载；minimum 最低配；nominal soak 用 `--soak-duration-s 7200` 覆盖时长）；
- 算法参数落点：`configs/defaults/platform.yaml` 的 `frontends.target_association` / `frontends.target_tracker` / `online_assist:`（稠密开关 + 预算 + 新旧度阈值）/ `runtime.lanes:`（四车道）四段；
- 规范：`docs/specifications/` 三份（ROV 竞赛在线系统需求 / HoloOcean 实时闭环仿真 / ROV 声光在线融合链路），权威顺序：正式赛事规则 > 需求规格 > 两份下位规格 > 路线图/配置/测试。

### 8. ROS2 在这套系统里的作用（专节）

**结论：ROS2 只在主线二出现，且只是进程间传输总线，不是算法框架。**

1. **主线一与 ROS2 零关系**：离线管线的数据层是 MCAP + Protobuf，`replay_demo` 不需要也不链接 ROS2（`UW_BUILD_ROS2` 默认 OFF）；lint 保证 `include/`、`src/` 生产代码无任何 ROS 头，ROS2 被锁在 `adapters/ros2/`。
2. **主线二为什么用 ROS2**：HoloOcean 官方对外接口就是 ROS2（`external_repos/holoocean-ros` 的 `holoocean_main` 把 UE5 传感器以话题暴露），消费它是既定路线；同时它把闭环拆成四个可独立崩溃/重启/替换的进程——上真机时把进程 1 换成真实传感器驱动进程，C++ 网关/飞手/评分器不用动。
3. **ROS2 承担的五件事**：传感器流发布（进程 1→2）；规范化事件的进入口（进程 2 订阅四个算法输入话题）；辅助结果出口（`/uw/hmi/status` JSON 字符串 + `/uw/hmi/overlay` 图像）；飞手命令回注（`/uw/pilot/thrusters`）；真值隔离与 `/clock`（`/uw/sim/ground_truth` 只有 scorer 可订）。
4. **ROS2 不承担的**：算法、状态管理、时间语义（pipeline 用自己的 capture/wall 时戳 + 单调时钟；receive 时戳在**出队后**打）、序列化（规范消息是 Protobuf，ROS 消息只是传输壳，两侧显式转换）。QoS 全 depth-1（"只要最新"）；背压/优先级/溢出策略由本仓库自己的 `LiveEventSource` 四车道承担，不依赖 ROS2 队列语义。
5. **分层代价与解法**：ROS 头只许在 `adapters/ros2/`，但网关要拥有全算法栈 → `holoocean_realtime_sink.hpp` 纯虚接缝（ROS TU 只见接口，实现在 `application` 角色），两边依赖各自合法。
6. **构建边界**：需要已 source 的 ROS2 Jazzy + 外部 colcon workspace 构建的 `holoocean_interfaces`（MIT，`external_repos/holoocean-ros`）；两节点已真实编译/链接/独立启动验证，但未跑过真实 HoloOcean/UE5 数据流。构建时的 PATH 顺序坑（colcon 要系统 Python 在前、cmake 要 conda cmake 4.x 在前）见 [CLAUDE.md](../CLAUDE.md)。

---

## 两条主线的关系

| 维度 | 主线一（离线 SLAM） | 主线二（ROV 在线辅助） |
|---|---|---|
| 输入来源 | `McapEventSource`（一次顺序扫全 bag） | `LiveEventSource`（四车道有界队列，实时流入） |
| 入口接口 | 同一个 `PipelineInputPort` / `PumpEvents` | 同左 |
| 消息模型 | 同一套 `schemas/proto` + `canonical_topics` | 同左 |
| 算法核心 | 因子图（VO/桩 + 声呐距离 + 深度 + 回环）→ 求解器 | 目标检测 → 关联 → 跟踪（**没有位姿图、没有求解器**） |
| 复用组件 | `SonarCfarFrontend`、`StereoOpticalDepthFrontend`、`AcousticOpticBuffer`（配对语义） | 同左（声呐前端是同一实例级复用） |
| 输出 | TUM 轨迹 + MapEvidence（submap/surfel）+ ATE + RunManifest | `OperatorAssistState`（航迹 + 偏移 + 健康）+ HMI 叠加图 |
| 确定性 | 同 seed 逐字节一致（determinism test 把关） | 确定性故障调度/随机化（RNG 显式持有），但整体在线时序不保证逐字节 |
| 真值角色 | `/gt/state` 仅评测支路（kReferenceOnly） | `/uw/sim/ground_truth` 仅 scorer 进程可订阅 |
| 成熟度 | 合成数据端到端跑通 + 确定性回归保护；真实数据 VO 调参中 | 代码全 + 单测/冒烟覆盖；未在真实仿真器上闭环 |

**共享的工程不变量**（改任何一条主线都要守）：单向依赖 + adapters 边界；先改 `.proto` 再动两侧；显式 seed 的局部 RNG；`Pose3` 无欧拉角；深度符号语义不混用；新 frontend/factor_builder 进对应层的合并 target。

---

## 维护约定

- 本文只记录**当前实现**的事实；机制背后的"为什么"来自源码头注释、`configs/` 头注释与 `NOTICE`，修改对应代码时请同步更新本文。
- 行号会漂移：更新时以函数名重新定位，行号仅作当日快照。
- 发现本文与代码不一致时，以源码为准并修正本文（仓库文档维护总约定见[文档中心](./README.md)）。
