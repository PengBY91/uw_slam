---
title: uw_slam 代码库参考文档
type: codebase-reference
status: current
scope: "what the code does today, file-by-file/type-by-type — not a design proposal"
updated: 2026-08-18
---

# uw_slam 代码库参考文档

本文是 `uw_slam` 的**代码级参考文档**：逐层、逐目录、逐类型地记录当前代码库里
**实际存在的东西**——真实的类名/函数签名/字段名/算法参数，以及它们如何连接成
一条可运行的端到端管线。

这份文档和仓库里已有的三份文档分工不同，互不重复：

| 文档 | 性质 | 回答的问题 |
|---|---|---|
| [`README.md`](../README.md) | 项目门面 | 这是什么、怎么编译、怎么跑 demo |
| [`acoustic-optic-slam-platform-architecture-2026-08-17.md`](./acoustic-optic-slam-platform-architecture-2026-08-17.md) | 长期架构设计（已批准） | 系统**应该**长成什么样、为什么这么设计 |
| [`holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md`](./holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md) | 第一阶段工程方案（草稿） | 怎么把三份外部代码接起来跑通第一版 |
| **本文** | 代码参考 | 代码**现在**长什么样：真实类型、真实函数、真实数据怎么流动 |

架构文档描述的是目标状态，很多设计尚未实现或只实现了一部分；本文只记录"读一遍
代码之后能确认的事实"，凡是设计与实现有出入的地方，都会明确标出"文档说 X，代码
实际是 Y"。

## 目录

1. [现状速览](#1-现状速览)
2. [架构总览：分层与依赖方向](#2-架构总览分层与依赖方向)
3. [目录结构地图](#3-目录结构地图)
4. [领域契约层：schemas/proto/](#4-领域契约层schemasproto)
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

骨架 + 每层至少一条真实可跑的垂直切片，13/13 C++ 测试、9/9 Python 测试通过，
有一条端到端可跑的合成数据 demo（`synth_bag_gen` → `replay_demo`，6 次迭代收敛，
ATE rmse ~3cm）。不是空骨架，也不是生产系统——具体缺什么见第 15 节。

---

## 2. 架构总览：分层与依赖方向

依赖只允许单向：

```
core → algorithms → runtime → adapters → apps
```

`core/` 和 `algorithms/` 下任何文件不允许出现 ROS/HoloOcean/第三方 vendor 头，由
`tools/lint/check_no_ros_in_core.sh`（[第 14 节](#14-工具链-tools)）静态检查这条不变量。
跨语言（C++/Python）领域契约的唯一事实源是 `schemas/proto/`；canonical 的录制/回放
格式是 MCAP（未压缩，因为 C++ 构建关掉了 zstd/lz4 后端，保证 Python 写的 bag 能被
C++ 直接读）。

| 层 | CMake / Python 包 | 依赖 | 作用 |
|---|---|---|---|
| `schemas/` | `uw_domain_proto`（生成） | 无 | 领域消息定义（protobuf），C++/Python 绑定的共同来源 |
| `core/domain` | `uw_domain` | `uw_domain_proto` | 生成类型的 C++ 人体工学层（Stamp 转换、oneof payload 访问器） |
| `core/sensor_models` | `uw_sensor_models` | `uw_domain`, Eigen3 | `Pose3`、声呐 beam 几何 |
| `core/measurement_api` | `uw_measurement_api`（INTERFACE） | `uw_domain`, `uw_sensor_models` | `SonarFrontend`/`FactorBuilder`/`ResidualBlock`/`*Provider` 抽象接口 |
| `algorithms/frontends/sonar_cfar_frontend` | `uw_sonar_cfar_frontend` | `uw_measurement_api` | CFAR + DBSCAN 声呐前端 |
| `algorithms/factor_builders/*` | `uw_sonar_range_factor`/`uw_relative_pose_factor`/`uw_depth_factor` | `uw_measurement_api` | 三种残差 + 雅可比 |
| `algorithms/estimation` | `uw_estimation` | `uw_measurement_api`, Eigen3 | Gauss-Newton/LM 求解器、`PoseGraphProblem`、`StateStore` |
| `algorithms/mapping/submap_manager` | `uw_submap_manager` | `uw_domain`, `uw_sensor_models` | 按 keyframe 存储的地图证据管理 |
| `runtime` | `uw_runtime` | `uw_domain`, `mcap_impl`, `protobuf`, `yaml-cpp`, Eigen3 | 状态机、四车道队列原语、分层配置加载、`RunManifest`、MCAP 读写封装 |
| `adapters/holoocean` | Python `uw_holoocean_adapter` | protobuf, mcap, numpy | 直连 HoloOcean Python API |
| `adapters/ros2` | `uw_holoocean_sonar_bridge_node`（`UW_BUILD_ROS2`） | ROS2 Jazzy, `holoocean_interfaces` | ROS2 话题 → `SonarFrame` 的传输层桥接 |
| `adapters/third_party/*` | `uw_holoocean_ros_bridge`/`uw_svin_bridge`/baseline stub | `uw_measurement_api` | 三个具体 provider 实现 + 一个未完成的 baseline |
| `apps/tools/synth_bag_gen`, `apps/replay_demo` | 可执行文件 | 上述所有层 | 唯一目前真实跑通的端到端管线 |
| `evaluation` | `uw_evaluation` | 无外部依赖 | ATE 轨迹指标（**没有 RPE**） |

---

## 3. 目录结构地图

```
schemas/proto/uw/domain/     11 个 .proto 文件，领域契约唯一事实源
core/
  domain/                    uw_domain：Stamp 助手、oneof payload 访问器模板
  sensor_models/             uw_sensor_models：Pose3、声呐 beam 几何
  measurement_api/           uw_measurement_api：Frontend/FactorBuilder/ResidualBlock/Provider 抽象（纯头文件）
algorithms/
  frontends/sonar_cfar_frontend/     CFAR + 极坐标转换 + DBSCAN（移植自 sonar_camera_reconstruction）
  factor_builders/
    sonar_range_factor/              残差移植自 SVIn，雅可比独立重导
    relative_pose_factor/            原生：6D 相对位姿残差
    depth_factor/                    原生：1D 深度残差
  estimation/                        Eigen 手写 LM 求解器、PoseGraphProblem、StateStore
  mapping/submap_manager/            按 keyframe 存储 MapEvidence（并非经典 submap 边界逻辑）
runtime/
  include/uw/runtime/
    state_machines.hpp               三个正交的滞回状态机
    bounded_queue.hpp                四车道有界队列原语
    config.hpp                       defaults→rig→scenario→experiment 分层配置类型
    run_manifest.hpp                 RunManifest（一次运行的不可变记录）
    mcap_io.hpp                      MCAP 读写的 protobuf 封装
adapters/
  holoocean/                         Python 包 uw_holoocean_adapter，直连 HoloOcean
  ros2/                              UW_BUILD_ROS2 开关保护的 ROS2 节点
  third_party/
    holoocean_ros_bridge/            真实、已测试：ImagingSonar → SonarFrame
    svin_bridge/                     真实、已测试注入点；ROS2 订阅层是全注释骨架
    sonar_camera_reconstruction_baseline/   纯 stub，脚本体是 TODO+exit 1
  datasets/                          纯 stub，只有 README
apps/
  tools/synth_bag_gen/                合成带真值的 MCAP bag
  replay_demo/                        bag 回放 → 前端 → 因子构建 → 求解 → 评测
evaluation/                          ATE（无 RPE）
configs/                             defaults/rig/scenario/experiment 四层 YAML
tests/
  l0_contracts/                      protobuf round-trip 契约测试
  l2_replay/determinism_test.sh      两次跑 replay_demo 逐字节比对
tools/
  lint/check_no_ros_in_core.sh       依赖不变量检查
  codegen/gen_py.sh                  生成 Python protobuf 绑定
  setup_dev_env.sh                   apt→conda-forge 回退安装脚本
cmake/
  UwProtobuf.cmake                   生成 uw_domain_proto
  UwMcap.cmake                       FetchContent_Populate 拉取 MCAP header-only SDK
```

---

## 4. 领域契约层：schemas/proto/

`package uw.domain;`，proto3。11 个文件，导入关系：`time.proto`/`ids.proto` 是叶子；
`observation.proto` 依赖两者；`sonar.proto` 依赖 `observation.proto`；
`measurement.proto` 依赖 `ids.proto`+`calibration.proto`；`factor.proto` 依赖
`ids.proto`+`time.proto`；`state.proto` 依赖三者；`map.proto` 依赖 `ids.proto`；
`hypothesis.proto` 依赖 `measurement.proto`；`health.proto` 依赖 `time.proto`。

### `time.proto`
- `enum ClockDomain`：`UNSPECIFIED/SIMULATION/SYSTEM_MONOTONIC/SENSOR_HARDWARE`
- `message Stamp { int64 seconds = 1; int32 nanos = 2; }` —— 故意不用
  `google.protobuf.Timestamp`（避免引入 well-known-types 依赖），字段布局照抄它。

### `ids.proto` —— 强类型 ID
每个 ID 都是**独立的单字段 message**，而不是裸 `string`/`uint64`：
`SensorId`、`FrameId`、`SequenceId`、`ObservationId`、`EvidenceId`、`KeyframeId`、
`StateId`、`SubmapId`、`CalibrationVersion`、`ModelVersion`、`StateVersion`。
protoc 因此为每一个生成独立的 C++/Python 类，类型系统直接阻止"把 SensorId 传去
需要 StateId 的地方"——**这不是 C++ 侧手写的 phantom type/strong typedef，强类型
完全来自 protobuf 的 wrapper-message 模式**，`core/domain/` 里没有任何手写的
`KeyframeId` 类。

### `observation.proto`
`ObservationHeader`：`observation_id` `sensor_id` `sequence_id` `capture_time`
`receive_time`（Stamp，capture/receive 分离，见第 8.1 节 `time_utils.py`）
`clock_domain` `sensor_frame` `calibration_version`
`validity`（嵌套 enum：`OK/DEGRADED/REJECTED`）`provenance`（string，core 不解析）。
每条 raw/measurement/evidence 消息都携带一个，只在 adapter 边界产生一次，下游不
重新推导。

### `sonar.proto`
`SonarFrame`：`header` `intensity_tensor`（bytes，行主序 `[num_ranges,num_beams]`）
`num_ranges` `num_beams` `encoding`（`UINT8_GRAY`）`range_bins`（repeated float）
`azimuth_angles`（repeated float，**必须严格递增**，由
`uw::domain::IsAzimuthAscending()` 校验）`min_range`/`max_range`/`range_resolution`
`horizontal_fov` `elevation_aperture`（永不被折叠成单点估计）`gain_metadata`
`sound_speed_assumption`。字段设计参照 `sonar_camera_reconstruction` 的
`OculusPing`/`OculusFire`。

### `measurement.proto` —— 带物理语义的 typed payload
- `SonarRangeBearing`：`range_m` `bearing_rad` `range_sigma_m` `bearing_sigma_rad`
  `sonar_frame`。**故意不含 elevation**——2D 前视声呐 ping 本来就观测不到。
- `RelativePoseMeasurement`：`from_keyframe` `to_keyframe` `relative_pose`
  （语义 `from_T_to`）`covariance_6x6_row_major`（36 个 double，顺序
  `[tx,ty,tz,rx,ry,rz]`）。
- `PressureDepthMeasurement`：`depth_m` `sigma_m`。
- `VisualTrackMeasurement`/`StereoDepthMeasurement`/`SonarRegistrationMeasurement`/
  `ImuPreintegrationMeasurement`：占位消息，暂无对应的 factor_builder 消费。
- `MeasurementEvidence`：`evidence_id` `source_observations`（repeated）
  `estimated_noise_scale`（**只是前端建议值，绝不是最终 information**）
  `quality_features`（map）`observable_subspace` `valid_domain`
  `algorithm_version` `model_version`，然后一个覆盖上述 7 种 payload 的 `oneof`。

### `factor.proto`
`FactorCandidate`：`associated_state_ids` `measurement_type` `residual_model`
（决定哪个 FactorBuilder 消费它）`proposed_noise` `observable_subspace`
`robust_policy_hint`（`NONE/HUBER/CAUCHY`）`evidence_ids` `valid_from`/`valid_to`。
前端只能提议 candidate，只有 typed FactorBuilder 才真正构建残差——前端不能直接
注入权重。

### `state.proto`
`StateSnapshot`：`state_id` `state_version` `capture_timestamp` `pose_wb`
`velocity_w_mps` `imu_bias`（6：gyro+accel）`marginal_uncertainty_row_major`
`tracking_status`（`TRACKING/DEGRADED/LOST/RELOCALIZING/RECOVERING`）
`calibration_version` `contributing_measurements`。单一权威 `StateStore`
（single-writer/multi-reader，见 [6.5](#65-algorithmsestimation--gauss-newtonlm-求解器)），真值永不进入这个消息。

### `map.proto`
`MapEvidence`：`evidence_id` `keyframe_id` `state_version` `local_frame`
`representation_type`（`POINT_CLOUD/OCCUPANCY/TSDF/SURFEL/SEMANTIC_MASK`）
`geometry_or_occupancy`（bytes；POINT_CLOUD 时是紧凑小端 float32 xyz 三元组）
`uncertainty` `source_observations` `reintegration_policy`（`TRANSFORM_ONLY`/
`FULL_REFUSE`）。**这是对 `sonar_camera_reconstruction` `merge.py` 的刻意反模式**：
保留局部坐标 + state_version 引用，而不是插入时就烘焙进一个会过期的全局帧
——同样的原则贯穿 `submap_manager`（见 [6.6](#66-algorithmsmappingsubmap_manager)）。

### `health.proto`
`HealthReport`：`component_id` `status`（`HEALTHY/SUSPECT/UNAVAILABLE/RECOVERING`）
`reason_code` `input_valid_rate` `queue_depth` `latency_p50/p95/p99_ms`
`residual_mean/stddev` `dropped_frame_count` `valid_domain_rate`
`out_of_distribution_rate` `last_recovery_time`。每个模块统一发布这个消息。

### `hypothesis.proto`
`HypothesisSet`：`candidates`（repeated `MeasurementEvidence`）
`calibrated_likelihoods`（与 candidates 等长同序）`rejected_candidates`
`ambiguity_reason` `out_of_distribution`。存在的意义是不让 FLS elevation 歧义/
多路径/误关联过早被折叠成一个点——但**v1 算法只消费 top-1**
（`uw::domain::TopCandidate<T>()`）。

### `calibration.proto`
`Transform3D { matrix_row_major: repeated double[16] }`（4x4 齐次矩阵，行主序）。
`FrameEdge { parent_frame, child_frame, transform }`（语义 `parent_T_child`，
v1 故意不带逐条不确定度）。`ImuNoiseModel`/`CameraIntrinsics`/`SonarBeamModel`
（含 `sonar_enabled`，对应 SVIn 的 `isSonarUsed`）/`DepthSensorModel`（含
`depth_enabled`，对应 `isDepthUsed`）。`RigCalibrationSnapshot`：
`calibration_version` `frame_tree`（repeated FrameEdge）`cameras` `imu_noise`
`sonar_beam_models` `depth_models` `time_offset_seconds`（map）`notes`——**唯一
标定事实源**，ROS 静态 TF / 各工具自己的 YAML 都应该从它派生，不应该另外维护
（`adapters/third_party/svin_bridge` 单向从它生成一次性的 SVIn yaml，绝不反向）。

---

## 5. core/ 层

### 5.1 `core/domain` —— `uw_domain`

`include/uw/domain/domain.hpp` + `src/domain.cpp`，是生成类型之上的一层薄薄的
C++ 人体工学封装，**不是第二套 schema**。

```cpp
Stamp ToStamp(std::chrono::system_clock::time_point);
std::chrono::system_clock::time_point ToTimePoint(const Stamp&);
double ToSeconds(const Stamp&);
Stamp FromSeconds(double);

bool IsAzimuthAscending(const SonarFrame&);   // sonar.proto 不变量的实现
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
> `core/measurement_api/include/uw/measurement_api/measurement_evidence.hpp`
> ——该文件**不存在**。实际实现在 `uw_domain` 目标里的 `domain.hpp`。

### 5.2 `core/sensor_models` —— `uw_sensor_models`

只依赖 `uw_domain` + `Eigen3::Eigen`。

**`Pose3`**（`geometry.hpp`/`.cpp`）——平移 + 四元数 xyzw，故意不引入 Sophus/manif，
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

**声呐 beam 几何**（`sonar_beam_model.hpp`/`.cpp`）——只有两个自由函数，纯几何，
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

### 5.3 `core/measurement_api` —— `uw_measurement_api`（纯头文件 INTERFACE 库）

**`frontend.hpp`**：
```cpp
class SonarFrontend {
 public:
  virtual ~SonarFrontend() = default;
  virtual uw::domain::HypothesisSet ProcessSonarFrame(const uw::domain::SonarFrame&) = 0;
  virtual uw::domain::HealthReport Health() const = 0;
};
```
**注意：没有通用的 `Frontend<T>` 模板**——设计上刻意不搞一个模板套所有模态
（声呐/视觉/立体视觉的输入输出物理上不同，架构文档 7.4 节），今天只有这一个非泛型
的 `SonarFrontend`。输出永远是 `HypothesisSet`，从不折叠成单一 6DoF 位姿。

**`factor_builder.hpp`**：
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
模型"这条架构不变量——前端永远不能直接注入权重。

**`residual_block.hpp`**（Ceres 风格契约）：
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
"顺手"去重构它）。**注意：仓库里没有任何抽象 `Solver` 基类**——`GaussNewtonSolver`
是具体的非虚类；"可替换"只是靠 `factor_builders` 只依赖 `ResidualBlock` 这一层
间接性来保证的，还没有真正的 `Solver` 接口。

**`providers.hpp`**（面向 adapters 的非阻塞轮询接口）：
```cpp
class LocalOdometryProvider { virtual std::optional<MeasurementEvidence> PollRelativePose() = 0; ... };
class MapObservationProvider { virtual std::vector<MapEvidence> PollMapEvidence() = 0; ... };
class SonarFrameProvider    { virtual std::optional<SonarFrame> PollSonarFrame() = 0; ... };
```
全部非阻塞/轮询式，方便调度器车道在不阻塞 adapter 线程的情况下抽干队列。具体实现
只存在于 `adapters/`，`core/`/`algorithms/` 只知道接口。

---

## 6. algorithms/ 层

### 6.1 `algorithms/frontends/sonar_cfar_frontend`

三个文件：`cfar_detector.{hpp,cpp}`、`dbscan.{hpp,cpp}`、
`sonar_cfar_frontend.{hpp,cpp}`。CMake 目标 `uw_sonar_cfar_frontend`。
`fixtures/` 目录**存在但是空的**——测试用的是构造出来的合成数据，不是 golden
fixture 文件。

**CFAR 检测器**（移植自 `sonar_camera_reconstruction` 的 `CFAR.py`+`cfar.cpp`）：
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
在基于 log-gamma 的 `GosocaCore` 函数上求根，**替代了上游的
`scipy.optimize.root` 多起点搜索**。`Detect()` 沿 range 轴逐 beam 滑窗，边界行
（首尾 `train_hs+guard_hs` 行）恒为 0；CA 对称求训练窗和，SOCA/GOCA 分前后段取
min/max，OS 用 `nth_element` 取排序统计量。

**DBSCAN**（`dbscan.hpp/cpp`）：**明确是原创重实现，不是移植**——教科书版
Ester et al. 1996，O(n²) 邻域查询，输入 `std::vector<Eigen::Vector2d>`。选择自实现
是因为上游 `cluster_scanline` 包了一层 sklearn，本仓库不想引入这个依赖。

**`SonarCfarFrontend`**（实现 `SonarFrontend`）：
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

**输出始终停留在声呐局部极坐标系（range,bearing），从不重映射到笛卡尔图像网格，
更不会烘焙进世界/`map` 坐标系**——这是相对上游 `imaging_sonar.py`/`merge.py` 的
刻意偏离（见 `NOTICE`）。

### 6.2 `algorithms/factor_builders/sonar_range_factor` —— 全仓库数学核心

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
符号翻转）计算，这**不等于**它自己那个残差公式的真实导数——直接照抄会引入错误。

有限差分测试（`sonar_range_residual_test.cpp`）：对位姿的 3 个平移分量各做
`±1e-6` 中心差分，与解析雅可比逐列比对（容差 `1e-5`），并且**精确断言**（`EXPECT_EQ`
而非 `NEAR`）朝向列恒为 0。

Builder：`kResidualModel = "sonar_range_v1"`；要求 evidence 携带
`SonarRangeBearing` 且 `context.nearby_points_W` 非空；
`sqrt_information = proposed_noise > 0 ? proposed_noise : 1.0`——builder 信任
已经被上游 cap 过的噪声值，不再自行判断（架构文档 8.4 节）。**v1 限制**：估计器
还不联合优化 3D 地标（图变量只有 keyframe 位姿），所以 `nearby_points_W` 目前由
外部提供（合成回放里来自 scenario 配置），不是来自实时地标库——这是
`submap_manager` 未来自然的扩展点。

### 6.3 `algorithms/factor_builders/relative_pose_factor`（原生，非移植）

6D 残差，两个 7 维位姿参数块 `[T_WBi, T_WBj]`：
```
predicted = T_WBi.Inverse() * T_WBj
translation_error = predicted.translation - measured.translation
rotation_error = measured_q.conjugate() * predicted.rotation
rotation_residual = 2 * rotation_error.vec()          // 小角度四元数误差近似
if rotation_error.w() < 0: rotation_residual = -rotation_residual   // 修正双覆盖符号翻转
residual = [sqrt_info_t * translation_error; sqrt_info_r * rotation_residual]
```
**雅可比用内部中心有限差分计算**（不是解析式）——对两个 7 参数块各扰动
`±1e-6`。头文件明确称这是"刻意的 v1 简化（构造即正确）"，等真正上 Ceres/GTSAM
后再换成闭式的最小 SO3 雅可比。Builder（`kResidualModel = "relative_pose_v1"`）
对平移/旋转两块**用同一个各向同性的标量** sqrt-information——注释关联到一个真实的
SVIn 审计发现（架构文档 22.4 节）：SVIn 的 `nav_msgs/Odometry` 没有可用的位姿
协方差，`LocalOdometryProvider` 包装层只能自己估一个噪声尺度，所以用单标量是
"诚实的 v1 选择"而不是伪造各向异性协方差。

### 6.4 `algorithms/factor_builders/depth_factor`（原生，非移植）

1D 残差，单个 7 维位姿参数块。世界系 Z 朝上，测得的深度（正值=水面以下）对应
位姿 Z 的负值：
```
residual = sqrt_information * (measured_depth_m + translation.z())
```
雅可比只有 tz 分量非零（`= sqrt_information`），线性关系，精确计算不需要有限差分。
**这就是 CLAUDE.md 里"z 轴 anchor bug"提到的那个因子**：一旦图里有深度因子，z 就
不再是 gauge freedom，固定/anchor keyframe 必须给自己真实的深度衍生 z，而不能
想当然地钉在 `Pose3::Identity()` 的 z=0（`apps/replay_demo` 的处理见
[第 9.2 节](#92-appsreplay_demo--端到端主流程)）。

### 6.5 `algorithms/estimation` —— Gauss-Newton/LM 求解器

**`GaussNewtonOptions`/`GaussNewtonSummary` 被放在 namespace scope**（不是嵌套在
`GaussNewtonSolver` 类里），专门规避 CLAUDE.md 记录的那个 GCC bug——嵌套聚合类型
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

**算法**：稠密 Eigen 实现的 Levenberg-Marquardt，直接在每个 keyframe 的原始 7 参数
块上操作——**不是严格的 6-DOF 切空间/流形更新**，每步接受后就地重归一化四元数
（`RenormalizeQuaternion`）。这是文档化的刻意 v1 简化（架构文档第 20 节）。

主循环（最多 `max_iterations=30` 次外层迭代）：
1. 线性化：遍历所有残差块绑定，累积稠密法方程 `JtJ`/`Jtr`（只对自由/非固定
   keyframe 的列有贡献，每个 7×7 分块由 `free_index` 索引）。
2. 内层阻尼重试（最多 `max_inner_retries=8` 次）：
   `damped(i,i) += lambda * max(damped(i,i), 1e-12)`（Marquardt 式对角缩放）；
   `delta = damped.ldlt().solve(-jtr)`——**稠密 LDLT（Cholesky）线性求解**，
   有意为 v1 问题规模（个位数到几百个 keyframe）设计；应用 delta 并重归一化
   四元数；重新算 cost；`trial_cost <= cost_at_linearization` 则接受
   （`lambda /= 3`），否则回滚 + `lambda *= 5` 重试。
3. 若内层重试全部失败：跳出外层循环，诚实报告"没收敛"（`converged=false`）。
4. 收敛判据：`|cost_at_linearization - current_cost| < 1e-12` → `converged=true`。

`PoseGraphProblem`：图变量**只有 keyframe 位姿，不联合优化 3D 地标**。
`AddKeyframe(id, initial_pose, fixed=false)`、`AddResidualBlock(block, involved_keyframes)`
（顺序必须匹配 `ResidualBlockSizes()`，未知 id 抛 `std::out_of_range`）、
`SetKeyframePose`/`GetKeyframePose`。`GaussNewtonSolver` 通过
`friend class GaussNewtonSolver` 直接访问 `PoseGraphProblem` 的私有 map，避免拷贝
参数数组。

`StateStore`：单写者/多读者的版本化快照环形缓冲（`std::deque<StateSnapshot>`，
默认容量 256），`Commit()` 分配单调递增的 `next_version_`。

集成测试 `ThreeKeyframeChainConvergesToTruth`：3-keyframe 链（`kf0` 固定于原点，
`kf1`/`kf2` 自由且初值有扰动），两个真实 `RelativePoseResidual` + 一个真实
`DepthResidual`，求解后 `final_cost < 1e-6`、平移误差 `< 1e-3`——证明
"FactorBuilder → ResidualBlock → PoseGraphProblem → 求解器" 这条链确实能拼起来
跑，而且用的是真实残差类型，不是 test double。

### 6.6 `algorithms/mapping/submap_manager`

**目录名叫"submap"，但实现粒度其实是按 keyframe**——没有距离/重叠/帧数触发的
"新建 submap" 逻辑。数据结构是
`std::unordered_map<keyframe_id, KeyframeMapState{pose_WB, evidence, stale}>`。

设计原则（对应架构 7.8/9/21 节）：`MapEvidence` 始终保存在**局部坐标系**并引用
源观测，**插入时绝不烘焙进全局位姿**——这是对 `sonar_camera_reconstruction`
`merge.py` 的刻意反模式（呼应 [4](#4-领域契约层schemasproto) 的 `map.proto`
注释）。世界系坐标是按需从 keyframe 当前已知位姿现算的。

- `AddMapEvidence(evidence)`：追加到对应 keyframe 的 evidence 列表。
- `UpdateKeyframePose(id, new_pose_WB)`：keyframe 位姿变化时调用（例如位姿图优化
  之后）。若该 keyframe 任一条 evidence 的 `reintegration_policy() ==
  FULL_REFUSE`，标记该 keyframe `stale=true`；`TRANSFORM_ONLY` 的 evidence 不受
  影响——因为 `WorldPointsForKeyframe` 每次调用都会用*当前*位姿重新变换，位姿
  修正会自动传播，不需要重跑前端。
- `WorldPointsForKeyframe(id)`：目前只解码 `POINT_CLOUD` 表示（其余类型返回空，
  "v1 未实现"），把 `geometry_or_occupancy` 重新解释为紧凑 `float[3]` 三元组，
  逐点应用 `pose_WB.Apply(local)`。

---

## 7. runtime/ 层

### 7.1 状态机 `state_machines.hpp`

架构文档 12.1 节的**三个正交状态机**——系统跟踪状态、单模态健康状态、建图节流
状态——用同一个滞回（hysteresis）模板实现：

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

`min_hold` 存在的目的是"避免权重在阈值附近震荡"（架构 12.1 节原话）。**建图状态
永远不能直接决定定位状态**这条规则，是靠三个类之间没有共享可变状态来保证的，不是
靠运行时检查——纯粹的类型/所有权层面的隔离。

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
[第 11 节](#11-配置系统-configs)）。每条车道的 overflow 策略不同——IMU 不能随便
丢，相机/建图可以丢旧/低价值项——所以策略是构造参数而不是硬编码。

> **代码 vs. CLAUDE.md 措辞的出入**：`BoundedQueue`/`Lane` 本身只是**队列原语和
> 车道枚举**，没有一个把四条车道real-time 调度、优先级抢占串起来的"调度器"类。
> `apps/replay_demo`/`apps/tools/synth_bag_gen` 目前都是单线程直接遍历 MCAP
> 消息，并不实际实例化 `BoundedQueue`/驱动四车道调度——这部分是运行时原语已经
> 就位、但尚未被任何 app 接线消费的一层（与 README"已知边界"里
> "`experiment` 层算法变体选择字段只读取未真正驱动分支"是同一类"骨架已搭、
> 未接线"的情况）。

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
  std::string estimator_mode = "black_box_vio";
  std::string map_backend = "submap_point_cloud_v1";
  bool write_run_manifest = true;
};

PlatformDefaultsConfig LoadPlatformDefaultsConfig(const std::string& path);
uw::domain::RigCalibrationSnapshot LoadRigConfig(const std::string& path);
ScenarioConfig LoadScenarioConfig(const std::string& path);
ExperimentConfig LoadExperimentConfig(const std::string& path);
```

**`rig` 层直接解析进 `RigCalibrationSnapshot` protobuf 消息**（不是另建一套
struct）——`LoadRigConfig` 里逐字段 `snapshot.mutable_xxx()->set_yyy(...)`，
保证"标定长什么样"只有一处定义。

**路径解析（`configs/experiment/*.yaml` 里的坑）**——`LoadExperimentConfig`
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
否则拼到 `base_dir` 后面。**这正是第一次实现时漏掌的一层 `parent_path()`**——
CLAUDE.md/README 都提到这个坑，这里是坑对应的确切代码。

`experiment` 层里 `frontends.sonar`/`estimator_mode`/`map_backend` 这三个"选算法
变体"字段目前**只被读取赋值给 `ExperimentConfig`，两个 app 都还是各自跑一条
写死的固定管线**，没有真正按这些字段分支。

**`warmup_seconds`**（`PlatformDefaultsConfig`）：一次运行最开始 N 秒内的
keyframe 只接受相对位姿（dead-reckoning）因子，不接受声呐 range/深度这类
"绝对参考"因子（0=禁用，即不做区分）。这个设计借鉴自一个姊妹 ROS2
SVIn+HoloOcean 部署（`workfiles_02` 的 `merge_node`）——VIO 的 IMU bias 还没
收敛前不能信绝对修正。本仓库没有在线 IMU 滤波器，所以类比实现是：warmup 窗口
内的 keyframe **仍然留在位姿图里**（仍会通过相对位姿因子被航位推算、仍会被
求解器优化），只是跳过声呐/深度因子的构建；`kf0` 不论 `warmup_seconds` 取值
如何，**始终**是固定 anchor——换成 warmup 窗口之后的某个 keyframe 做 anchor
需要知道它真实的 x/y/yaw，而这在图里根本不可观测，只有 `kf0` 因为
`synth_bag_gen` 把它放在世界系原点这个构造事实才能用 `Pose3::Identity()`
（具体应用见 [9.2 节](#92-appsreplay_demo--端到端主流程)）。

### 7.4 `RunManifest`

架构 14.2 节：每次运行产出一个**不可变** RunManifest；动态参数变化必须变成带时间戳
的事件，绝不能静默覆盖一份已写出的 manifest——调用方应把它当 write-once 对待。

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
通用转义器是不必要的复杂度。`apps/replay_demo` 里 `run_id` 具体是
`replay_demo_<unix秒>`，`simulator` 写死为
`"synthetic (apps/tools/synth_bag_gen)"`（见 [9.2](#92-appsreplay_demo--端到端主流程)）。

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
实现了 `CanonicalMcapWriter`，并且**同样强制 `CompressionType.NONE`**——因为 C++
构建禁用了 zstd/lz4（`MCAP_COMPRESSION_NO_ZSTD/LZ4`），压缩过的 Python bag 在
C++ 侧读不出来。这保证了 Python 写的 bag 能被 `replay_demo` 零转换直接消费。

`cmake/UwMcap.cmake`（见 [13 节](#13-构建系统)）里唯一 `#define MCAP_IMPLEMENTATION`
的翻译单元是 `cmake/mcap_impl.cpp`，任何调用 `mcap::McapWriter`/`McapReader`
的目标必须链接 `mcap_impl`（不是仅 `mcap`）。

---

## 8. adapters/ 层

### 8.1 `adapters/holoocean` —— Python 包 `uw_holoocean_adapter`

直连 HoloOcean Python API，取代 `ocean_t` 的脚本集合。

- **`coordinates.py`**：`Pose` dataclass（`translation:(3,)`,
  `quaternion_xyzw:(4,)`），有 `compose`/`inverse`/`apply`/`to_matrix4`/
  `identity()`，与 C++ 侧 `Pose3` 对齐。**修复了 `ocean_t/src/svin2_pipeline.py`
  里 `CoordTransformer._SE3_to_pose` 的一个真实 bug**：欧拉角万向锁分支
  （`cos(pitch) <= 1e-6` 时强制把 yaw 设为 0）。`matrix_to_quaternion` 用
  Shepperd 方法，穿过万向锁数值稳定。坐标约定：HoloOcean/UE 是左手 Z-up、单位
  cm；本仓库 body/world 系是右手 Z-up，两者关系 `T_ue_to_auv = diag(1,-1,-1)`
  ——和 `ocean_t` 的约定相同，只是重新实现、去掉了欧拉角分支。
- **`holoocean_driver.py`**：`HoloOceanSession`，`ocean_t/src/main.py` 的替代。
  惰性/带保护地导入 `holoocean` 包（缺失时抛出清晰的 `RuntimeError`——它是可选
  依赖，`pyproject.toml` 的 `holoocean` extra）。持有一个用显式 `seed` 一次性
  播种的 `numpy.random.Generator`（**不做全局 `np.random.seed()` 中途重新播种**
  ——这是修复 `ocean_t` 一个确定性 bug 的直接产物：`svin2_pipeline.py` 每帧不带
  参数调 `np.random.seed()`，破坏了 L2 回放确定性）。`apply_randomization()`
  是 `NotImplementedError`——明确未完成，等真机 HoloOcean 环境再补。**本机没有
  HoloOcean/Unreal 安装，`HoloOceanSession` 本身未被真实跑过**，包里其余部分都
  有测试覆盖。
- **`canonical_writer.py`**：见 [7.5 节](#75-mcap-io-封装-mcap_iohpp)。
- **`scenario_randomization.py`**：类型化、可采样的 `ScenarioRandomization`
  dataclass（嵌套 `VisualDegradation`/`SonarDegradation`/
  `TimingAndCalibrationDegradation`），取代 `ocean_t` 的 `water_control_panel.py`
  GUI（原来只有 2 个滑条/4 个硬编码预设，没法程序化驱动）。预设：
  `PRESET_CLEAR`/`PRESET_TURBID`/`PRESET_DEEP`/`PRESET_CRITICAL_DEGRADED`。
  `sample_uniform_sweep(rng, low, high)` 对每个数值字段递归均匀采样，同样接受
  显式 `rng`，与 driver 一样不用全局种子。
- **`time_utils.py`**：唯一生产 `Stamp`/clock-domain 值的地方，修复了另一个
  `ocean_t` 审计发现（`main.py` 和 `svin2_pipeline.py` 用两种互不一致的方式算
  时间戳，没有 capture/receive 区分）。`wall_clock_seconds()` 明确只用于
  receive time，从不用于估计。

依赖（`pyproject.toml`）：`protobuf>=4.21`、`mcap>=1.0`、`numpy>=1.24`，`pytest`
是 dev extra，`holoocean` 是可选 extra（保证没装 HoloOcean 的机器也能装/测其余
部分）。9 个测试覆盖 `coordinates`/`canonical_writer`/`scenario_randomization`，
`HoloOceanSession` 本身因缺少仿真环境未被测试覆盖。

### 8.2 `adapters/ros2` —— `UW_BUILD_ROS2` 开关保护

仓库里唯一允许出现 ROS2 头的地方。两个目标：

| 目标 | 状态 |
|---|---|
| `uw_ros2_svin_bridge`（`ros2_svin_odometry_bridge.hpp`） | 整个类体全部注释掉——一份文档化骨架，从未真正编译过 |
| `uw_holoocean_sonar_bridge_node`（`ros2_holoocean_sonar_bridge.hpp` + `holoocean_sonar_bridge_main.cpp`） | 真实代码，**已在本机对 ROS2 Jazzy + `holoocean_interfaces` 编译链接成功、启动无报错**，但**从未接过真实 `holoocean_main` 数据流**（需要 UE5/GPU，本机未做） |

`Ros2HoloOceanSonarBridge : public rclcpp::Node` 订阅参数 `input_topic`（默认
`/holoocean/auv0/ImagingSonar`，`SensorDataQoS`），消息类型
`holoocean_interfaces::msg::ImagingSonar`。每条消息回调里调用
`provider_.PushImagingSonar(msg.timestamp, msg.bins_range, msg.bins_azimuth,
msg.image_range, observation_id)`（合成的自增 `"holoocean_sonar_N"` observation
id），除此之外**什么都不做**（注释："这里没有估计/信号处理"）。`main()` 硬编码
`HoloOceanImagingSonarParams{horizontal_fov_rad=2.0943951(120°), min_range_m=0.5,
max_range_m=30.0}`（注释说这个值"匹配 holoocean_bridge 的默认场景 FOV"）。

编译门槛：`find_package(holoocean_interfaces REQUIRED)`——它**不在公共 ROS2 包
索引里**，必须从 `external_repos/holoocean-ros/holoocean_interfaces` colcon
build 到独立的 `~/ros2_ws`（symlink 而非拷贝），把它的 `install/` 加进
`CMAKE_PREFIX_PATH`。这个节点被明确称为"纯传输层"——下游（`SonarFrontend::
ProcessSonarFrame`）完全没有接线，`sonar_cfar_frontend` 目前只在
`replay_demo` 的 bag 回放路径里被用到，跟这条 ROS2 路径完全不相交。

### 8.3 `adapters/third_party/*` —— 三个 provider 适配器

统一模式：一个不依赖 ROS2/ROS1 的 C++ 类实现某个 `core/measurement_api` provider
接口，带一个显式 `Push*()` 注入点（不需要任何传输层就能单测），外面再套一层
真实的（或骨架的）ROS2 订阅接线。

**`holoocean_ros_bridge`**（真实、已测试）：
`HoloOceanRosBridgeSonarFrameProvider : SonarFrameProvider`。`PushImagingSonar()`
把一个展平的行主序 `[num_ranges,num_beams]`、值域 `[0,1]` 的 float32 强度图转成
`uw::domain::SonarFrame`——从 `HoloOceanImagingSonarParams`（fov/min/max）填充
`range_bins`/`azimuth_angles`（因为 `ImagingSonar.msg` 两者都不携带），**镜像每一行**
（`num_beams-1-c`）修正 HoloOcean 的列序与本平台升序方位角约定的差异
（`IsAzimuthAscending`），量化/裁剪到 `uint8`。尺寸不匹配或维度为 0 的畸形帧被
静默丢弃并计数。镜像翻转和字段映射细节（用 `image_range` 而非
`image_azimuth`；FOV/range 来自场景 JSON 而非消息本身）是读一个同事独立维护、
未 vendor 进本仓库的 `holoocean_bridge` 包（对 `sonar_oculus/OculusPing` 做同样
转换）反推出来的。

**`svin_bridge`**（真实、已测试注入点，里程计侧未对真实 SVIn 验证）：
`SvinBridgeLocalOdometryProvider : LocalOdometryProvider`，包装 SVIn 的
`okvis_odometry`（`nav_msgs/Odometry`）为黑盒相对位姿证据，自己估一个协方差代理
（因为 SVIn 不提供真实位姿协方差，架构 22.4 节审计发现）。只有注入点单测跑过；
ROS2 订阅侧（`ros2_svin_odometry_bridge.hpp`）完全注释，从未编译。

**`sonar_camera_reconstruction_baseline`**（纯 stub，诚实标注为不可用）：意图是
跑*未修改*的上游 `sonar_camera_reconstruction` ROS1 包作为外部对比 baseline，
从不链接进 `core`/`algorithms`。两个未解决的阻塞点：① 上游依赖未 vendor 的
`bruce_slam`，原仓库本身编不过；② 上游是 ROS1 Noetic，本机既没有 ROS1 也没有
`bruce_slam`。`run_baseline.sh` 是真实脚本骨架（有 `--bag`/`--out` 参数解析），
但**函数体就是三行 `echo "TODO: ..."` 加一行 `exit 1`**——不做任何真实工作。

### 8.4 `adapters/datasets/`

纯 stub，只有一个 `README.md`，说明意图：把公开数据集（EuRoC 风格、SVIn 自己的
公开数据集、RUSSO/Tank/SonarSweep）转换成 `adapters/holoocean` 和
`apps/tools/synth_bag_gen` 产出的同一套 canonical MCAP/protobuf schema，让
`replay_demo` 不关心 bag 来源。**"本轮未实现：还没有转换/测试过任何公开数据集"**。

---

## 9. apps/ 与 evaluation/

### 9.1 `apps/tools/synth_bag_gen` —— 合成数据生成

CLI：`--experiment <yaml>`（第一遍解析，通过 `ApplyScenarioConfig` 叠加
`ScenarioConfig`）→ `--out`/`--num-keyframes`/`--seed`（第二遍解析，覆盖
experiment 里的值——和 `replay_demo` 一样"CLI 参数最后生效"）。

`ScenarioOptions` 默认值：`num_keyframes=12, radius_m=8.0, arc_radians=1.4,
depth_m=12.0, relative_pose_noise_m=0.02, sonar_range_noise_m=0.03,
sonar_bearing_noise_rad=0.01, seed=42`；`sonar_targets_world` 默认空 →
`BuildSonarTargets` 退回到 3 个硬编码的类海底点。

**真值轨迹**（`BuildGroundTruthTrajectory`）：一段圆弧——`t∈[0,1]` 按
`num_keyframes` 插值，`theta = t*arc_radians`；位置
`= (radius·sinθ, radius·(1-cosθ), -depth_m)`（深度恒定，平面圆弧），朝向
`= AngleAxis(theta, UnitZ())`（只有 yaw，与弧线相切）。固定 5Hz 间隔：
`t_ns = i * 200_000_000`（每 keyframe 0.2s）。

**噪声模型**：`std::mt19937_64 rng(seed)`，三个独立的
`std::normal_distribution<double>`：`pose_noise(0, relative_pose_noise_m)`、
`range_noise(0, sonar_range_noise_m)`、`bearing_noise(0, sonar_bearing_noise_rad)`
——由 seed 完全确定，不做全局 RNG 重新播种（与 Python adapter 同样的纪律）。

**相对位姿证据**（`/evidence/relative_pose`，每个 `i>0`）：
`true_relative = trajectory[i-1].Inverse() * trajectory[i]`；给平移 x/y/z 各自
独立加 `N(0, relative_pose_noise_m)`（旋转不加噪，噪声只作用于平移）。

**声呐**（`/raw/sonar_frame`，每个在 12m 范围内的目标各产生一个合成 ping，
写的是真实像素而不是预烘焙的证据）：算 `local = trajectory[i].Inverse().Apply(target)`，
`range = |local|`（超过 12.0 跳过），`bearing = atan2(local.y, local.x)`，
range/bearing 各加噪声后调 `BuildSyntheticSonarFrame`：固定传感器几何
`num_ranges=600, num_beams=300, min_range=0, max_range=15m, horizontal_fov=6.0 rad`
（故意偏宽/不真实，只是用来练 `sonar_cfar_frontend`，不是标定过的设备模型），
背景强度 `5`，在量化后的 `(row,col)` bin 上画一个 3 列宽的强度 `200` 光斑（够宽
让 DBSCAN 的 `min_samples=2` 能聚出簇——单像素聚不成簇）。

**深度**（`/evidence/depth`，每个 keyframe）：
`PressureDepthMeasurement{depth_m = -pose.z, sigma_m = 0.05}`（`sigma_m` 只是
声明值，代码没有真的按它采样噪声）。

`/scenario/sonar_targets` 一次性写成 `MapEvidence`（`POINT_CLOUD`，紧凑 float32
xyz）。只依赖 `uw_domain`/`uw_sensor_models`/`uw_runtime`——不依赖
`algorithms/estimation`，纯数据合成。

### 9.2 `apps/replay_demo` —— 端到端主流程

CLI：`--bag <path>`（必填）、`--experiment <yaml>`（可选）、
`--out <prefix>`（默认 `/tmp/replay_demo`）、`--max-iterations N`
（CLI 覆盖 experiment，experiment 覆盖内建默认——三层覆盖链）。

`main()` 真实流程：

1. **加载配置**：给了 `--experiment` 就 `LoadExperimentConfig` → 拿到
   `PlatformDefaultsConfig`（求解器 max_iterations/initial_lambda、三种因子的
   sqrt-information 常数、`warmup_seconds`、`write_run_manifest`）。
2. **`ReadSonarTargets(bag)`**：读 `/scenario/sonar_targets`
   （`MapEvidence`，`geometry_or_occupancy()` 里的紧凑 float32 xyz）——这是对
   "真实地标/submap 查询"的替代品（v1 限制，文件头注释已标注）。
3. **预热窗口**：`warmup_keyframes = ceil(warmup_seconds / 0.2s)`
   （0.2s 是 `synth_bag_gen` 固定的 5Hz keyframe 间隔）；这些 keyframe 只获得
   相对位姿（航位推算）因子，被排除在声呐 range/深度这类"绝对参考"因子之外——
   对应"VIO bias 收敛前不融合绝对修正"这条工程经验。
4. **`kf0` anchor 的 z**：扫 `/evidence/depth`，取第一条
   `source_observations(0) == "kf0"` 的 `PressureDepthMeasurement`，
   `kf0_z = -depth_m`。**这就是 CLAUDE.md"已经踩过的坑"里那个 z 轴 anchor
   bug 的修复代码**——`kf0` 固定位姿的平移/旋转其余部分是
   `Pose3::Identity()`，但 z 用它自己真实的深度证据种下，而不是留在 0，因为一旦
   图里有深度因子，z 就不再是 gauge freedom。
5. `PoseGraphProblem problem`；`AddKeyframe("kf0", kf0_pose, fixed=true)`。
6. **相对位姿一遍**：读 `/evidence/relative_pose`；若 `from` keyframe 已存在，
   航位推算出 `to` 的初始猜测
   `problem.GetKeyframePose(from) * measured_relative`，
   `AddKeyframe(to, guess)`，用
   `RelativePoseFactorBuilder::Build(...)`（`proposed_noise =
   relative_pose_sqrt_info`）构建残差块，绑定 `{from, to}`。
7. **声呐一遍**：配置 `SonarCfarFrontend`
   （`num_training_cells=16, num_guard_cells=4, pfa=1e-2,
   detector_threshold=50`——与 `sonar_cfar_frontend_test` 的合成 fixture 参数
   一致）。读 `/raw/sonar_frame`，跳过图里不存在或在预热窗口内的 keyframe 对应
   的帧，调用 `sonar_frontend.ProcessSonarFrame(frame)`（**真实**跑一遍
   CFAR+极坐标+DBSCAN，不是预算好的证据）→ `HypothesisSet`，只用
   `candidates(0)`（top-1，按 `hypothesis.proto` 的 v1 规则）。数据关联：对
   `targets` 线性搜索，取哪个 scenario 目标从当前位姿估计出发算出的 range
   离测得的 range 最近（文档明确标注这是合成 demo 的替代做法，不是通用算法）。
   用 `FactorBuildContext{nearby_points_W = {best_target}}` 构建
   `SonarRangeFactorBuilder` 残差块。
8. **深度一遍**：读 `/evidence/depth`，跳过预热窗口，构建
   `DepthFactorBuilder`（`proposed_noise = depth_sqrt_info`）。
9. **求解**：`GaussNewtonSolver::Solve(problem, {max_iterations,
   initial_lambda})`，打印迭代次数/初始与最终 cost/是否收敛。
10. **状态/地图接线**：遍历 `problem.KeyframeOrder()`，逐个提交
    `StateSnapshot` 到 `StateStore`，调用
    `submap_manager.UpdateKeyframePose(kf_id, pose)`，把
    `{timestamp_s = i*0.2, pose}` 追加进 `estimated_trajectory`。
11. **真值**：读 `/gt/state`（`StateSnapshot`）进 `ground_truth_trajectory`，
    时间戳取自 `capture_timestamp`。
12. **评测**：`uw::evaluation::ComputeAte(estimated, ground_truth)`，打印
    rmse/mean/max/匹配数。
13. **输出**：写 `<out_prefix>_trajectory.tum`（TUM 格式：
    `timestamp tx ty tz qx qy qz qw`），除非配置里 `write_run_manifest=false`，
    否则再写 `<out_prefix>_run_manifest.json`
    （`run_id = replay_demo_<unix秒>`，`dataset_or_scenario = bag路径`，
    `simulator = "synthetic (apps/tools/synth_bag_gen)"`）。

文件头注释里明确列出的 v1 限制：没有真实的可靠性调度器（sqrt-information
常数是固定值，不是标定出来的）；没有真实地标/submap 查询；只消费 top-1 声呐假设；
分层配置驱动求解器/噪声参数，但不驱动 frontend/factor_builder/map_backend 的
*选择*（管线写死为 `relative_pose_v1 + sonar_range_v1 + depth_v1`）。

链接：`uw_domain, uw_sensor_models, uw_runtime, uw_estimation, uw_evaluation,
uw_relative_pose_factor, uw_sonar_range_factor, uw_depth_factor,
uw_submap_manager, uw_sonar_cfar_frontend`。

### 9.3 `evaluation/` —— 轨迹指标

**只实现了 ATE，仓库里没有任何 RPE 代码**（grep 过 `Rpe/RPE/relative_pose_error`
均无命中）。

```cpp
struct TrajectoryPose { double timestamp_s; Pose3 pose_WB; };
struct AteResult { double rmse_m, mean_m, max_m; int num_matched_poses; };
AteResult ComputeAte(const std::vector<TrajectoryPose>& estimated,
                     const std::vector<TrajectoryPose>& ground_truth,
                     double max_time_diff_s = 0.05);
```
实现：对每个估计位姿，按 `|时间戳差|` 线性扫描最近邻匹配真值（v1 对小规模合成
场景够用），超过 `max_time_diff_s` 未匹配则跳过。**每次匹配的误差只是平移
欧氏距离**（`(est.translation - gt.translation).norm()`）——**完全不计算旋转
误差**。累积 `rmse_m = sqrt(Σerr²/matched)`、`mean_m`、`max_m`。

文件头明确标注的 v1 限制：比较前**没有做 Umeyama SE3/Sim3 对齐**——假定估计
轨迹和真值轨迹已经共享同一个世界系/尺度，这对合成场景成立（两者本来就在同一个
世界系里造出来的），但拿真实 VIO/SLAM 输出去对另外测绘的真值时就不成立了，标注
为评测真实数据前的自然后续工作。

---

## 10. 端到端运行时序

这是把 [9.1](#91-appstoolssynth_bag_gen--合成数据生成) 和
[9.2](#92-appsreplay_demo--端到端主流程) 串成一条时间线，也是目前仓库里唯一真实
跑通的完整数据流：

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
  ├─ ReadSonarTargets(bag)  ← /scenario/sonar_targets
  ├─ kf0 anchor：从 /evidence/depth 找 kf0 自己的深度，种 kf0 的 z
  ├─ PoseGraphProblem：AddKeyframe("kf0", fixed=true)
  ├─ 相对位姿一遍：/evidence/relative_pose → 航位推算初值 → RelativePoseFactorBuilder
  ├─ 声呐一遍：/raw/sonar_frame → SonarCfarFrontend::ProcessSonarFrame（真跑 CFAR+DBSCAN）
  │              → top-1 假设 → 最近目标数据关联 → SonarRangeFactorBuilder
  ├─ 深度一遍：/evidence/depth → DepthFactorBuilder
  ├─ GaussNewtonSolver::Solve（LM，稠密 LDLT，≤30 次迭代）
  ├─ 逐 keyframe：StateStore::Commit + submap_manager.UpdateKeyframePose
  ├─ /gt/state → ground_truth_trajectory
  ├─ ComputeAte(estimated, ground_truth)  → rmse/mean/max
  └─ 写 demo_trajectory.tum（TUM 格式）+ demo_run_manifest.json（RunManifest）
```

典型结果：6 次迭代内收敛，ATE rmse ~3cm（合成噪声量级本就是厘米级）。
`tests/l2_replay/determinism_test.sh` 就是把这整条流程跑两遍、diff
`_trajectory.tum`，验证其中没有藏着全局可变随机状态（见
[第 12 节](#12-测试体系-tests)）。

---

## 11. 配置系统 configs/

四层，每层一个 YAML 文件，`--experiment <path>` 驱动 `LoadExperimentConfig` 一次
性加载全部（详细的路径解析机制见 [7.3 节](#73-分层配置加载-confighpp--configcpp)）。

**`configs/defaults/platform.yaml`**（平台级默认值，不含任何具体机体/场景信息）：
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
```
（`runtime.lanes` 描述的是 [7.2 节](#72-四车道有界队列原语-bounded_queuehpp)
`BoundedQueue`/`Lane` 该怎么配置；目前没有任何 app 真正读取这一段去实例化队列
——文件里写着，代码里还没接线消费。）

**`configs/rig/example_auv.yaml`**（标定唯一事实源，对应 `RigCalibrationSnapshot`；
`replay_demo`/`synth_bag_gen` v1 都还不消费这一层，只加载不使用）：
```yaml
calibration_version: "example_auv_v1"
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

**`configs/scenario/synthetic_smoke.yaml`**（"跑什么数据"，完整接入
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

**`configs/experiment/synthetic_smoke.yaml`**（"怎么跑"；`rig`/`scenario`/
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

四层消费程度总结（README 已有，这里复述一遍方便对照代码）：`defaults` 完整接入
`replay_demo`；`rig` 完整加载但两个 app 都还不用；`scenario` 完整接入
`synth_bag_gen`；`experiment` 里选算法变体的字段只读取打印，不驱动分支。

---

## 12. 测试体系 tests/

三层，对应架构文档验收面设计。**只有跨模块的 L0/L2 测试放在 `tests/` 下**，L1
单元测试就近放在各自模块自己的 `test/` 目录（已经在第 5/6/7 节逐个列过）。

### L0 —— `tests/l0_contracts/domain_contract_test.cpp`

只链接 `uw_domain` + gtest，测的是 protobuf round-trip 和几个域不变量助手：
- `ObservationHeaderRoundTrips`：序列化/解析一个 `ObservationHeader`，字段相等
  性在 round-trip 后保持。
- `SonarFrameAscendingAzimuthAccepted`/`...Rejected`：分别用升序/非升序方位角
  数组测 `IsAzimuthAscending`。
- `MeasurementEvidencePayloadRoundTripsThroughOneof`：构造一个
  `SonarRangeBearing`，经 `MakeEvidence<>()` 包进 `MeasurementEvidence`，
  round-trip 后 `HasPayload<SonarRangeBearing>` 为真、
  `HasPayload<PressureDepthMeasurement>` 为假。

### L2 —— `tests/l2_replay/determinism_test.sh`

对真实二进制（不是 mock）跑两遍：
```bash
"$SYNTH_BAG_GEN" --out "$WORKDIR/scenario.mcap" --seed 7
"$REPLAY_DEMO" --bag "$WORKDIR/scenario.mcap" --out "$WORKDIR/run1" >/dev/null
"$REPLAY_DEMO" --bag "$WORKDIR/scenario.mcap" --out "$WORKDIR/run2" >/dev/null
diff -q "$WORKDIR/run1_trajectory.tum" "$WORKDIR/run2_trajectory.tum"
```
"逐字节一致"的意思是：同一个 bag/config/seed，跑两次 `replay_demo`，两份
`_trajectory.tum` 必须 diff 干净。这是"没有藏着全局可变随机状态"的直接验证——
求解器、RNG 使用、任何 map/hash 迭代顺序里的不确定性一旦泄漏进轨迹输出，这个
测试就会挂，这也是 CLAUDE.md 强调"不要用全局 `np.random.seed()`"这条纪律的实际
把关机制。

### L1（分布在各模块自己的 test/ 下）

已在第 5/6/7 节列出，汇总一份速查表：

| 模块 | 测试焦点 |
|---|---|
| `sonar_range_factor` | 有限差分验证解析雅可比；朝向列精确为 0 |
| `relative_pose_factor` | 残差在真值处为零；有限差分雅可比非零 |
| `depth_factor` | 残差公式与雅可比模式 |
| `estimation` | 三 keyframe 链收敛到真值（用真实 factor_builders，不是 test double） |
| `sonar_cfar_frontend` | 合成声呐图上的 CFAR+DBSCAN（构造数据，非 fixture 文件） |
| `submap_manager` | `TRANSFORM_ONLY` vs `FULL_REFUSE` 的 stale 行为 |
| `runtime/config` | 直接读真实 `configs/experiment/synthetic_smoke.yaml` 逐字段断言 |
| `runtime/mcap_io` | protobuf 消息经 MCAP 写入/读回的 round-trip |
| `evaluation` | 零误差自比较；已知 1m 偏移轨迹对 rmse 精确为 1.0 |

```bash
ctest --test-dir build --output-on-failure   # C++：13/13
cd adapters/holoocean && pytest               # Python：9/9
tools/lint/check_no_ros_in_core.sh            # 依赖不变量
```

---

## 13. 构建系统

顶层 `CMakeLists.txt`：C++17，默认 `RelWithDebInfo`，
`CMAKE_EXPORT_COMPILE_COMMANDS ON`。两个 option：`UW_BUILD_ROS2`（默认 OFF，
需要 sourced ROS2 环境）、`UW_BUILD_TESTS`（默认 ON）。依赖：
`find_package(Eigen3 REQUIRED NO_MODULE)`、`find_package(yaml-cpp REQUIRED)`，
`include(UwProtobuf)`/`include(UwMcap)` 各自产出对应目标。子目录按严格 DAG 顺序
`add_subdirectory`：`core/domain → core/sensor_models → core/measurement_api →
factor_builders(*) → frontends/sonar_cfar_frontend → estimation →
mapping/submap_manager → runtime → evaluation →`（若 `UW_BUILD_ROS2`）
`adapters/ros2 →` `adapters/third_party/{svin_bridge,holoocean_ros_bridge}`
（无条件）`→ apps/tools/synth_bag_gen, apps/replay_demo →`（若
`UW_BUILD_TESTS`）`tests`。

**`cmake/UwProtobuf.cmake`**：`find_package(Protobuf REQUIRED)` +
`find_package(absl CONFIG REQUIRED)`。glob（`CONFIGURE_DEPENDS`，新增 .proto
自动生效）`schemas/proto/uw/domain/*.proto`，`protobuf_generate(LANGUAGE cpp ...
PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}/generated)`，构建 `uw_domain_proto`
STATIC 库。**显式链接一整组 absl 组件**
（`flat_hash_map/hash/strings/status/statusor/synchronization/time/base/log/cord`）
——工作绕开 `protobuf::libprotobuf` 的 `INTERFACE_LINK_LIBRARIES` 在
conda-forge 工具链多层静态库传递时不可靠的问题（"DSO missing from command
line"，CLAUDE.md 记录的那个坑）。生成代码的警告靠 `COMPILE_OPTIONS "-w"` 抑制。

**`cmake/UwMcap.cmake`**：MCAP C++ SDK 是 header-only、单 TU 实现模式
（`#ifdef MCAP_IMPLEMENTATION`），没有自己的 CMakeLists.txt（上游假定用
Bazel/Conan/vendoring），所以用 `FetchContent_Declare` + `FetchContent_Populate`
（不是 `FetchContent_MakeAvailable`，因为没有子目录可 `add_subdirectory`），配
`cmake_policy(SET CMP0169 OLD)` 保住这种经典用法在新版 CMake 上还能跑。手动搭
两个目标：`mcap`（INTERFACE，含 `MCAP_COMPRESSION_NO_ZSTD`/`_NO_LZ4` 编译宏——
v1 写不压缩的 chunk，避免引入 zstd/lz4）和 `mcap_impl`（STATIC，编译
`cmake/mcap_impl.cpp`，唯一 `#define MCAP_IMPLEMENTATION` 的翻译单元，注释警告
绝不能在别的文件重复定义）。

---

## 14. 工具链 tools/

**`tools/lint/check_no_ros_in_core.sh`**：解析出脚本自身所在的仓库根目录，
对 `core/` 和 `algorithms/` 下的 `.hpp/.cpp/.h/.cc` grep 禁止的 include 模式：
```bash
FORBIDDEN_PATTERNS=(
  '#include\s*[<"](rclcpp|ros|rmw)/'
  '#include\s*[<"]holoocean'
  '#include\s*[<"]okvis/'
  '#include\s*[<"]sonar_oculus'
)
check_dir "core" "${FORBIDDEN_PATTERNS[@]}"
check_dir "algorithms" "${FORBIDDEN_PATTERNS[@]}"
```
命中任一模式即 `FAIL=1`，退出码 1 并打印违规文件。只检查 `core/`/`algorithms/`
——`runtime/`/`adapters/` 不检查，正好匹配"依赖单向"规则里 ROS 头本该出现的地方。

**`tools/codegen/gen_py.sh`**：唯一的 codegen 脚本，`protoc -I ...
--python_out=...` 把 `schemas/proto/uw/domain/*.proto` 重新生成到
`adapters/holoocean/uw_holoocean_adapter/schema_pb2/`。生成的 `*_pb2.py` 是
gitignored 的——这是开发环境搭建的便捷步骤，不是检查进版本控制的产物，保持
`.proto` 单一事实源。`protoc` 不在 PATH 上时快速失败并给出清晰提示。

**`tools/setup_dev_env.sh`**：两段回退。① `try_apt()`：
`timeout 60 sudo apt-get update -qq && timeout 300 sudo apt-get install -y -qq
protobuf-compiler libprotobuf-dev libeigen3-dev libgtest-dev cmake
build-essential`——60s 超时专门用来探测卡住/被限速的镜像（对应 CLAUDE.md 记录的
沙箱环境 HTTP(80) apt 镜像卡死、HTTPS(443) 正常的问题）。② 只有 apt 失败才走
`try_conda()`：建/复用一个 `uw_slam_build` conda-forge 环境
（`eigen libprotobuf protobuf gtest cmake`），打印出确切的
`PATH`/`cmake -DCMAKE_PREFIX_PATH` 调用方式——这正是本仓库在沙箱开发环境里实际
被编译/测试所走的那条路径。

---

## 15. 已知边界

（与 README.md「已知边界」一致，这里从代码事实的角度复述一遍）

- `adapters/ros2/uw_holoocean_sonar_bridge_node` 真实编译+启动验证过，但没有接过
  真实 `holoocean_main` 进程（需要 UE5+Epic EULA），也没有接到
  `SonarFrontend`/`replay_demo` 下游——它是纯传输层。`uw_ros2_svin_bridge` 是全
  注释骨架，从未编译。
- `--experiment` 目前完整接入 `defaults`/`scenario`（分别驱动求解器参数和
  `synth_bag_gen`），`rig` 完整加载但两个 app 都不消费，`experiment` 层里
  `frontends`/`estimator_mode`/`map_backend` 只读取打印，不驱动分支——两个 app
  各自只有一条写死的固定管线。
- `runtime/bounded_queue.hpp`/`state_machines.hpp` 的原语已经实现，但没有任何
  app 真正实例化四车道队列或驱动状态机转换——这层运行时基础设施是"已搭、未接线"。
- 求解器是 Eigen 手写 LM，不是 Ceres/GTSAM（架构第 20 节延后决策），且直接在原始
  7 参数块上做加法更新+事后重归一化，不是严格的流形更新。
- 位姿图只优化 keyframe 变量，不联合优化路标点；`nearby_points_W` 目前由外部
  （合成 scenario 或未来的 submap_manager 查询）提供。`sonar_range_factor` 的
  数据关联用"按当前航迹推算位姿选最近已知目标"，只在合成数据下成立。
- `algorithms/mapping/submap_manager` 尽管叫"submap"，实现粒度是按 keyframe，
  没有距离/重叠/帧数触发的 submap 边界逻辑。
- `evaluation/` 只有 ATE（平移 RMSE/mean/max），没有 RPE，也没有 Umeyama
  SE3/Sim3 对齐——假定估计轨迹和真值已经共享同一世界系/尺度，只对合成场景成立。
- `sonar_camera_reconstruction_baseline` 是纯 stub（`run_baseline.sh` 函数体是
  TODO+`exit 1`），因为上游依赖未 vendor 的 `bruce_slam` 且需要 ROS1 Noetic。
