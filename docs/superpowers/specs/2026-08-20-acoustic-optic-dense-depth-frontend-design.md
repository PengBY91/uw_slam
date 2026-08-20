---
title: 几何优先的声光 Dense Depth 前端规格
created: 2026-08-20
updated: 2026-08-20
type: engineering-design
status: approved-design
implementation_status: not-started
codebase_reference: 725e718
---

# 几何优先的声光 Dense Depth 前端规格

## 1. 文档定位

本文定义 `uw_slam` 首个可实现的声光前端融合 MVP：在小范围静态仿真场景中，融合
双目相机的度量深度先验与 2D forward-looking sonar（FLS）的 range-bearing 观测，
输出相机局部坐标系下带不确定性的增强深度和局部表面证据。

本文是目标规格，不代表所述模块已经实现。当前代码事实以
[代码库参考](../../uw-slam-codebase-reference-2026-08-18.md)、源码和测试为准；长期平台
边界以[声光 SLAM 平台架构](../../acoustic-optic-slam-platform-architecture-2026-08-17.md)
为准。

## 2. 决策摘要

- MVP 使用双目 camera + 2D FLS，输出增强 dense depth / 局部点云。
- 首版只做几何融合；语义分割、语义地标和学习式 fusion 不进入 MVP。
- IMU 不是主链路依赖，但 schema、录制和后续运动补偿接口不得封死 IMU 扩展。
- 前端输出 keyframe-local evidence，不维护 world-frame 全局地图。
- 采用几何优先的概率融合：保留 FLS elevation ambiguity，以多候选关联和 posterior
  depth update 取代最近邻深度覆盖。
- optical frontend 使用通用 `OpticalDepthPrior` 接口；MVP 实现双目，允许后续替换为
  metric monocular、relative-depth monocular 或 multi-view frontend。
- MVP 只消费 `METRIC` optical prior；单目相对尺度与 sonar scale alignment 属于后续
 决策门。
- 首版以 HoloOcean/合成小范围静态场景验收，并以 stereo-only 消融为主要基线。

## 3. 目标与非目标

### 3.1 目标

1. 为双目图像建立 canonical raw observation contract，并与现有 `SonarFrame` 使用相同
   `ObservationHeader` 时间、frame、标定版本和 provenance 语义。
2. 输出逐像素度量 depth、variance 和 valid mask 的 optical prior。
3. 将 FLS range-bearing-elevation-aperture 测量投影为相机图像中的候选弧带，不把检测
   解释成唯一 3D 点。
4. 通过时间、标定、几何、可见性和不确定性 gate 建立可审计的声光 association。
5. 对通过 gate 的候选执行 robust posterior depth update，输出 depth、variance、valid
   mask、contribution mask 和完整来源。
6. 在视觉退化且有 sonar 覆盖的区域，相对 optical-only baseline 明确降低 depth error。
7. live/replay 使用同一算法接口，相同 bag/config/calibration 的结果确定性一致。
8. 保持 optical depth source 可替换，后续接入单目时不重写声呐投影、关联和融合核心。

### 3.2 非目标

- 语义分割、语义 landmark、动态物体语义治理；
- 学习式 cross-modal feature fusion 或端到端 dense-depth 网络；
- 依赖 IMU 的帧间运动补偿、视觉惯性初始化或完整 6DoF VIO；
- 单帧 2D FLS 独立填满所有 stereo 无效像素；
- 从 2D FLS 伪造唯一 elevation；
- 全局地图、回环、全局 pose graph 或后端求解器改造；
- 在线外参/time-offset 自标定；
- 真实水池数据作为首版完成条件；
- 单目 relative-scale alignment 的实际算法实现。

## 4. 当前实现与差距

### 4.1 已有能力

- `schemas/proto/uw/domain/sonar.proto` 定义完整 `SonarFrame`，包含强度张量、range bins、
  azimuth、elevation aperture、gain 与 sound-speed assumption。
- `SonarCfarFrontend` 已实现 CFAR、first-contact、极坐标转换和 DBSCAN，输出
  `HypothesisSet<SonarRangeBearing>`。
- `ObservationHeader` 已统一 capture/receive time、clock domain、sensor frame、
  calibration version 和 provenance。
- `RigCalibrationSnapshot` 已预留 camera intrinsics、frame tree 和 time offset。
- `MeasurementEvidence` 已提供 source observations、noise suggestion、quality、valid domain、
  algorithm/model version 和 typed payload envelope。
- MCAP C++/Python 读写、确定性回放、HoloOcean 网关骨架和轨迹评测已经存在。
- `SubmapManager` 与 `MapEvidence` 已坚持局部证据 + 版本化位姿，而不是在前端固化 world
  frame。

### 4.2 缺失或不完整能力

| 层面 | 当前状态 | MVP 所需变化 |
|---|---|---|
| Raw camera contract | 没有 `ImageFrame` | 新增图像尺寸、编码、stride、pixels 和 header |
| Camera calibration | proto 有 intrinsics | rig 增加双目内参、base-camera/sonar 外参和 time offset |
| Providers | 只有 sonar provider | 增加 camera provider 和相机帧组抽象 |
| Synchronization | MCAP topic 分别读取 | 按校正后的 capture time 生成声光 bundle |
| Optical frontend | 不存在 | 增加可替换 `OpticalDepthFrontend` 和双目实现 |
| Optical evidence | `StereoDepthMeasurement` 只是占位数组 | 增加通用、带 scale semantics 的逐像素 prior |
| Cross-modal geometry | 不存在 | 增加 FLS arc-band projector 和 candidate gate |
| Association | `replay_demo` 中用 z=0 点做最近邻 | 改成多候选、可见性和概率一致性审计 |
| Fusion | 不存在 | 增加 scalar posterior depth optimizer 和 fallback |
| Fused evidence | 不存在 | 增加 fused depth、variance、masks 和 association records |
| Mapping bridge | 只有通用 bytes | 明确 depth-to-local-surface 转换与来源语义 |
| Synthetic data | 无 camera image/depth GT | 生成同步 stereo、sonar、GT depth 和退化场景 |
| Evaluation | 只有 trajectory ATE/RPE | 增加 depth、coverage、false-fusion、NLL 和 latency |

现有 `replay_demo` 为声呐数据关联构造 `z=0` 的局部点。这一做法只适合当前 range-factor
垂直切片的已记录限制，不得复用于 dense-depth 融合，因为它会把 FLS 不可观测的
elevation 固定为虚假值。

## 5. 架构与模块边界

### 5.1 在线数据流

```text
Left ImageFrame ─┐
Right ImageFrame ┼─→ AcousticOpticSynchronizer ─┐
SonarFrame ──────┘                               │
                                                ├─→ OpticalDepthFrontend
                                                │     → OpticalDepthPrior
                                                │
                                                └─→ SonarCfarFrontend
                                                      → Sonar hypotheses

OpticalDepthPrior + Sonar hypotheses + RigCalibrationSnapshot
  → AcousticOpticDepthFusionFrontend
  → FusedDepthMeasurement + AssociationAudit
  → local surface samples / evaluation
```

`AcousticOpticSynchronizer` 使用校正后的 capture time，而非 receive time。MVP 对超过配置
容差的 bundle 直接拒绝，不在没有运动估计的情况下外推或插值。

时间偏移统一定义为 `t_reference = t_sensor_capture + time_offset_seconds[sensor_id]`。adapter
不得自行采用相反符号；缺少对应 offset 时只允许使用显式配置的零偏移，并在 manifest
和 health 中记录该假设。

### 5.2 代码依赖

新增模块继续遵守 `core → algorithms → runtime → adapters → apps`：

- `schemas/proto/uw/domain/`：wire contract 的唯一事实源；
- `core/measurement_api/`：camera provider、optical frontend、fusion frontend 的抽象接口；
- `core/sensor_models/`：camera projection、FLS arc geometry 和 frame transform；
- `algorithms/frontends/`：stereo depth、sonar projection/association、概率融合的具体实现；
- `runtime/`：有界队列、capture-time synchronizer、配置和 replay 编排；
- `adapters/`：HoloOcean/ROS2/vendor image 转 canonical `ImageFrame`；
- `apps/`：生成合成数据、运行 acoustic-optic replay 和输出实验产物；
- `evaluation/`：depth 与 fusion 专用指标。

算法模块不得 include ROS2、HoloOcean 或 vendor image message。同步器不包含 stereo 或
sonar 算法；它只负责数据分组、容差和丢弃策略。

### 5.3 所有权

- Optical frontend 拥有 optical prior 及其 uncertainty 建议，不拥有全局 pose/map。
- Sonar frontend 拥有检测和多假设，不拥有 elevation 或全局 landmark。
- Fusion frontend 拥有 cross-modal association 与 posterior depth，不拥有 world pose。
- Mapping 消费 local evidence 和版本化 state，负责变换、重定位或重融合。
- Reliability/estimation 层最终拥有 information cap；学习 confidence 将来不能直接变成
  后端 information matrix。

## 6. 数据契约

### 6.1 `ImageFrame`

新增 `schemas/proto/uw/domain/image.proto`：

```text
ImageFrame {
  ObservationHeader header
  uint32 width
  uint32 height
  uint32 row_stride_bytes
  ImageEncoding encoding       // MONO8, RGB8, BGR8; MVP 至少支持 MONO8
  bytes pixel_data
  bool is_rectified
  double exposure_seconds
}
```

适配器必须验证 `pixel_data` 与 height/stride 一致。左右目各自拥有 sensor ID、frame 和
observation ID，不通过 topic 名隐式推断左右关系。

### 6.2 `CameraFrameBundle`

`CameraFrameBundle` 是 core API 的进程内值类型，不作为新的录包消息：

```text
CameraFrameBundle {
  ImageFrame primary
  optional<ImageFrame> secondary
}
```

canonical bag 保留原始独立 `ImageFrame`；runtime 根据配置和 capture time 重建 bundle。
这样可以用同一 bag 调整同步容差，并避免永久记录某次运行的配对决定。

### 6.3 `OpticalDepthPriorMeasurement`

新增通用 optical prior payload；实现后迁移并废弃当前只占位的
`StereoDepthMeasurement`，不在 C++/Python 侧另建平行 wire struct：

```text
OpticalDepthPriorMeasurement {
  FrameId reference_camera_frame
  uint32 width
  uint32 height
  repeated float depth_m               // row-major, width*height
  repeated float variance_m2           // row-major, width*height
  bytes valid_mask                      // one byte per pixel
  ScaleStatus scale_status              // METRIC, RELATIVE_SCALE, UNOBSERVED_SCALE
  string producer_type                  // stereo, monocular_metric, monocular_multiview
}
```

MVP 只允许 `METRIC` 进入 fusion。`RELATIVE_SCALE` 被记录但拒绝融合，作为未来 sonar
scale alignment 的输入；`UNOBSERVED_SCALE` 直接拒绝。

无效像素的 depth/variance 数值不具有语义，消费者必须先检查 `valid_mask`。有效像素的
depth 必须有限且大于零，variance 必须有限且大于零。

### 6.4 `FusedDepthMeasurement`

```text
FusedDepthMeasurement {
  FrameId reference_camera_frame
  uint32 width
  uint32 height
  repeated float depth_m
  repeated float variance_m2
  bytes valid_mask
  bytes contribution_mask       // OPTICAL_ONLY, ACOUSTIC_OPTIC, INVALID
  repeated AcousticOpticAssociationRecord associations
}
```

输出使用 primary camera local frame。`MeasurementEvidence.source_observations` 必须包含
primary/secondary image 和实际参与更新的 sonar observation；未参与更新的邻近 sonar
frame 不得列为贡献来源。

### 6.5 `AcousticOpticAssociationRecord`

每个 sonar hypothesis 至少记录：

```text
AcousticOpticAssociationRecord {
  EvidenceId sonar_evidence_id
  AssociationStatus status      // ACCEPTED, AMBIGUOUS, CONFLICT, REJECTED
  repeated uint32 candidate_pixel_indices
  uint32 selected_pixel_index
  double best_score
  double second_best_score
  double time_delta_seconds
  double prior_depth_m
  double posterior_depth_m
  double prior_variance_m2
  double posterior_variance_m2
  string reason_code
}
```

非 `ACCEPTED` 记录中的 selected pixel 不具有语义。reason code 使用稳定枚举或受控字符串，
不能把自由文本作为控制逻辑。

### 6.6 Calibration 与配置

`RigCalibrationSnapshot`/rig YAML 必须包含：

- left/right camera intrinsics 与 distortion model；
- `base_link → camera_left`、`base_link → camera_right`、`base_link → sonar_link`；
- camera 与 sonar 的 time offsets；
- FLS horizontal FOV、elevation aperture、range resolution 和 sound speed assumption；
- calibration version，且所有输入 observation 必须引用同一兼容版本。

experiment/defaults 配置至少提供：

- primary/secondary camera sensor IDs；
- secondary camera 是否必需；
- 最大声光 capture-time delta；
- stereo disparity/depth 有效范围；
- range、bearing、reprojection 与 visibility gates；
- best/second-best association margin；
- robust loss 类型与尺度；
- posterior variance 改善下限；
- 最大每帧 sonar hypotheses/candidates；
- P95 latency budget。

默认配置面向双目。单目配置可省略 secondary camera，但 MVP 中没有可用的单目 frontend
实现，配置校验必须明确拒绝选择不存在的实现。

## 7. 抽象接口与单目扩展

### 7.1 Optical frontend

```text
OpticalDepthFrontend::Process(
  CameraFrameBundle,
  RigCalibrationSnapshot
) -> optional<MeasurementEvidence<OpticalDepthPriorMeasurement>>
```

MVP 实现 `StereoOpticalDepthFrontend`。融合模块只消费通用 optical prior，不依赖 disparity、
baseline 或 stereo 实现细节。

### 7.2 Fusion frontend

```text
AcousticOpticDepthFusionFrontend::Fuse(
  MeasurementEvidence<OpticalDepthPriorMeasurement>,
  HypothesisSet<SonarRangeBearing>,
  RigCalibrationSnapshot
) -> FusedDepthResult
```

`FusedDepthResult` 是进程内返回类型，包含 fused evidence、association audit 和 health delta；
可持久化内容必须落入 protobuf，而不是只存在于 C++ struct。

### 7.3 单目扩展成本

| 单目路线 | 预计改动 | Fusion 核心变化 |
|---|---|---|
| Metric monocular depth model | 新 frontend、模型/OOD/uncertainty calibration | 无 |
| Relative-depth monocular model | 新 frontend + sonar scale alignment | optical prior 接受新 scale gate |
| Monocular multi-view geometry | tracking/window/motion/scale initialization | 若输出 metric prior则无 |
| Raw monocular + FLS joint estimator | 新的联合状态与估计问题 | 超出本 MVP，不能视为 frontend 替换 |

单张单目图像 + 单帧 2D FLS 一般不能恢复唯一 dense 3D：单目缺 metric scale，FLS 缺
elevation。该扩展必须引入多帧视差、metric-depth prior、可靠 correspondence 或其他运动/
结构约束。

为验证接口没有写死双目，MVP 测试提供 fake metric-monocular frontend；它只验证
`OpticalDepthFrontend` 可替换性，不宣称实现真实单目算法。

## 8. 几何关联与概率融合

### 8.1 FLS 测量集合

对 range `rho`、bearing `theta` 和 elevation aperture `alpha`，sonar frame 中的理想
候选集合为：

```text
p_S(phi) = rho [cos(phi) cos(theta), cos(phi) sin(theta), sin(phi)]
phi in [-alpha/2, +alpha/2]
```

公式采用 sonar local frame 的 `x` 前、`y` 左、`z` 上约定。`alpha` 来自该 observation
引用的 calibration version 中对应 `SonarBeamModel`；当前 `SonarRangeBearing` payload
不复制 beam model，避免 measurement 与 calibration 保存两份可能冲突的 aperture。

range/bearing sigma、beam width、range-bin resolution、sound-speed uncertainty 和外参
uncertainty将理想弧扩张为候选弧带。MVP 固定外参，但必须把标定版本和配置的 gate
记录在 RunManifest。

### 8.2 候选生成

1. 使用 `T_CS` 把采样或解析表示的 FLS 弧带变换到 primary camera frame。
2. 使用 camera intrinsics 投影为 image candidate band，剔除负深度和图像外区域。
3. 仅访问 valid optical prior pixel。
4. 将 pixel + optical depth 反投影成 3D，再变换回 sonar frame。
5. 对 range、bearing、elevation aperture、reprojection 和 visibility 执行 gate。
6. 对保留候选计算结合 optical variance 与 sonar noise 的归一化分数。
7. 按分数排序并保留配置上限内的多候选。

候选生成必须有固定遍历和 tie-break 顺序，不能依赖 unordered iteration 或未 seeded 的
随机采样。

association score 定义为归一化残差的负对数似然，数值越低越好。接受 margin 定义为
`second_best_score - best_score`，必须大于等于配置阈值；只有一个候选时仍需通过全部
绝对物理 gate，不能因缺少 second-best 自动接受。

### 8.3 关联接受

只有以下条件同时成立才接受：

- best candidate 通过全部物理 gate；
- best score 与 second-best score 的 margin 超过配置阈值；
- candidate posterior 有限；
- posterior variance 相对 prior 有配置规定的最小改善；
- 更新后 cross-modal normalized innovation 仍在接受域；
- 可见性/遮挡检查没有发现更近的冲突表面。

否则输出 `AMBIGUOUS`、`CONFLICT` 或 `REJECTED`，保留 optical prior，不强制融合。

### 8.4 Posterior depth update

对选中的 primary-camera pixel `u` 和 optical prior depth `d_o`，沿 camera ray 优化标量
depth `d`：

```text
min_d
  (d - d_o)^2 / sigma_d^2
  + robust_range(range(T_SC * ray(u, d)) - rho) / sigma_rho^2
  + robust_bearing(bearing(T_SC * ray(u, d)) - theta) / sigma_theta^2
```

约束为 `d > 0`、预测 elevation 在 aperture 内、投影可见。MVP 使用确定性一维求解或
有界 Gauss-Newton；失败、非有限结果或未改善 variance 时回退 optical prior。

posterior variance 从最优点附近的局部 Hessian/曲率估计，并受物理 sensor、标定和
cross-modal consistency cap 限制。frontend confidence 不能直接覆盖该 variance。

### 8.5 Dense 的能力边界

MVP 的“增强 dense depth”以 optical dense/semi-dense prior 为底图。sonar 用于校正已
存在且可关联的 optical hypothesis。没有 optical support 时，单个低角分辨率 2D FLS
detection 不足以选择唯一 camera pixel/elevation，因此首版不创建新像素深度。

未来可以通过多帧 surface tracking、plane prior、semantic correspondence 或学习式
cross-modal features 填补空洞，但必须作为独立规格和消融，不得悄悄改变本 MVP claim。

## 9. 失败处理与健康状态

系统采用“不能证明一致，就不融合”的策略：

| 情况 | 行为 |
|---|---|
| 缺少内参/外参或 calibration version 不兼容 | 拒绝整个同步 bundle |
| 校正后的 camera-sonar 时间差超过容差 | 拒绝 fusion，不做运动补偿 |
| optical scale 为 `RELATIVE_SCALE` | MVP 记录后拒绝，留给未来 scale alignment |
| optical scale 为 `UNOBSERVED_SCALE` | 拒绝进入 fusion |
| optical pixel 无效 | 保持无效；sonar 不单独生成 pixel depth |
| SonarFrame/schema/azimuth 非法 | 隔离 sonar frame，optical 分支继续 |
| 多候选分数接近 | `AMBIGUOUS`，保留 optical prior |
| cross-modal residual 超过 gate | `CONFLICT`，不更新 |
| posterior 非有限或 variance 未改善 | 回退 optical prior |
| 单帧失败 | 记录 health，不永久禁用整个模态 |

Health 至少报告：

- sync match/reject rate 与 time-delta 分布；
- optical valid coverage 与 scale-status 计数；
- sonar frame valid rate、detection/hypothesis 数；
- 每帧 candidate 数、accepted/ambiguous/conflict/rejected rate；
- depth update magnitude 与 normalized innovation 分布；
- P50/P95/P99 frontend latency、队列深度和丢帧数；
- 当前 calibration、algorithm 和 model version。

## 10. 仿真数据与场景矩阵

HoloOcean/合成 adapter 必须写入同步的 left/right `ImageFrame`、`SonarFrame`、GT pose 和
primary-camera GT depth。GT 只进入 evaluation plane，不进入 frontend 或 association。

首版固定场景集至少包含：

1. `clean_textured`：清晰、高纹理，验证 fusion 不损害健康 optical depth；
2. `low_texture_sonar_visible`：低纹理但目标有回波，验证 sonar correction；
3. `turbid_sonar_visible`：视觉退化、声呐正常，验证互补性；
4. `repeated_structure`：多个几何候选，验证 ambiguity rejection；
5. `elevation_stress`：目标不在 sonar 水平面，防止回归到 z=0 假设；
6. `time_offset_fault`：超过同步容差，验证拒绝；
7. `extrinsic_perturbation`：使用错误标定运行，验证 conflict/metric 敏感性；
8. `sonar_dropout`：验证 optical-only fallback；
9. `optical_invalid_region`：验证 sonar 不无条件填洞。

所有随机退化使用 experiment seed 和显式传递的 RNG；同一 seed 的 raw bag 必须确定。

## 11. 测试策略

### 11.1 L0 contract

- `ImageFrame` encoding、dimensions、stride 和 payload size；
- optical/fused depth arrays 与 mask shape；
- scale status 和有效值约束；
- source observations、frame 和 calibration version round-trip；
- association status 与 reason code round-trip。

### 11.2 L1 unit/component

- camera projection/unprojection 与 frame transform；
- FLS ideal arc、uncertainty band 和 elevation aperture；
- camera-ray/sonar-range 求交与数值边界；
- unique、ambiguous、occluded、out-of-image 和 conflict association；
- posterior depth/variance 与 robust fallback；
- fixed iteration/tie-break 的确定性；
- stereo frontend fixture；
- fake metric-monocular frontend 通过同一 fusion API。

### 11.3 L2 replay/integration

- canonical bag → synchronize → optical/sonar frontends → fusion → local evidence；
- 同一 bag/config/calibration 重跑产物逐字节一致；
- 修改同步容差后从原始独立 frame 重建不同 bundle；
- calibration version、time offset、dropout 和错误输入故障注入；
- optical-only、sonar-only-observable 和 acoustic-optic fused 三路消融。

## 12. 指标与验收标准

所有 depth 指标分别在全图、sonar 投影覆盖区和视觉退化区报告，避免全图平均掩盖融合
贡献。至少报告 RMSE、MAE、valid coverage、false-fusion rate、posterior NLL 和 P95
latency。

### 12.1 定义

- `false fusion`：accepted update 使绝对 depth error 增加超过
  `max(0.05 m, 0.03 * GT depth)`。
- `sonar covered`：GT 可见并位于对应 detection 的带不确定性投影弧带内，不使用算法
  自己的接受结果反向定义覆盖区。
- `visual degraded`：由 scenario ground-truth mask 定义，不由 stereo 失败结果定义。
- `optical-only`：完全相同的 camera input、calibration 和 optical frontend，只关闭
  sonar fusion。

### 12.2 MVP Gate

| 指标 | 通过条件 |
|---|---|
| 退化场景 sonar 覆盖区 depth RMSE | 相对 optical-only 平均下降至少 15% |
| clean 场景 depth RMSE | 相对 optical-only 退化不超过 2% |
| valid depth coverage | 不低于 optical prior 超过 1 个百分点 |
| false-fusion rate | 不超过 accepted updates 的 5% |
| ambiguity 场景错误接受率 | 不超过 1% |
| 非法 time delta/calibration version | 100% 拒绝 |
| uncertainty | accepted 且 GT-valid 的 sonar 覆盖像素平均 posterior NLL 优于 optical prior |
| determinism | 重复 replay 的正式产物逐字节相同 |
| performance | 默认 5 Hz keyframe 下 P95 小于 200 ms，并记录硬件/profile |

若某一固定场景没有足够 accepted updates，不能以低 false-fusion rate 宣称成功；结果必须
同时报告 accepted count/rate。阈值变更需要版本化更新本规格和实验配置，不得只修改
测试常量。

## 13. 分阶段交付边界

本规格描述一个 MVP，但实现按可独立验收的垂直阶段推进：

1. Contracts/calibration：camera raw schema、optical/fused evidence、rig 与 L0 tests；
2. Optical baseline：canonical stereo、stereo depth prior、GT depth 与 optical-only metrics；
3. Cross-modal geometry：同步、FLS arc projection、候选生成与 association audit；
4. Probabilistic fusion：posterior update、uncertainty、fallback 与 health；
5. Replay/evaluation：完整仿真场景矩阵、三路消融、determinism 和性能报告；
6. Mapping handoff：fused depth 转 local surface evidence，并验证 pose correction 后可重变换。

每阶段只声明实际完成的能力。语义、单目 scale alignment、学习式 fusion 和真实水池 Gate
分别需要后续独立规格。

## 14. 风险与缓解

| 风险 | 缓解 |
|---|---|
| FLS elevation ambiguity 造成多候选 | 保留 arc band、多假设与 margin gate，不压成 z=0 点 |
| camera-sonar overlap 小 | 在 rig/scenario 中显式验证共同 FOV，报告 covered-area ratio |
| 时间错位被误认为几何误差 | capture-time sync、time-offset version 与 fault scenario |
| 外参误差被 depth update 吸收 | MVP 固定外参，冲突时拒绝；单独做 perturbation sensitivity |
| stereo variance 未校准 | 先做 optical-only NLL/coverage，posterior 必须相对改善 |
| sonar speckle/multipath 错配 | CFAR hypotheses、robust gate、ambiguity/conflict audit |
| “dense”被误解为 sonar 填洞 | 在接口、指标和 claim 中限定为校正 optical prior |
| 单目扩展重写 fusion | 通用 optical prior + scale status + fake monocular contract test |
| 学习 confidence 过度自信 | 学习模块不进 MVP；未来仍受物理/标定/一致性 cap |

## 15. 后续决策门

### Gate A：单目 metric-depth frontend

只有当 metric monocular prior 在独立 holdout 上具备可校准 uncertainty 和 OOD 行为，才
允许作为 `OpticalDepthFrontend` 实现进入 fusion；否则只可 shadow 记录。

### Gate B：Sonar-assisted monocular scale alignment

只有当跨模态 association 在不使用 GT 的条件下达到本规格的错误接受率要求，才允许
消费 `RELATIVE_SCALE` prior 并估计尺度。

### Gate C：语义前端融合

语义进入候选 gate 前，必须先完成静态/动态类别策略、类别置信度校准、错误语义 fallback
和 geometry-only 消融，不能让语义标签覆盖物理一致性检查。

### Gate D：学习式 dense fusion

需要 synthetic holdout、real calibration、OOD、uncertainty calibration、P95 latency 和
shadow-mode 结果均通过；否则模型只生成对比 evidence，不进入正式 depth output。

### Gate E：真实水池验收

在仿真 MVP 通过后，先验证时钟、外参、折射模型、sonar conformance 和 GT/参考测量，再
冻结真实数据 Gate。不得把真实传感器系统误差静默吸收到 fusion threshold。

## 16. 规格完成条件

本设计实现完成必须同时满足：

- 第 6 节契约有 schema validation 和 round-trip tests；
- 第 5 节模块边界落地且依赖 lint 通过；
- 第 8 节没有 z=0 elevation fabrication 或无审计 hard association；
- 第 9 节每类失败都有可测试 reason/status；
- 第 10 节固定场景可由单命令生成和回放；
- 第 12 节所有 MVP Gate 通过并生成版本化报告；
- local fused evidence 可在 state 更新后重新变换，不被前端固化到 world frame；
- README/代码库参考明确区分“已实现”与“目标规格”。
