# 前端正确性闭环设计

> 日期：2026-08-23
> 状态：用户已确认并修订书面规格，进入逐步实施计划
> 范围：OpenCV 双目校正、声光同步失效、VO 连续性、不确定性传播、状态与质量门禁

## 1. 背景

当前仓库的模块边界、Protobuf 消息、确定性回放和合成端到端链路已经成立，但真实双目与运行状态仍存在几类会直接破坏结果可信度的问题：

- `StereoGeometry` 仅检查左右相机旋转近似相同，并把平移模长当作水平基线；真实 rig 同时具有前向、水平和垂直平移分量，左右内参也不相同。
- 稠密深度和 landmark VO 仍假设输入已经完成极线校正，却没有强制检查或生成与校正图像匹配的内参和虚拟相机外参。
- replay 在同步器拒绝声呐—图像组合后，把时间差替换为零继续调用融合。
- `StereoLandmarkVoFrontend` 在一次估计失败后仍推进 previous frame，可能让后续证据都从未入图 keyframe 出发而永久断链。
- schema 已携带 sigma、协方差和质量特征，但因子权重仍主要来自固定 sqrt-information 常数。
- replay 使用固定 0.2 秒构造轨迹时间，并把 solver stalled 的结果继续标为 `TRACKING`。
- “非空地图”门禁无法区分 optical-only 地图与真正获得声学修正的地图。

本设计把这些问题作为一个完整的“前端正确性闭环”处理。目标是在更换后端或引入学习式模型之前，先确保送入后端的数据在几何、时间、不确定性和状态语义上自洽。

## 2. 目标与非目标

### 2.1 目标

1. 使用 OpenCV 对不同内参、带 plumb-bob 畸变、非水平基线的双目图像执行完整校正。
2. 让校正后的图像、内参和虚拟相机外参使用同一个派生标定快照，禁止 raw/rectified 几何混用。
3. 保留原始 capture/receive time 和 observation ID，所有轨迹与状态使用真实 capture time。
4. 同步失败必须产生可审计的拒绝，不允许把失败时间差改写为零。
5. 单帧 VO 失败不得自动破坏后续所有关键帧；连续失败必须进入明确的 `LOST` 状态。
6. 相对位姿、压力深度和声呐距离因子消费实际协方差或 sigma，并把配置常数作为信息上限或缺省回退。
7. 增加能够证明双目、同步、VO 恢复、状态语义和融合有效性的自动化门禁。

### 2.2 非目标

- 不在本阶段替换 Harris/NCC、BlockMatcher 或 RANSAC 的算法主体。
- 不引入 ORB、SIFT、StereoSGBM、学习式深度或学习式可靠性模型。
- 不重构声呐路标为联合优化变量；该工作保留为下一独立阶段。
- 不替换现有 Gauss-Newton/LM 后端，不实现滑窗、IMU 预积分、回环或重定位。
- 不承诺从长时间或大视差 VO 丢失中完成全局重定位；本阶段只保证短期缺帧后仍能从最后成功参考帧恢复，并在超限后如实进入 `LOST`。

## 3. 方案选择

采用“独立 OpenCV 适配层 + 纯核心算法接口”方案。

不把 OpenCV 类型暴露到 `domain`、`sensor_models`、`measurement_api` 或 `frontends`。新增的 OpenCV target 只负责从规范化 rig 和 ImageFrame 构造校正映射，并输出仍由仓库自身类型表示的 rectified ImageFrame 与派生 RigCalibrationSnapshot。应用层负责在原始输入与算法前端之间编排该适配器。

不采用以下方案：

- 直接在 `core/frontends` include OpenCV：实现简单，但会使算法接口、测试和替换成本绑定 OpenCV。
- 同时迁移到 OpenCV 特征、匹配和深度算法：一次改变过多，无法区分几何修复和算法替换各自造成的精度变化。

## 4. 模块与依赖

### 4.1 OpenCV 适配层

新增独立 `opencv_adapters` target，源码位于 `adapters/opencv/`，依赖：

- OpenCV `core`、`calib3d`、`imgproc`；
- `uw::domain`、`uw::core`、`uw::measurement_api`（公开接口返回 `uw::measurement_api::CameraFrameBundle`，因此这一依赖是显式的，不是隐含允许就可以不声明）；
- 不依赖 frontends、estimation、mapping 或 application。

`tools/lint/check_layer_dependencies.py` 目前的 `source_files()` 只扫描 `include/`、`src/`、`adapters/ros2/`、`apps/`——顶层新增的 `adapters/opencv/` 不在这个扫描列表里，不会被 lint 自动检查。实施单元 1 必须同时把 `adapters/opencv/` 加入该脚本的扫描集合（并声明它的允许依赖，同 `adapters/ros2/` 一样只到 `measurement_api`/`sensor_models`/`domain`），否则本节反复强调的"不暴露 cv::Mat""不依赖 frontends"这些边界约束实际上没有自动化保护。

公开接口不暴露 `cv::Mat`。Rectification context 持有不可变派生标定，避免每帧复制完整 protobuf；每帧返回值只携带图像：

```cpp
// RectifiedStereoBundle：Process() 每帧返回值，只携带图像本体。
struct RectifiedStereoBundle {
  uw::measurement_api::CameraFrameBundle images;
};
```

```cpp
// StereoRectificationContext：初始化阶段构造一次的不可变上下文，
// 持有派生标定与缓存 remap；下面三个是它的只读访问器。
const uw::domain::RigCalibrationSnapshot& DerivedRig() const;
const std::string& LeftRectifiedFrame() const;
const std::string& RightRectifiedFrame() const;
```

初始化阶段生成不可变 `StereoRectificationContext`，缓存 `R1/R2/P1/P2/Q` 和左右 remap。缓存键至少覆盖 calibration version、左右 sensor ID、frame ID 和图像尺寸。运行中尺寸变化不复用旧映射。

新增 OpenCV 这一硬依赖需要同步更新 `tools/setup_dev_env.sh`（conda-forge 回退路径）、`README.md` 构建一节和 CI（`.github/workflows/ci.yml`）；当前工作树的 cmake、setup 脚本和 README 里都不存在任何 OpenCV 引用，这不是"顺带装一下"的小事，实施单元 1 应把它列为明确任务，避免重演 CLAUDE.md 记录过的那类环境配置坑。

### 4.2 核心双目契约

`StereoGeometry` 继续是 frontend 使用的纯 Eigen/Pinhole 模型，但其解析目标改为派生 rig 中的 rectified cameras。它必须验证：

- 左右内参完整且有限；
- 左右图像尺寸相同；
- 虚拟相机旋转一致；
- 基线在 rectified optical x 方向，非水平分量低于数值容差；
- 焦距和投影矩阵能够形成正的 metric baseline。

Frontend 必须同时检查左右 `ImageFrame.is_rectified == true`。合成数据已经满足该条件，可继续使用现有零畸变 rig 的快速路径。

### 4.3 派生标定

OpenCV 适配器保留原始 rig 中的 sonar、IMU、depth、time offsets 和原始 frame edges，并增加独立虚拟 frame：

- `camera_left_rectified_link`；
- `camera_right_rectified_link`。

派生 cameras 使用校正后的 `P1/P2` 内参、空畸变系数和原 sensor ID。校正后 ImageFrame 保留原 sensor ID，但把 `sensor_frame` 改为对应 rectified frame，并设置 `is_rectified=true`。

虚拟 frame 的 `base_link_T_camera_rectified` 由原始 optical extrinsic 与 OpenCV `R1/R2` 严格组合得到。声光投影和 map bridge 因而能够从 rectified optical frame 正确变换到 sonar/base_link，而不把虚拟图像误认为原始相机坐标。

派生 `calibration_version` 由原始版本和 rectification 参数摘要组成。RunManifest 新增独立 `derived_calibration_hash`，与现有原始 `calibration_hash` 同时记录。

## 5. 数据流

### 5.1 启动

1. 加载并验证原始 experiment/rig。
2. 若配置选择 stereo VO 或 stereo depth，创建 OpenCV rectification context。
3. 初始化失败视为配置错误，程序非零退出，不回退到未经校正的真实图像。
4. 零畸变且原 rig 已满足 rectified 几何契约时允许 identity fast path，但仍生成明确的派生 rig。

### 5.2 每个关键帧

1. 从 MCAP 读取原始左右图像，先执行 `ValidateImageFrame`。
2. 使用原始 header 和原始 rig 做时间同步决策。
3. 将左右图像转换为 MONO8，然后通过缓存 remap 生成 rectified bundle。
4. VO 和 StereoOpticalDepthFrontend 消费相同的 rectified bundle 与 derived rig。
5. SonarFrontend 仍消费原始 SonarFrame。
6. AcousticOpticDepthFusionFrontend 使用 sonar evidence、rectified optical depth 和 derived rig。
7. Map bridge 使用同一 derived rig 将 rectified optical 点转换到 base_link。

所有输出 evidence 继续引用原始 observation ID。图像校正属于确定性派生处理，不生成新的传感器观测 ID。

## 6. 同步与声光拒绝

现有 `optional<SynchronizedAcousticOpticBundle>` 无法区分“缺少声呐”和“时间差超限”，也会丢失实际 delta。改为显式结果：

```cpp
enum class SynchronizationStatus {
  kSynchronized,
  kNoSonar,
  kTimeDeltaExceeded,
  kInvalidTimestamp,
};

struct SynchronizationDecision {
  SynchronizationStatus status;
  double max_pairwise_time_delta_s;
  std::optional<SynchronizedAcousticOpticBundle> bundle;
};
```

同步器接受 optional sonar：

- 无 sonar 返回 `kNoSonar`，光学链正常运行；
- 时间差合法返回 `kSynchronized` 和 bundle；
- 时间差超限返回 `kTimeDeltaExceeded` 以及真实 delta；
- 非法时间戳返回 `kInvalidTimestamp`。具体定义为 `Stamp.nanos` 不在 `[0, 1e9)`、sensor ID 为空，或应用 rig offset 后的 corrected time 不是有限数；全零 Stamp 仍是合法的相对时间原点，不因数值为零而拒绝。

Associator 增加与 Synchronizer 共用的 `max_time_delta_s`。当 sonar evidence 存在但 delta 超限时，在几何投影之前生成 `REJECTED/TIME_DELTA` 记录。Fusion 输出保持 optical-only。应用层禁止把任何失败状态转换为 delta=0。

## 7. VO 连续性与健康状态

`StereoLandmarkVoFrontend` 将“最近处理帧”与“最后成功参考关键帧”分开：

- 第一帧有足够 stereo landmarks 时建立参考帧，但不输出相对位姿；
- 当前帧成功与参考帧拟合后，输出 `reference_keyframe -> current_keyframe`，然后推进参考帧；
- 当前帧无法三角化或拟合失败时，不推进参考帧；
- 后续帧继续尝试与最后成功参考帧匹配；
- 连续失败次数达到配置阈值后，Health 进入 unavailable/lost 语义，并停止假装产生连续跟踪；
- 一次成功拟合将连续失败计数清零。

8.1 节的协方差条件数/秩检查明确算作本节的"拟合失败"：RANSAC 姿态拟合本身收敛、但协方差秩不足/条件数超阈值的帧，同样不推进参考帧、同样计入连续失败计数。不这样定义的话，参考帧会推进到一个协方差不合格（例如近似共线点集）的病态锚点上，而这类帧因为"姿态拟合技术上成功了"不会被计入失败计数，系统可能连续多帧既不产出可信 evidence、又永远不会因为达到失败阈值而如实进入 `LOST`——参见第 12 节错误表，"VO 单帧失败"与"covariance 秩不足"是同一个计数器下的两种触发原因，不是两条独立状态机。

Pipeline 不再依赖“上一物理帧一定已入图”的假设。Evidence 的 `from_keyframe` 必须是 frontend 当前参考关键帧，应用仍保留 `from` 存在性检查作为边界保护。

本阶段不在 LOST 后自动创建新的不相连 pose graph 分量。恢复需要仍能匹配最后成功参考帧；超过该能力范围时如实维持 LOST，留给后续重定位设计。

## 8. 不确定性与因子权重

### 8.1 VO 质量摘要

RANSAC rigid fit 返回 pose 之外，还返回：

- correspondence count；
- inlier count 与 ratio；
- inlier RMSE；
- 最小奇异值或等价条件指标；
- 6x6 covariance。

协方差在最优位姿附近使用 minimal SE(3) 六维扰动构造残差 Jacobian，通过 `sigma^2 (J^T J)^{-1}` 或 SVD 伪逆计算。秩不足、非有限或条件数超过阈值时不输出伪造的高置信协方差，整条 VO evidence 被拒绝或标为不可用。

质量摘要写入 `MeasurementEvidence.quality_features`；协方差写入 `RelativePoseMeasurement.covariance_6x6_row_major`。

### 8.2 白化策略

- RelativePoseResidual 使用 6x6 sqrt-information 矩阵对白化 minimal residual。
- PressureDepthMeasurement 使用 `sigma_m`。
- SonarRangeBearing 的 range factor 使用 `range_sigma_m`。
- 配置的 default sqrt-information 是最大允许信息强度。标量因子使用 `min(1/sigma, cap)`；相对位姿 covariance 做对称特征分解，将 covariance 特征值下限截到 `1/cap^2` 后再构造 sqrt-information，使任何白化方向的增益都不超过 cap。translation/rotation 分别使用各自 cap。
- payload sigma/covariance 缺失或非法时，使用当前配置常数作为兼容回退，并在 health/quality feature 中记录 fallback。

这一语义会同步更新 `FactorCandidate.proposed_noise` 的 schema 注释和 FactorBuilder 测试：字段仍保持 tag/type 不变，但从“调用者已经计算好的最终权重”收敛为“调用者允许的最大 sqrt-information”。

本阶段不实现 robust kernel。`robust_policy_hint` 保持未消费，待后端替换阶段统一处理。

## 9. 低纹理与视差有效性

保留现有 SAD BlockMatcher，但补齐最低正确性过滤：

- block texture variance 下限；
- best 与 second-best SAD 的唯一性 margin；
- left-right consistency；
- 正 disparity 与配置范围检查。

任一检查失败即把像素标为 invalid。平坦或重复纹理不得因为某个 disparity 恰好先被遍历就获得有效深度。`disparity_sigma_px` 后续可依据匹配曲率细化；本阶段仍允许固定 sigma，但只应用于通过上述质量检查的像素。

## 10. 时间、状态与输出语义

应用建立 `keyframe_id -> capture_time` 映射，来源优先为左相机原始 header。StateSnapshot 和 TUM 轨迹均使用该真实时间；ATE 匹配不再使用 `index * 0.2s`。

状态发布规则：

- solver converged 且 VO/输入链健康：`TRACKING`；
- solver stalled 或短期 VO 退化但仍有可用图：`DEGRADED`；
- VO 连续失败超过阈值且不能继续连接图：`LOST`。

StateSnapshot 填充 `capture_timestamp`、`calibration_version` 和 contributing evidence IDs。系统不得仅因函数返回了 Pose3 就宣称 tracking。

`DEGRADED` 是诚实上报机制，不代表 solver stalled 在真实数据上是可接受的终态：roadmap 文档（`docs/uw-slam-production-readiness-and-roadmap-2026-08-21.md` P1 验收）明确要求真实数据 solver 必须收敛、不得 stalled，且不能被"轨迹和地图非空"掩盖过去。本节只负责状态语义不失真（第 12 节的 gate 非零是配套的强制手段），收敛问题本身的修复不在本规格范围内（见 2.2 非目标）。

## 11. 配置

新增配置属于现有分层 YAML：

- rectification alpha/crop policy；
- rectified frame suffix；
- VO max consecutive failures；
- RANSAC covariance conditioning threshold；
- block texture、uniqueness 和 left-right consistency thresholds；
- acoustic-optic max time delta；
- expected fusion coverage gate；
- relative pose translation sqrt-information cap 与 rotation sqrt-information cap（当前 `default_sqrt_information.relative_pose` 是单个标量，8.2 节要求 translation/rotation 分用各自 cap，需要把这一个字段拆成两个，不是复用现有字段就能实现）。

所有字段进入类型化配置并有范围验证。未知或非法值启动失败。不得再在 replay 内硬编码仅适合 synthetic fixture 的前端参数。

## 12. 错误处理

| 失效 | 行为 | 状态/审计 |
|---|---|---|
| rig 缺相机、矩阵或 frame edge | 启动失败 | 配置错误 |
| OpenCV stereoRectify/map 初始化失败 | 启动失败 | rectification error |
| 单帧 payload/尺寸/编码非法 | 丢弃该帧 | frontend suspect，计数增加 |
| 运行中分辨率变化 | 拒绝帧，不复用旧 map | dimension mismatch |
| sonar 缺失 | optical-only | no-sonar，不算同步故障 |
| 时间差超限 | optical-only | association TIME_DELTA |
| VO 单帧失败 | 保留最后成功参考 | degraded/reject counter |
| VO 连续失败超限 | 停止跟踪输出 | LOST |
| covariance 秩不足 | 拒绝 VO evidence | conditioning failure |
| solver stalled | 保留诊断输出，gate 非零 | DEGRADED |

## 13. 测试设计

### 13.1 OpenCV adapter

- 使用非水平基线、不同内参和 plumb-bob 畸变构造左右投影，校正后对应点垂直误差小于 0.25 px。
- 对已知 3D 点进行校正、匹配和三角化，深度相对误差小于 1%。
- 验证派生 frame、K、baseline、calibration version 和 `is_rectified`。
- 验证缺标定、非法畸变、尺寸不一致和运行中尺寸改变均 fail closed。
- 验证合成 identity rig 不改变像素和现有几何结果。

### 13.2 同步

- 合法 offset 修正后返回 synchronized 和真实 delta。
- sonar absent 返回 no-sonar。
- 时间超限返回 rejection 和未被清零的 delta。
- replay 集成测试确认 `TIME_DELTA` 对应 optical-only 输出。

### 13.3 VO

- good kf0、损坏 kf1、good kf2：kf2 能相对 kf0 输出证据。
- 连续失败达到阈值：Health/状态进入 LOST。
- 成功后失败计数清零。
- 退化共线点集不输出高置信 pose。
- 更少内点、更高 RMSE 的输入产生更大 covariance。

### 13.4 因子

- 完整 6x6 covariance 的白化残差符合手算结果。
- 增大 covariance 会减小因子贡献。
- depth/sonar sigma 改变会按反比例改变 sqrt-information。
- 配置 cap 能限制过度自信量测。
- 非法/缺失 covariance 走明确 fallback。

### 13.5 视差

- 平坦图像输出零有效像素。
- 重复纹理的非唯一匹配被拒绝。
- 左右不一致匹配被拒绝。
- 干净常视差 fixture 继续恢复 metric depth。

### 13.6 应用与门禁

- 真实 rig 参数驱动的可控场景完成 rectification -> VO/depth。
- 缺一帧不导致后续所有 keyframe 消失。
- StateSnapshot/TUM 使用输入 capture time。
- solver stalled 发布 DEGRADED 并触发非零 gate。
- 预期有声光覆盖的场景必须有 accepted association。
- 报告分别统计 optical-only 与 acoustic-optic map points。
- 现有 synthetic smoke ATE 保持不高于 0.15m，确定性输出保持一致。

## 14. 验收标准

本阶段完成必须同时满足：

1. OpenCV 仅存在于独立 adapter target，核心 public API 无 OpenCV 类型。
2. 当前真实 rig 通过 rectification 初始化，校正几何达到 0.25 px 极线误差和 1% 深度误差的合成标定测试。
3. out-of-sync sonar 不产生 acoustic-optic depth update，并保留真实拒绝原因。
4. 单个损坏 VO 帧后至少一个后续帧能够重新连接最后成功 keyframe。
5. 相对位姿、depth、sonar 因子使用 payload uncertainty，并受到配置 cap 限制。
6. 平坦图像不生成有效 stereo depth。
7. stalled solver 不发布 TRACKING。
8. StateSnapshot、轨迹和 ATE 使用真实 capture time。
9. 合成回放 ATE 与确定性门禁通过。
10. 全量 CTest、Python adapter tests、layer lint、ASan/UBSan 和端到端 replay 通过。

## 15. 实施拆分

本规格拆为三个顺序执行、各自可验证的实施单元：

1. OpenCV stereo rectification adapter、派生标定和双目契约；同时把 `adapters/opencv/` 加入 `tools/lint/check_layer_dependencies.py` 的扫描集合，并更新 `tools/setup_dev_env.sh`/`README.md`/CI 以安装 OpenCV（见 4.1 节）。
2. SynchronizationDecision、VO 参考帧连续性、真实时间戳与 tracking 状态。
3. 协方差传播（含新增的 relative pose translation/rotation cap 配置字段，见 11 节）、因子白化、视差质量过滤和融合/地图质量门禁。

每个单元独立执行 TDD red-green-refactor；一个单元验证通过后再进入下一个。后端替换与声呐联合路标优化不混入本规格。
