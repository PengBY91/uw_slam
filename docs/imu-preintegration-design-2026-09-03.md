# IMU 预积分 frontend / factor builder 设计短文（PREP-B-01）

> 状态：v2 2026-09-03 · 第 2 周交付设计短文 + frontend + 核心数学；**第 3 周交付第 4–6 节：15 维残差、factor builder、估计器状态扩展（技术负责人拍板取方案 A）**，两个求解器后端都已支持并各有一条 IMU-only 链路收敛测试。第 8 节（`estimator_mode` + `synth_bag_gen` IMU 输出 + 端到端 demo）留到第 4 周。
> 出自 `docs/ROV平台到货前准备工作规格-2026-09-02.md` PREP-B-01；架构决策 D-1（阶段 1 相对位姿来源：IMU 预积分 + 声呐配准）
> 参考文献：Forster, Carlone, Dellaert, Scaramuzza, "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", IEEE T-RO 2017。**公式按论文自行推导并用数值测试验证，没有移植 GTSAM/OKVIS/VINS 任何代码**（与 `sonar_range_residual` 雅可比同一规则，见 CLAUDE.md）。

## 0. 一句话

把两个关键帧之间的 `ImuSample` 序列在机体系里预积分成一条 `ImuPreintegrationMeasurement` 证据（ΔR、Δv、Δp、15×15 协方差、五个偏置雅可比），作为因子图里第三类相对运动约束；估计器为每个关键帧增加速度 + 偏置 9 维状态，残差 15 维。到货形态（单目、无 DVL）下这是唯一的高频相对位姿来源。

## 1. 已落地的部分（第 2 周）

| 文件 | 内容 | 测试 |
|---|---|---|
| `schemas/proto/uw/domain/measurement.proto` | `ImuPreintegrationMeasurement` 追加字段 7–15：`from_keyframe`/`to_keyframe`、五个 3×3 偏置雅可比、`gravity_mps2`、`sample_count`；协方差/残差顺序定为 `[δR(3), δv(3), δp(3), bg(3), ba(3)]` | contract round-trip |
| `include/sensor_models/so3.hpp` + `.cpp` | `Hat/Vee/Exp/Log/RightJacobian/RightJacobianInverse`，小角度分支 | `tests/core/so3_test.cpp`：Exp/Log 往返（含近 π）、Jr 有限差分、Jr·Jr⁻¹ = I |
| `include/sensor_models/imu_preintegration.hpp` + `.cpp` | `ImuPreintegrationNoise::FromRig`、`PreintegratedImuDelta`（含一阶偏置修正 `Corrected*` 和 proto 往返）、`ImuPreintegrator`（增量积分 + 协方差 + 偏置雅可比） | `tests/core/imu_preintegration_test.cpp`：静止/匀角速闭式解、解析轨迹定义式对照、偏置雅可比对比重积分（误差 < 2%）、2000 次蒙特卡洛协方差对角/耦合项（±15%）、proto 往返/拒绝 |
| `include/measurement_api/frontend.hpp` | 新窄接口 `InertialFrontend` + `ImuPreintegrationRequest` | — |
| `include/frontends/imu_preintegration_frontend.hpp` + `.cpp` | `ImuPreintegrationFrontend`：窗口选样、零阶保持、imu_link 外参（旋转 + 向心杠杆臂）、fail-closed 规则、健康状态、证据封装 | `tests/frontends/imu_preintegration_frontend_test.cpp`：区间/关键帧 id/样本计数、确定性、外参旋转与杠杆臂、缺口/样本不足/区间非法/畸形样本/rig 噪声为零全部拒绝 |
| `cmake/Libraries.cmake`、`cmake/Tests.cmake` | 并入 `core`/`frontends` target | `tools/lint/check_layer_dependencies.py` 通过 |

## 2. 约定（所有后续代码必须一致）

- **坐标系**：world/body 均 Z-up（仓库不变量），重力 `g_w = (0, 0, −gravity_mps2)`，常数取 rig `imu_noise.gravity_mps2`。所有 Δ 量在 **`from_keyframe` 的 base_link 系**下，frontend 已把 imu_link 读数转到 base_link（陀螺/加速度计旋转 + 向心项 `ω×(ω×r)`，忽略角加速度项 `α×r`）。
- **定义**：`ΔR_ij = R_iᵀ R_j`，`Δv_ij = R_iᵀ (v_j − v_i − g Δt)`，`Δp_ij = R_iᵀ (p_j − p_i − v_i Δt − ½ g Δt²)`。
- **积分格式**：零阶保持；旋转按 `ΔR ← ΔR·Exp(ω dt)`；加速度项用**半步旋转** `ΔR·Exp(ω dt/2)`——论文原式用步首 ΔR，在 200 Hz、0.5 rad/s 下会留下每秒约 1 cm/s 的系统性速度漂移（`DeltasMatchAnalyticTrajectoryDefinitions` 首次实跑抓到 6 mm/s@1.2 s），改半步后降到 1e-4 量级。协方差与偏置递推按半步旋转重新推导（`.cpp` 注释有推导），蒙特卡洛和有限差分都通过。
- **噪声**：连续时间密度（rad/s/√Hz、m/s²/√Hz），离散方差 `σ_c²/dt`。偏置随机游走优先取 rig `sigma_*_bias_walk_c`，为 0 时退化取 `sigma_*_bias`（所有现有 rig 都只填了后者）。白噪声密度为 0（rig 漏写键的默认值）**直接拒绝**，不给奇异协方差留门。
- **扰动模型**：右扰动 `R ← R·Exp(δφ)`，与 `rigid_transform_fit.hpp`、`camera_body_conjugation.hpp` 注释一致；残差里位姿块仍用现有 7 参数 `[t, q_xyzw]` 布局。
- **证据**：`estimated_noise_scale = 1.0`（估计器可再缩放）；`source_observations` 只放区间内首末样本 id（200 Hz × 数秒否则上百条）；`quality_features`：`sample_count`、`delta_time_s`、`max_hold_s`、`mean_rate_hz`、`held_backwards_at_start`、`lever_arm_m`。

## 3. Frontend 的 fail-closed 规则

| 条件 | 结果 | `last_rejection_reason()` |
|---|---|---|
| 区间 ≤ 1 ms 或 > 5 s（可配） | 拒绝 | `interval_out_of_range` |
| rig `sigma_gyro_c`/`sigma_accel_c` ≤ 0 或非有限 | 拒绝 | `rig_imu_noise_invalid` |
| `require_extrinsic` 且无 `imu_link` 边 | 拒绝（默认不严格：按单位阵） | `imu_extrinsic_missing` |
| 样本向量非 3 维或非有限 | 拒绝 | `imu_sample_malformed` |
| 任一保持段 > `max_sample_gap_s`（默认 50 ms = 200 Hz 丢 10 样本） | 拒绝 | `imu_gap_too_large` |
| 区间内样本数 < `min_samples`（默认 2） | 拒绝 | `too_few_imu_samples` |
| 连续失败 ≥ 3 | `Health()` = `STATUS_UNAVAILABLE` | — |

不做任何"常加速度补桥"或插值：缺数据就没有这条边，由声呐配准/深度/航向兜底。

## 4. 残差设计（已落地，`include/factor_builders/imu_preintegration_residual.hpp` + `.cpp`）

参数块（沿用 `ResidualBlock` 的 Ceres 式多块接口）：`[pose_i(7), inertial_i(9), pose_j(7), inertial_j(9)]`，其中 `inertial = [v(3), bg(3), ba(3)]`（世界系速度，机体系偏置）。

残差 15 维，按线性化偏置 `b̄` 与当前偏置差 `δb = b_i − b̄` 用证据里的雅可比一阶修正（`PreintegratedImuDelta::Corrected*` 已实现）：

```
r_R = Log( (ΔR̄ Exp(J_R^{bg} δbg))ᵀ · R_iᵀ R_j )
r_v = R_iᵀ (v_j − v_i − g Δt) − (Δv̄ + J_v^{bg} δbg + J_v^{ba} δba)
r_p = R_iᵀ (p_j − p_i − v_i Δt − ½ g Δt²) − (Δp̄ + J_p^{bg} δbg + J_p^{ba} δba)
r_bg = bg_j − bg_i
r_ba = ba_j − ba_i
```

白化：`Σ^{-1/2}` 来自证据 15×15 协方差的 LLT（偏置块为随机游走 `σ_walk² Δt`）。雅可比对 `[δφ_i, δp_i, v_i, bg_i, ba_i, δφ_j, δp_j, v_j, bg_j, ba_j]` 手推（r_R 用 `JrInv`），再用链式法则接到 7 参数四元数原始块——与 `relative_pose_residual` 现在的做法一致，并按 `sonar_range_residual` 惯例写数值差分测试。**若第 6 节选择 GN 改流形更新，则直接输出切空间雅可比。**

## 5. Factor builder（第 3 周，已落地）

`ImuPreintegrationFactorBuilder`（`kResidualModel = "imu_preintegration_v1"`）：从证据 payload `FromProto`（失败即拒绝，不默认零协方差），`proposed_noise` 由估计器按 `estimated_noise_scale` 缩放，返回上面的残差块；`involved` 列表扩到 4 块（见第 6 节）。

## 6. 状态扩展（已落地，技术负责人 2026-09-03 拍板取方案 A）

现状：`PoseGraphProblem::Keyframe` 固定 7 参数；GN 求解器直接在 7 参数上做增量并归一化四元数（v1 有意简化）；Ceres 适配器同样按 7 参数块。

**方案 A（推荐）：独立 9 维惯性参数块，按需添加。**
- `PoseGraphProblem::AddInertialState(keyframe_id, v, bg, ba)`，内部 `inertial_states_` 与 `keyframes_` 平行；`GetInertialState/SetInertialState`。
- `AddResidualBlock` 的 `involved_keyframes` 泛化为 `ParameterRef{kind: kPose|kInertial, keyframe_id}`，保留旧签名重载（全 `kPose`），现有 factor builder 一行不改。
- `MutableParameterBlocks()` 返回 `{id, kind, double*, size, fixed}`；GN 求解器的自由变量索引按块大小累计，只对 `kPose` 块做四元数归一化。纯位姿图（没有惯性块）的求解路径**逐字节不变**，`synthetic_smoke*` 的 ATE 数字不会动，这是选它的主要理由。
- Ceres 适配器：惯性块作为普通 9 维参数块添加。
- 代价：GN `EvaluateAll` 的列装配要从"每关键帧 7 列"改成"按块偏移"，约 60 行。

**方案 B：把 Keyframe 扩成 16 参数。** 改动集中但所有残差的 `ParameterBlockSizes()` 都要从 7 变 16 或做偏移映射，且合成 demo 数值会因自由度增加而变（即使没有 IMU 因子，也需要给多出的 9 维加先验或固定）。不推荐。

**顺带决策**：是否趁机把 GN 位姿更新改成 6 维切空间（`Exp(δφ)·q`）。现在 7 参数 + 归一化对小步长可用，但 15 维 IMU 残差的雅可比若直接对切空间推会更干净；建议**本任务不改**（保持 v1 求解器行为、避免同时动两处），把切空间更新列为 PREP-B-01 之后的独立项，用 `tools/bench/solver_benchmark.sh` 对比。

## 7. 初值与 anchor 陷阱（CLAUDE.md z 轴 anchor 的惯性版）

- anchor 关键帧的位姿 z 已按深度证据设置；**速度不要钉 0**：用 `ImuPreintegrationFrontend` 之前的静止检测（加速度计模长与 g 偏差 < 0.05 m/s²、陀螺模长 < 0.01 rad/s 持续 ≥ 0.5 s）判定静止后再把 `v_0 = 0` 作为先验加入，否则给宽先验（σ_v = 0.5 m/s）让图自己定。
- 偏置初值：静止段陀螺均值作为 `bg_0`；`ba_0 = 0` 加先验 σ = rig `sigma_accel_bias`（HWT9053 标称，到货后 PREP-B-05 回填）。
- 重力方向由深度 + 加速度计静止段共同可观；roll/pitch 有绝对参考后 yaw 仍是规范自由度（与现在一致），PREP-B-02 航向因子接管。
- 每个惯性块在图里必须至少被一条 IMU 边或先验约束，否则奇异——`AddInertialState` 只在 frontend 真的产出证据时调用。

## 8. 管线接入与验收（第 4 周）

- `estimator_mode: imu_preintegration`（`config.cpp` 校验注册；`replay_pipeline.cpp` 新分支：以现有 `capture_time_by_keyframe` 为区间边界，从 `ReplayInputData::imu_samples` 调 frontend，逐段 `AddInertialState` + 因子）。注意 `replay_pipeline.cpp` 目前只在 rig 有相机时才保留 `rig`，IMU-only 模式要放开这个条件。
- `synth_bag_gen`：新增独立 salt 的 `imu_rng`（不改动现有三条流的 salt 与抽取顺序），按 rig `rate_hz` 生成含重力、含偏置随机游走的 `/raw/imu`；bag_audit 已期望 `/raw/imu` 按传感器速率写入。
- 验收（规格原文）：仅 IMU 预积分 + 深度 + 声呐 range（无 VO）4–10 次迭代收敛，ATE ≤ 0.15 m；HoloOcean 真实 IMU 段（PREP-A-03 fidelity profile，200 Hz）30 s 内 ATE 有界且随时间增长符合预积分漂移预期；`check_layer_dependencies.py` 通过。

## 8.5 第 3 周实际落地记录（2026-09-03）

| 文件 | 内容 | 测试 |
|---|---|---|
| `include/estimation/pose_graph_problem.hpp` + `.cpp` | 方案 A：`ParameterKind{kPose,kInertial}` + `ParameterRef`、`AddInertialState/Set/Get/Has/IsInertialFixed`、`MutableAllParameterBlocks()`（位姿在前、惯性块在后）、`ResidualBinding.involved_parameters` | `PoseOnlyGraphExposesTheSameBlocksThroughBothAccessors`、`InertialBlocksAreOrderedAfterAllPoses`、`InertialStateApiRejectsUnknownKeyframesAndUnbackedRefs` |
| `include/estimation/gauss_newton_solver.hpp` + `.cpp` | 列装配从"每关键帧 7 列"改成按块偏移（`FreeBlock{offset,size}`），只对 `kPose` 块归一化四元数；备份/回滚按 `free_order` 插入序遍历（不再遍历 hash map，去掉一处非确定性） | 既有 10 条估计器测试全绿；`synthetic_smoke` 实跑 ATE 0.0999 m / 4 迭代，落在 README 记录的 0.08–0.10 m 区间内 |
| `include/factor_builders/imu_preintegration_residual.hpp` + `.cpp` | 15 维残差 + 全解析雅可比 | `imu_preintegration_residual_test.cpp` 7 条：零残差一致性、偏置行、白化、两组中心差分（含远离线性化偏置）、四元数列投影回极小雅可比、参数列表过短拒绝 |
| `include/factor_builders/imu_preintegration_factor_builder.hpp` + `.cpp` | `imu_preintegration_v1`，LLT 白化，fail-closed | `imu_preintegration_factor_builder_test.cpp` 7 条：白化矩阵反演协方差、wire 往返、两个噪声旋钮只能放大不能收紧、错载荷/畸形消息/奇异协方差全部返回 nullptr |
| `adapters/ceres/src/ceres_pose_graph_solver.cpp` | 惯性块作为 9 维欧氏参数块加入，位姿仍带四元数流形；两个后端共用 `GaussNewtonSolver::ParameterKey` | `CeresPoseGraphSolver.ImuOnlyChainRecoversPosesAndInertialStates` |

三处实施中与原设计不同、值得记下的地方：

1. **`AddResidualBlockOnParameters` 而不是 `AddResidualBlock` 重载**。原计划"保留旧签名重载"在 C++ 里行不通：`{"kf0", "kf1"}` 这样的花括号实参对 `vector<string>` 的 initializer_list 构造和 `vector<ParameterRef>` 的 `(InputIt, InputIt)` 构造同时可行（`const char*` 在重载决议阶段就是合法的输入迭代器），GCC 直接报 ambiguous。改成不同函数名后，现有 factor builder 调用点确实一行没动。
2. **四元数列的链式法则是精确的，不是近似**。残差只看 `Pose3::FromParameterBlock` 归一化后的四元数，所以沿 q 方向本来就不变；正交方向上 `dr/dq_raw = (dr/dδφ)·2Q(q̂)ᵀ/|q_raw|`，其中 `Q = [wI+[v]_x ; -vᵀ]` 满足 `QᵀQ = I`、`Qᵀq = 0`。Ceres 的 `EigenQuaternionManifold` 会再右乘 `dq/dδφ = 0.5Q`，正好还原极小雅可比——`QuaternionColumnsProjectBackToTheMinimalJacobian` 就是在验证这一点，也是两个后端能对同一个因子看到同样曲率的原因。GN 侧沿 q 方向那一列为零（真实导数就是零），LM 阻尼保证正定，无害。
3. **两个噪声旋钮只放大不收紧**。`relative_pose_factor_builder` 那套"各向同性上限 + SVD 截断"在这里不适用——IMU 协方差是真实传播出来的量而不是配置猜测，所以 `estimated_noise_scale` 和 `proposed_noise` 一律作为方差膨胀系数相乘，没有任何路径能让因子比积分出来的协方差更自信。

## 9. 开放问题（需拍板）

1. ~~第 6 节方案 A/B~~ — 2026-09-03 拍板取**方案 A**，已实施，见第 8.5 节。
2. ~~GN 是否改切空间更新~~ — 2026-09-03 决定**不在本任务内做**，列为 PREP-B-01 之后的独立项，用 `tools/bench/solver_benchmark.sh` 对比后再定。当前 7 参数 + 归一化在 IMU-only 链路上收敛到 cost < 1e-12，没有暴露出必须先改的问题。
3. `sigma_*_bias` 与 `sigma_*_bias_walk_c` 的语义：现有 rig 只有前者，`FromRig` 目前把前者当随机游走密度用。到货后 PREP-B-05 会同时给出两者，届时建议 rig 明确填 `*_walk_c` 并把前者改回"偏置不稳定性/先验 σ"的原意。
4. 杠杆臂角加速度项：当前忽略；若 IMU 安装距机体原点 > 10 cm 且需要高动态机动，再加 `α×r`（需要陀螺差分）。
