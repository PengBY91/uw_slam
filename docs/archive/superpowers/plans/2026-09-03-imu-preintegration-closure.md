# PREP-B-01 合法端到端闭环实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans task-by-task. Apply superpowers:test-driven-development to every production change and superpowers:verification-before-completion before changing status.

**Goal:** 在没有 `/gt/state` 或旧 relative-pose/VO 提示参与算法路径的前提下，用显式关键帧边界、IMU 预积分、深度和 sonar range 因子完成合成回放，达到 ATE ≤0.15 m。

**Architecture:** 新增不携带位姿的 `KeyframeBoundary` canonical 输入事件，作为 IMU 分段和图节点身份的唯一合同。首帧前静止预卷经独立 initializer 生成速度/偏置初值与先验；后续位姿只由惯性传播并由 IMU、depth、sonar range 因子优化。`reference_states` 只在求解后进入 evaluator。

**Tech Stack:** C++17、Eigen、protobuf、yaml-cpp、MCAP、GoogleTest/CTest；构建目录 `build_task`。

**Status (2026-09-03):** Tasks 1–7 executed in `.worktrees/week4-b01-clean`. Local acceptance met — `initialization=stationary`, imu 11 / sonar_range 36 / depth 12 factors, `relative_pose_factor_count=0`, solver converged in 13 iterations, **ATE rmse = 0.0858 m ≤ 0.15 m**, four algorithm trajectories byte-identical across original / ground-truth-deleted / ground-truth-displaced / ground-truth-retimed bags; 708 CTest green, both lints pass, `synthetic_smoke` byte-identical. HoloOcean 200 Hz / 30 s remains **external, not accepted**. Six deliberate deviations from this plan (smoothstep angular profile, inflated stationary bias sigma, mean-based stationary criterion, zero-Stamp span fix, bounded stationary look-back with a 0.75 s pre-roll, minimal-DOF observability counting) are recorded in `docs/imu-preintegration-design-2026-09-03.md` §8.6.

**Execution baseline:** `.worktrees/week4-prep` 中 `src/application/replay_pipeline.cpp`、`apps/synth_bag_gen.cpp` 和 IMU smoke 含有已确认不合格的实验实现，只保留作诊断对照。已创建 `.worktrees/week4-b01-clean`（branch `codex/week4-b01-clean`），只复制“前三周成果 + 文档 v1.3”，并确认旧实验的四个文件/配置不存在；Ceres-enabled 基线构建成功，657/657 CTest 通过。后续任务只在该干净工作树执行。

---

## Task 1：锁定配置语义与噪声字段

**Files:**

- Modify: `src/runtime/config.cpp`
- Modify: `tests/runtime/config_test.cpp`
- Modify: `include/sensor_models/imu_preintegration.hpp`
- Modify: `src/sensor_models/imu_preintegration.cpp`
- Modify: `tests/core/imu_preintegration_test.cpp`
- Modify: `configs/rig/example_auv_sonar_only.yaml`
- Create: `configs/experiment/synthetic_imu_preintegration.yaml`

### Step 1：写失败测试

在 config 测试中增加：

```cpp
TEST(Config, AcceptsImuPreintegrationEstimatorMode) {
  auto config = uw::runtime::LoadExperimentConfig(
      std::string(UW_REPO_ROOT) +
      "/configs/experiment/synthetic_imu_preintegration.yaml");
  EXPECT_EQ(config.estimator_mode, "imu_preintegration");
}

TEST(ImuPreintegration, NoiseFromRigDoesNotTreatBiasPriorAsRandomWalk) {
  uw::domain::ImuNoiseModel rig;
  rig.set_sigma_gyro_c(1e-4);
  rig.set_sigma_accel_c(1e-3);
  rig.set_sigma_gyro_bias(1e-3);
  rig.set_sigma_accel_bias(1e-2);
  EXPECT_FALSE(ImuPreintegrationNoise::FromRig(rig).has_value());
}
```

测试名按现有 fixture 辅助函数调整，但断言语义不能改变。先运行：

```bash
cmake --build build_task -j"$(nproc)"
ctest --test-dir build_task -R 'unit\.(runtime|core)\..*(Config|ImuPreintegration)' --output-on-failure
```

预期：模式白名单或 bias-walk 回退测试失败。

### Step 2：最小实现

- `ValidateExperimentConfigSelections()` 接受 `imu_preintegration`。
- `ImuPreintegrationNoise::FromRig()` 要求两个 `sigma_*_bias_walk_c` 均有限且大于 0；不再回退到 `sigma_*_bias`。
- `example_auv_sonar_only.yaml` 补充 `sigma_gyro_bias_walk_c: 1.0e-5` 与 `sigma_accel_bias_walk_c: 1.0e-4`；已有 `sigma_*_bias` 保持为初始偏置先验。
- 实验配置使用该 sonar-only rig、`synthetic_smoke` scenario 和平台 defaults。

### Step 3：验证

重复上述 CTest；再运行：

```bash
git diff --check -- src/runtime/config.cpp tests/runtime/config_test.cpp include/sensor_models/imu_preintegration.hpp src/sensor_models/imu_preintegration.cpp tests/core/imu_preintegration_test.cpp configs/rig/example_auv_sonar_only.yaml configs/experiment/synthetic_imu_preintegration.yaml
```

---

## Task 2：新增显式关键帧边界 canonical 合同

**Files:**

- Create: `schemas/proto/uw/domain/keyframe.proto`
- Modify: `include/domain/domain.hpp`
- Modify: `include/runtime/canonical_topics.hpp`
- Modify: `include/runtime/canonical_event.hpp`
- Modify: `include/runtime/canonical_event_validation.hpp`
- Modify: `src/runtime/canonical_event_validation.cpp`
- Modify: `src/runtime/mcap_event_source.cpp`
- Modify: `src/runtime/live_event_source.cpp`
- Modify: `include/application/pipeline_input_port.hpp`
- Modify: `src/application/event_pump.cpp`
- Modify: `include/application/online_assist_pipeline.hpp`
- Modify: `src/application/online_assist_pipeline.cpp`
- Modify: `src/application/holoocean_realtime_sink.cpp`
- Modify: `include/application/replay_input_accumulator.hpp`
- Modify: `src/application/replay_input_accumulator.cpp`
- Modify: `tests/runtime/canonical_event_test.cpp`
- Modify: `tests/runtime/canonical_event_validation_test.cpp`
- Modify: `tests/runtime/mcap_event_source_test.cpp`
- Modify: `tests/runtime/live_event_source_test.cpp`
- Modify: `tests/application/event_pump_test.cpp`
- Modify: `tests/application/online_assist_pipeline_test.cpp`
- Modify: `tests/application/replay_input_accumulator_test.cpp`
- Modify: `tests/integration/event_source_parity_test.cpp`

### Step 1：先写合同测试

测试必须覆盖：topic/schema 精确匹配、MCAP round-trip、live source 进入 localization lane、event pump 调用专用入口、accumulator 保持事件顺序、空 keyframe id/重复 id/非递增 capture time 拒绝。

目标 proto：

```proto
syntax = "proto3";
package uw.domain;

import "uw/domain/ids.proto";
import "uw/domain/observation.proto";

message KeyframeBoundary {
  ObservationHeader header = 1;
  KeyframeId keyframe_id = 2;
  string source = 3;
}
```

topic 固定为 `/keyframe/boundary`，角色为 `kAlgorithmInput`。先运行：

```bash
cmake --build build_task -j"$(nproc)"
ctest --test-dir build_task -R 'unit\.(runtime|application)\..*(Canonical|McapEventSource|LiveEventSource|EventPump|ReplayInputAccumulator)' --output-on-failure
```

预期：新类型尚未进入 variant/dispatch 时编译或测试失败。

### Step 2：最小实现

- `CanonicalEventKind` 增加 `kKeyframeBoundary`。
- `CanonicalPayload` 增加 `uw::domain::KeyframeBoundary`。
- `PipelineInputPort` 增加 `OnKeyframeBoundary()`；所有实现和测试 fake 显式实现。
- `ReplayInputData` 增加 `std::vector<KeyframeBoundary> keyframe_boundaries`。
- accumulator 校验 header、keyframe id 唯一和 capture time 严格递增；不从 log time 派生 capture time。
- live source 把该事件放 localization lane；`RawHeader()` 返回其 header。

### Step 3：验证

重复本任务 CTest，并确认已有 `/gt/state` 仍为 `kReferenceOnly`，live source 仍拒绝它进入算法 lane。

---

## Task 3：生成静止预卷、关键帧事件和独立 IMU 流

**Files:**

- Modify: `apps/synth_bag_gen.cpp`
- Modify: `tests/runtime/bag_audit_checks_test.cpp`
- Create: `tests/integration/synthetic_imu_fixture_test.sh`
- Modify: `cmake/Tests.cmake`

### Step 1：写失败的 fixture smoke

脚本生成 IMU experiment bag，并使用现有 MCAP 读取/审计工具断言：

- `/keyframe/boundary` 数量等于 keyframe 数；时间从 0.5 s 开始且严格递增。
- `/raw/imu` 从 0 s 开始，0–0.5 s 的真值为静止、200 Hz，覆盖首个边界。
- 运动段 IMU、depth、sonar、GT 的时间整体平移 0.5 s。
- 同 seed 的 IMU 字节一致；改变是否生成相机/relative-pose 不改变 IMU 流。

运行：

```bash
cmake --build build_task -j"$(nproc)"
ctest --test-dir build_task -R 'integration.synthetic_imu_fixture' --output-on-failure
```

预期：缺少 keyframe topic 和静止预卷而失败。

### Step 2：最小实现

仅当 experiment 的 `estimator_mode == "imu_preintegration"` 时启用 `pre_roll_s = 0.5`。静止段输出比力 `(0, 0, gravity_mps2)` 和零角速度，再叠加同一 IMU RNG 的白噪声/偏置；运动段沿用解析圆弧。写边界事件的核心形式：

```cpp
uw::domain::KeyframeBoundary boundary;
*boundary.mutable_header()->mutable_capture_time() = uw::domain::FromSeconds(t_s);
boundary.mutable_header()->mutable_observation_id()->set_value("boundary_" + kf_id);
boundary.mutable_header()->mutable_sensor_id()->set_value("keyframe_scheduler");
boundary.mutable_header()->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
boundary.mutable_keyframe_id()->set_value(kf_id);
boundary.set_source("synthetic_fixed_interval_v1");
writer.WriteMessage(uw::runtime::kTopicKeyframeBoundary, t_ns, boundary);
```

不得从 ground truth pose 填任何 boundary 字段。

### Step 3：验证非回归

除 fixture smoke 外，再运行现有确定性和 synthetic smoke，确认非 IMU experiment 的输出/ATE 未变化：

```bash
ctest --test-dir build_task -R 'integration\.(synthetic_imu_fixture|synthetic_smoke|replay_determinism)' --output-on-failure
```

---

## Task 4：实现静止初始化和惯性 anchor 先验

**Files:**

- Create: `include/frontends/imu_stationary_initializer.hpp`
- Create: `src/frontends/imu_stationary_initializer.cpp`
- Create: `tests/frontends/imu_stationary_initializer_test.cpp`
- Create: `include/factor_builders/inertial_prior_residual.hpp`
- Create: `src/factor_builders/inertial_prior_residual.cpp`
- Create: `tests/factor_builders/inertial_prior_residual_test.cpp`
- Modify: `cmake/Libraries.cmake`
- Modify: `cmake/Tests.cmake`

### Step 1：写失败测试

initializer 用例：0.5 s 静止成功；0.495 s 失败；陀螺超阈值失败；加速度模长偏差超阈值失败；输出 `bg0`、`ba0` 和确定的初始化模式。

prior residual 用例：参数布局 `[v(3), bg(3), ba(3)]`；目标处 9 维残差为零；sqrt information 正确缩放；解析雅可比等于对角矩阵；非有限/非正 sigma 拒绝。

接口目标：

```cpp
struct ImuStationaryInitialization {
  Eigen::Quaterniond rotation_WB;
  Eigen::Vector3d velocity_W;
  Eigen::Vector3d bias_gyro;
  Eigen::Vector3d bias_accel;
  Eigen::Matrix<double, 9, 1> sigma;
  enum class Mode { kStationary, kWideVelocityPrior } mode;
};
```

运行：

```bash
cmake --build build_task -j"$(nproc)"
ctest --test-dir build_task -R 'unit\.(frontends\.ImuStationaryInitializer|factor_builders\.InertialPriorResidual)' --output-on-failure
```

### Step 2：最小实现

- 静止阈值严格使用设计值：持续时间 ≥0.5 s、`abs(|a|-g) < 0.05 m/s²`、`|gyro| < 0.01 rad/s`。
- 静止成功：`v0=0`，`bg0=mean(gyro)`，`ba0=mean(accel)-expected_specific_force`。
- `rotation_WB` 只由平均比力确定 roll/pitch，yaw 固定为规范零值；initializer 保持 frontend 层纯数据结构，不依赖 `PoseGraphProblem`。
- 失败：`v0=0` 只是初值，速度 sigma 固定 0.5 m/s；bias sigma 来自 rig。
- anchor pose 固定以移除位姿 gauge，但 inertial block 不 fixed；通过一条 9 维 residual 约束。

### Step 3：验证

运行上述单测，并运行现有 IMU residual/双求解器测试，确认新先验没有改变已落地的 15 维因子：

```bash
ctest --test-dir build_task -R 'unit\.(frontends\.Imu|factor_builders\.(Imu|Inertial)|estimation\.PoseGraphSolver\.Imu)' --output-on-failure
```

---

## Task 5：按合法输入接入 replay，并恢复三类因子

**Files:**

- Modify: `src/application/replay_pipeline.cpp`
- Modify: `include/application/replay_pipeline.hpp`
- Modify: `tests/application/replay_pipeline_test.cpp`

### Step 1：先写图装配测试

构造不含 GT、不含 relative-pose evidence 的 `ReplayInputData`，含 3 个 boundary、覆盖区间的 IMU、3 个 depth evidence 和可检测 sonar frames。断言运行摘要：

```text
keyframe_boundary_count=3
imu_factor_count=2
depth_factor_count>0
sonar_range_factor_count>0
initialization=stationary
```

再加三个隔离测试：

1. 同一算法输入分别搭配“无 GT”“正确 GT”“时间和位姿被大幅篡改的 GT”，求解输出逐元素一致。
2. 加入与真实运动冲突的 relative-pose evidence，IMU 模式输出不变且 `relative_pose_factor_count=0`。
3. 删除 boundary 后即使 GT 完整也 fail closed，错误为 `imu_preintegration: fewer than two keyframe boundaries`。

先运行：

```bash
cmake --build build_task -j"$(nproc)"
ctest --test-dir build_task -R 'unit\.application\..*(Replay|Imu)' --output-on-failure
```

### Step 2：最小图装配实现

- ground-truth loop 只构造 `ground_truth_trajectory`，不得写 `capture_time_by_keyframe` 或 `problem`。
- IMU 分支从 `input.keyframe_boundaries` 建有序 keyframe 列表。
- anchor x/y/yaw 取规范零值，z 取首个对应 depth evidence；roll/pitch 由静止重力方向初始化。
- 每个后续位姿/速度只由上一个已估状态和当前 `PreintegratedImuDelta` 传播。
- 调 `AddInertialState()` 后添加 IMU residual；anchor 添加 inertial prior。
- 删除 IMU 模式跳过 sonar loop 的条件。sonar/depth 只按 boundary 的 keyframe id 关联。
- 用结构体而不是正则友好散文保存统计，最后一次性打印 `key=value` 摘要。

核心禁止项可直接形成代码审查 grep：

```bash
rg -n 'reference_states|RelativePoseMeasurement' src/application/replay_pipeline.cpp
```

命中允许存在于 evaluator 和非 IMU mode；IMU 分支内不得命中。

### Step 3：可观测性诊断

求解前检查每个非固定参数块至少被一个 residual 引用；打印自由参数维数、残差维数以及 solver 报告。若残差维数小于自由维数，或任一惯性块没有 IMU/先验约束，fail closed，不靠 LM damping 掩盖结构奇异。

### Step 4：验证

重复本任务 CTest，并运行：

```bash
ctest --test-dir build_task -R 'unit\.(application|estimation|factor_builders)' --output-on-failure
```

---

## Task 6：建立无泄漏端到端硬门槛

**Files:**

- Replace: `tests/integration/imu_preintegration_smoke_test.sh`
- Modify: `apps/synth_bag_gen.cpp`
- Modify: `cmake/Tests.cmake`

### Step 1：写严格 smoke

给 `synth_bag_gen` 增加仅影响 reference branch 的 `--omit-ground-truth`、`--ground-truth-time-offset-s` 和 `--ground-truth-pose-offset-m`。脚本用临时目录分别生成原始 bag、无 GT bag、GT capture timestamp/pose 被确定性篡改但 MCAP log time 与算法输入不变的 bag，再执行三次回放。每次保存完整 stdout/stderr，命令失败时先打印日志再退出。

解析器必须锚定 `key=value` 行，不使用 `imu_preintegration.*factor` 这类宽松正则。硬断言：

```text
solver_converged=true
keyframe_boundary_count>=2
imu_factor_count=keyframe_boundary_count-1
depth_factor_count>0
sonar_range_factor_count>0
relative_pose_factor_count=0
ate_rmse_m<=0.15
```

三次算法轨迹输出必须逐字节一致；ATE 只在含参考真值的原始 bag 上计算。迭代数打印为诊断，不设 4–10 的硬门槛。

### Step 2：运行并修复最小问题

```bash
cmake --build build_task -j"$(nproc)"
ctest --test-dir build_task -R 'integration\.imu_preintegration_smoke' --output-on-failure
```

若 ATE 不通过，先用因子计数、初始化模式、可观测性和残差分项定位；禁止重新引入 GT/relative-pose 初值或关闭 sonar 因子。

### Step 3：回归

```bash
ctest --test-dir build_task -R 'integration\.(imu_preintegration_smoke|synthetic_smoke|replay_determinism)' --output-on-failure
```

---

## Task 7：完整验证与状态更新

**Files:**

- Modify: `docs/imu-preintegration-design-2026-09-03.md`
- Modify: `docs/ROV平台到货前准备工作规格-2026-09-02.md`
- Modify: `docs/traceability/rov-realtime-closed-loop.csv`
- Modify: `docs/archive/superpowers/plans/2026-09-03-week4-pre-delivery-prep.md`

### Step 1：完整验证

```bash
cmake --build build_task -j"$(nproc)"
ctest --test-dir build_task --output-on-failure
python3 tools/lint/check_layer_dependencies.py .
python3 tools/lint/check_realtime_traceability.py docs/traceability/rov-realtime-closed-loop.csv .
git diff --check
```

记录测试总数、失败数、原始/无 GT/篡改 GT 三次轨迹摘要、三类因子计数、初始化模式、迭代数和 ATE。

### Step 2：更新状态

只有全部本地门槛通过，B-01 才标为“已本地验证”。HoloOcean 200 Hz、30 s 运行仍标为“外部待验收”，直到有真实 MCAP 和逐秒漂移报告。不得因为本地 synthetic smoke 通过而写“B-01 已外部验证”。

### Step 3：代码审查门

使用 `superpowers:requesting-code-review` 检查：

- GT 是否只在 evaluator。
- IMU mode 是否完全忽略 relative-pose evidence。
- 每个惯性块是否有 IMU 或先验约束。
- sonar range 因子是否真实非零。
- smoke 是否在失败时保留日志并使用严格字段解析。

审查问题全部关闭后，才允许进入 B-02 回放接线或 C-03 W5 外部导航计划。
