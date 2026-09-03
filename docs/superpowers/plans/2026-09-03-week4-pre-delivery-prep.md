# ROV 到货前准备第四周实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 完成第四周所有可在当前工作区验证的准备任务，并为 Windows/HoloOcean 专属验收项提供可复现入口。

**Architecture:** 先在离线 C++ 管线完成 IMU 预积分和航向因子闭环，再实现 MAVLink/IMU 的设备格式适配与伪装流。仿真、外观退化和训练台沿用现有 manifest、PilotCommand、MCAP 和评分器边界；真实 HoloOcean/UE5 验收单独记录，不与本地单测混淆。

**Tech Stack:** C++17、Eigen、protobuf、yaml-cpp、MCAP；Python 3、pymavlink、pytest；HoloOcean 2.3.0/UE5（仅外部验收）。

---

## 文件地图

- 修改 `src/runtime/config.cpp`、`include/runtime/config.hpp`：注册新估计模式及实验配置。
- 修改 `src/application/replay_pipeline.cpp`、`include/application/replay_pipeline.hpp`：接入 IMU 预积分、航向因子及验证输出。
- 修改 `apps/synth_bag_gen.cpp`、`cmake/Applications.cmake`：生成可回放的 IMU/航向合成证据。
- 新增/修改 `include/factor_builders/`、`src/factor_builders/`、`tests/factor_builders/`：航向因子及测试。
- 新增/修改 `adapters/mavlink/`、`adapters/holoocean/uw_holoocean_adapter/device_emulation/`：真机线上格式 adapter 和 MAVLink/IMU 伪装后端。
- 修改 `adapters/holoocean/uw_holoocean_adapter/fault_injector.py` 或新增退化模块及对应 pytest：相机退化评估。
- 修改 `configs/scenario/pool_example.yaml`、`adapters/holoocean/scenarios/`、训练/评分脚本：A-08/A-12 本地编排。
- 修改 `cmake/Libraries.cmake`、`cmake/Tests.cmake`、`docs/ROV平台到货前准备工作规格-2026-09-02.md`、`docs/specifications/holoocean-realtime-closed-loop-simulation-spec.md`、`docs/traceability/rov-realtime-closed-loop.csv`：构建注册、状态和追溯。

### Task 1: 完成 B-01 估计模式和配置选择器

**Files:**
- Modify: `include/runtime/config.hpp`
- Modify: `src/runtime/config.cpp`
- Modify: `tests/runtime/config_test.cpp`
- Create: `configs/experiment/synthetic_imu_preintegration.yaml`

- [ ] **Step 1: 写配置失败测试**：在 `config_test.cpp` 增加合法值 `imu_preintegration` 被接受、未知值仍被拒绝、实验文件能加载该值三项测试。
- [ ] **Step 2: 运行测试确认失败**：运行 `ctest --test-dir build -R 'unit.runtime.*Config|unit.runtime.config' --output-on-failure`；预期新合法值测试因校验白名单未更新而失败。
- [ ] **Step 3: 实现最小配置支持**：将 `imu_preintegration` 加入 `ValidateExperimentConfigSelections()` 的允许集合，错误信息同步列出三个合法值；新增实验配置复用现有 synthetic rig/scenario/defaults，并关闭视觉前端依赖。
- [ ] **Step 4: 运行配置测试**：运行同一 ctest 命令；预期全部通过。
- [ ] **Step 5: 提交前检查变更范围**：运行 `git diff --check -- include/runtime/config.hpp src/runtime/config.cpp tests/runtime/config_test.cpp configs/experiment/synthetic_imu_preintegration.yaml`。

### Task 2: 接入 B-01 回放分支和合成 IMU 端到端 demo

**Files:**
- Modify: `src/application/replay_pipeline.cpp`
- Modify: `apps/synth_bag_gen.cpp`
- Modify: `tests/application/replay_pipeline_test.cpp`
- Modify: `cmake/Applications.cmake`
- Create: `tests/integration/imu_preintegration_smoke_test.sh`

- [ ] **Step 1: 写回放分支测试**：构造包含相邻关键帧、IMU 预积分 evidence、深度和声呐 range evidence 的最小输入，断言 `imu_preintegration` 模式建立惯性状态/因子，并断言 `black_box_vio` 与 `stereo_landmark_vo` 路径输出不变。
- [ ] **Step 2: 写合成 IMU 输出测试**：在 `synth_bag_gen` 的测试或 smoke 脚本中断言 `/raw/imu` 消息按 200 Hz 采样、capture time 单调、首个关键帧前后样本数足够覆盖预积分区间。
- [ ] **Step 3: 运行新测试确认失败**：运行 `ctest --test-dir build -R 'unit.application|integration.imu_preintegration_smoke' --output-on-failure`；预期新模式尚未被回放消费时失败。
- [ ] **Step 4: 实现合成 IMU 流**：在 `apps/synth_bag_gen.cpp` 使用独立 seed/salt RNG 生成与 ground truth 一致的加速度、角速度和偏置扰动，写入规范 `/raw/imu`，不复用 pose/sonar RNG。
- [ ] **Step 5: 实现回放分支**：按关键帧 capture time 聚合 `/raw/imu`，调用现有 `InertialFrontend` 产生 `ImuPreintegrationMeasurement`，把 builder 绑定到相邻关键帧；缺样本、时间倒退和跨越过大间隙时 fail-closed 并记录原因。
- [ ] **Step 6: 加入 smoke 脚本**：生成 bag 后运行 `replay_demo --experiment configs/experiment/synthetic_imu_preintegration.yaml`，解析输出的收敛状态、迭代次数和 ATE。
- [ ] **Step 7: 运行红绿验证**：依次运行 `cmake --build build -j"$(nproc)"`、`ctest --test-dir build -R 'unit.application|integration.imu_preintegration_smoke' --output-on-failure` 和 smoke 脚本；预期 4–10 次迭代收敛且 ATE ≤ 0.15 m。

### Task 3: 实现 B-02 绝对航向因子

**Files:**
- Create: `include/factor_builders/heading_prior_residual.hpp`
- Create: `src/factor_builders/heading_prior_residual.cpp`
- Create: `include/factor_builders/heading_prior_factor_builder.hpp`
- Create: `src/factor_builders/heading_prior_factor_builder.cpp`
- Modify: `src/application/replay_pipeline.cpp`
- Modify: `cmake/Libraries.cmake`
- Modify: `cmake/Tests.cmake`
- Create: `tests/factor_builders/heading_prior_residual_test.cpp`

- [ ] **Step 1: 写数值雅可比和门控测试**：覆盖 yaw=0/90°、±π 环绕、四元数归一化、正常电流、高电流跳过、anchor yaw 用观测初始化。
- [ ] **Step 2: 运行测试确认失败**：运行 `ctest --test-dir build -R 'unit.factor_builders.*Heading|unit.application.*Replay' --output-on-failure`；预期因子类型尚未存在而失败。
- [ ] **Step 3: 实现残差**：在残差内部用 quaternion 与重力轴计算最短 yaw 差，输出一维白化残差和解析雅可比；不添加欧拉角状态。
- [ ] **Step 4: 实现 builder**：按 `sigma0 + current_a * slope` 生成信息量，高于 current gate 时返回拒绝结果，并保留拒绝原因/计数。
- [ ] **Step 5: 接入回放**：从 `VehicleState`/航向 evidence 读取磁航向，初始化 anchor yaw；普通关键帧添加航向先验，高电流帧跳过并打印统计。
- [ ] **Step 6: 运行测试和 lint**：运行 `cmake --build build -j"$(nproc)"`、`ctest --test-dir build -R 'unit.factor_builders|unit.application' --output-on-failure`、`tools/lint/check_layer_dependencies.py .`；预期测试通过且核心层无第三方依赖倒置。

### Task 4: 实现 C-03 MAVLink adapter 契约

**Files:**
- Create: `adapters/mavlink/README.md`
- Create: `adapters/mavlink/uw_mavlink_adapter/messages.py`
- Create: `adapters/mavlink/uw_mavlink_adapter/telemetry.py`
- Create: `adapters/mavlink/uw_mavlink_adapter/setpoint.py`
- Create: `adapters/mavlink/uw_mavlink_adapter/transport.py`
- Create: `adapters/mavlink/tests/test_telemetry.py`
- Create: `adapters/mavlink/tests/test_setpoint.py`
- Create: `adapters/mavlink/tests/test_fail_closed.py`

- [ ] **Step 1: 写 transport fake 测试**：用内存 transport 输入 `HEARTBEAT`、`ATTITUDE`、`SCALED_PRESSURE2`、`SYS_STATUS`、`BATTERY_STATUS`、`SYSTEM_TIME`，断言输出 canonical vehicle/health 消息；输入外部导航和设定值，断言发送正确 MAVLink message。
- [ ] **Step 2: 运行 Python 测试确认失败**：运行 `adapters/mavlink/.venv/bin/pytest -q adapters/mavlink/tests`；预期模块尚未创建而失败。
- [ ] **Step 3: 实现消息映射**：集中定义 MAVLink 到 protobuf 的字段和坐标转换，所有消息都带 source、capture time、sequence；不在核心层 include MAVLink 头。
- [ ] **Step 4: 实现命令输出**：将 `PilotCommand` 的手动轴/模式映射到 `MANUAL_CONTROL`，将 `MotionSetpoint` 映射到 `SET_POSITION_TARGET_LOCAL_NED`，将 SLAM 相对增量映射到 `VISION_POSITION_DELTA`。
- [ ] **Step 5: 实现 fail-closed**：连接失活、时间戳倒退、未知坐标系、必需字段缺失时不发控制命令并生成 `HealthReport`。
- [ ] **Step 6: 运行测试**：运行 `adapters/mavlink/.venv/bin/pytest -q adapters/mavlink/tests`；预期全部通过，并在 README 记录 SITL 连接命令和消息契约。

### Task 5: 实现 A-13 MAVLink/IMU 设备伪装流

**Files:**
- Create: `adapters/holoocean/uw_holoocean_adapter/device_emulation/__init__.py`
- Create: `adapters/holoocean/uw_holoocean_adapter/device_emulation/mavlink_emitter.py`
- Create: `adapters/holoocean/uw_holoocean_adapter/device_emulation/imu_udp_emitter.py`
- Modify: `adapters/holoocean/uw_holoocean_adapter/holoocean_bridge_sensor_host.py`
- Modify: `adapters/holoocean/uw_holoocean_adapter/scenario_manifest.py`
- Create: `adapters/holoocean/tests/test_device_emulation.py`
- Modify: `tests/integration/live_ingress_smoke_test.sh`

- [ ] **Step 1: 写伪装流测试**：断言 MAVLink emitter 能产生 C-03 所需设备消息，IMU emitter 与 D-02 wire format 完全一致，两个 emitter 使用仿真时间且不包含 ground truth。
- [ ] **Step 2: 运行测试确认失败**：运行 `(cd adapters/holoocean && .venv/bin/pytest -q tests/test_device_emulation.py)`；预期新 emitter 尚未实现而失败。
- [ ] **Step 3: 实现 MAVLink emitter**：优先接入 PREP-A-05 SITL bridge；没有 SITL 时提供 fake transport，只生成同样的消息序列，不复制控制逻辑。
- [ ] **Step 4: 实现 IMU emitter**：复用 `adapters/wit_imu` 的 protobuf 序列化和序号/健康字段，使用 manifest 中的仿真速率和时间源。
- [ ] **Step 5: 增加 backend 选择**：给 sensor host 增加 `--backend device_emulation`，保留原始 TCP backend；manifest 校验伪装流的 algorithm topics 不得等于 ground truth topic。
- [ ] **Step 6: 扩展 smoke**：为 `live_ingress_smoke_test.sh` 增加 MAVLink/IMU 伪装流用例，检查消息数量、时间单调性、真值隔离和断流后的 fail-closed。
- [ ] **Step 7: 运行测试**：运行 `(cd adapters/holoocean && .venv/bin/pytest -q)` 和 `tools/lint/check_layer_dependencies.py .`；真实 30 分钟 HoloOcean 运行记录为待外部验收，不伪报为本地通过。

### Task 6: 完成 A-09 相机外观退化离线评估

**Files:**
- Modify: `adapters/holoocean/uw_holoocean_adapter/sensor_perturbation.py`
- Create: `adapters/holoocean/uw_holoocean_adapter/camera_degradation.py`
- Create: `adapters/holoocean/tools/camera_degradation_report.py`
- Create: `adapters/holoocean/tests/test_camera_degradation.py`
- Create: `docs/camera-degradation-baseline-2026-09-03.md`

- [ ] **Step 1: 写确定性退化测试**：覆盖浑浊度、低照度、色偏、运动模糊、压缩伪影和分辨率/帧率档位；同 seed 输出字节一致，强度为零时逐像素不变。
- [ ] **Step 2: 运行测试确认失败**：运行 `(cd adapters/holoocean && .venv/bin/pytest -q tests/test_camera_degradation.py)`；预期模块不存在而失败。
- [ ] **Step 3: 实现退化管线**：按固定顺序应用颜色/照度、浑浊散射、模糊、编码/尺寸退化；所有随机数从显式 RNG 传入，不调用全局 seed。
- [ ] **Step 4: 实现报告工具**：输入录制图像或 MCAP，输出退化档位、检测成功率/跟踪成功率、图像质量指标和推荐码率下限；没有 HoloOcean 输入时明确输出“仅工具校验”。
- [ ] **Step 5: 运行测试并生成基线**：运行 pytest，生成 `docs/camera-degradation-baseline-2026-09-03.md`；Windows HoloOcean 实测曲线作为后续补充，不在本地验收中虚构数值。

### Task 7: 完成 A-08 水池关卡和 A-12 飞手训练台本地编排

**Files:**
- Modify: `configs/scenario/pool_example.yaml`
- Modify: `adapters/holoocean/uw_holoocean_adapter/scripted_pilot.py`
- Modify: `adapters/holoocean/uw_holoocean_adapter/pilot_ros_bridge.py`
- Modify: `adapters/holoocean/uw_holoocean_adapter/task_scorer.py`
- Create: `adapters/holoocean/tools/run_pilot_training.py`
- Create: `adapters/holoocean/tests/test_pilot_training.py`
- Create: `docs/pool-scene-and-pilot-training-2026-09-03.md`

- [ ] **Step 1: 写本地编排测试**：用 fake sensor/pilot backend 验证 scripted pilot 产生合法 `PilotCommand`，训练运行能写 MCAP、调用 scorer，并在缺失传感器/非法模式时停止发命令。
- [ ] **Step 2: 运行测试确认失败**：运行 `(cd adapters/holoocean && .venv/bin/pytest -q tests/test_pilot_training.py)`；预期训练入口不存在而失败。
- [ ] **Step 3: 固化水池场景配置**：在 `pool_example.yaml` 明确池体边界、起点、深度、控制点和评分阈值；保留 `spawn_prop` fallback，不把不存在的 UE5 资产当作已加载。
- [ ] **Step 4: 实现训练入口**：串联 pilot command、录制器、评分器和 run report，支持 dry-run/fake backend；所有运行参数写入 manifest。
- [ ] **Step 5: 运行本地验证**：运行 pytest 和 `python adapters/holoocean/tools/run_pilot_training.py --dry-run --scenario configs/scenario/pool_example.yaml`；预期有录制、评分和可诊断失败输出。
- [ ] **Step 6: 写外部验收步骤**：文档给出 Windows HoloOcean 加载水池关卡、真实手柄训练、两名飞手各三次录制的命令、输出位置和判定标准。

### Task 8: 更新规格、追溯和执行记录

**Files:**
- Modify: `docs/ROV平台到货前准备工作规格-2026-09-02.md`
- Modify: `docs/specifications/holoocean-realtime-closed-loop-simulation-spec.md`
- Modify: `docs/traceability/rov-realtime-closed-loop.csv`
- Modify: `docs/README.md`

- [ ] **Step 1: 对照本计划逐项更新状态**：每个第四周任务填写已完成内容、证据文件、本地验证结果和外部待验收条件。
- [ ] **Step 2: 更新仿真规格**：增加设备伪装流、IMU profile、航向噪声/门控和相机退化的需求条目，标注仿真时间与墙钟时间语义。
- [ ] **Step 3: 更新 traceability**：为新增/变更的 B-01、B-02、C-03、A-09、A-08、A-12、A-13 记录实现文件和测试入口。
- [ ] **Step 4: 运行文档和追溯检查**：运行 `python3 tools/lint/check_realtime_traceability.py docs/traceability/rov-realtime-closed-loop.csv .` 和 `git diff --check`；预期无追溯或格式错误。

### Task 9: 完整验证和交付检查

**Files:**
- Verify: `build/`, `adapters/holoocean/`, `adapters/mavlink/`, `docs/`

- [ ] **Step 1: 编译**：运行 `cmake --build build -j"$(nproc)"`；预期 exit code 0。
- [ ] **Step 2: C++ 全量测试**：运行 `ctest --test-dir build --output-on-failure`；记录总数和失败数。
- [ ] **Step 3: Python 测试**：运行 `(cd adapters/holoocean && .venv/bin/pytest -q)` 以及 `adapters/mavlink/.venv/bin/pytest -q adapters/mavlink/tests`；记录总数和失败数。
- [ ] **Step 4: 架构检查**：运行 `tools/lint/check_layer_dependencies.py .`；预期 exit code 0。
- [ ] **Step 5: 端到端检查**：运行 IMU smoke、现有 `synthetic_smoke` replay 和 live ingress smoke；对每个指标保存输出路径。
- [ ] **Step 6: 对照验收总表**：确认每项都有证据或明确标为 Windows/HoloOcean 待验收；只有在命令输出确认后才在规格中写“已完成”。

