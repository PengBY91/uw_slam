# 测试与验证指南

> 状态：**当前说明**。回答"每一块功能怎么测、需要加载什么运行环境、怎么判断跑对了"。
> 不是构建入门——先看[根 README](../README.md)的「快速开始」。这里假设依赖已经装好
> （`./tools/setup_dev_env.sh` 已跑过），只讲验证流程本身。

## 先跑一键验证

大多数场景只需要这一条命令，它按顺序跑完 README「构建 / 运行端到端 Demo / 测试
策略」三节描述的每一步，并把每步的命令、完整日志、耗时和 PASS/FAIL 状态写到
`--out-dir` 下：

```bash
tools/verify_pipeline.sh --out-dir /tmp/uw_slam_verify/check --with-ros2
cat /tmp/uw_slam_verify/check/summary.txt
```

不加 `--with-ros2` 时脚本只跑不需要真实仿真器/ROS2 的部分（第 1~6 步）；`~/ros2_ws`
或 `/opt/ros/jazzy/setup.bash` 不存在时，即使加了 `--with-ros2` 也会自动跳过第
7/8 步而不报错。脚本本身会探测 `uw_slam_build` conda env 并加入 `PATH`，不需要手动
`export`。

2026-08-20 在本机（commit `90b7752`）实测过完整 8 步（含 `--with-ros2`），全部
PASS：编译 → 23 项 ctest → 11 项 pytest → lint → `synth_bag_gen` → `replay_demo`
（ATE rmse ≈0.213m）→ ROS2 桥 configure → ROS2 桥 build。

## 分项验证：每个功能怎么单独测

改动只涉及某一模块时，不必每次跑全量脚本，按下表挑对应命令即可（ctest 用例现在按
架构层分组注册为 `unit.<layer>.*`/`contract.*`/`integration.*`，用 `-L <label>` 选层比
拼单个用例名字更稳）；跨 `include/`/`src/` 架构层接口的改动仍建议跑一次全量
`verify_pipeline.sh`。

| 功能 | 验证命令 | 需要的运行环境 | 判定标准 |
|---|---|---|---|
| 领域契约（protobuf round-trip） | `ctest --test-dir build -L contract` | `uw_slam_build` conda env 在 `PATH` 里 | 通过 |
| 声呐 CFAR 前端 | `ctest --test-dir build -R SonarCfarFrontend` | 同上 | 通过（fixture 回归） |
| 因子构建（雅可比数值验证） | `ctest --test-dir build -L factor_builders` | 同上 | 通过 |
| 状态估计（Gauss-Newton/LM） | `ctest --test-dir build -L estimation` | 同上 | 通过 |
| 地图 + 轨迹评测 | `ctest --test-dir build -L "mapping\|evaluation"` | 同上 | 通过 |
| 端到端声呐 pipeline demo | 见下方「手动跑端到端 Demo」 | 同上 | 6~7 次迭代收敛，ATE rmse 0.15~0.22m |
| 光学立体深度 baseline（acoustic-optic plan 2） | `build/bin/synth_stereo_gen --out /tmp/stereo.mcap`，再 `build/bin/optical_baseline_eval --bag /tmp/stereo.mcap --experiment configs/experiment/synthetic_smoke.yaml --max-rmse-m 0.05 --min-coverage 0.9` | 同上 | 打印一行 `rmse_m=... coverage=... OK`；退出码 0 |
| 声光融合场景矩阵（plan 5） | `build/bin/acoustic_optic_scenario_matrix --experiment configs/experiment/synthetic_smoke.yaml --seed 4242 --trials-per-scenario N` | 同上 | 每个 scenario 打印一行统计；**退出码非 0 是正常的**——反映的是场景本身触发的 MVP gate 结果，不是命令执行失败，见 `tests/integration/acoustic_optic_scenario_matrix_determinism_test.sh` 顶部注释 |
| 确定性回放（集成测试） | 已含在 ctest 里：`integration.replay_determinism`、`integration.optical_baseline_smoke`、`integration.acoustic_optic_scenario_matrix_determinism`（`ctest --test-dir build -L integration`）；也可单独 `bash tests/integration/<name>.sh <对应二进制路径...>` | 同上 | 同 seed 两次运行输出逐字节一致（scenario matrix 一项排除 `p95_latency_ms`，因为它是真实墙钟耗时，本来就不该要求确定性） |
| HoloOcean Python 网关（坐标变换 / MCAP writer / 场景随机化，不含真实仿真器调用） | `cd adapters/holoocean && .venv/bin/pytest`（首次需要 `.venv/bin/pip install -e ".[dev]"`，见 [HoloOcean 适配器 README](../adapters/holoocean/README.md)） | `adapters/holoocean/.venv`，**不能**用 conda env 里的 `pytest`——会解析到 base conda 环境，既缺 `uw_holoocean_adapter` 包，又会因为 protobuf gencode/runtime 版本不一致直接报 `VersionError` | 25/25 通过 |
| ROS2 桥接节点（传输层） | 编译：`cmake -S . -B build_ros2 -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build" -DUW_BUILD_ROS2=ON && cmake --build build_ros2 --target holoocean_sonar_bridge_node`；独立启动：`source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ./build_ros2/bin/holoocean_sonar_bridge_node` | `uw_slam_build` env + 已 source 的 ROS2 Jazzy + 已 colcon build 好 `holoocean_interfaces` 的 `~/ros2_ws` | 编译链接成功；节点启动后常驻不退出/不崩溃即算正常——它是纯传输层，没有真实 `holoocean_main` 喂话题、也没接 `SonarFrontend`，安静地空转就是预期行为，见 [ROS2 适配器 README](../adapters/ros2/README.md) |
| Protobuf schema 改动后重新生成 Python 绑定 | `tools/codegen/gen_py.sh` | `adapters/holoocean/.venv` | 生成/更新 `schema_pb2/`；跑完之后 pytest 应仍然全过 |
| 依赖方向不变量（`include`/`src` 生产代码不许 include ROS/HoloOcean，也不许用旧 `uw/...` 手写头路径） | `tools/lint/check_no_ros_in_core.sh`（等价于 `tools/lint/check_layer_dependencies.py .`） | 无特殊环境 | 打印 `OK: ...` |

### 手动跑端到端 Demo

```bash
export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"

build/bin/synth_bag_gen \
  --experiment configs/experiment/synthetic_smoke.yaml \
  --out /tmp/synthetic.mcap

build/bin/replay_demo \
  --bag /tmp/synthetic.mcap \
  --experiment configs/experiment/synthetic_smoke.yaml \
  --out /tmp/demo

cat /tmp/demo_trajectory.tum
```

期望输出里能看到 `solver: N iterations, cost ... -> ... (converged)` 和一行
`ATE: rmse=...m ...`；不同 seed 下 rmse 会在 0.15~0.22m 之间波动，这不是验收阈值
（原因见根 README「运行端到端 Demo」一节：sonar_range_factor 目前不联合优化路标，
首次观测的 elevation 误差会摊到 x/y 上）。

## 目前测不了的部分（不是环境配置问题，是仓库当前阶段本来没做）

- **HoloOcean 真实仿真器**：`HoloOceanSession` 实际调用 HoloOcean/UE5 的部分写了但
  没跑过——这台机器没有 Unreal Engine 二进制和 Epic Games 账号联动。能测的只有上表
  里坐标变换/writer/随机化那一行（纯逻辑，不需要仿真器）。
- **公开数据集 adapter**（`adapters/datasets/`）：只是接口占位，未实现，没有可跑的
  测试。
- **ROS2 桥下游数据流**：节点能编译、能独立启动，但没有真实 `holoocean_main` 进程
  喂话题，也没有接到 `SonarFrontend::ProcessSonarFrame`，所以"跑起来看到声呐数据被
  处理"这件事目前做不到——这是[架构文档](./acoustic-optic-slam-platform-architecture-2026-08-17.md)
  记录的已知边界，不是没测对。

## 环境速查

| 环境 | 用途 | 加载方式 |
|---|---|---|
| `uw_slam_build` conda env | C++ 构建、ctest、所有 `build/` 下的可执行文件 | `export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"` |
| `adapters/holoocean/.venv` | Python 适配器 pytest、`gen_py.sh` | 直接用 `.venv/bin/pytest` / `.venv/bin/python`，不要指望 `PATH` 上的全局 `pytest` |
| ROS2 Jazzy + `~/ros2_ws` | 编译/运行 `uw_holoocean_sonar_bridge_node` | `source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash`，两者都要 source，缺一个 `find_package(holoocean_interfaces)` 就会失败 |

三个环境互不重叠，也不需要同时加载——按你要测的那一项对号入座即可。
