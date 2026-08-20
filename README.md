# uw_slam — 水下声光融合 SLAM 平台

`uw_slam` 是水下自主平台声呐 + 光学融合 SLAM 的长期代码框架，是
[`acoustic-optic-slam-platform-architecture-2026-08-17.md`](./docs/acoustic-optic-slam-platform-architecture-2026-08-17.md)
（长期架构设计，下称"架构文档"）的第一次真实代码落地，按架构文档第 5-7 节的
依赖不变量、领域契约、仓库边界独立搭建。`ocean_t`/`SVIn`/`sonar_camera_
reconstruction` 三个既有代码库保持不动，只把其中有价值的具体实现（声呐 range
因子、CFAR 前端）移植或参考过来。配套的第一阶段工程方案见
[`holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md`](./docs/holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md)。

现在的状态：骨架加上每一层至少一条可编译、可运行、有测试覆盖的真实垂直切片，
不是接口占位。13/13 C++ 测试、9/9 Python 测试通过，有一条端到端可跑的合成数据
demo。离生产可用还远，尚未做的部分列在"已知边界"一节。

## 目录

- [与仓库内其他目录的关系](#与仓库内其他目录的关系)
- [许可与代码出处](#许可与代码出处)
- [架构总览](#架构总览)
- [仓库结构](#仓库结构)
- [构建](#构建)
- [运行端到端 demo](#运行端到端-demo)
- [分层配置系统](#分层配置系统)
- [测试策略](#测试策略)
- [已知边界](#已知边界)
- [延伸阅读](#延伸阅读)

## 与仓库内其他目录的关系

`external_repos/{SVIn,sonar_camera_reconstruction,ocean_t}/` 是独立的参考仓库，
本仓库不修改它们，各自保留自己的 git 历史。整个 `external_repos/` 已加入
`.gitignore`，不纳入本仓库版本控制：

| 目录 | 关系 |
|---|---|
| `external_repos/SVIn/` | 上游第三方仓库（GPLv3）。`sonar_range_factor` 移植了其 `SonarError` 残差公式，见 [`NOTICE`](./NOTICE) 第 1 节。其余部分（双目 VIO 内部状态估计）不移植，是架构文档明确的非目标，只作为外部黑盒对比 baseline。 |
| `external_repos/sonar_camera_reconstruction/` | 上游第三方仓库（MIT）。`sonar_cfar_frontend` 移植了其 CFAR 检测 + 极坐标转换 + DBSCAN 聚类，见 [`NOTICE`](./NOTICE) 第 2 节。其 `merge.py` 里丢弃 pitch、把点云直接转换并固定到 `map` frame（不保留局部坐标引用，后续轨迹修正无法回溯）的部分没有被移植（架构文档 22.1 节的审计发现），本仓库的前端只输出声呐局部坐标系下的证据。 |
| `external_repos/ocean_t/` | 团队早期的 HoloOcean 驱动原型，已被 `adapters/holoocean/` 取代，不再维护，仅保留供历史参考。 |
| `external_repos/holoocean-ros/` | 上游第三方仓库（MIT，byu-holoocean/holoocean-ros）。HoloOcean 官方 ROS2 接口，是当前实际在用的仿真工具。`holoocean_main` 加载场景并把传感器数据发到 ROS2 话题（`holoocean_interfaces` 定义的消息类型），`holoocean_examples` 是操作示例。`adapters/third_party/holoocean_ros_bridge` + `adapters/ros2` 消费其 `ImagingSonar` 话题（见下）。 |
| `external_repos/holoocean_bridge/` | 一个同事独立维护的 HoloOcean→ROS2 桥接包，未 vendor 进本仓库（不同 provenance/license 链）。`adapters/third_party/svin_bridge`、`adapters/third_party/sonar_camera_reconstruction_baseline` 的 README 已经把它记录为「可参考、不可直接抄」；`adapters/third_party/holoocean_ros_bridge` 的镜像翻转等转换细节也是读它推导出来的，见该目录 README。 |

## 许可与代码出处

本仓库整体采用 GPLv3（见 [`LICENSE`](./LICENSE)），原因是直接移植了 GPLv3 许可的
`SVIn` 代码。移植文件保留原始文件头版权声明，每一处移植的具体来源、移植了什么、
刻意没有移植什么，都记录在 [`NOTICE`](./NOTICE) 里，修改移植代码前请先读一遍，
避免破坏可追溯性。

## 架构总览

依赖只允许单向从下往上：`core → algorithms → runtime → adapters → apps`；`core/` 和
`algorithms/` 内不允许出现任何 ROS/HoloOcean/第三方 vendor 头文件（由
`tools/lint/check_no_ros_in_core.sh` 强制检查）。跨语言（C++/Python）领域契约的唯一
事实源是 Protobuf（`schemas/proto/`），canonical 的录制/回放格式是 MCAP。

| 层 | 作用 | 本仓库中的具体实现 |
|---|---|---|
| `schemas/` | 领域契约唯一事实源 | `ObservationHeader`/`SonarFrame`/`MeasurementEvidence`/`FactorCandidate`/`StateSnapshot`/`MapEvidence`/`HealthReport`/`RigCalibrationSnapshot` 等 Protobuf 消息，生成 C++/Python 绑定 |
| `core/` | 领域类型 + 传感器物理模型 + 前端/后端抽象接口，零 ROS/仿真器依赖 | `uw_domain`（protobuf 生成类型 + 强类型 ID）、`uw_sensor_models`（`Pose3`、声呐 beam 模型）、`uw_measurement_api`（`Frontend<T>`/`FactorBuilder`/`ResidualBlock` 抽象） |
| `algorithms/` | 具体算法实现，各自独立 target，新增一个不改已有的 | `frontends/sonar_cfar_frontend`（移植自 sonar_camera_reconstruction）、`factor_builders/{sonar_range_factor（移植自 SVIn）, relative_pose_factor, depth_factor}`、`estimation`（Eigen 手写 Gauss-Newton，Solver 接口可插拔）、`mapping/submap_manager` |
| `runtime/` | 状态机、四车道 bounded-queue 调度器、`RunManifest`、分层配置加载 | `state_machines.hpp`、`bounded_queue.hpp`、`run_manifest.hpp`、`config.hpp`（yaml-cpp） |
| `adapters/` | ROS2、HoloOcean、第三方厂商消息只存在于这一层 | `holoocean/`（Python，直连 HoloOcean Python API，取代 `ocean_t`）、`ros2/`（`UW_BUILD_ROS2` 开关保护，含已验证可编译运行的 `holoocean_sonar_bridge_node`）、`third_party/{svin_bridge, sonar_camera_reconstruction_baseline, holoocean_ros_bridge}`、`datasets/` |
| `apps/` | 可执行入口 | `replay_demo`（端到端回放+估计+评测）、`tools/synth_bag_gen`（合成带 GT 的 MCAP bag） |
| `evaluation/` | 评测指标 | ATE/RPE 等轨迹指标 |

## 仓库结构

```
schemas/proto/     领域契约的唯一事实源（Protobuf），生成 C++/Python 绑定
core/               uw_domain / uw_sensor_models / uw_measurement_api：无 ROS/仿真器依赖
algorithms/         frontends / factor_builders / estimation / mapping
runtime/            状态机、四车道调度器、RunManifest、分层配置加载
adapters/           holoocean / ros2 / datasets / third_party：ROS2、厂商消息只存在于这一层
apps/               replay_demo、synth_bag_gen 等可执行入口
configs/            defaults → rig → scenario → experiment 分层配置
evaluation/         ATE/RPE 等评测指标
tests/              L0 契约（跨模块）/ L2 确定性回放；L1 单元测试随各模块自己的 test/ 目录
tools/              codegen、lint、开发环境安装脚本
```

## 构建

依赖：`cmake`（≥3.22）、`g++`（C++17）、`protobuf-compiler`/`libprotobuf-dev`、
`libeigen3-dev`、`libgtest-dev`、`libyaml-cpp-dev`。MCAP C++ SDK 没有包管理器分发，
通过 CMake `FetchContent` 从源码拉取（见 `cmake/UwMcap.cmake`，只用其 header-only
实现，`MCAP_COMPRESSION_NO_{ZSTD,LZ4}` 关闭压缩后端以避免额外依赖）。

`./tools/setup_dev_env.sh` 优先尝试 `apt-get`。本仓库在一个仅 HTTPS 出网正常、
HTTP(80) 镜像大量超时的沙箱环境中开发，apt 的包镜像常不可达，`apt-get` 会卡住不动，
脚本会自动回退到用 `conda-forge` 建一个独立的 `uw_slam_build` 环境。两条路径二选一
即可：

```bash
./tools/setup_dev_env.sh

# 走 apt 路径（系统级安装）：
cmake -S . -B build
cmake --build build -j"$(nproc)"

# 走 conda 回退路径：
export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build -j"$(nproc)"

ctest --test-dir build --output-on-failure   # 13/13 通过
```

Python 侧（`adapters/holoocean/`）：

```bash
cd adapters/holoocean
uv sync   # 或 pip install -e .
pytest    # 9/9 通过
```

`adapters/ros2`（`-DUW_BUILD_ROS2=ON`）需要一个 source 好的 ROS2 Jazzy 环境，
外加 `external_repos/holoocean-ros/holoocean_interfaces`（HoloOcean 官方 ROS2 消息
包）colcon build 到 `CMAKE_PREFIX_PATH` 上，因为它不在公共 ROS2 包索引里，
`find_package(holoocean_interfaces REQUIRED)` 才能成功。下面是本机验证过的路径：

```bash
# 1) ROS2 Jazzy（apt，见 https://docs.ros.org/en/jazzy/Installation.html）
sudo apt-get install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep

# 2) colcon workspace：symlink 进 holoocean-ros 的三个包（不拷贝，external_repos/ 保持只读）
mkdir -p ~/ros2_ws/src
ln -s "$(pwd)/external_repos/holoocean-ros/holoocean_interfaces" ~/ros2_ws/src/
ln -s "$(pwd)/external_repos/holoocean-ros/holoocean_main" ~/ros2_ws/src/
ln -s "$(pwd)/external_repos/holoocean-ros/holoocean_examples" ~/ros2_ws/src/

# 3) 建这一步之前 conda deactivate ——base conda 环境会让 CMake 优先选中缺
#    catkin_pkg 的 conda Python，报 ModuleNotFoundError（见 CLAUDE.md「已经踩过的坑」）
conda deactivate
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws && colcon build --symlink-install && cd -

# 4) 编译 uw_slam 本体时反过来要用 conda 自己的 cmake（系统 apt 的 cmake 3.28
#    的 FindProtobuf.cmake 生成路径有 bug，见 CLAUDE.md）
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build" -DUW_BUILD_ROS2=ON
cmake --build build -j"$(nproc)" --target uw_holoocean_sonar_bridge_node
```

见 `adapters/ros2/README.md`：这条路径已经编译+启动验证过（不需要 UE5/HoloOcean
仿真本体也能跑通传输层），但还没接过真实的 `holoocean_main` 数据流。

## 运行端到端 demo

`synth_bag_gen` + `replay_demo` 的合成数据 demo 不需要 HoloOcean/ROS2 环境。用
`synth_bag_gen` 生成带已知真值的合成 MCAP bag，再用 `replay_demo` 跑完整的
前端 → 因子构建 → 位姿图求解 → 评测 流程。
两种驱动方式在默认参数下应给出相同结果：命令行参数（快速试验），或者
`--experiment` 接入分层 YAML 配置。显式传入的 CLI 参数会覆盖 `--experiment` 里
对应的值。

```bash
# 方式一：命令行参数
build/apps/tools/synth_bag_gen/synth_bag_gen --out /tmp/synthetic.mcap
build/apps/replay_demo/replay_demo --bag /tmp/synthetic.mcap --out /tmp/demo

# 方式二：分层配置
build/apps/tools/synth_bag_gen/synth_bag_gen \
  --experiment configs/experiment/synthetic_smoke.yaml --out /tmp/synthetic.mcap
build/apps/replay_demo/replay_demo \
  --bag /tmp/synthetic.mcap --experiment configs/experiment/synthetic_smoke.yaml --out /tmp/demo

# 典型输出：solver 在 6~7 次迭代内收敛，ATE rmse ~0.15-0.21m（跨 seed 有波动）
cat /tmp/demo_trajectory.tum
```

ATE 不再是早期版本的 ~3cm。原因是 `sonar_range_factor` 的路标关联从"直接读
`/scenario/sonar_targets` 的真值坐标"换成了真实的 `SubmapManager::QueryNearestPoint()`
在线发现：第一次看到某个路标时只能假设它和当前 pose 同一个 elevation（声呐没有
仰角信息），z 被 depth 因子固定后，这个初值误差会转嫁到 x/y 分量上。这是架构文档明确记录
的 v1 限制（没有联合路标估计），不是回归，细节见 `CODEBASE_GUIDE.md` 8.2 节。

## 分层配置系统

对应架构文档第 14.2 节的 `defaults → rig → scenario → experiment` 四层叠加，详细字段
说明见 [`configs/README.md`](./configs/README.md)。解析代码（yaml-cpp）在
[`runtime/include/uw/runtime/config.hpp`](./runtime/include/uw/runtime/config.hpp) /
`runtime/src/config.cpp`：

| 层 | 描述的是 | 目前的消费程度 |
|---|---|---|
| `defaults/` | 平台级默认值（求解器参数、因子 sqrt-information 常数、runtime 车道配置），不含任何具体机体/场景信息 | 完整接入 `replay_demo` |
| `rig/` | 标定唯一事实源，对应 `RigCalibrationSnapshot`（外参、内参、噪声模型） | 直接解析进 protobuf 消息（不是另建一套 struct），完整加载 |
| `scenario/` | "跑什么数据"：world、控制、退化、故障、seed | 完整接入 `synth_bag_gen` |
| `experiment/` | "怎么跑"：选择 frontend、estimator mode、reliability policy、map backend、算力预算 | 引用其他三层的字段完整消费；`frontends`/`estimator_mode`/`map_backend` 这类选算法变体的字段目前只读取并打印，还没有真正驱动分支选择 |

每次运行都会产出一个不可变 `RunManifest`（见
[`runtime/include/uw/runtime/run_manifest.hpp`](./runtime/include/uw/runtime/run_manifest.hpp)），
记录实际生效的配置/标定/代码/模型哈希。

## 测试策略

对应架构文档验收面设计，三层测试：

- L0 契约（`tests/l0_contracts/`）：Protobuf round-trip，保证 schema 是唯一事实源
  这件事在代码层面成立。
- L1 单元：每个移植/新写的因子、frontend、solver 各自的单元测试（就近放在各自
  模块的 `test/` 目录下），包括：`sonar_range_factor` 用有限差分验证解析雅可比、
  `sonar_cfar_frontend` 用固定 fixture 声呐图做 CFAR + DBSCAN 的 golden-value 回归、
  `config_test` 直接读真实 `configs/experiment/synthetic_smoke.yaml` 逐字段断言解析
  结果。
- L2 确定性回放（`tests/l2_replay/determinism_test.sh`）：同一 bag/config/seed 跑
  两次 `replay_demo`，结果必须逐字节一致。这是"没有藏着全局可变随机状态"的直接
  验证，也是修过的一类真实 bug（见下）的回归测试思路来源。

```bash
ctest --test-dir build --output-on-failure   # C++：13/13
cd adapters/holoocean && pytest               # Python：9/9
tools/lint/check_no_ros_in_core.sh            # 依赖不变量：core/algorithms 不含 ROS 头
```

## 已知边界

- `adapters/ros2/` 现在有一个真实编译+启动验证过的目标
  （`uw_holoocean_sonar_bridge_node`，对着 ROS2 Jazzy + `holoocean_interfaces` 编译
  链接成功、启动无报错），但没有接过真实的 `holoocean_main`/HoloOcean 仿真进程
  （需要 UE5 二进制 + Epic Games EULA，本机没做这一步），也没有把它接到
  `SonarFrontend`/`apps/replay_demo` 的下游。`uw_ros2_svin_bridge`（svin_bridge 的
  ROS2 包装）仍然是全注释的文档骨架，没有真实代码可编译，两者状态不同，见
  `adapters/ros2/README.md` 的对照表，不要混为一谈。
  `adapters/third_party/svin_bridge/` 本身（不含 ROS2 包装那层）在当前开发机上没有
  真实 SVIn 进程可对，只做到接口和文档层面的正确性，代码与本文档中均有标注，不假装
  已验证。
- `--experiment` 目前把 `defaults` 层的求解器参数/因子 sqrt-information 常数，以及
  `rig`/`scenario` 层的数据完整接进 `synth_bag_gen`/`replay_demo`；`experiment` 层里
  `frontends`/`estimator_mode`/`map_backend` 等算法变体选择字段只是被读取和打印，
  尚未真正驱动"选哪个 frontend/estimator"这类分支（目前两个 app 各自只实现一条
  固定管线）。
- 第一版求解器是 Eigen 手写的 Gauss-Newton/LM，还没换成 Ceres/GTSAM。这是架构文档第
  20 节明确记录的延后决策，`Solver` 接口留了替换口子，但目前只有一种实现。
- 位姿图只优化 keyframe 变量，不联合优化路标点（架构文档 10.4.5 节）；数据关联
  现在走真实的 `SubmapManager::QueryNearestPoint()`（不再直接读 ground truth），
  但新路标第一次被发现时的 elevation 只能假设和当前 pose 一致（声呐没有仰角
  信息），之后也不会被后续因子精化，ATE 变化的解释见上一节。
- `reliability` 的 `final_information = min(learned, physical, calibration, cross_modal)`
  多路 cap（架构文档 8.4 节）v1 只实现了固定常数上限，真正的多路自适应是后续工作。

## 延伸阅读

- [`uw-slam-codebase-reference-2026-08-18.md`](./docs/uw-slam-codebase-reference-2026-08-18.md) ——
  代码级参考文档：逐层记录当前代码库里实际存在的类型/函数签名/算法参数，以及
  它们如何连成 `synth_bag_gen`→`replay_demo` 这条端到端管线，含若干处「文档说
  X、代码实际是 Y」的出入标注；和下面两份「应该长成什么样」的设计文档不同，只
  记录读一遍代码能确认的事实。
- [`acoustic-optic-slam-platform-architecture-2026-08-17.md`](./docs/acoustic-optic-slam-platform-architecture-2026-08-17.md) ——
  长期架构设计：模块依赖 DAG、领域契约、状态机与调度、可靠性策略、Gate 划分。
- [`holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md`](./docs/holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md) ——
  第一阶段工程方案，含对 `ocean_t`/`SVIn`/`sonar_camera_reconstruction` 三份代码的
  逐文件审计发现。
- [`configs/README.md`](./configs/README.md) —— 分层配置每个字段的详细说明。
- [`NOTICE`](./NOTICE) —— 移植代码的逐文件出处、许可证、移植范围。
- [`CLAUDE.md`](./CLAUDE.md) —— 给 Claude Code 在本仓库工作时的约定和背景速览。
