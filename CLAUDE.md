# CLAUDE.md

在 `uw_slam/` 里工作时的约定和背景速览。完整背景见 `README.md` 和延伸阅读里的两份
架构文档；这里只记那些"读代码读不出来、但会影响你怎么改代码"的东西。

## 这个仓库是什么

水下声光融合 SLAM 平台的长期代码框架，是架构文档
(`docs/acoustic-optic-slam-platform-architecture-2026-08-17.md`) 的代码落地。**不是**在
`ocean_t`/`SVIn`/`sonar_camera_reconstruction` 上小修小补，是独立重建的新仓库，
允许把后两者的具体实现移植进来（已经移植了两处，见下）。当前是"骨架 + 每层至少
一条真实可跑的垂直切片"阶段，不是生产系统。

## 硬性规则

- **不要修改 `external_repos/` 下的任何子目录**（`SVIn/`、`sonar_camera_
  reconstruction/`、`ocean_t/`，以及后来加入的 `holoocean-ros/`——HoloOcean 官方
  ROS2 接口包，含 `holoocean_main`/`holoocean_interfaces`/`holoocean_examples`——
  和 `holoocean_bridge/`——一个同事的 HoloOcean→ROS2 桥接包，供
  `sonar_camera_reconstruction_baseline`/`svin_bridge` 的 README 引用为参考）。
  它们是只读的参考/移植来源，各自有自己的来源，整个 `external_repos/` 已被
  `.gitignore` 排除在本仓库版本控制之外。`holoocean-ros` 是当前实际在用的仿真
  工具（HoloOcean + Unreal Engine 5，通过 ROS2 暴露话题），`include/adapters/
  holoocean_ros_bridge_sonar_frame_provider.hpp` + `adapters/ros2/include/adapters/
  ros2_holoocean_sonar_bridge.hpp` 是本仓库消费它的接入点——本机的 colcon
  workspace（`~/ros2_ws`，symlink 进 `external_repos/holoocean-ros` 的三个包）
  和 ROS2 Jazzy 装在系统 apt 里，都在这个仓库目录之外，不受 `.gitignore`/版本
  控制影响。
- **依赖只能单向**：`domain → core → {frontends, factor_builders, estimation,
  mapping, runtime, evaluation, adapters} → apps`，ROS2 隔离在 `adapters/ros2/`。
  `include/`、`src/` 下任何生产代码都不能 include ROS/HoloOcean/第三方 vendor
  头，也不能再用旧的 `uw/...` 手写头路径——这是 `tools/lint/
  check_layer_dependencies.py`（`tools/lint/check_no_ros_in_core.sh` 是它的
  兼容入口）强制检查的不变量，改完代码顺手跑一下。
- **移植第三方代码前先读 `NOTICE`**。仓库整体是 GPLv3（因为移植了 SVIn 的 GPLv3
  代码），移植文件必须保留原始版权头，新移植内容要在 `NOTICE` 里补一节说明来源、
  移植了什么、刻意没移植什么。目前已移植两处：
  - `include/factor_builders/sonar_range_residual.hpp` +
    `src/factor_builders/sonar_range_residual.cpp`：残差公式来自 SVIn 的
    `SonarError`，但雅可比是**独立重新推导的**——上游雅可比和自己的残差在数学上
    不自洽，直接抄会引入错误。
  - `include/frontends/{cfar_detector,dbscan,sonar_cfar_frontend}.hpp` +
    `src/frontends/{cfar_detector,dbscan,sonar_cfar_frontend}.cpp`：CFAR +
    极坐标转换 + DBSCAN 来自 `sonar_camera_reconstruction`，但**没有**移植它
    `merge.py` 里丢弃 pitch、直接烘焙到 `map` frame 的部分——前端只应输出声呐
    局部坐标系下的证据，不应该自己决定全局位姿。
- **除非用户明确要求，不要 `git commit`**。当前仓库只做过 `git init`，历史上没有
  任何提交；不要主动创建第一个提交。
- Protobuf（`schemas/proto/`）是跨语言领域契约的唯一事实源。需要新字段时改
  `.proto`，不要在 C++/Python 任一侧另建一套平行 struct 去绕过它——`rig` 配置层
  直接解析进 `RigCalibrationSnapshot` protobuf 消息就是照这个原则做的。

## 构建与测试

```bash
# 依赖装好之后（见 README.md「构建」一节，本机走的是 conda-forge 回退路径）：
export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure   # 应该全过（当前 106 个 case，随代码增长会变，实跑为准）

cd adapters/holoocean && pytest              # 应该全过（当前 25 个 case）

tools/lint/check_no_ros_in_core.sh           # 依赖不变量检查（兼容入口，实际跑 tools/lint/check_layer_dependencies.py）
```

改完代码后按这个顺序验证：编译 → C++ 测试 → Python 测试（如果碰了 `adapters/
holoocean/`）→ lint。**端到端 demo 也值得实际跑一遍**，不要只信单元测试——下面
「已经踩过的坑」里那个 z 轴 anchor bug就是单元测试全绿、但实跑 demo 才发现的。

```bash
build/bin/synth_bag_gen --experiment configs/experiment/synthetic_smoke.yaml --out /tmp/synthetic.mcap
build/bin/replay_demo --bag /tmp/synthetic.mcap --experiment configs/experiment/synthetic_smoke.yaml --out /tmp/demo
# 期望：6~7 次迭代内收敛，ATE rmse ~0.15-0.21m（跨 seed 有波动；不是 ~3cm 了，
# 见 README「运行端到端 demo」一节——sonar_range_factor 的路标关联换成真实
# SubmapManager 在线发现之后，v1 没有联合路标估计的 elevation 误差会摊到 x/y 上）
```

## 代码约定

- 位姿统一用 `Pose3`（平移 + 四元数 xyzw），C++ (`uw::sensor_models::Pose3`) 和
  Python (`uw_holoocean_adapter.coordinates.Pose`) 两边一致。**不要引入欧拉角**
  ——之前 `ocean_t` 用欧拉角是被审计出来的具体问题（万向锁风险），新代码刻意避开。
- 随机数用显式传入的、有 seed 的 RNG，一路传到调用点；**不要用全局
  `np.random.seed()` 或在运行中途重新 seed**——这是从 `ocean_t` 审计里改掉的模式，
  也是确定性回放测试 (`tests/integration/determinism_test.sh`) 实际在把关的东西。
- 新增一个 frontend/factor_builder，头文件放 `include/frontends/`（或
  `include/factor_builders/`）、实现放 `src/frontends/`（或 `src/factor_builders/`）、
  测试放 `tests/frontends/`（或 `tests/factor_builders/`），并把新增的 `.cpp`/测试
  `.cpp` 加进 `cmake/Libraries.cmake`/`cmake/Tests.cmake` 里对应 target 的源文件
  列表——这两个 target 是按架构层合并的（所有 frontend 共用 `frontends` target，
  所有 factor builder 共用 `factor_builders` target），不要为新实现单独建
  target 或 `CMakeLists.txt`，也不要改动其他已有实现的代码或接口。
- 求解器（`include/estimation`/`src/estimation`）目前是 Eigen 手写的
  Gauss-Newton/LM，`Solver` 接口留了替换口子（后续换 Ceres/GTSAM 属于架构文档第
  20 节明确记录的延后决策）——不要因为"手写的不够好"就顺手去重构成别的库，除非
  用户明确要这么做。
- YAML 配置分层（`defaults → rig → scenario → experiment`，见
  `include/runtime/config.hpp`）：`experiment/*.yaml` 里的 `rig`/
  `scenario`/`defaults` 三个 key 是相对 `configs/` 目录（不是相对 experiment 文件
  自己所在目录）写的路径，加新 experiment 文件时注意这一点。

## 已经踩过的坑（省得重新踩一遍)

- **GCC 对同一个类里"嵌套 struct 做默认参数"处理有 bug**：`const Options& options
  = {}` 若 `Options` 嵌套在同一个类里会编译失败。解决方式是把这类 options/summary
  struct 提到 namespace scope（见 `GaussNewtonOptions`/`GaussNewtonSummary`），不要
  再往类里塞嵌套 struct 当默认参数类型。
- **`protobuf::libprotobuf` 的 `INTERFACE_LINK_LIBRARIES` 不会自动透传 absl 符号**
  ——静态库链接会报 "DSO missing from command line"。`domain_proto` 已经显式
  `PUBLIC` 链接了一组 absl 组件，新增用到 protobuf 生成类型的 target 通常不需要
  再手动处理，但如果遇到类似链接错误，先检查是不是漏了某个 absl 组件而不是怀疑
  protobuf 本身坏了。
- **MCAP C++ SDK 没有自己的 CMakeLists.txt**，`FetchContent_MakeAvailable` 用不了，
  要用 `FetchContent_Populate` 手动接（见 `cmake/UwMcap.cmake`）。
- **z 轴不是规范自由度**：x/y/yaw 对"相对位姿 + 声呐 range-only"的图确实是 gauge
  freedom，但一旦有 depth 因子，z 就有了绝对参考。之前把 `kf0` 固定在纯
  `Pose3::Identity()`（z=0）而其他 keyframe 被 depth 证据拉向真实深度，造成约束
  冲突、求解器 30 次迭代不收敛、ATE 高达 4.6m——这是靠**实跑 demo** 而不是单元测试
  发现的。修法是给 fixed 的 anchor keyframe 也用它自己的真实 depth 证据设 z，而不是
  想当然地钉在 0。加新的绝对参考因子（比如未来的绝对朝向）时留意同样的陷阱。
- **本机沙箱环境 apt 的 HTTP(80) 镜像会卡住不动，HTTPS(443) 正常**：根因是本机有个
  本地 HTTP(S) 代理（`127.0.0.1:8019`，`$HTTP_PROXY`/`$HTTPS_PROXY` 已经设好），但
  `apt-get`/`rosdep` 不会自动读这两个环境变量——需要显式写
  `/etc/apt/apt.conf.d/95proxy`（`Acquire::http::Proxy "http://127.0.0.1:8019";` 同理
  加 `https`）配置 apt 的代理，`rosdep update` 则是直接靠 shell 里的
  `$HTTP_PROXY`/`$HTTPS_PROXY` 生效。**`sudo` 默认会清空这两个变量**，所以
  `sudo rosdep init` 这类命令要用 `sudo -E`，否则代理配置对子进程不生效又卡住。
  不要在 `apt-get`/`rosdep` 卡住时傻等，先检查代理配置好了没有；`tools/setup_dev_env.sh`
  给 C++ 依赖走的是切 conda-forge 的回退逻辑，但装 ROS2（apt-only，没有 conda 包）
  就得走这条代理配置路径。
- **装 ROS2 后跑 `colcon build`/`cmake -DUW_BUILD_ROS2=ON` 之前先 `conda deactivate`**：
  base conda 环境默认自动激活，`$CONDA_PREFIX`/`$CONDA_PYTHON_EXE` 会让 CMake 的
  `find_package(Python3)`（以及 `colcon build` 内部生成 `holoocean_interfaces` 消息代码
  要用到的 `python3`）优先选中 conda 的 Python——但 `catkin_pkg`（ament_cmake 生成
  package.xml 元数据要用）只装在系统 Python 里，选错 Python 会在配置/生成阶段报
  `ModuleNotFoundError: No module named 'catkin_pkg'`。**并且**：`cmake`/`colcon` 这
  两步对 Python 的要求是反的——`colcon build` 要系统 Python 在前（有 `catkin_pkg`），
  而 `cmake -S . -B build -DUW_BUILD_ROS2=ON` 要用 conda 自己的 `cmake`（4.x）而不是
  系统 apt 装的 `cmake`（3.28，`FindProtobuf.cmake` 里 `protobuf_generate()` 算生成
  文件相对路径时有旧 bug，不认 `IMPORT_DIRS`，只认 `CMAKE_CURRENT_SOURCE_DIR`，会把
  `.pb.h` 生成到多一层 `generated/schemas/proto/...` 的错误路径，导致 `domain_proto`
  互相 include 找不到文件）——所以两步要分别调整 `PATH` 顺序，不能一套环境变量走到底。
- **`configs/experiment/*.yaml` 里 `rig`/`scenario`/`defaults` 路径是相对 `configs/`
  的**，不是相对 experiment 文件自己所在目录——第一次实现时按后者算漏了一层
  `parent_path()`，读文件报错才发现。

## 目录速查

不复述 `README.md` 已有的表格，只列会话中容易忘记具体在哪的：

- 领域契约 protobuf 定义：`schemas/proto/uw/domain/*.proto`
- Pose3、声呐 beam 模型：`include/sensor_models/`
- Frontend/FactorBuilder/ResidualBlock 抽象接口：`include/measurement_api/`
- 分层配置加载：`include/runtime/config.hpp` + `src/runtime/config.cpp`
- RunManifest：`include/runtime/run_manifest.hpp`
- 集中式 CMake（library/application/test target 图）：`cmake/Dependencies.cmake`、
  `cmake/Libraries.cmake`、`cmake/Applications.cmake`、`cmake/Tests.cmake`
- 移植代码出处总账：`/NOTICE`
- lint 脚本：`tools/lint/check_no_ros_in_core.sh`（兼容入口）/
  `tools/lint/check_layer_dependencies.py`（实际实现）
- 确定性回放测试：`tests/integration/determinism_test.sh`
