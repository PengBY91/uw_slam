# CLAUDE.md

在 `uw_slam/` 里工作时的约定和背景速览。完整背景见 `README.md` 和延伸阅读里的两份
架构文档；这里只记那些"读代码读不出来、但会影响你怎么改代码"的东西。

## 这个仓库是什么

水下声光融合 SLAM 平台的长期代码框架，是架构文档
(`docs/acoustic-optic-slam-platform-architecture-2026-08-17.md`) 的代码落地。**不是**在
`ocean_t`/`SVIn`/`sonar_camera_reconstruction` 上小修小补，是独立重建的新仓库，
允许把后两者的具体实现移植进来（已经移植了两处，见下）。当前是"骨架 + 每层至少
一条真实可跑的端到端链路"阶段，不是生产系统。

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
  mapping, runtime, evaluation, adapters} → application → apps`，ROS2 隔离在
  `adapters/ros2/`；`apps/` 只做参数解析和进程入口，用例编排放在 `application`。
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
- **除非用户明确要求，不要 `git commit`**。保留用户已有改动；提交、变基、清理
  工作树都必须在授权范围内进行。
- Protobuf（`schemas/proto/`）是跨语言规范化消息模型的唯一来源。需要新增跨语言
  规范化消息字段时改
  `.proto`，不要在 C++/Python 任一侧另建一套平行 struct 去绕过它——`rig` 配置层
  直接解析进 `RigCalibrationSnapshot` protobuf 消息就是照这个原则做的。

## 构建与测试

```bash
# 依赖装好之后（见 README.md「构建」一节，本机走的是 conda-forge 回退路径）：
export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure   # 当前工作树 136 个 case；会变化，以实跑为准

(cd adapters/holoocean && .venv/bin/pytest -q)  # 当前 35 个 case；先按 README 安装 dev extra

tools/lint/check_no_ros_in_core.sh           # 依赖不变量检查（兼容入口，实际跑 tools/lint/check_layer_dependencies.py）
```

改完代码后按这个顺序验证：编译 → C++ 测试 → Python 测试（如果碰了 `adapters/
holoocean/`）→ lint。**端到端 demo 也值得实际跑一遍**，不要只信单元测试——下面
「已经踩过的坑」里那个 z 轴 anchor bug就是单元测试全绿、但实跑 demo 才发现的。

```bash
build/bin/synth_bag_gen --experiment configs/experiment/synthetic_smoke.yaml --out /tmp/synthetic.mcap
build/bin/replay_demo --bag /tmp/synthetic.mcap --experiment configs/experiment/synthetic_smoke.yaml --out /tmp/demo
# 期望：6~7 次迭代内收敛，ATE rmse ~0.06-0.07m（跨 seed 有波动；不是 ~3cm 了，
# 见 README「运行端到端 demo」一节——sonar_range_factor 的路标关联换成真实
# SubmapManager 在线发现之后，v1 没有联合路标估计的 elevation 误差会摊到 x/y 上）。
# 换成 configs/experiment/synthetic_smoke_vo.yaml 可以跑 estimator_mode:
# stereo_landmark_vo 变体（相对位姿从左右相机帧实时算，而不是从桩读取），
# ATE 量级相当。
```

`integration.acoustic_optic_scenario_matrix_determinism` 不只比较同 seed 输出：它也保留
矩阵程序第一次运行的退出码，因此最低有效覆盖 gate 失败会让 CTest 失败。可单独运行：

```bash
ctest --test-dir build -R integration.acoustic_optic_scenario_matrix_determinism --output-on-failure
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
- `PressureDepthMeasurement.depth_m` 是**正向下**的水深量；仓库 world/body frame 是
  Z-up，所以位姿 z 使用 `-depth_m`。`OpticalDepthPriorMeasurement`、
  `FusedDepthMeasurement` 和关联记录中的 `depth_m` 则是相机 optical frame 的
  **正向前**距离。字段同名但坐标与符号语义不同，不要直接混用。
- `ValidateExperimentConfigSelections()` 会拒绝未知的 frontend/map backend/estimator/
  detector 标识符。`estimator_mode` 和 `landmark_detector` 已真正分支；sonar/optical
  frontend 与 map backend 当前只有一个被接受的实现，fail-fast 不等于已有多后端切换。

## 已经踩过的坑（省得重新踩一遍)

- **GCC 对同一个类里"嵌套 struct 做默认参数"处理有 bug**：`const Options& options
  = {}` 若 `Options` 嵌套在同一个类里会编译失败。解决方式是把这类 options/summary
  struct 提到 namespace scope（见 `GaussNewtonOptions`/`GaussNewtonSummary`），不要
  再往类里塞嵌套 struct 当默认参数类型。
- **`protobuf::libprotobuf` 的 `INTERFACE_LINK_LIBRARIES` 不会自动透传 absl 符号**
  ——静态库链接会报 "DSO missing from command line"。`domain_proto` 已经显式
  `PUBLIC` 链接了一组 absl 组件，新增用到 protobuf 生成类型的 target 通常不需要
  再手动处理，但如果遇到类似链接错误，先检查是不是漏了某个 absl 组件而不是怀疑
  protobuf 本身坏了——2026-08-22 开 `UW_COVERAGE=ON`（`--coverage` 改变链接顺序）
  时就实测触发过一次新的缺口（`absl::log_internal_check_op` 的
  `MakeCheckOpString`），加进 `cmake/UwProtobuf.cmake` 那份显式链接列表后修复；
  平时不开这些少见的编译选项组合不会碰到，但说明这份列表并不能保证已经穷尽。
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
- **相机 optical frame 和 body frame 的旋转方向容易搞反**：`stereo_landmark_vo_frontend`
  从相机坐标系解出的相对位姿要用 rig 标定的 camera→body 外参做共轭
  （`T_body = T_cam_body * T_cam * T_cam_body^-1`）才能喂给以 body frame 定义的
  相对位姿因子；第一版把方向搞反，单元测试全绿但实跑 demo 时 ATE 停在 6.67m 不收敛，
  修对之后降到 0.061m——这也是**实跑 demo** 而非单元测试才发现的问题，同一类坑见
  上面的 z 轴 anchor bug。
- **声光关联的并列几何分数不必然表示两个真实假设**：近 boresight 时 elevation 对
  bearing 无影响、对 range 只有二阶影响，同一平面上的弧采样点会合理打平。关联器只有
  在前两名深度也不满足 `depth_agreement_sigma` 合并方差门时才返回 `AMBIGUOUS`；不要
  删除这条深度一致性判定，也不要靠放宽 `ambiguity_margin` 让场景矩阵变绿。
- **场景矩阵中的预期拒绝不是回归**：`time_offset_fault`、
  `extrinsic_perturbation`、`sonar_dropout`、`optical_invalid_region` 都刻意验证
  fail-closed/回退语义。最低有效覆盖 gate 排除这些场景；其他场景出现 0 accepted
  才应失败。`--min-fusion-improvement-fraction` 和延迟 gate 仍是 opt-in。
- **`camera_rectifier` 目前是有限去畸变原语，不是通用 stereo rectifier**：它支持
  plumb-bob 0/4/5 个系数和 MONO8/RGB8/BGR8，在 `StereoGeometry` 的平行基线前提下
  去除镜头畸变；不处理任意相机相对旋转。它尚未接入 `replay_demo`，因为对现有真实
  bag 的双线性重采样会削弱细纹理、让 VO 跟踪从 50/50 降到 8/50，后续需要联合调参。
- **ThreadSanitizer（`-DUW_SANITIZER=thread`）在本机沙箱里有两层坑**：(1) 不先关
  ASLR 会直接 `FATAL: ThreadSanitizer: unexpected memory mapping`——连二进制都跑不
  起来，跟代码无关；构建（`gtest_discover_tests` 会在构建期跑一次二进制枚举用例）
  和运行都要套 `setarch $(uname -m) -R`。(2) 即使绕过 (1)，TSan 也会在完全单线程的
  测试体里报 `heap-use-after-free`（比如
  `DomainContract.ImageFrameRoundTripsWithCanonicalHeader`）——根因是这个 conda-forge
  工具链的 `libprotobuf.so`/`libgtest.so` 是预编译的动态库，没有用
  `-fsanitize=thread` 重新编译，TSan 看不到库内部的同步操作，会在 protobuf 自己的
  atomic/arena 分配上报假阳性；同一段代码在 `UW_SANITIZER=address` 下（对真实
  use-after-free 至少一样敏感）跑得干干净净，佐证了这是工具链问题不是真 bug。
  `tools/run_quality_checks.sh` 因此只跑 `address`（ASan+UBSan）不跑 `thread`——要
  让 TSan 真正可信，得从源码重新编译一套开 `-fsanitize=thread` 的
  protobuf/gtest，目前没有这么做。

## 目录速查

不复述 `README.md` 已有的表格，只列会话中容易忘记具体在哪的：

- 规范化 Protobuf 消息定义：`schemas/proto/uw/domain/*.proto`
- Pose3、相机/去畸变与声呐 beam 模型：`include/sensor_models/`
- Frontend/FactorBuilder/ResidualBlock 抽象接口：`include/measurement_api/`
- 分层配置加载：`include/runtime/config.hpp` + `src/runtime/config.cpp`
- RunManifest：`include/runtime/run_manifest.hpp`
- 集中式 CMake（library/application/test target 图）：`cmake/Dependencies.cmake`、
  `cmake/Libraries.cmake`、`cmake/Applications.cmake`、`cmake/Tests.cmake`
- 移植代码出处总账：`/NOTICE`
- lint 脚本：`tools/lint/check_no_ros_in_core.sh`（兼容入口）/
  `tools/lint/check_layer_dependencies.py`（实际实现）
- 确定性回放测试：`tests/integration/determinism_test.sh`
- 声光场景矩阵 gate：`tests/integration/acoustic_optic_scenario_matrix_determinism_test.sh`
