# 配置分层

对应架构文档第 14.2 节：`defaults → rig → scenario → experiment` 四层叠加。
叠加关系与三个真正驱动分支的选择器画在一起，见根 README 的
[分层配置图](../README.md#分层配置)（那张图是这套字段的唯一图示，改字段时改那一处）。

**当前状态**：`apps/synth_bag_gen` 和 `apps/replay_demo` 都可以通过 `--experiment <path>` 加载
这里的分层配置（解析代码见 `include/runtime/config.hpp`，用 yaml-cpp）。仍是 v1 明确限制
的部分：`experiment/*.yaml` 里选择算法变体的字段（`frontends`/`map_backend`）大多只对应一条
固定管线，两个 app 各自仍只实现这一条，还没有真正按配置切换 frontend/map backend 的实现——这
是紧接着的下一步，不应该修改这里定义的分层结构或字段命名。**但从
`uw::runtime::ValidateExperimentConfigSelections`（`apps/replay_demo` 加载 `--experiment` 后立即调
用）开始**，这些字段不再是"读取但不驱动"：`frontends.sonar`/`frontends.optical`/`map_backend`
只要不等于目前唯一实现的那个值（分别是 `sonar_cfar_frontend_v1`/`stereo_depth_frontend_v1`/
`submap_point_cloud_v1`），`replay_demo` 会在跑之前直接报错退出，而不是静默按硬编码管线继续
跑——关掉了生产就绪度文档第 10 节"配置存在但不驱动实现"那条风险里"未识别或未实现的算法选择
必须启动失败"这一半；"真正切换到另一条实现"仍然是待办，因为除了下面两个例外，压根还没有第二
条实现可切换。`map_backend` 是预留的地图实现选择字段，目前只支持
`submap_point_cloud_v1`。**第一个例外**：`estimator_mode` 是保留兼容性的历史字段名。当前它只选择
相对位姿输入来源：`black_box_vio` 读取 bag 中外部或预生成的相对位姿量测，
`stereo_landmark_vo` 从双目图像在线计算相对位姿；这条字段本身不切换估计求解器，两条路径
产出的相对位姿量测最终喂给同一个求解器实例——但求解器本身现在是独立可切换的，见下面
第三个例外。`stereo_landmark_vo` 需要 rig 带相机，并使用
`include/frontends/stereo_landmark_vo_frontend.hpp` 从 `/raw/camera/left,right` 实时计算量测，
替代默认 `black_box_vio` 从 `/evidence/relative_pose` 读取的量测；见
`configs/experiment/synthetic_smoke_vo.yaml`。**同一分支下的第二个例
外**：`frontends.landmark_detector`（`bright_blob` 默认值 / `harris_corner`）也真的会被消费，选择
`stereo_landmark_vo_frontend` 内部用哪个 landmark 检测器——`bright_blob`
（`LandmarkBlobDetector`）是给 `synth_bag_gen` 的合成高亮方块场景调的，`harris_corner`
（`HarrisCornerDetector`）是给真实相机画面（没有理由出现孤立高亮色块）用的，见两者各自的头文件
注释。**第三个例外**：`estimation.solver`（`gauss_newton_v1` 默认值 / `ceres_v1`）真的会被
消费，选择 `PoseGraphProblem` 实际用哪个求解器求解——这是
`docs/archive/superpowers/specs/2026-08-23-solver-and-mapping-oss-adoption.md` 描述的基准决策门
工作单元：`ceres_v1` 需要编译时打开 `UW_BUILD_CERES_SOLVER`（默认关闭，因为 Ceres 依赖较重
——SuiteSparse/glog/gflags），选了 `ceres_v1` 但没编译进 Ceres 支持会在启动时直接报错退出，
不会静默回退到 `gauss_newton_v1`。默认值目前仍是 `gauss_newton_v1`，切换默认值本身是一个
需要实测基准数据支撑的决策，还没有做。

*注：`configs/defaults/platform.yaml` 里 `estimation.solver` 旁边的注释仍写着"第一版图优化
库未定，见架构文档第 20 节延后决策"——这不是过时未更新，是刻意保留：`ceres_v1` 只是让这条
延后决策第一次有了可以实测比较的第二个选项,决策本身（默认值该不该换）仍然延后，等基准工具
产出数据后再关闭。*

- `defaults/`：平台级默认值，不含任何具体机体/场景信息。`estimation.warmup_seconds`
  （默认 0，即不启用）用于让开局前 N 秒的 keyframe 只做 dead reckoning（继续吃
  relative-pose 因子）、暂不融合 sonar/depth 这类"绝对参考"因子——批处理位姿图场景下
  对"VIO 偏置还没收敛前先别信绝对修正"这条工程经验的落地方式，具体见
  `apps/replay_demo.cpp` 里的 warmup 代码块。
- `rig/`：标定唯一事实源（对应 `RigCalibrationSnapshot`），描述一台具体机体的传感器外参、内参、
  噪声模型。
- `scenario/`：world、控制、退化、故障、seed ——描述"跑什么数据"，不描述"用什么算法"。
- `experiment/`：选择 frontend、相对位姿输入模式（`estimator_mode`）、reliability policy、
  地图实现（`map_backend`）、model version 和算力预算 ——描述"怎么跑"。

每次运行仍然会产出一个不可变 `RunManifest`（见 `include/runtime/run_manifest.hpp`），
记录实际生效的配置/标定/代码/模型哈希，即使当前配置文件本身还没被程序读取。

## 声光消息与接口字段

`rig/*.yaml` 的 `cameras`、camera/sonar `frame_tree` 边和
`time_offset_seconds` 已解析进 `RigCalibrationSnapshot`。时间偏移采用
`t_reference = t_sensor_capture + time_offset_seconds[sensor_id]` 的符号约定。

`experiment/*.yaml` 的 `frontends.optical` 已被解析。`stereo_depth_frontend_v1` 现在有真正的
实现（`include/frontends/stereo_optical_depth_frontend.hpp`，`StereoOpticalDepthFrontend`）。
`include/runtime/acoustic_optic_synchronizer.hpp`（capture-time 配对）、
`include/frontends/acoustic_optic_associator.hpp`（FLS 弧带候选生成 + 几何关联审计）、
`include/frontends/acoustic_optic_depth_fusion_frontend.hpp`（posterior 深度优化，真正产出
`FusedDepthMeasurement`）和 `include/mapping/acoustic_optic_map_bridge.hpp`（转成
`MapEvidence`，base_link 系）都已经落地并有单测覆盖，`apps/acoustic_optic_scenario_matrix.cpp`
把它们接成一条真实跑通的流水线，覆盖架构文档第 10 节的 9 场景矩阵（真实结果见代码库参考
文档 6.10/6.11 节）——至此声光系列六个 plan 全部完成。

**`apps/replay_demo`/`apps/synth_bag_gen` 现在也真正接了这些组件**（后续独立于六个
plan 的一次集成，见代码库参考文档 6.12 节和对应 plan 文档）：`--experiment` 加载的 rig
含相机时，`replay_demo` 会真正构造并跑上述整条流水线，把结果存进 `submap_manager` 的第三个
`MapEvidence` bucket；没有相机（或没传 `--experiment`）时两个 app 行为逐字节不变
（`integration.replay_determinism` 把关）。**明确没做的事**：稠密深度没有变成位姿图的新
factor 类型——`PoseGraphProblem`/求解器/轨迹 ATE 完全不受这次集成影响。

## P0 非放空 gate（`gates:`）

`experiment/*.yaml` 根层的 `gates:` 覆盖 `defaults/*.yaml` 的同名字段（`include/runtime/
config.hpp` 的 `PlatformDefaultsConfig`），`apps/replay_demo` 求解完之后逐条检查
（`application::EvaluateReplayGates`），任何一条不满足就非零退出（exit 2）——outputs（轨迹/
manifest）仍然照常写出，只影响退出码，方便失败时仍能排查。除 `require_converged`（默认开，
求解器不收敛任何时候都不可接受）外全部默认关闭（阈值 `<=0`/`false`），需要每个 experiment
自己按实测结果决定开不开、开多严：

- `require_converged` / `max_ate_rmse_m` / `min_matched_ate_poses` / `require_nonempty_map`：
  求解器收敛性、ATE、地图非空——`require_nonempty_map` 只要求"有地图内容"，optical-only 深度
  点就能满足它，不要求声光关联本身成功。
- `min_acoustic_optic_accepted` / `min_acoustic_optic_map_points`：专门针对声光关联本身
  （`contribution_mask == DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC`，在
  `mapping::BuildMapEvidenceFromFusedDepth` 抹掉逐点来源信息**之前**由
  `application::CountDepthContributions` 计数，见该函数注释）——**只在
  `configs/experiment/acoustic_optic_demo.yaml` 开启**，因为只有它的 scenario
  （`scenario/acoustic_optic_demo.yaml`）把声呐目标特意放在相机窄视场内，`apps/synth_bag_gen.cpp`
  会把该目标同时画进双目图像（不止画普通 VO landmark），使真实 ACCEPTED 关联可达（seed 42
  实测 12 个 keyframe 中 3 个 accepted / 3 个 acoustic-optic map point）。`synthetic_smoke.yaml`
  的默认三个目标经真实几何计算不在相机视场内（见该文件注释），这两个 gate 必须保持关闭——不能
  为了让实验"变绿"而放宽关联判定或伪造 acceptance。控制台汇总（`acoustic-optic: ... accepted,
  ... map evidence points added (N optical-only, M acoustic-optic)`）和 gate 失败信息都会打印
  这两类点数，方便区分"完全没有地图内容"和"有地图内容但没有真正的声光融合"。

## 回环闭合对比 demo（`frontends::LoopClosureFrontend`）

`scenario/synthetic_loop_closure.yaml` 跟 `scenario/synthetic_smoke.yaml` 结构完全一样，唯一区别
是 `arc_radians` 覆盖整整一圈（`2*pi`）而不是 80 度短弧——`apps/synth_bag_gen.cpp` 的合成
landmark 云是按世界坐标生成一次、每个 keyframe 复用同一份（不是逐帧重新生成），所以真正转一圈
回到起点时会看到同一批 landmark，是一次真正的视觉重访，不只是位置数值上接近。

`configs/experiment/synthetic_loop_closure_vo.yaml`（`loop_closure` 关闭，`defaults/
platform.yaml`）和 `synthetic_loop_closure_vo_enabled.yaml`（`loop_closure` 开启，`defaults/
platform_loop_closure.yaml`）除 `defaults:` 外完全一样，用同一份 bag 对比两者的 ATE 就是端到端
效果验证——两份文件头部注释记录了实测数字和一个重要发现：production 默认参数
（`candidate_search_radius_m=3.0`、`min_keyframe_index_gap=15`）在这个场景下只找到 1 条回环边，
ATE 改善很小；手工把搜索半径放宽到 8~20 后回环边数涨到 2~34 条，但 ATE 反而从 1.26m 恶化到
9.24m——`LoopClosureFrontend` 固定用 `HarrisCornerDetector`（不像 `stereo_landmark_vo_frontend`
那样有 `bright_blob`/`harris_corner` 双模选择），跟 `synth_bag_gen` 给 `bright_blob` 调的合成
高亮图案不是同一套外观假设，半径放宽后会把很多外观模糊/错误的角点匹配当成候选收进来，Huber
在数量多的时候压不住——这是 v1 default 刻意保守（而不是随便定）的直接证据，见
`synthetic_loop_closure_vo_enabled.yaml` 头部注释的完整记录。
