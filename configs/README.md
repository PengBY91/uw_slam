# 配置分层

对应架构文档第 14.2 节：`defaults → rig → scenario → experiment` 四层叠加。

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
`stereo_landmark_vo` 从双目图像在线计算相对位姿；两条路径最终使用同一个
`GaussNewtonSolver`，并不切换估计求解器。`stereo_landmark_vo` 需要 rig 带相机，并使用
`include/frontends/stereo_landmark_vo_frontend.hpp` 从 `/raw/camera/left,right` 实时计算量测，
替代默认 `black_box_vio` 从 `/evidence/relative_pose` 读取的量测；见
`configs/experiment/synthetic_smoke_vo.yaml`。**同一分支下的第二个例
外**：`frontends.landmark_detector`（`bright_blob` 默认值 / `harris_corner`）也真的会被消费，选择
`stereo_landmark_vo_frontend` 内部用哪个 landmark 检测器——`bright_blob`
（`LandmarkBlobDetector`）是给 `synth_bag_gen` 的合成高亮方块场景调的，`harris_corner`
（`HarrisCornerDetector`）是给真实相机画面（没有理由出现孤立高亮色块）用的，见两者各自的头文件
注释。

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
