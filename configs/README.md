# 配置分层

对应架构文档第 14.2 节：`defaults → rig → scenario → experiment` 四层叠加。

**当前状态**：`apps/tools/synth_bag_gen` 和 `apps/replay_demo` 都可以通过 `--experiment <path>` 加载
这里的分层配置（解析代码见 `runtime/include/uw/runtime/config.hpp`，用 yaml-cpp）。仍是 v1 明确限制
的部分：`experiment/*.yaml` 里选择算法变体的字段（`frontends`/`estimator_mode`/`map_backend`）目前
只被读取和打印，两个 app 各自仍只实现一条固定管线，还没有真正按配置切换 frontend/estimator/map
backend 的实现——这是紧接着的下一步，不应该修改这里定义的分层结构或字段命名。

- `defaults/`：平台级默认值，不含任何具体机体/场景信息。`estimation.warmup_seconds`
  （默认 0，即不启用）用于让开局前 N 秒的 keyframe 只做 dead reckoning（继续吃
  relative-pose 因子）、暂不融合 sonar/depth 这类"绝对参考"因子——批处理位姿图场景下
  对"VIO 偏置还没收敛前先别信绝对修正"这条工程经验的落地方式，具体见
  `apps/replay_demo/src/main.cpp` 里的 warmup 代码块。
- `rig/`：标定唯一事实源（对应 `RigCalibrationSnapshot`），描述一台具体机体的传感器外参、内参、
  噪声模型。
- `scenario/`：world、控制、退化、故障、seed ——描述"跑什么数据"，不描述"用什么算法"。
- `experiment/`：选择 frontend、estimator mode、reliability policy、map backend、model version 和
  算力预算 ——描述"怎么跑"。

每次运行仍然会产出一个不可变 `RunManifest`（见 `runtime/include/uw/runtime/run_manifest.hpp`），
记录实际生效的配置/标定/代码/模型哈希，即使当前配置文件本身还没被程序读取。

## 声光前端契约字段

`rig/*.yaml` 的 `cameras`、camera/sonar `frame_tree` 边和
`time_offset_seconds` 已解析进 `RigCalibrationSnapshot`。时间偏移采用
`t_reference = t_sensor_capture + time_offset_seconds[sensor_id]` 的符号约定。

`experiment/*.yaml` 的 `frontends.optical` 已被解析。`stereo_depth_frontend_v1` 现在有真正的
实现（`algorithms/frontends/stereo_optical_depth_frontend`，`StereoOpticalDepthFrontend`），
由 `apps/tools/optical_baseline_eval` 独立跑通并用 `uw_l2_optical_baseline_smoke_test` 把关，
但**没有**被 `apps/replay_demo` 动态构造——`frontends.optical` 仍然只是一个配置选择器，不是
`replay_demo` 里可切换的运行能力。`runtime/acoustic_optic_synchronizer.hpp`（capture-time
配对）、`algorithms/frontends/acoustic_optic_associator`（FLS 弧带候选生成 + 几何关联审计）
和 `algorithms/frontends/acoustic_optic_depth_fusion`（posterior 深度优化，真正产出
`FusedDepthMeasurement`）都已经落地并有单测覆盖。`apps/tools/acoustic_optic_scenario_matrix`
现在把这四者接成一条真实跑通的流水线（含真实、非新写的 `SonarCfarFrontend`），覆盖
架构文档第 10 节的 9 场景矩阵，见代码库参考文档 6.10 节的真实结果表——但**仍然没有**接进
`apps/replay_demo`；把声光证据接入位姿图 replay 属于后续 plan（mapping handoff）。
