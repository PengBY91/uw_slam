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
