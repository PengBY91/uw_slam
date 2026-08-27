# ROV 在线驾驶辅助（实时闭环）代码审查记录

> 日期：2026-08-27
> 范围：主线二"ROV 在线驾驶辅助"——HoloOcean ROS2 话题 → `holoocean_realtime_node` →
> `LiveEventSource` → `OnlineAssistPipeline` → HMI + 飞手命令回注，含配套的 HoloOcean
> 测试/评分/gate 工具链（`adapters/holoocean/`）。
> 方法：5 个并行 review agent 覆盖入口队列、融合管线、HMI/飞手回注、故障注入与验收 gate、
> 追溯表，再追加 2 个 agent 专项审查性能与精度；对最高严重度的结论均用 grep/直接读码复核过，
> 不是单纯转述 agent 输出。
> 前提：这条主线**从未在真实 HoloOcean/UE5 上跑过端到端闭环**，只有单元/冒烟测试，
> 因此下面很多问题是"单测覆盖不到、只有实跑才会暴露"的类型。

## 结论摘要

架构设计本身合理：分层职责边界清晰，"飞手接管闭环"这条硬约束在代码里被严格遵守（没有任何
路径让感知/融合层绕过飞手直接写推进器指令）。但存在若干**会导致首次实跑直接失效**的问题，
以及一批会限制实际精度/性能上限的结构性设计问题。按严重度分四类记录，每条附问题描述、
证据位置、影响，以及可行的解决思路。

---

## A. 阻塞性问题（会导致真实 HoloOcean 首次运行直接失效）

### A1. 时钟域不一致：在线管线用墙钟比对仿真时间，会永久判定"全部不可用" [已修复 2026-08-27]

> 修复方式：新增 `uw::adapters::SimWallClockEstimator`（`include/adapters/
> sim_wall_clock_estimator.hpp` + `src/adapters/sim_wall_clock_estimator.cpp`），
> 用"最近一次收到的 (capture_time, 墙钟接收时刻)"持续重新锚定，`EstimateNow()`
> 用墙钟流逝时间从锚点外推当前仿真时间估计。`holoocean_realtime_sink.cpp` 的
> `Submit()` 在入队前用每条消息的 header 调用 `Observe()`，`deps.now` 改为
> `sim_clock_.EstimateNow()`。新增单测 `tests/adapters/
> sim_wall_clock_estimator_test.cpp`（4 例，覆盖首次观测前兜底、外推、重新锚定、
> 非仿真时钟域不干扰锚点）。`adapters`/`application` 两个 target 编译通过，
> `adapters_tests`（53 例）、`application_tests`（37 例）全绿。

- **证据**：`src/application/holoocean_realtime_sink.cpp:203`（`deps.now` 用
  `std::chrono::system_clock::now()`，墙钟）；`src/adapters/holoocean_live_conversion.cpp:24-27`
  （`capture_time` 明确标注 `CLOCK_DOMAIN_SIMULATION`，仿真时间从每局开始时接近 0 计）；
  `src/application/online_assist_pipeline.cpp` 的 `VisualLive`/`SonarLive`/
  `DenseCurrentlyFresh`/`ComputeDegradation`（约 329-425 行）直接算 `wall_s - capture_s`，
  未做时钟域转换。
- **影响**：真实 HoloOcean 跑起来后，`wall_s - capture_s` 是天文数字，`ComputeDegradation`
  从第一个 tick 起就返回 `STATUS_UNAVAILABLE`。所有单测/冒烟测试用的都是同步好的
  `FakeClock`/墙钟合成夹具，结构性测不到这个问题。**这是当前最高优先级阻塞项。**
- **解决思路**：
  1. 在管线内部维护一个"sim-time ↔ wall-time"锚点：用首次收到的观测的
     `(capture_time, receive_time)` 建立初始 offset，之后所有 staleness 判断先把
     `capture_time` 按 offset 换算成等效墙钟时间再比较。
  2. 由于 HoloOcean 的 RTF 可能不等于 1（甚至可暂停/变速），单一 offset 会随时间漂移，建议
     用最近 N 条观测的 `(capture_time, receive_time)` 做滑动线性回归，周期性重估 offset 而不是
     只用第一条锚定。
  3. 判断逻辑应先读 `header.clock_domain()`，只有确认是同一时钟域才能直接相减；不同域必须
     经过上面的换算——这也是把 `ObservationHeader.clock_domain` 字段真正用起来，而不是写了
     不用。
  4. 为这个转换单独写单测：构造一个 `capture_time` 与 `receive_time` 存在明显非零 offset（且
     offset 会漂移）的合成场景，验证 staleness 判断在这种情况下仍然正确——现有 `FakeClock`
     天然同步，测不出这类回归。

### A2. `realtime_gate.py` 验收 gate 是空壳，会直接抛未捕获异常 [部分修复 2026-08-27]

> 已修复：`evaluate_gate()` 缺字段时现在抛出命名清晰的 `GateFailure`（"字段名:
> missing from run report"），不再是裸 `KeyError`（`run_report.py` 新增
> `_get_required()`）。`run_gate()` 会读回 `TaskScorer` 写的分数文件并塞进
> `report["task_score"]`。`main()` 新增 `required_passes()`：标称 campaign 按
> 8/10、扰动 campaign 按 7/10 判定整体是否通过（SIM-ACC-003），不再是"必须
> 每个 seed 都过"。新增 `tests/test_realtime_gate.py`（12 例）+
> `test_run_report.py` 补一例，`adapters/holoocean` 全量 196 例 pytest 通过。
>
> **仍未修复（范围较大，见下方"整体处理顺序"更新）**：`evaluate_gate()`
> 需要的另外约 15 个遥测字段（`result_age_p95_ms`、`state_age_p95_ms`、
> `rtf_p95`、`deadline_miss_fraction`、`queue_high_watermark`、
> `recovery_duration_s_max`、`rss_growth_after_warmup_mib`、
> `cpu/gpu_headroom_fraction_avg`、`sonar/visual/fused_track_count`、
> `guidance_marked_stale_when_overdue`）仍然没有任何进程回填——这些本质上要
> 靠 C++ 网关自己在运行期采集（复用已有的 `rolling_latency`、
> `LiveEventSource::HealthReports()`，加新的 RSS/CPU 采样、RTF 估计、检测计数
> 器），是一个独立的、量级不小的 C++ instrumentation 工作，没有在这次改动里
> 仓促实现——`evaluate_gate` 现在至少会对着缺失字段给出诚实、可读的失败原因，
> 而不是崩溃或悄悄放过。GPU headroom 尤其没有把握（这台开发机上也没有可靠
> 手段验证），如果实现应明确标注数据来源与局限，不能编造数值。

### A2（原始记录，供追溯）. `realtime_gate.py` 验收 gate 是空壳，会直接抛未捕获异常

- **证据**：`adapters/holoocean/uw_holoocean_adapter/realtime_gate.py` 的 `run_gate()`
  （约 252-257 行）只返回 `{profile, seed, task_id, duration_s}` 四个字段；`evaluate_gate()`
  （`run_report.py`）需要约 20 个字段（`result_age_p95_ms`、`rtf_p95`、
  `deadline_miss_fraction`、`rss_growth_after_warmup_mib` 等）从未被任何进程回填；
  `main()` 只捕获 `(RealtimeGateError, GateFailure)`，第一次 `_require` 会抛未捕获的
  `KeyError`。另外 `main()` 只是逐 seed 打印 PASSED/FAILED，SIM-ACC-003 要求的"标称 8/10、
  扰动 7/10"成功率判据从未被真正比较。`realtime_gate.py` 目前没有任何单测覆盖。
- **影响**：即使仿真器装好了，这个模块现在也**产生不出任何 pass/fail 结论**——它是脚手架，
  不是能用的 gate。
- **解决思路**：
  1. 让 gateway（C++ 侧）在运行期持续统计 `result_age_p95`、`state_age_p95`、
     `deadline_miss_fraction`、队列高水位等，运行结束时写一份 run-report JSON（复用现有
     `RunManifest`/`run_report.py` 里已经定义好的字段结构）。
  2. `_run_scripted_pilot_process`/`_run_scorer_process` 需要把它们各自采集的指标
     （任务分数、RTF、资源占用等）写到共享文件/管道，`run_gate()` 结束时读回这些产物并
     和 gateway 的 run-report 合并成完整 dict，再传给 `evaluate_gate()`。
  3. 给 `evaluate_gate()` 补上"缺字段时抛出明确的 `GateFailure`而不是裸 `KeyError`"的防御；
     `main()` 增加跨 seed 的成功率统计（`passed/len(seeds)` 和 8/10、7/10 的显式比较）。
  4. 补 `test_realtime_gate.py`：至少覆盖"完整字段走 evaluate_gate 正常判定"和"缺字段时给出
     可读的 GateFailure 而不是崩溃"两类用例。

### A3. 飞手/评分进程从未真正接通 ROS2，闭环连模拟意义上都没跑通过 [已修复 2026-08-27]

> 修复方式：
> - 新增 `uw_holoocean_adapter.pilot_ros_bridge.run_scripted_pilot_bridge()`：
>   真正的 rclpy 节点，订阅 `/uw/hmi/status`，用 `hmi_status_bridge.
>   parse_guidance_status()` 解析后喂给 `ScriptedPilot.command()`，发布
>   `/uw/pilot/thrusters`；解析失败时保守发零命令而不是崩溃或悬挂上一条命令。
> - 新增 `uw_holoocean_adapter.scorer_ros_bridge.run_scorer_bridge()`：同时
>   订阅 `/uw/sim/ground_truth` 和 `/uw/hmi/status`，喂给 `TaskScorer`，每秒
>   把 `report()` 写盘一次、退出时再写一次。内部有一个 `_SimTimeTracker`
>   ——复现 A1 同样的"仿真时钟 vs 墙钟"问题在 Python 侧的等价修复（真值时间戳
>   是仿真时间，`/uw/hmi/status` 没有绝对时间戳，如果分别用墙钟/仿真时间喂给
>   `TaskScorer` 内部同一个时间轴会把 completion_time/误报率算错）。
> - 新增 `ros_message_conversion.truth_pose_to_odometry()` + 在
>   `realtime_ros_session.py`'s `build_realtime_messages()` 里接入
>   `PoseSensor` → `/uw/sim/ground_truth`（`nav_msgs/Odometry`，完整无噪声
>   位姿，故意放在 `fault_injector.apply()` 之后追加，结构上保证不会被任何
>   故障配置扰动）；这条 truth 话题此前只在 `TopicMap`/校验逻辑里存在定义，
>   从未被任何代码实际发布过。`main()` 也补上了对应的 ROS2 publisher。
> - `realtime_gate.py` 的 `_run_scripted_pilot_process`/`_run_scorer_process`
>   改为调用这两个真实 bridge（而不是空转 `sleep`）。
>
> 新增/修改测试：`test_hmi_status_bridge.py`（9 例）、
> `test_scorer_ros_bridge.py`（5 例，覆盖 `_SimTimeTracker`）、
> `test_ros_message_conversion.py`/`test_realtime_ros_session.py` 各加 2-3
> 例覆盖 `truth_pose_to_odometry`/ground-truth 话题的有条件发布。rclpy 相关
> 的节点封装本身（`pilot_ros_bridge.py`/`scorer_ros_bridge.py`/
> `realtime_ros_session.py` 的 `main()`）延续仓库既有约定不做单测——本机没装
> rclpy/ROS2，需要真实原生主机才能跑，和 `RealtimeRosSession`/
> `HoloOceanSession` 一直以来的状态一致；已确认这些模块在没有 rclpy 的机器上
> 至少能正常 import（懒加载）。`adapters/holoocean` 全量 196 例 pytest 通过。

### A3（原始记录，供追溯）. 飞手/评分进程从未真正接通 ROS2，闭环连模拟意义上都没跑通过

- **证据**：`realtime_gate.py:155-186` 的 `_run_scripted_pilot_process`/`_run_scorer_process`
  目前只是构造对象后 `sleep`——从未订阅 `/uw/hmi/status`、从未发布 `/uw/pilot/thrusters`，
  代码注释中承认这是等 ROS2 bridge 的占位。`ScriptedPilot`（`scripted_pilot.py`）、
  `PilotCommandModel`（`pilot_command_model.py`）、`OperatorOverlayRenderer` 各自单独实现且
  逻辑合理，但从未被接到一起过。
- **影响**：`/uw/hmi/status → 飞手 → /uw/pilot/thrusters` 这条路径目前是三个互不相连的零件。
- **解决思路**：
  1. 给 `ScriptedPilot` 包一层 `rclpy` node：订阅 `/uw/hmi/status`（JSON 解析成
     `AssistGuidanceStatus` 或直接复用现有解析函数），调用 `ScriptedPilot.step()`，发布
     `/uw/pilot/thrusters`。
  2. `_run_scorer_process` 同理需要真正订阅相关话题 + ground truth 话题（注意维持 A 类以外
     的隔离约束，见 SIM-ARCH-002），定时写分数文件；`run_gate()` 要真正读取这个文件而不是
     丢弃。
  3. 这块工作量相对独立、值得单开一个子任务/PR 来做，因为涉及给 Python 侧新增 rclpy
     依赖和明确的 pub/sub 话题契约。

---

## B. 架构/集成缺口（不阻塞跑通，但会让"跑起来"和"跑得对/跑得可信"脱节）

### B1. 生产网关用占位标定 rig（identity 外参） [已修复 2026-08-27]

> 修复方式：`MakeOnlineAssistRealtimeSink` 签名从直接接收
> `RigCalibrationSnapshot` 改为接收 `HoloOceanRealtimeSinkRigConfig{
> rig_config_path, fallback_rig}`；`ros2` 角色不允许依赖 `runtime`（
> `LoadRigConfig` 所在层），所以 ROS2 网关只负责把 `rig_config_path` 参数
> 值透传下去，真正的 YAML 加载在 `application` 角色的
> `holoocean_realtime_sink.cpp` 里用已有的 `uw::runtime::LoadRigConfig`
> 完成——和离线 `replay_demo` 读取 `configs/rig/*.yaml` 走的是同一个函数、
> 同一份格式。`holoocean_realtime_node.cpp` 新增 `rig_config_path` ROS2
> 参数。没给这个参数时打印醒目 WARNING 并回退到占位 rig；给了但加载失败
> （文件不存在/格式错）时直接抛异常终止，不悄悄回退——操作者显式要求了真实
> 标定却拿到错的,应该立刻发现,而不是被placeholder 悄悄接住。
>
> **本机这次意外发现是真的装了 ROS2 Jazzy + 已 colcon build 好的
> `holoocean_interfaces`**（`/opt/ros/jazzy` + `~/ros2_ws/install`），所以
> 这次修复没有停留在"只保证 CMake 依赖方向对"，而是实际跑了
> `adapters/ros2/README.md` 记录的编译命令，编译通过、`adapters_tests`（53
> 例）/`application_tests`（37 例）全绿，并且真的启动了
> `holoocean_realtime_node` 二进制验证了三种路径：不给参数（打印 WARNING，
> 正常起停）、给真实 `configs/rig/example_auv.yaml`（无报错正常起停）、给
> 不存在的路径（`YAML::BadFile` 抛出，进程终止）——行为符合设计。
>
> **顺带发现一个和这次改动无关的新问题**：不管有没有传 `rig_config_path`、
> 不管加载成功与否，只要进程收到 SIGINT/SIGTERM 就会 "dumped core"（用
> `timeout --signal=SIGINT` 复现过，不是 SIGTERM 独有）——包括从未收到过
> 任何传感器消息的最简单场景。这是 `rclcpp`/节点析构顺序相关的既有问题，
> 和这次的 rig 加载逻辑无关（有效 rig 路径下同样复现），没有在这次改动里
> 深挖修复，记录在此以免遗漏；下次碰这块代码时应该优先查一下 `~HoloOcean
> RealtimeGatewayNode`/`~OnlineAssistRealtimeSink` 的析构顺序和
> `pump_thread_.join()` 是否在 rclcpp 的信号处理路径里发生了未定义行为。

### B1（原始记录，供追溯）. 生产网关用占位标定 rig（identity 外参）

- **证据**：`adapters/ros2/include/adapters/ros2_holoocean_realtime_gateway.hpp:54-61`
  的 `BuildIdentityRig()` 被直接接入真实 ROS2 节点，文档注释自己承认"不携带真实内外参"。
  规格 `FUS-CAL-001` 明确禁止占位外参进入实机验收。
- **影响**：在替换前，所有方位/距离投影在几何上都是错的。
- **解决思路**：
  1. 短期：在 `BuildIdentityRig()` 处加运行清单标记（"non-calibrated placeholder rig"），
     并在 `realtime_gate.py` 里加一条硬校验——检测到 rig 是 identity 占位符时禁止把 gate
     状态标记为 `verified`。
  2. 长期：让 `holoocean_realtime_node` 像 `replay_demo` 一样接受 `--rig` 参数，复用离线
     管线里 `RigCalibrationSnapshot` protobuf 的解析路径，从 `configs/rig/*.yaml` 加载真实
     标定，而不是硬编码 identity。

### B2. 队列背压/丢弃统计算出来了，但没人读 [已修复 2026-08-27]

> 修复方式：把 `BuildStatusJson` 这段原本藏在 `holoocean_realtime_sink.cpp`
> 匿名命名空间里、完全没有单测覆盖的 JSON 拼装逻辑，整体搬到新文件
> `include/application/holoocean_status_json.hpp` +
> `src/application/holoocean_status_json.cpp`（`uw::application::
> BuildOnlineAssistStatusJson`）——只依赖 `domain` 类型，不依赖
> runtime/opencv/ROS2，复用这个仓库既有的"可移植核心 + 薄包装"分层方式
> （类比 `holoocean_live_conversion.hpp` vs `holoocean_realtime_node.cpp`）。
> 新增 `QueueHealthToJson`，函数签名新增 `queue_health` 参数
> （`std::array<HealthReport, 4>`，顺序固定 localization/correction/
> mapping/evidence，和 `LiveEventSource::HealthReports()` 自己文档的顺序
> 一致），JSON 顶层新增 `"queue_health"` 一节，每条车道给出
> `queue_depth`/`queue_high_watermark`/`dropped_frame_count`/
> `rejected_frame_count`/`sequence_gap_count`/`oldest_message_age_ms`/
> `latency_p50/p95/p99_ms`。`RealtimeAssistOutputSink` 新增对 `source_`
> 的常引用（`source_` 在成员声明顺序里排在 `output_sink_` 之前，构造期
> 已经可用；析构靠现有的"先 `Close()`+`join()` 泵线程、再走隐式成员析构"
> 顺序保证不会有悬垂引用），`Publish()` 里改为
> `BuildOnlineAssistStatusJson(state, source_.HealthReports())`。
>
> **范围说明**：只做了"可观测"这一半——JSON 里现在能看到每条车道的丢弃/
> 拒绝/年龄统计了。**没有**把这些指标接回 `OnlineAssistPipeline::
> ComputeDegradation()` 去改 `guidance_valid`/`system_health`——评估后
> 认为这个联动本身价值有限且有风险：姿态队列一旦真的在拒绝消息，
> `last_vehicle_state_capture_s_` 很快就会因为收不到新姿态而触发既有的
> `vehicle_state_stale` 降级路径，安全侧的最终结果是一样的，真正的差距
> 只是"运营方多早能看出原因"，而这正是暴露 JSON 字段已经解决的问题；反过来
> 硬把队列统计接进降级状态机，会在没有真实数据验证阈值的情况下改变
> `guidance_valid` 这个直接影响飞手 `ScriptedPilot` 推进器输出的信号,
> 风险和收益不成比例,所以没有做。
>
> Python 侧 `hmi_status_bridge.parse_hmi_status` 用 `dict.get()` 按已知
> key 取值，新增的顶层 `queue_health` key 是纯新增、向后兼容，不需要跟着
> 改。新增 5 个单测（`tests/application/holoocean_status_json_test.cpp`，
> 手写子串检查——仓库测试依赖里没有 JSON 解析库，和
> `holoocean_live_conversion_test.cpp` 一类手写序列化器的验证方式一致），
> `application_tests`（新增后 46 例）、完整 CTest 567 例全绿，ROS2 网关
> 重新编译通过。

### B2（原始记录，供追溯）. 队列背压/丢弃统计算出来了，但没人读

- **证据**：`LiveEventSource::Stats()/HealthReports()`（`live_event_source.cpp:262-311`）
  完整统计了 `overflow_rejected_count`、`dropped_*`、`sequence_gap_count` 等，但
  `holoocean_realtime_sink.cpp` 的 `BuildStatusJson` 和 `OnlineAssistPipeline::
  ComputeDegradation` 都没有调用这两个方法；`Submit()` 在非 `kClosed` 的拒绝状态下
  （`holoocean_realtime_sink.cpp:262-270`）直接静默 `return`。
- **影响**：真实高负载下队列已经在丢/拒消息，但飞手在 HMI 上完全看不出来，直到数据年龄
  自己超时才会显现——违反 FUS-Q-003"容量不足时必须触发显式背压/降级状态"。
- **解决思路**：
  1. 把 `HealthReports()` 的结果周期性（配合 B3/性能一节的节流机制）塞进
     `BuildStatusJson`，新增 `queue_reject_count`/`queue_drop_count`/`queue_oldest_age_ms`
     等字段。
  2. `ComputeDegradation` 增加一类新的降级原因："某车道拒绝率超过阈值"，不再只看
     capture-time staleness 一种信号源。
  3. `Submit()` 对拒绝状态至少要做计数，即使不立刻改变行为。

### B3 / C3. 入口车道没有"最大驻留时间"这一维度 / 队列没有"先丢后处理"检查 [已修复 2026-08-27]

> 修复方式：`LaneQueueConfig` 新增 `std::optional<double> max_residence_s`
> （nullopt = 不启用，保持原有行为）。`correction`（声呐，CFAR/聚类较贵）
> 和 `mapping`（相机，视觉检测较贵）两条车道默认给 0.5s（对齐 FUS-RT-002
> 的硬过期上限——已经比系统自己的硬过期预算还老的消息，处理得再快也产不出
> 有效结果，不值得付处理成本）；`localization`（姿态/深度，处理便宜，且
> 已经用 `kReject` 提供显式背压）和 `evidence`（同样便宜）默认不启用，
> 避免和 FUS-Q-003"姿态通道不得静默丢失"的精神冲突。
>
> 检查点放在 `Run()` 里、`PopNextLocked()` 弹出之后、交给 consumer 之前
> ——超预算直接丢弃（计入新的 `stats_.stale_dropped_count` +
> 按车道的 `stale_dropped_counts_`，通过 `HealthReports()` 折进已有的
> `dropped_frame_count`）、循环回 `cv_.wait` 而不是把它交给下游的
> CFAR/视觉检测。**实现过程中踩了一个真实的并发正确性坑**：第一版把
> "取 now 用于陈旧判断"这一步挪到了 `PopNextLocked` 之前，结果打破了
> `DequeuesBeforeSamplingClocksWhenAHigherPriorityEventArrivesConcurrently`
> 这个既有的、专门验证"一旦某条消息被选中弹出，并发到达的更高优先级消息
> 不能抢占它"的并发测试——被测试当场抓住并改了回来：现在严格保持"先在
> 锁内按调度优先级弹出并提交，再在锁外采样时钟"这个原有顺序不变，陈旧
> 判断作为弹出之后、锁外的一个后置检查，本身触发时才重新加锁计数、continue
> 回到 `cv_.wait`。
>
> 新增 5 个单测（过预算丢弃且保留新鲜消息、预算内不丢、未配置车道无论多老
> 都不丢、`HealthReports()` 里能看到 `dropped_frame_count` 增加、构造期
> 拒绝非正数预算），加上原有的并发测试验证行为没有被破坏。生产网关
> `holoocean_realtime_sink.cpp` 用的是 `LiveSourceConfig{}` 默认值，直接
> 自动拿到新默认值，不需要额外接线。`runtime_tests`（含 25 例
> `LiveEventSource*`）、完整 CTest 560 例全绿，ROS2 网关重新编译通过。

### B3（原始记录，供追溯）. 入口车道没有"最大驻留时间"这一维度

- **证据**：`LaneQueueConfig`（`live_event_source.hpp:21-24`）只有 `capacity` +
  `overflow_policy`，没有基于年龄的驱逐；`oldest_message_age_ms` 算出来了但没人用它主动
  清队头。
- **影响**：结合下面性能一节的 D3，过载时可能出现"消息已经超过 500ms 硬过期预算，仍被完整
  处理一遍才在下游被丢弃"，正是规格禁止的延迟堆积。
- **解决思路**：给 `LaneQueueConfig` 增加 `max_residence_s`，`PopNextLocked` 弹出队头前先
  检查年龄，超预算直接计入丢弃统计、`continue` 取下一条，不再进入下游处理——这个改动同时
  解决 B3 本身和性能一节的 D3。

### B4. 故障注入矩阵覆盖不全 [已修复 2026-08-27]

> 修复方式：`fault_injector.py` 新增 `SensorDegradationWindow`
> （start_s/duration_s）、`SensorDegradationSchedule`
> （visual_windows/sonar_windows 两条独立时间表——真实的浑浊事件和真实的
> 声呐多径/增益事件没有理由同时发生，分开调度也才能单独练习"只有一种
> 模态退化"这条路径）、`build_sensor_degradation_schedule`（复用已有
> `outage_count`/`outage_duration_s` 的采样风格：给定 seed/时长/窗口数/
> 窗口时长，确定性地在 `[0, duration_s)` 内随机放置窗口）、
> `sensor_degradation_active`（半开区间 `[start, start+duration)` 判断）、
> `resolve_active_degradation`（纯函数：给定 schedule + 窗口内 profile +
> 窗口外 baseline + 当前仿真时间，返回这一刻该用哪个 —— `schedule=None`
> 时永远返回 baseline，对所有没有传 schedule 的既有调用方是纯加法、零
> 行为变化）。
>
> `RealtimeRosSession`（本机没有 rclpy/HoloOcean，历来不直接单测，只测它
> 调用的可移植核心）新增 `sensor_degradation_schedule`/
> `visual_degradation_profile`/`sonar_degradation_profile` 三个可选构造
> 参数；`tick()` 里在 `frame.sim_time_s` 拿到之后，唯一新增的一行是调用
> `resolve_active_degradation(...)`——真正的调度判断逻辑不在这个不可单测
> 的类里，风险面收得很小。`main()` 新增 `--sensor-fault-schedule
> {none,scheduled}`（默认 `none`，保证现有调用方式行为完全不变）+
> `--{visual,sonar}-fault-window-{count,duration-s}` 四个参数；选
> `scheduled` 后，原本"`--visual-degradation critical` = 整个运行期间
> 持续退化"变成"窗口内退化、窗口外自动恢复清澈/干净"，`--visual-degradation`/
> `--sonar-degradation` 的语义从"是否退化"变成"窗口内用哪个 profile"。
>
> 新增 9 个单测覆盖 `build_sensor_degradation_schedule`（确定性、默认
> 窗口数为 0、窗口不越界、拒绝非法参数）、`sensor_degradation_active`
> （半开区间边界）、`resolve_active_degradation`（无 schedule 时恒等于
> baseline、窗口内外切换、**窗口结束后真正恢复**——这条是和"整个运行期间
> 常开"这种旧行为的关键区别）。`adapters/holoocean` 全量 pytest
> 205 例（含新增 9 例）全绿。

### B4（原始记录，供追溯）. 故障注入矩阵覆盖不全

- **证据**：`fault_injector.py` 实现了时间类（时钟偏移/抖动/乱序/重复）、设备中断恢复、单
  推进器降效，但视觉/声呐类扰动（浑浊、后向散射、散斑、增益漂移、距离尺度偏差）目前是
  `scenario_randomization.py`/`holoocean_driver.py` 里场景启动时的静态参数，不是可在运行期
  调度、有 start/duration/recovery 的故障事件。
- **影响**：FUS-FAULT 表要求的"3 周期内降级、2 秒内恢复"这条对视觉/声呐类故障没有被实际
  练习到。
- **解决思路**：把这些参数做成 `fault_injector.py` 里的运行时可调度事件（如果 HoloOcean
  桥不支持运行期改这些底层渲染/声学参数，退而求其次在 `scenario_randomization` 里做"分段
  时间表"——特定时间窗口内切到扰动参数值再切回，通过重启渲染管线的轻量参数而非改变整个
  场景来模拟"故障期"）。

### B5. 一般性传感器掉线恢复缺显式重确认状态 [已修复 2026-08-27]

> 修复方式：新增私有方法 `MarkRecoveringIfModalityWasDropped(last_capture_s,
> new_capture_s)`，在 `RunVisualDetection`/`RunSonarDetection` 开头调用——
> 比较这次检测的 capture time 和该模态**上一次**检测的 capture time
> 之间的间隔，超过 `modality_stale_after_s` 就复用 `UpdateRig()` 已有的
> `recovering_` 标志位。**刻意没有**像标定变化那样做
> `fusion_.emplace(...)`/`pending_*.clear()` 全量重置——模态短暂中断不会
> 让 rig 几何失效，也不该连带清掉另一个模态还在正常跟踪的航迹；
> `FlushAssociation()` 里已有的"任一航迹重新变成 CONFIRMED 才清除
> recovering_"逻辑本身就是 FUS-HEALTH-002 要的"重新确认"闸门，不需要
> 额外清缓存。用的是 capture-time 间隔（同一模态两次检测之间的采集时间
> 差），不是 wall-clock staleness，这样和排队/处理延迟无关，只反映
> "这个模态自己的采集节奏真的断过"。
>
> **写测试时才发现的真实边界情况**：`TargetTracker` 的 `hits` 计数器
> 只会累加、永远不会因为 miss/STALE 被清零，而且 `Predict()`/C2 的
> 过期裁剪都只在 `TargetTracker::Update()` 里跑，`Update()` 只有
> `FlushAssociation()` 会调用，`FlushAssociation()` 只有
> `RunVisualDetection`/`RunSonarDetection` 会调用——如果某个模态完全
> 停止收帧（不是"收到帧但没检测到目标"），它的航迹会在停摆期间被
> 完全冻结（既不被裁剪也不被判 STALE），一旦恢复、只要第一条新检测
> 还是重新关联上这条冻结的老航迹（这次用的测试替身传感器固定在
> boresight 0.0 输出，几乎必然关联上），就会在**同一个 tick 内**
> 触发又立刻清除 recovering_——外部通过 `sink.Latest()` 根本看不到
> 中间态。这不是 bug（recovering_ 确实起了闸门作用，只是这次清除得快），
> 但让"断言某个具体 tick 一定处于 recovering 状态"的测试变得不可靠。
> 改成给 `OnlineAssistPipelineDiagnostics` 新增
> `modality_recovery_count` 计数器，直接、稳定地验证触发条件本身是否
> 命中，不用去赌 tracker 重新确认的时机。
>
> 新增 2 个单测（`ModalityDropoutRecoveryIncrementsRecoveryCounter`
> 验证真实掉线恢复会让计数器 +2（视觉+声呐各触发一次）且系统最终恢复
> `guidance_valid`；`ShortModalityGapBelowStaleThresholdDoesNotEnterRecovering`
> 验证正常调度抖动不会误触发，计数器保持 0）。`application_tests`
> （新增后 14 例 OnlineAssistPipeline 用例）、完整 CTest 572 例全绿，
> ROS2 网关重新编译通过。

### B5（原始记录，供追溯）. 一般性传感器掉线恢复缺显式重确认状态

- **证据**：`UpdateRig()`（`online_assist_pipeline.cpp:176-190`）在标定版本变化时会设置
  `recovering_=true`，要求新的 CONFIRMED 航迹出现后才恢复正常状态；但普通的"传感器车道从
  stale 变回 live"没有等价处理，只是隐式靠卡尔曼协方差自然增长兜底。
- **影响**：不严格满足 FUS-HEALTH-002"恢复时不得直接复用故障前的过期缓存，必须重新确认"
  的文字要求。
- **解决思路**：复用已有的 `recovering_` 机制，只是把触发条件从"仅标定变化"扩展到"任意
  传感器车道从 stale→live 的转换"。

### B6. 追溯表个别行引用不实 [已修复 2026-08-27]

> 修复方式：`tools/lint/_build_realtime_traceability_csv.py` 里改了 4 行
> `ROWS`/`_fus_state` 映射，重新跑该脚本生成 CSV：
> - **FUS-CAL-001**：原引用是 `apps/online_assist_smoke.cpp (BuildRig)`
>   （实现文件不是测试）。改成 `tests/frontends/target_associator_test.cpp`
>   里 `FailsClosedForInvalidCovarianceCalibrationAndFrame`/
>   `RejectsCyclicOrMultiParentRigInsteadOfChoosingAPath` 两个真正测试
>   "rig 帧树不可达/成环/多父节点时 fail-closed"的用例，状态升级为
>   `verified`。
> - **FUS-CAL-002**：同样问题，改引用
>   `AppliesRigClockOffsetsAndKeepsUnmatchedSingleSensorMeasurements`
>   （真正验证了 `time_offset_seconds` 被使用），但如实注明
>   `calibration_version`/`time_offset_provenance` 本身没有被独立断言过，
>   状态维持 `implemented` 不虚报为 `verified`。
> - **FUS-OUT-002**：原引用 `tests/application/*` 是个 glob，不是具体测试；
>   而且原来点的 `latest_assist_sink.hpp` 其实只是测试用的替身 sink，真正
>   在线路径的 replace-latest 实现是 `holoocean_realtime_sink.cpp` 的
>   `RealtimeAssistOutputSink` + ROS2 侧 `rclcpp::QoS(1)`——这两者都没有
>   直接测试。新增 `tests/application/latest_assist_sink_test.cpp`（3 例，
>   真正验证"发布两次只留最新一次"）给替身 sink 补上真测试，module 字段
>   如实写清楚 ROS2 侧实现"未直接测试",不掩盖这个已知的证据缺口。
> - **SIM-SON-002**：原来的 scenario 字段虚标成"全部四档负载都覆盖"，但
>   实际上 `blue_rov_aid_sv1213_base.json` 只写死了一个 10Hz（标称档），
>   压根没有 5Hz 最低档/20Hz 过载档的独立场景文件。改成如实标注只有
>   `rov_realtime_nominal` 一档，并在 module 字段里明确写"no separate
>   5Hz-minimum/20Hz-overload scenario variant exists yet"。
>
> 跑了 `tools/lint/check_realtime_traceability.py` 确认改完还是全绿（CI
> 门槛：只检查 `verified` 行的 evidence_path 是真实存在的文件，不检查
> 引用是否真正对应该需求——这次是手工核对着改的，不是脚本能自动保证的）。
> `application_tests` 新增 3 例、完整 CTest 570 例全绿。

### B6（原始记录，供追溯）. 追溯表个别行引用不实

- **证据**：`FUS-CAL-001/002`、`FUS-OUT-002`、`SIM-SON-002` 这几行的"test"列引用的是实现
  文件或跑题的测试，不是真正验证该行为的测试。
- **影响**：不影响整体可信度（追溯表没有虚报任何"已在真实 HoloOcean 验证"的条目），但个别
  行的证据强度弱于表面看起来的样子。
- **解决思路**：为这几行补上真正验证对应行为的测试，或者把 status 从 `implemented` 降级为
  更如实的状态，跑一遍 `check_realtime_traceability.py` 确认没有破坏 lint 约束。

---

## C. 性能问题

### C1.（最大风险）`PublishNow()` 对每条到达消息无条件全量重渲染 [已修复 2026-08-27]

> 修复方式：`OnlineAssistPipelineConfig` 新增 `min_publish_interval_s`（默认
> 0.1s=10Hz，对应 FUS-RT-003 标称目标输出频率；0 = 不节流，可配置）。
> `PublishNow()` 改为 `PublishNow(bool force = false)`：内部先算好
> `wall_s`，若未到间隔且非强制则直接 return，跳过后面的渲染/JSON 构建/
> `sink_->Publish()` 整段；`Flush()` 和 `UpdateRig()`（"recovering" 状态
> 转换要立刻可见）用 `force=true` 绕过节流。内部状态更新（tracker/关联/
> `last_*_capture_s_`）在节流判断**之前**由调用方各自完成，不受节流影响
> ——节流只影响"要不要真的发布"，不影响"内部状态是否最新"。
>
> 新增 3 个针对性单测（`PublishIsThrottledButPublishedStateStaysCurrent`/
> `PublishThrottlingCanBeDisabledViaZeroInterval`/
> `FlushAlwaysPublishesRegardlessOfThrottleWindow`），验证：50 个事件
> 10ms 间隔只产生个位数发布、`min_publish_interval_s=0` 时逐条发布、
> `Flush()` 无视节流窗口。`configs/defaults/platform.yaml` 补了这个字段
> 及注释。`application_tests`（41 例，含新增 3 例）、`runtime_tests`
> 全绿，完整 CTest 556 例全绿，ROS2 网关重新编译通过。

### C1（原始记录，供追溯）.`PublishNow()` 对每条到达消息无条件全量重渲染

- **证据**：`online_assist_pipeline.cpp:127,136,147,158,172,189` —— `OnImageFrame`/
  `OnSonarFrame`/`OnVehicleState` 等 6 个回调全部无条件调用 `PublishNow()`；
  `RealtimeAssistOutputSink::Publish`（`holoocean_realtime_sink.cpp:128-141`）无条件重渲染
  HMI 叠加（`operator_overlay_renderer.cpp` 里 `canvas.clone()` + 逐 track `putText` +
  声呐面板逐像素 `sqrt`/`atan2`/`lower_bound` 循环，最多 320×320=102,400 像素/帧）+ 重建
  JSON 状态串。
- **影响**：过载工况（相机 1.25×、声呐 20Hz、状态 100Hz）下最多约 145 次/秒全量渲染，而
  画面/声呐帧实际只以各自 10-25Hz 更新——直接挤占 20% CPU 余量要求，可能导致截止期违约。
- **解决思路**：
  1. 加最小发布间隔节流：记录 `last_publish_wall_s_`，若
     `wall_s - last_publish_wall_s_ < min_publish_interval_s`（对齐规格里目标输出 10Hz 的
     标称要求，比如设 80-100ms）且没有"强制刷新"的理由（新目标 CONFIRMED、健康状态跳变）
     就跳过渲染/发布，但内部状态（track/degradation）仍然照常更新，不影响融合本身的实时性。
  2. 也可以走"脏标记"路线：只有当 track set 或健康状态相比上次发布真的变化时才重渲染，
     配合最小间隔一起用，兼顾"新有效观测触发"这条规格要求（FUS-RT-003）。

### C2. `TargetTracker` 从不清理过期航迹 [已修复 2026-08-27]

> 修复方式：`TargetTrackerParams` 新增 `retention_after_s`（默认 5.0s，
> 构造期校验必须严格大于 `stale_after_s`，否则抛
> `std::invalid_argument`/YAML 加载抛 `std::runtime_error`）。新增私有
> `TargetTracker::PruneExpired(reference_time_s)`，在 `Update()` 的两个
> 分支末尾都调用——空批次（只有 miss，无检测）用 `now_s` 做参照，
> 有检测的批次用批内最大 `corrected_time_s`（`batch_capture_time_s`）
> 做参照，和这两个分支原本推进 `Predict()`/`state_time_s` 用的时间基准
> 完全一致。裁剪只删除长期空闲的航迹，不影响 `next_track_id_`/
> `accepted_observation_ids_` 等其它不变量。
>
> 新增 4 个单测：验证"过了 stale_after_s 但没过 retention_after_s 仍保留
> 且标 STALE"、"过了 retention_after_s 就彻底消失"、"裁剪在有新检测的
> batch 分支里同样生效（不只是空批次分支）"、"retention_after_s 不大于
> stale_after_s 时构造直接拒绝"。为了不破坏已有测试对长时间预测数学的
> 覆盖，`LongPredictionUsesWholeElapsedTimeIndependentOfEmptyFrequency`
> 显式把 `retention_after_s` 调大到 20.0（该测试本身要预测到 t=5.1s，
> 跟裁剪窗口无关）。`configs/defaults/platform.yaml` 补了这个字段和
> 中文注释。`frontends_tests`（新增后 25 例）、完整 CTest 556 例全绿。
>
> **顺带发现但本次未处理**：`accepted_observation_ids_`（拒绝重复观测 ID
> 用的 `std::set<std::string>`）同样永久增长，2 小时标称/30 分钟过载连续
> 运行下也是内存增长的候选来源——但它的语义是"永久去重"（防止旧
> observation_id 被重放接受两次），直接按时间裁剪会改变这个去重保证的
> 强度，需要专门评估要不要改成有时间窗口的去重而不是无脑加一行
> erase，所以没有在这次改动里顺手做，留作后续单独决策。

### C2（原始记录，供追溯）. `TargetTracker` 从不清理过期航迹

- **证据**：`target_tracker.cpp` 只有 `std::unique` 做合并去重，没有基于年龄的 erase 路径。
- **影响**：任何一次性虚警（CFAR 误检、瞬时视觉噪声）产生但从未被合并确认的航迹会永久留在
  容器里，每次 `Predict()`/`ToProtoSet()`（被 C1 的高频 `PublishNow()` 反复触发）都要为它
  做矩阵运算；是 2 小时/30 分钟连续运行 RSS 256MiB 增量上限最直接的候选违规点。
- **解决思路**：增加一个"彻底过期"阈值（比 `stale_after_s`=0.5s 大得多，比如 5-10s），对
  超过这个阈值的 STALE 航迹做 erase；阈值要留够余量，避免短暂丢失后立刻被删、重新出现时又
  分配新 ID、打断航迹连续性。

### C3. 队列没有"先丢后处理"检查

- 与 B3 是同一个改动的两面：`PopNextLocked` 目前弹出队头就无条件交给下游做完整处理
  （CFAR、立体匹配等）。**解决思路已在 B3 中给出**——加 `max_residence_s` 后，弹出前先判断
  年龄，超预算的直接计入丢弃统计，不进入下游处理。

### 性能方面确认没问题的部分（无需改动）

- ROS 回调线程只做入队，真正的感知计算在独立 `pump_thread_` 上跑，不会阻塞消息接收。
- 四车道调度是 8:4:2:1 加权轮询（`kWeightedSchedule`），不是严格优先级，高频状态流不会
  饿死相机/声呐车道。
- `TargetAssociator::Associate` 是 O(|视觉|×|声呐|)，在预期的个位数检测量级下没有风险。

---

## D. 精度/效果问题

### D1.（结构性上限）在线视觉检测器是写死的 HSV 绿色阈值分割

- **证据**：`adapters/opencv/src/opencv_visual_assist_frontend.cpp:16,26-30` ——
  `kImplementationLabel = "sim_fixture_detector_v1"`，`hsv_hue_min/max` 硬编码色相带，默认
  `class_label = "aquaculture_zone"`。这是在线目标搜索的**主检测器本身**，不是回环闭合那种
  次要分支。
- **影响**：真实水下图像浑浊/后向散射下红橙色最先被吸收、饱和度整体塌陷，色相判别力会
  直接消失，很可能是拖累 `SYS-ACC-001`（precision/recall ≥90%）在真实场景下达标的最主要
  单一原因。
- **解决思路**：
  1. 短期（仿真验收够用）：至少让检测器的目标外观范围可配置，并在验收报告里明确写清楚
     这是占位检测器、不能用于宣称真实精度（呼应仓库已有的"不得用仿真精度代替实机精度结论"
     原则）。
  2. 长期：替换为不强依赖固定色相的检测方法——比如结合形状/纹理特征，或者用声呐先验区域
     去引导视觉在候选区域内做确认（降低对色彩通道的依赖，声呐已经能提供距离和粗方位）。
     这块工作量大，且需要真实 AI-D 素材/训练数据才能真正验证效果，应该单独立项，不要
     和其他问题混在一次改动里。

### D2. 生产 sink 从未接入版本化配置 [已修复 2026-08-27]

> 修复方式：`HoloOceanRealtimeSinkRigConfig` 扩展/更名为
> `HoloOceanRealtimeSinkConfig`，新增 `platform_config_path` 字段，策略和
> B1 的 `rig_config_path` 完全一致（空则用硬编码默认值 + 醒目 WARNING，
> 给了但加载失败则直接抛异常终止）。`holoocean_realtime_sink.cpp` 新增
> `ResolvePlatformDefaults()`，用已有的 `uw::runtime::LoadPlatformDefaultsConfig()`
> （和离线 `replay_demo` 读 `configs/defaults/platform.yaml` 走的是同一个
> 函数）。`sonar_frontend_` 改用已有的 `uw::application::
> BuildSonarCfarFrontendParams()`（`replay_pipeline.cpp` 里现成的转换函数，
> 直接复用，没有重新发明）转换 `PlatformDefaultsConfig.sonar_frontend`；
> `deps.target_association`/`deps.target_tracker`/`deps.pipeline`
> 三个字段本来就和 `PlatformDefaultsConfig` 对应字段同类型，直接赋值
> 不需要转换。`holoocean_realtime_node.cpp` 新增 `platform_config_path`
> ROS2 参数。
>
> 范围说明：`VisualAssistParams`（在线视觉检测器，HSV 阈值那个）**没有**
> 接进来——不是漏了，是这个仓库里**任何地方**（包括离线管线）都还没有
> 给它建过 YAML 配置 schema，接进来是新建 schema 而不是"接上已有的"，
> 工作量和决策权重都不一样；而且这个检测器本身处于 D1 记录的"可能要整体
> 替换"状态，在那个决策之前先建一套配置 schema 性价比存疑，已在代码注释
> 和本文档里明确记录为待定，不是遗漏。
>
> 验证：`application`/`application_tests` 在本机默认 CMake 配置下编译通过，
> `application_tests` 37 例全绿；额外用真实 ROS2 Jazzy +
> `holoocean_interfaces` 重新编译了 `holoocean_realtime_node` 并实际启动
> 二进制验证了"给两个真实配置文件正常起停"和"platform_config_path 指向
> 不存在文件时立即抛异常终止"两条路径，行为符合设计（同 B1，异常抛出很快
> 发生，`timeout` 报的退出码只是因为 core dump 写入耗时超过测试用的 2 秒
> 窗口，不代表进程真的多等了 2 秒才失败）。

### D2（原始记录，供追溯）. 生产 sink 从未接入版本化配置

- **证据**：`holoocean_realtime_sink.cpp:193-194` 用 C++ struct 默认值构造
  `VisualAssistParams{}`/`SonarCfarFrontendParams{}`；`config.hpp` 里
  `deps.target_association`/`deps.target_tracker` 也留空，从未从
  `configs/defaults/platform.yaml` 加载。当前数值与配置文件一致纯属手工同步巧合。
- **影响**：违反 FUS-AC-002（CFAR/聚类/关联参数必须来自版本化配置）；改配置文件对已编译
  的在线二进制没有任何实际效果，未来调参会静默失效。
- **解决思路**：`holoocean_realtime_node` 增加类似 `replay_demo` 的配置加载路径（`--experiment`
  或专门的在线配置参数），复用 `include/runtime/config.hpp` 的分层加载逻辑，把
  `VisualAssistParams`/`SonarCfarFrontendParams`/`TargetAssociatorConfig`/
  `TargetTrackerConfig` 都从配置文件构造，而不是默认构造。

### D3. 声呐方位/距离不确定度是固定常数，无子波束细化 [已修复 2026-08-27]

> 修复方式：`sonar_cfar_frontend.cpp` 新增 `ExtentAdaptiveSigma(extent,
> default_sigma)`：把簇内检测点近似看作在簇的角度/距离范围内均匀分布，
> 用连续均匀分布的标准差公式 `extent/sqrt(12)` 和配置的 `default_*_sigma`
> 取较大值——`default_*_sigma` 对窄/单波束簇仍然是下限（保证不会比传感器
> 本身的基础分辨率还乐观），簇越宽这个值就越接近真实的不确定度而不是一个
> 固定数。簇的 `angular_extent_rad`/`range_extent_m` 本来就已经在算
> `quality_features` 时算出来了，只是之前没有喂回 `range_sigma_m`/
> `bearing_sigma_rad`，这次顺带把 `quality_features` 里两处重复计算也
> 去掉了（直接复用同一份局部变量）。
>
> 新增 2 个针对性单测（更宽的簇报出比默认值更大的 sigma；更宽的簇比更窄
> 的簇报出更大的 sigma），复用已有的 `MakeSyntheticFrame` 合成声呐帧
> 生成器（新增一个 `column_half_width` 参数控制目标的角度展宽，默认值和
> 原来的硬编码行为一致，不影响已有测试）。**没有做**基于信噪比/强度的
> 修正——那需要一个目前没有实测数据支撑的经验公式，容易凭空编出一个错误
> 假设,留给真机标定数据到位后再做,不在这次改动里编造。
>
> `frontends_tests`（新增后 12 例声呐相关）、完整 CTest 562 例全绿，
> ROS2 网关重新编译通过。声呐前端也被离线管线共用，
> `acoustic_optic_scenario_matrix_determinism`（比较重跑同 seed 输出是否
> 逐位一致）依然全绿，确认这次改动没有引入不确定性或打乱候选排序（排序
> 依据簇大小 `n`，跟 sigma 无关）。

### D3（原始记录，供追溯）. 声呐方位/距离不确定度是固定常数，无子波束细化

- **证据**：`sonar_cfar_frontend.cpp:238-239` —— 每个检测的 `bearing_sigma_rad`/
  `range_sigma_m` 直接用配置常量（约 0.01 rad≈0.57°、0.05m），跟波束宽度、簇范围、信噪比
  无关；方位角直接取原始波束 bin 中心，没有做子波束插值。
- **影响**：如果 SV1213 实际波束间隔有几度，既丢失了本可达到的方位精度，又向融合层报出
  过度自信（偏窄）的协方差，导致 `Fuse()` 系统性过度信任声呐——直接威胁 `SYS-ACC-002` 的
  3° P95 方位门限。
- **解决思路**：把固定常数换成 `f(波束宽度, 簇的角度/距离范围, SNR)` 的函数——DBSCAN 已经
  算出簇的统计量，可以直接用簇宽度/强度做一个更真实的 sigma 下限，不需要等实机数据也能
  先把"明显过度自信"这个问题缓解；后续有实测数据后再做真正的标定校准。

### D4. 运动补偿算出来了，但被丢弃 [评估后暂缓 2026-08-27]

> 评估结论：`bundle.interpolated_vehicle_state` 只在 `AcousticOpticBuffer`
> 凑齐"双目+声呐+状态"三路完全同步的稀有 bundle 时才会产生（目前唯一
> 消费方是稠密深度门控），而 `RunVisualDetection`/`RunSonarDetection` →
> `FlushAssociation` 这条**真正驱动关联的常规路径**根本不会经过
> `HandleBundle`、拿不到这个插值结果——要接上常规路径，需要在
> `AcousticOpticBuffer` 内部新开一个"任意时刻插值姿态"的查询接口
> （目前的插值逻辑封在一次性 bundle 形成流程里，没有独立暴露），再改
> `TargetAssociator::Associate()` 接口签名把这份姿态/角度差传进去做
> bearing 补偿。
>
> 没有在这次改动里做的原因：这类旋转/坐标系补偿正是 CLAUDE.md
> "已经踩过的坑"里明确记录过**两次**独立真实翻车的类型（z 轴 anchor、
> 相机 body frame 共轭方向）——都是单元测试全绿、只有实跑才发现符号/
> 方向搞反。本机没有真实 HoloOcean/UE5，没有办法用真实数据验证一个新写的
> 旋转补偿实现到底对不对；如果符号或参考系搞反，结果是让方位精度从"关联
> 门限悄悄吸收掉一部分偏差"变成"主动引入一个方向错误的修正量"，反而可能
> 比不修更差。这个判断本身也是这次审查要交付的一部分：与其仓促写一个
> 没法验证的修复，不如清楚记录"为什么现在不做、要做的话前提条件是什么"。
>
> 建议后续处理方式：(1) 先给 `AcousticOpticBuffer` 加一个独立的、有完整
> 单测覆盖的"给定时刻插值/外推姿态"查询接口（不涉及 bundle 形成逻辑，
> 风险可控，可以先做）；(2) 用构造出来的合成场景（比如"已知 ROV 以固定
> 角速度自转，验证补偿前后关联精度差异"）在单测里把符号约定钉死，参照
> `camera_body_conjugation.cpp` 的共轭方向验证方式；(3) 接入
> `target_associator.cpp` 时保持关联门限不变或适度收紧（因为运动补偿
> 生效后不再需要靠门限松弛吸收姿态变化），并用一个独立的开关/配置项
> 控制，方便出问题时快速回退到当前行为。这三步建议拆成独立的、有真实
> HoloOcean 数据可验证时再做的后续工作，不属于这次连续会话里应该赶工
> 完成的部分。

### D4（原始记录，供追溯）. 运动补偿算出来了，但被丢弃

- **证据**：`AcousticOpticBuffer::InterpolateState`（`acoustic_optic_buffer.cpp:393-443`）
  实现规范（只在括号内插值、拒绝外推、正确 slerp），产出
  `bundle.interpolated_vehicle_state`；但 grep 确认 `online_assist_pipeline.cpp` 和
  `target_associator.cpp` 全文从未引用这个字段——`vehicle_state` 只被用来判断"新不新鲜"
  （`vehicle_state_ok`），关联/投影时只用静态 rig 外参，从未用来补偿两次采集之间 ROV 自身
  的姿态变化。
- **影响**：正常驾驶时的偏航角速度，在最大 50ms 关联窗口内产生的偏差就可能吃掉 3° 精度
  预算的大半，目前这部分偏差只是被关联门限的松弛量"悄悄吸收"，不是被主动修正。
- **解决思路**：把 `bundle.interpolated_vehicle_state` 传给 `target_associator` 的
  `ProjectOne`，用两个采集时刻之间的姿态差做旋转补偿（body frame 在两个时间点的相对旋转），
  而不是假设静止。需要调整 `target_associator` 接口签名（增加 vehicle_state/delta_rotation
  参数），并针对性写单测：构造一个有明显偏航角速度的合成场景，验证补偿前后关联精度差异。

### D5. 贪心（非匈牙利）关联在密集场景有具体失配风险

- **证据**：`target_associator.cpp:775-808`、`target_tracker.cpp:494-509` 是贪心
  按代价升序分配，不是全局最优的匈牙利算法。
- **影响**：养殖区搜索场景目标稀疏，风险低；结构物巡检场景里平行管道/构件密集，噪声下
  交叉最近邻可能锁死错误配对，正确的另一方被孤立成单模态航迹，丢失距离融合、甚至产生
  重复航迹——集中威胁结构巡检任务的 precision/recall。
- **解决思路**：先加一个"同一 tick 内候选数超过阈值"的可观测计数，用实测数据量化风险
  大小，而不是马上重写；如果结构巡检任务的实测密度确实经常超过 2-3 个候选，再按仓库既定
  原则（"该不该换要靠实测数据关闭，不要因为'手写的不够好'就顺手换"，参见求解器默认值的
  同类决策）替换成匈牙利算法，这个改动相对独立、风险可控。

---

## 建议的整体处理顺序

1. **A1（时钟域）→ A2/A3（gate 与飞手接通）**：这三项不修，整条主线在真实仿真器上完全
   跑不起来，也产不出任何验收证据，必须最先做。
2. **B1（占位 rig）→ D2（版本化配置接入）**：这两项一起做成本较低（都是"把已有的加载路径
   接上"，不需要新算法），做完才能让后续的精度问题在一个"接线正确"的基线上被正确观察到。
3. **C1/C2/C3（性能三件套）**：改动范围集中在 `online_assist_pipeline.cpp` 和
   `live_event_source.hpp/cpp`，建议一次性做，避免 2 小时稳定性测试因为这几个问题而无法
   跑完。
4. **D4（运动补偿接入）→ D3（声呐不确定度自适应）**：这两项不需要等实机数据就能显著改善
   精度上限，值得在第一次真实 HoloOcean 验收之前做完。
5. **D1（视觉检测器）**：工作量最大、且真正见效需要真实/更真实的素材验证，建议单独立项，
   不要卡在第一次端到端验收之前。
6. **B2/B3/B4/B5/B6**：可观测性和鲁棒性方面的加固，可以和上面几项并行推进，不阻塞主线程。
