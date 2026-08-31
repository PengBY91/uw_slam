# uw_slam ROV 在线驾驶辅助深度走读（主线二：实时闭环）

> 逐文件、逐阶段拆解 ROV 在线驾驶辅助（HoloOcean ROS2 话题 → 实时网关 → 四车道
> 事件源 → 在线融合管线 → HMI/飞手回注）的代码与逻辑，是
> [两条主线代码走读](./uw-slam-two-mainlines-walkthrough-2026-08-26.md)主线二部分的
> 深化版，与[离线 SLAM 管线深度走读](./uw-slam-offline-slam-pipeline-deep-dive-2026-08-28.md)
> （主线一）配对。
>
> **最重要的前提**（CLAUDE.md 原话）：这条线的代码写完了，但**没在真实
> HoloOcean/UE5 上跑过**（本机没装仿真器）。别把"有实现 + 有单测"当成
> "闭环验证过"。真正需要原生主机的验收都在 `docs/traceability/rov-realtime-
> closed-loop.csv` 里标为 `gated`，不是 `verified`。
>
> 核对于当前工作树（HEAD `f4d3f3e`），2026-08-28。文中 `file:line` 以该版本为准，
> 后续会漂移——以函数名/文件为准。

## 目录

- [术语速查表](#术语速查表)
- [0. 全景与数据流](#0-全景与数据流)
- [1. 规范基础：三份规格 + 追溯表](#1-规范基础)
- [2. 仿真源侧：HoloOcean 实时会话（Python）](#2-仿真源侧)
- [3. ROS2 网关：`holoocean_realtime_node`](#3-ros2-网关)
- [4. 依赖倒转接缝：`HoloOceanRealtimeSink`](#4-依赖倒转接缝)
- [5. 四车道事件源：`LiveEventSource`](#5-四车道事件源liveeventsource)
- [6. 在线融合管线：`OnlineAssistPipeline`](#6-在线融合管线onlineassistpipeline)
- [7. 目标关联：`TargetAssociator`](#7-目标关联targetassociator)
- [8. 目标跟踪：`TargetTracker`](#8-目标跟踪targettracker)
- [9. 输出侧：HMI overlay、状态 JSON、运行时报告](#9-输出侧)
- [10. 飞手与评分：ScriptedPilot、TaskScorer、真值隔离](#10-飞手与评分)
- [11. Gate 体系：realtime_gate.py 与 run_report.py](#11-gate-体系)
- [12. 无仿真器的测试路径](#12-无仿真器的测试路径)
- [13. v1 简化边界汇总](#13-v1-简化边界汇总)

---

## 术语速查表

正文里反复出现的实时系统/目标跟踪行话，在此给中文对照与一句解释。**代码标识符
（类型名、函数名、文件名、topic 名）保留英文**——它们要与代码对得上、可全文
检索，不做翻译；需要解释的是它们背后的概念。与主线一共用的 SLAM 术语（位姿/
四元数/声呐/标定等）见[离线管线走读的术语速查表](./uw-slam-offline-slam-pipeline-deep-dive-2026-08-28.md#术语速查表)。

| 术语 | 中文 | 一句话解释 |
|---|---|---|
| ROS2 / topic / QoS | 机器人操作系统 2 / 话题 / 服务质量 | ROS2 是机器人消息中间件；topic 是命名消息通道；QoS(1) = 只保留最新一条的投递策略（replace-latest，不积压） |
| WSL2 | Windows Subsystem for Linux 2 | Windows 里的 Linux 子系统；本仓库 C++ 跑在 WSL2、HoloOcean/UE5 跑在 Windows 侧 |
| UE5 / HoloOcean | 虚幻引擎 5 / 基于 UE5 的水下仿真器 | HoloOcean 是搭在 UE5 上的水下机器人仿真平台，本项目的数据源 |
| tick | 仿真步 | HoloOcean 按固定周期推进一步仿真、吐出一批传感器数据，一步叫一个 tick |
| RTF (real-time factor) | 实时率 | 仿真时间流逝速度 ÷ 真实时间流逝速度；RTF=1 表示仿真与真实同步 |
| canonical event | 规范化事件 | 统一包装成"topic + 载荷 + 时间戳 + 序号"的消息对象，让回放和实时共用同一入口契约 |
| lane | 车道（优先级队列通道） | 按处理优先级分组的有界消息队列：定位/校正/建图/证据四条 |
| bounded queue / overflow | 有界队列 / 溢出策略 | 容量有限的队列；满了之后的策略：丢最旧（kDropOldest）、拒收新消息（kReject）等 |
| backpressure | 背压 | 下游处理不过来时向上游显式反压（拒收/丢帧），而不是无限积压 |
| staleness / stale | 过期 / 陈旧的 | 数据年龄超过预算即视为不可用；`stale_after_s` 是过期阈值 |
| watermark | 水位线 | 流式系统里"更早的数据应该都到齐了"的进度标记；本仓库 v1 没有实现，是记录在案的边界 |
| front end / backend | 前端 / 后端 | 前端 = 把传感器原始数据变成测量证据的算法；后端 = 用证据做优化/跟踪的算法 |
| detection / track | 检测 / 航迹 | 检测 = 单帧里"这里有个目标"；航迹 = 跨帧持续跟踪的同一个目标（带 id 和状态） |
| track ID / confirm | 航迹编号 / 确认 | 新检测先成"试探航迹"，连续命中 `confirm_hits` 次才升级 CONFIRMED，防止误检成航 |
| TENTATIVE / CONFIRMED / DEGRADED / STALE | 试探 / 确认 / 降级 / 过期 | 航迹生命周期的四个状态（proto 枚举） |
| Kalman filter / predict / update | 卡尔曼滤波 / 预测 / 更新 | 用运动模型外推状态（预测）、再用测量修正状态（更新）的经典递归估计器 |
| CV model | 匀速模型 | 假设目标速度不变的简化运动模型（本仓库 tracker 用的） |
| state / covariance | 状态 / 协方差 | 状态 = 目标当前的运动量（方位/距离/速率）；协方差 = 对这些量的不确定度矩阵 |
| innovation | 新息 | 测量值 − 预测值（卡尔曼滤波里的"惊喜程度"） |
| Mahalanobis distance | 马氏距离 | 考虑不确定度形状的距离度量：残差按协方差白化后的范数；门限用它的平方（如 9.0 = 3σ²） |
| Joseph form update | Joseph 形式更新 | 协方差更新的一种数值更稳的写法，保证协方差始终对称正定 |
| PSD (positive semi-definite) | 半正定 | 合法协方差矩阵的数学性质；`SanitizeCovariance` 强制修复数值漂移出的非法协方差 |
| gating / gate | 门控 / 门 | 关联前的快速筛选：时间门、马氏门、类别门——过不了门的候选对直接不进分配 |
| greedy assignment | 贪心分配 | 多目标对多航迹的简化分配法：按代价从小到大逐个认领，不保证全局最优（对比匈牙利算法） |
| JPDA / MHT | 联合概率数据关联 / 多假设跟踪 | 学术界的两种多目标关联框架；本仓库 v1 都没做（记录在案） |
| fusion (measurement-level) | （测量级）融合 | 两条模态的测量加权合成一条更准的测量（`TargetAssociator::Fuse`） |
| fused track | 融合航迹 | 同时被视觉和声呐来源喂养的航迹（sources 含 VISUAL+SONAR） |
| dropout | 掉线/断供 | 某个传感器流中断（故障或遮挡）；`--drop-sonar-at-s` 冒烟参数就在制造它 |
| degradation / degraded mode | 降级 / 降级模式 | 部分模态失效后系统降低承诺继续工作（guidance 仍给但标 SUSPECT） |
| fail-closed / fail-loud | 失败即关闭 / 失败即报错 | fail-closed = 出错时不给结果（不给错误的结果）；fail-loud = 配置坏了就拒绝启动，不静默回退占位符 |
| health report / liveness | 健康报告 / 存活 | `HealthReport` proto 是组件自检报告；liveness = "最近还有没有它的消息"（多久没听到就算断） |
| overlay | 叠加显示 | 把航迹/状态画到飞手相机画面上的合成图像（`/uw/hmi/overlay`） |
| replace-latest | 只留最新 | 输出槽永远只存最新一条状态，读端永远拿当前值、不排队积压 |
| throttle / min_publish_interval | 节流 / 最小发布间隔 | 限制昂贵输出（渲染+序列化）的频率，内部状态照常即时更新 |
| pilot / scripted pilot | 飞手 / 脚本飞手 | 驾驶 ROV 的角色；脚本飞手 = 读 HMI 状态、按固定策略发布推进器命令的自动化替身 |
| thruster | 推进器 | BlueROV2 的 8 个水下推进器；飞手命令最终翻译成各推进器推力 |
| ground truth / truth isolation | 真值 / 真值隔离 | 仿真器知道的绝对事实；隔离 = 全系统只有评分器能读真值，算法永远碰不到 |
| scorer / gate / campaign | 评分器 / 验收门 / 战役 | 评分器对照真值打分；验收门 = 指标硬性标准；战役 = 多 seed 的批量验收运行（10 seed 至少 8 过） |
| soak run | 浸泡测试 | 长时间连续运行（如 2 小时）检测内存泄漏/性能退化的测试 |
| fault injection | 故障注入 | 仿真里刻意制造传感器掉线/噪声/时钟偏移，验证系统的降级与恢复 |
| seed (campaign) | 随机种子（战役） | 每个战役用不同 seed 生成不同随机场景，统计成功率而非单次运气 |
| manifest | 清单文件 | 描述仿真资产/传感器/任务的 JSON/YAML 配置（`scenarios/*.json`） |
| rig / calibration version | 传感器rig / 标定版本 | 与主线一同一概念；标定版本号变化会触发在线管线全量重置（见 §6.6） |
| dense depth | 稠密深度 | 对图像逐像素算深度（块匹配），贵；对比"稀疏"= 只对检测出的目标算 |
| block matching | 块匹配 | 稠密立体的经典算法：在右图中滑动搜索与左图小块最相似的块，视差→深度 |
| HSV threshold | HSV 颜色阈值分割 | 按色调/饱和度/亮度范围抠出目标色块——视觉前端目前的占位级做法 |
| Hough transform | 霍夫变换 | 在图像里检测直线/几何形状的经典算法（结构物巡检用） |
| depth prior | 深度先验 | 提供给视觉前端的可选深度信息，用来精化目标的距离估计 |
| Interop / seam / dependency inversion | 互操作 / 接缝 / 依赖倒转 | "接缝"= 隔离两个不许互相依赖模块的抽象接口层（`HoloOceanRealtimeSink`） |
| monotonic clock / wall clock | 单调钟 / 墙钟 | 单调钟 = 只前进不跳变（测时长用）；墙钟 = 系统日期时间（打日志用） |
| saturation (counter) | 计数器饱和 | 计数到类型上限就停（`SaturatingUint64Add`），防止长跑溢出回绕 |
| FNV-1a / hash | FNV-1a 哈希 | 一种简单快速的非加密哈希，用来检测"文件/配置字节变没变" |

---

## 0. 全景与数据流

```
Windows/UE5 主机                          WSL2 (本仓库 C++)
─────────────────────                    ─────────────────────────────────────────────
realtime_ros_session.py (Python)         holoocean_realtime_node (ROS2, adapters/ros2)
  HoloOcean tick → ROS2 话题 ══════════▶   │ 订阅 4 个算法输入话题 + PilotCamera
  /holoocean/auv0/{LeftCamera,            │ holoocean_live_conversion（Raw→domain）
    RightCamera, ImagingSonar,            ▼
    VehicleState}                       HoloOceanRealtimeSink（接缝, src/application/）
                                        │ Submit() 进四车道 LiveEventSource
/scripted_pilot ◀══ /uw/hmi/status ═══  │     │ PumpEvents（后台泵线程）
  （吃 HMI 状态，吐 /uw/pilot/thrusters）│     ▼
                                        │ OnlineAssistPipeline（PipelineInputPort）
/task_scorer ── 唯一可读 /uw/sim/ground_truth   │  视觉检测 ─┐
                                        │  声呐检测 ─┼→ TargetAssociator → TargetTracker
                                        │  稠密深度 ─┘        （仅同步 bundle 时）
                                        ▼
                                       OperatorAssistState → overlay + JSON 状态
                                       ═══▶ /uw/hmi/overlay + /uw/hmi/status（QoS 1）
```

与主线一的关系：**同一套 proto 消息模型 + 同一个 `PipelineInputPort` 事件入口**
（`application/pipeline_input_port.hpp`）。主线一的 `ReplayInputAccumulator` 和
主线二的 `OnlineAssistPipeline` 是同一个接口的两个实现——差别只在下游编排，不在
消息或入口契约上。这就是"回放和实时共用同一套算法代码"的全部机制。

关键文件分层（依赖单向，lint 强制）：

| 角色 | 文件 | 层 |
|---|---|---|
| 仿真会话/飞手/评分/gate | `adapters/holoocean/uw_holoocean_adapter/*.py` | Python 适配器（独立 venv + pytest） |
| ROS2 网关 | `adapters/ros2/src/holoocean_realtime_node.cpp` | adapters/ros2（**唯一**可 include ROS 头的地方） |
| Raw→domain 转换 | `include/adapters/holoocean_live_conversion.hpp` | adapters（可移植、无 ROS 类型，可单测） |
| 依赖倒转接缝 | `include/adapters/holoocean_realtime_sink.hpp` + `src/application/holoocean_realtime_sink.cpp` | 声明在 adapters，实现在 application |
| 四车道事件源 | `runtime/{bounded_queue,live_event_source}.hpp/cpp` | runtime |
| 在线管线 | `application/online_assist_pipeline.hpp/cpp` | application |
| 关联/跟踪 | `frontends/{target_associator,target_tracker,target_fusion_components,sonar_target_extractor}.hpp/cpp` | frontends |
| 时钟桥 | `include/adapters/sim_wall_clock_estimator.hpp` | adapters |
| 无 ROS 冒烟 | `apps/{live_ingress_smoke,online_assist_smoke}.cpp` | apps |

---

## 1. 规范基础

三份规范性文档（`docs/specifications/`，权威顺序见 `docs/README.md`）：

1. **ROV 竞赛在线系统需求规格**——硬件基线（BlueROV2 Heavy + SV1213 声呐 + AI-D
   双目）、基线任务（寻找养殖区 / 巡检水下结构物）、实时性能与降级验收；
2. **HoloOcean 实时闭环仿真规格**——仿真资产、传感器、时间/随机化/故障模型、
   真值隔离；
3. **ROV 声光在线融合链路规格**——在线输入、校验、缓存、同步、前端、关联、航迹、
   输出与健康契约。

规格里的每条需求（`FUS-*`/`SIM-*`/`FUS-RT-*` 等前缀）在
`docs/traceability/rov-realtime-closed-loop.csv`（125 行）逐条对应实现模块 + 测试 +
状态。状态三档：`verified`（有自动化测试实跑）/ `implemented`（实现+单测在，验收
证据待补）/ `gated`（需要原生 HoloOcean/GPU/ROS2 主机，本机不可验证）。跑
`tools/lint/check_realtime_traceability.py` 检查这个 CSV 的一致性——碰了规格或
CSV 才需要跑。

后文每个机制标注对应的需求号（如 FUS-Q-005），可回 CSV 查证。

---

## 2. 仿真源侧

**`adapters/holoocean/uw_holoocean_adapter/realtime_ros_session.py`**：以 manifest
的 tick 率驱动真实 HoloOcean 会话，把每个 tick 的传感器输出转成 ROS2 消息发布——
是 `record_session.py`（离线录制，主线一的真实数据来源）的实时对应物。

结构遵循同一套可测试性拆分：

- `build_realtime_messages`：**可移植、完全可单测的核心**——只消费已取到的
  `RawSensorFrame`，不碰 rclpy/HoloOcean；真正的 ROS2 节点类是底部薄壳，`rclpy`
  在 `main()` 里惰性 import（本机两者皆无）。
- Task 5 扩展：`build_realtime_messages` 可在转换前扰动相机/声呐数组
  （`sensor_perturbation.py`）、把消息过 `FaultInjector`（`fault_injector.py`）——
  都默认 no-op，老调用点行为逐位不变。
- 每 tick 先把 `/uw/pilot/thrusters` 的命令经 `PilotCommandModel` 应用，再步进
  会话——这就是"闭环"的回注半边。
- 已记录的缺口：洋流故障注入未在此接线（`ScenarioRandomization` 没有流速字段，
  修它属于 Task 2 的文件，跨任务边界，留作文档化 gap）。

配套模块：`fault_injector.py`（定时故障：释放时刻/时长/类型，含推进器故障与
传感器退化时间表）、`sensor_perturbation.py`（声呐/双目噪声扰动）、
`scenario_randomization.py`（按 seed 随机化场景）、`pilot_command_model.py`
（飞手命令 → 推进器模型）。

**注意**：本文件以及整个 `adapters/holoocean/` 是 Python，测试走
`(cd adapters/holoocean && .venv/bin/pytest -q)`，与 C++ 的 ctest 互不覆盖。

---

## 3. ROS2 网关

**`adapters/ros2/src/holoocean_realtime_node.cpp`**（`UW_BUILD_ROS2=ON` 时编译）。

订阅（`rclcpp::SensorDataQoS`，前缀 `/holoocean/<agent_name>/`）：

| 话题 | ROS 消息 | 去向 |
|---|---|---|
| `LeftCamera` / `RightCamera` | `sensor_msgs/Image` | 算法输入 |
| `ImagingSonar` | `holoocean_interfaces/ImagingSonar` | 算法输入 |
| `VehicleState` | `nav_msgs/Odometry`（噪声位姿） | 算法输入 |
| `PilotCamera` | `sensor_msgs/Image` | **仅显示**，永不进算法管线（"独立飞手路径"需求 FUS-OUT-004） |

刻意**永不订阅** `/uw/sim/ground_truth`（真值隔离，SIM-ARCH-002 的另一面），
**永不触碰** `/uw/pilot/thrusters`（realtime_ros_session.py 独占）。

处理链：ROS msg → `RawHolo{Image,Sonar,VehicleState}`（纯数据 struct）→
`holoocean_live_conversion.hpp` 的 `ConvertHolo*`（可移植、无 ROS 类型、可单测）
→ `uw::domain` 消息（带递增 `sequence_id`、单调接收时刻）→ `HoloOceanRealtimeSink`。

发布：`/uw/hmi/overlay`（`sensor_msgs/Image`）与 `/uw/hmi/status`（`std_msgs/String`
JSON），都是 **`rclcpp::QoS(1)`**——replace-latest 语义，不积压旧帧。

配置经 ROS2 参数（`declare_parameter`，默认值镜像 manifest
`blue_rov_aid_sv1213_base.json` 的 ImagingSonar 传感器：方位 140°、量程
0.30–30 m、声速 1480 m/s）。两个关键参数：

- `rig_config_path`：真实标定 rig YAML（与 replay_demo 的 rig 层同格式）；
- `platform_config_path`：算法参数（`configs/defaults/platform.yaml` 同格式）。

两个参数的空/坏路径策略见 §4。

为什么 ROS 头只能出现在这个文件：`tools/lint/check_layer_dependencies.py` 的
`ros2` 角色只允许依赖 `{adapters, measurement_api, sensor_models, domain,
domain_proto}`——**不含 runtime/application**，所以这个翻译单元看不到
`LiveEventSource`/`OnlineAssistPipeline`。跨过这条边靠 §4 的接缝。

---

## 4. 依赖倒转接缝

**`include/adapters/holoocean_realtime_sink.hpp`**（纯 domain 类型声明）+
**`src/application/holoocean_realtime_sink.cpp`**（实现，可 include 全栈）。

接口只有两个：

```cpp
class HoloOceanRealtimeSink {          // ROS2 网关调用的入口
  virtual void OnLeftCamera(ImageFrame);
  virtual void OnRightCamera(ImageFrame);
  virtual void OnPilotCamera(ImageFrame);   // 实现必须只做显示缓存
  virtual void OnSonar(SonarFrame);
  virtual void OnVehicleState(VehicleState);
};
class HoloOceanRealtimeOutput {        // 实现回调 ROS2 网关发布
  virtual void PublishOverlay(ImageFrame);
  virtual void PublishStatus(std::string json);
};
```

工厂 `MakeOnlineAssistRealtimeSink(output, config)` 返回的
`OnlineAssistRealtimeSink` 内部**拥有**：`LiveEventSource` + `OpenCvVisualAssistFrontend`
+ `SonarCfarFrontend` + `OnlineAssistPipeline` + `ForwardingPort` + **泵线程**
（线程里跑 `PumpEvents(source_, port_)`，异常有 log + `source_.Close()` 兜底，不会
静默死线程）+ `RuntimeMetricsCollector` + overlay 合成 sink。

**配置解析的 fail-loud 策略**（`ResolveRig`/`ResolvePlatformDefaults`，
FUS-CAL-001/FUS-AC-002）：

- 路径为空 → 用 fallback（identity 外参占位 rig / C++ 硬编码默认参数）+ **大字
  warning**：几何投影全错/编辑 YAML 无效——dev/smoke 可接受，但**这种 run 不得
  当作真机验收证据**；
- 路径非空但加载失败（缺文件/坏 YAML）→ **硬错误，节点拒绝启动**——操作员明确
  要了真配置就别给他静默换占位符。

**时钟域桥**（容易踩的坑）：HoloOcean 转换出的 capture_time 是
`CLOCK_DOMAIN_SIMULATION`（量级 ~0s），不是 Unix 墙钟（~1.7e9 s）。管线的
staleness/降级检查全靠 `deps.now`，若直接接 `system_clock::now()`，第一次 tick 起
每个"数据年龄"都是天文数字、系统永远报不可用。所以这里注入
`sim_clock_.EstimateNow()`（`include/adapters/sim_wall_clock_estimator.hpp`）：
用（sim capture, wall receipt）样本对锚定并外推 sim 时钟。RTF 指标则刻意用**原始
样本对**而非 `EstimateNow()` 的回读（否则只是"验证自己假设的 RTF=1"）。锚定发生在
`Submit()` 入口——入队时刻是最新鲜的配对点，不是泵线程晚些处理时。

`Submit()` 里还有身份语义：PilotCamera 只进 overlay 缓存；每条消息先
`sim_clock_.Observe(header)` 再入队；只有 `kClosed` 状态特殊处理（停机），其余
accept/drop 状态由 LiveEventSource 自己统计。

---

## 5. 四车道事件源：`LiveEventSource`

架构 §13 的落地：ROS 回调只做**有界队列入队**，调度器决定处理顺序。

### 5.1 车道划分（`runtime/bounded_queue.hpp` 的 `Lane`）

| 车道 | 内容 | 容量 | 溢出策略 | max_residence_s |
|---|---|---|---|---|
| kLocalization | IMU/DVL/VehicleState | 64 | **kReject**（显式背压，绝不静默丢，FUS-Q-003） | 无 |
| kCorrection | SonarFrame（→CFAR/聚类，贵） | 32 | kDropOldest | 0.5 s |
| kMapping | ImageFrame（→视觉检测，贵） | 16 | kDropOldest | 0.5 s |
| kEvidence | evidence/health/map（记录类） | 256 | kDropOldest | 无 |

`/gt/state`（StateSnapshot）没有车道——Submit 时被 `kReferenceRejected` 拒收。

溢出策略是构造参数不是硬编码（架构要求：IMU 不能随机丢，相机可以丢旧帧）。

### 5.2 Submit 的语义校验链（`live_event_source.cpp`）

```
closed? → kClosed
topic 不在 canonical_topics 词表 → kSemanticRejected
ValidateCanonicalEvent 失败（时间头/几何有限性等，canonical_event_validation）→ kSemanticRejected
role == kReferenceOnly（真值）→ kReferenceRejected          ← 真值进不了算法车道
同 sensor 的 sequence_id 不增（同 calibration_version 下）→ kDuplicateOrOutOfOrderRejected
入队：kEnqueued / kDroppedOldestAndEnqueued / kDroppedNewest / kOverflowRejected
```

逐传感器维护 `{calibration_version, sequence_id}`——换标定版本后序列号重新对齐，
不会误报乱序。序列间隙计数（gap）单独统计（丢包可见性）。

### 5.3 调度与陈旧丢弃

- `PopNextLocked` 按固定**加权轮转表**扫 15 槽：localization ×8 → correction ×4 →
  mapping ×2 → evidence ×1（FUS-Q-005）。cursor 记住上次位置，空车道跳过。
- `Run()` 单发（与所有 EventSource 一致）；二次调用抛 `std::logic_error`，防止
  两个消费者抢队列。
- **max_residence 在 pop 时检查、在昂贵的 consumer 回调之前**（FUS-Q-004，
  code review findings B3/C3）：已超系统数据年龄预算的消息不值得花钱处理。注意
  细节：一旦 `PopNextLocked` 选中了某条消息，即使它因过期被丢，**也不会**回头
  让更高优先级的并发新事件插队——"已选定的交付不因迟到者改期"有专门的单测钉住
  这个顺序。
- 驻留时延（pop − ingress，单调钟）进 128 样本滚动窗，出每车道 p50/p95/p99。

### 5.4 每车道健康报告

`HealthReports()`（固定顺序 localization/correction/mapping/evidence）：队列深度/
高水位、丢帧/拒帧/序列间隙计数、延迟分位数、**最老消息年龄**、最近有效
capture/receive/processed 时间。这些直接进 §9 的状态 JSON——HMI 上看到的队列
背压就来自这里（"surface queue backpressure in HMI status"，最近一次 commit）。

---

## 6. 在线融合管线：`OnlineAssistPipeline`

**`include/application/online_assist_pipeline.hpp` + `src/application/online_assist_pipeline.cpp`**。
一个 `PipelineInputPort` 实现，输入是 CanonicalEvent，输出是 replace-latest 的
`OperatorAssistState`（经 `AssistOutputSink`）。

设计目标（文件头注释）：**视觉与声呐检测彼此独立**——声呐断供不停视觉-only 航迹
发布，反之亦然；这是"降级模式辅助系统"的存在意义。唯一例外是稠密立体深度：
真正贵到需要实时预算决策的计算，被门在 AcousticOpticBuffer 的完整同步 bundle
后面。

### 6.1 三个入口的骨架

```cpp
OnImageFrame:  bundle = buffer_.AddImage(image);
               if (sensor == rig.cameras(0)) RunVisualDetection(image);  // 只处理左相机
               if (bundle) HandleBundle(*bundle);   // 立体+声呐+状态齐全才触发（dense）
               PublishNow();

OnSonarFrame:  bundle = buffer_.AddSonar(sonar);
               RunSonarDetection(sonar);
               if (bundle) HandleBundle(*bundle);
               PublishNow();

OnVehicleState: 记录 capture 时间；bundle = buffer_.AddVehicleState(state);
               if (bundle) HandleBundle(*bundle);
               PublishNow();
```

OnHealthReport：外部组件健康按 component_id 存 {report, **接收时刻**}——用自己
的 now 而不是报告者的 capture_time，与"多久没听到它"的判断口径一致，且对报告方
时钟漂移鲁棒。IMU/DVL/evidence/GT/map：accept-and-ignore（不阻塞在线环，但绝不
进跟踪）。

### 6.2 视觉检测：`RunVisualDetection`

1. **模态掉线恢复标记**（FUS-HEALTH-002，见 6.6）；
2. 查 rig 里该相机的内参，查不到就返回；
3. 深度先验：仅当 `DenseCurrentlyFresh(capture_s)` 才把 `pending_dense_depth_`
   传给前端；
4. `visual_frontend_->Process(left_image, depth, intrinsics)`——生产实现是
   `opencv_adapters::OpenCvVisualAssistFrontend`（HSV 阈值找养殖区色块 + Hough
   直线找结构物 + 可选深度先验算路径横向偏移；`VisualAssistParams` 里的
   `class_label="aquaculture_zone"`、Hough 参数、`path_offset_sigma_m` 等都是
   它的旋钮。注意：这个前端是**占位级实现**，HSV 阈值假设特定颜色目标——
   code review finding D1 记录了它对真实水下图像的适用性本身是未决问题，也因此
   它的参数还没有 YAML schema）；
5. `pending_visual_` **替换而非追加**：只保留最新一帧自己的检测。追加会让同一
   目标的两次重检出进同一批 Associate+Update——tracker 每批每航迹至多配一个
   检测，第二个重检出会生成幽灵重复航迹。接受的代价：两次声呐到达之间的视觉帧
   可能被顶掉——正常工况下声呐驱动关联批次（见下），真正的声呐断供时反而改为
   每个视觉帧立即 flush；
6. **声呐驱动批次**：正常时视觉检测只是暂存，等下一次声呐到达配对。急切 flush
   会先消费掉视觉检测的 observation_id——tracker 对已接受的 id 整批拒绝，一条
   急着 flush 又被复用的旧检测会毒死之后所有配对尝试。仅当
   `!SonarRecentlyLive(capture_s)`（声呐看起来断了）才立即 flush，让真正的声呐
   断供也能及时产出视觉-only 航迹。

### 6.3 声呐检测：`RunSonarDetection`

```
hypotheses = sonar_frontend_->ProcessSonarFrame(sonar)   // 与主线一同一个 CFAR 前端类
targets    = sonar_extractor_.Extract(hypotheses, sonar) // 注意：保留所有簇，不只 top-1
pending_sonar_ 替换；FlushAssociation(capture_s)          // 声呐是关联批次的驱动方
```

`SonarTargetExtractor`（`src/frontends/sonar_target_extractor.cpp`）与主线一的
"top-1 消费"规则**不同**（FUS-AC-001：在线系统保留每个簇）：

- 校验 payload（有限 sigma、range ≥ 0、质量指标有限非负）；
- 置信度 = `cfar_score/(cfar_score+1)`（簇大小 n 的饱和映射）；
- 协方差 = diag(bearing_sigma², range_sigma²)（主线一 §7 的 extent 自适应 sigma
  在这里直接受益）；
- `class_label = "sonar_target"`、`source = ASSIST_SOURCE_SONAR`；
- **确定性排序**：bearing → range → observation_id → 证据的**规范化序列化字节**
  （`SerializeToCodedStream` + `SetSerializationDeterministic`）——同一输入永远
  同一输出顺序。

### 6.4 关联批次：`FlushAssociation`

```cpp
association = fusion_->associator().Associate(pending_visual_, pending_sonar_, rig_);
if (!fusion_->tracker().Update(association.measurements, now_s))
    ++diagnostics_.association_reject_count;
pending_visual_.clear();  pending_sonar_.clear();   // 已提交的 id 不得重提交
if (recovering_) { 若任一航迹 CONFIRMED 则清除 recovering_; }
```

两个必须理解的点：

- **pending 无论成败都清空**：tracker 的批次是原子的——任何非有限/乱序/已接受的
  observation_id 会整批拒绝且**不产生任何变更**。这个拒绝从 FlushAssociation 的
  返回类型看是静默 no-op，`association_reject_count` 是它发生过的唯一信号。
  已提交的 id 若滞留 pending，下一拍会成为永久的过期配对伙伴。
- **恢复完成的判据**是"存在 CONFIRMED 航迹"而非"收到过检测"——对应 FUS-HEALTH-002
  的"航迹必须经过重确认才能再信"。

### 6.5 稠密深度：`HandleBundle` 与门

`AcousticOpticBuffer`（`include/runtime/acoustic_optic_buffer.hpp`）在 rig 参考钟
里做有界在线配对：t_reference = t_sensor_capture + time_offset_seconds[sensor_id]；
车辆状态在立体对中点做插值（四元数最短路径 slerp、角速度/深度线性、协方差取最近
邻）；半纳秒平局向较早的 capture 取整。要求恰好两相机 + 一启用声呐 + 一状态源。
每次 Add 立即对"当时已缓冲的观测"做决定——没有水位/批末 API，无法预知未来到达
（诚实记录的边界）。诊断计数器齐全（candidate/accepted/no_pair/over_window/
invalid/integrity/capacity/expiry + 校正后时差分位数）。

`HandleBundle` 的门链（FUS-DENSE-001/003）：

1. `dense.enabled`？默认 **false**（repo 全局，configs/defaults/platform.yaml；
   FUS-DENSE-004：没有实测收益研究之前不冒实时预算风险）；
2. 上一帧 dense 还在飞？（同步实现下总是 false——`dense_task_in_flight_` 的存在
   是为了让未来异步 provider 不需要改管线）；
3. provider 为 null？（= 永久 dense_deadline_missed）；
4. 双目都 rectified？（不满足则**不计新失败**——降级健康信号由
   `pending_dense_depth_` 自己的新鲜度老化自然浮出）；
5. `RunBounded(bundle.images, rig, budget_ms)`：`StereoBlockMatchDenseDepthProvider`
   里跑 `StereoOpticalDepthFrontend`（块匹配不可抢占，**超时事后检测、结果丢弃**，
   质量拒绝/预算超限/失败统一呈现为 `dense_deadline_missed`——操作员词表不区分
   为什么没拿到）；
6. 成功 → 存 `pending_dense_depth_` + capture 时刻；失败 → 清空 + 计数。

`DenseCurrentlyFresh`：enabled + 有值 + 距今 ≤ `modality_stale_after_s`（1 s）。
注意它和 `RunVisualDetection` 里的使用检查用的是**两个不同的 now**（发布时刻
wall_s ≥ 那一帧的 capture_s），健康检查永远不会比使用检查更乐观——错也错在
保守方向。

### 6.6 降级状态机：`ComputeDegradation`

优先级链（从上到下第一个命中者生效）：

| # | 条件 | status / reason | guidance_valid |
|---|---|---|---|
| 1 | `recovering_` | RECOVERING / "recovering" | **false** |
| 2 | 视觉、声呐都断 | UNAVAILABLE / "all_assist_unavailable" | false |
| 3 | 车辆状态过期（>0.5 s） | UNAVAILABLE / "vehicle_state_stale" | false |
| 4 | 声呐断（>1 s） | SUSPECT / "sonar_unavailable" | true |
| 5 | 视觉断（>1 s） | SUSPECT / "visual_unavailable" | true |
| 6 | 视觉前端自报不健康 | SUSPECT / 视觉 reason | true |
| 7 | 声呐前端自报不健康 | SUSPECT / 声呐 reason | true |
| 8 | dense 开了但不新鲜 | SUSPECT / "dense_deadline_missed" | true |
| 9 | — | HEALTHY | true |

两个刻意决定：#6 里视觉**排在**声呐前（双方都降级时操作员 top-line 理由取视觉，
声呐报告仍完整出现在 sensor_health 列表里——固定优先级，不是巧合）；"断"对
视觉/声呐用 capture-time 判（`VisualLive`/`SonarLive`），对外部健康用接收时刻判。

**`recovering_` 的两个触发源**（同一标志、两种语义，FUS-HEALTH-002）：

- `UpdateRig` 且 calibration_version 变了：**全量重置**（fusion_ 重建、pending_
  清空、dense 缓存清空）+ 立即强制发布 "recovering"（状态切换不能被发布节流
  拖延）——rig 几何本身变了，旧航迹全部不可信，`calibration_reset_count` 计数；
- `MarkRecoveringIfModalityWasDropped`：某模态相邻两次检测的 **capture-time 间隔**
  > `modality_stale_after_s`——只设标志 + `modality_recovery_count` 计数，
  **刻意不重置** fusion_/pending_/dense（单模态抖动没有让 rig 或另一模态的
  好航迹失效，全量重置是无谓破坏）；重确认门（6.4 的 any_confirmed）就是
  规格要求的那个 gate。比较的是传感器自身节拍的间隔而非墙钟 staleness——
  与排队/处理延迟无关。

这个计数器的语义要读准：它数的是"掉线+恢复**触发器**开火了几次"，与"恢复状态
实际持续多久"无关（后者取决于 tracker 重确认时机，外部不可靠观测）。

### 6.7 发布：`PublishNow` 与节流

**节流**（`min_publish_interval_s`，默认 0.1 s）：每个 On* 调用都会走到
PublishNow——过载时（相机 1.25x + 声呐 20 Hz + 状态 100 Hz）每秒约 145 次，
而每次发布 = HMI overlay 全图渲染 + JSON 重建，都不免费（code review finding
C1）。节流只作用于 **publish 到 sink**；内部跟踪状态（fusion_/pending_/last_*）
照常每条消息即时更新。`Flush()` 与 `UpdateRig` 的恢复切换**强制发布**，不受
节流。

发布的 `OperatorAssistState` 内容：

- 航迹集：`tracker.ToProtoSet(wall_s)`；统计"含 VISUAL+SONAR 双源的航迹出现过的
  发布次数"（`fused_track_publish_count`——是"融合航迹可见"的近似代理，同一航迹
  在多次发布中重复计数；下游验收只查非零，SIM-ACC-002/FUS-ACC-001）；
- 路径横向偏移：有值**且** `VisualLive(wall_s)` 才写——否则相机断供前算的偏移会
  永远重发布且无自己的 staleness 信号；
- system_health = ComputeDegradation 的结果；`data_age_ms` = 墙钟 − 最新
  capture（三类输入取最大）；
- sensor_health：视觉 + 声呐自报 + 外部报告（**过期的不重发布**——外部报告者停
  发时没有别的信号标记其最后一条报告过期，与自家模态的 liveness 判法不同）。

---

## 7. 目标关联：`TargetAssociator`

**`include/frontends/target_associator.hpp` + `src/frontends/target_associator.cpp`**。
职责边界（文件头）：只做**跨模态几何关联**——后验深度更新属于
AcousticOpticDepthFusionFrontend（主线一的声光融合，不在这条线上）。

### 7.1 关联前的坐标与时间归一

`Associate(visual, sonar, rig)` 的第一步是把两种检测统一到 **base_link 极坐标
(bearing, range)** + **校正时间**：

- **帧解析**（`ResolveRigFrames`，FUS-CAL-001）：从 base_link 出发 BFS 验证过的
  frame_tree，算出每个 frame 的 base_from_pose。帧树有环/多父/不可达 → 整体
  失败（fail-closed，**不猜一条路径**）。传感器↔帧匹配规则：`<sensor_id>_link`
  精确匹配（声呐0 的 `sonar_link` 是唯一遗留例外）；视觉输入额外允许唯一的有向
  后代。
- **独占角色检查**（`SensorHasExclusiveRole`）：视觉检测必须来自 rig 里恰好的
  一个相机且不是声呐；声呐反之——防止把非传感器身份的东西混进关联。
- **校正时间**（FUS-CAL-002）：`t_corrected = t_capture + rig.time_offset_
  seconds[sensor_id]`；offset 必须存在、有限、provenance 非空。全程 long double
  防精度损失，上限 2^53（double 能精确表示的整数界）。
- **投影**（带雅可比）：
  - 视觉 bearing-only（无 range）：光学系射线 → body → 传感器系旋转 → base_link
    系方位角；协方差只传方位项（数值导数 × 原方位方差）。
  - 带 range（视觉 tan 投影 / 声呐极坐标）：3D 点 → base_link；
    `(bearing, range)` 和 2×2 协方差都经**数值中心差分雅可比**传播
    （`ProjectWithJacobian`；bearing 差分前先 wrap，range 步长自适应）。
- 全部校验失败都有结构化诊断（`AssociationReason`/`AssociationMetric`/
  value/threshold——每条诊断只报一个指标、值必须有限，调用方不从 NaN 猜语义）。

### 7.2 配对门链（每个 (visual, sonar) 候选对，按序）

1. **时间门** `kCorrectedTimeDelta`：|Δt| ≤ 0.05 s（`max_corrected_time_delta_s`）；
2. **类别兼容** `kClassIncompatible`：同名或任一方是通用标签（`target`/
   `sonar_target`）；
3. **不确定度上限** `kUncertainty`：bearing 方差 ≤ 0.25 rad²、range 方差 ≤ 4.0 m²
   （各方差对自己的阈值取最坏比值）；
4. **统计门**（分两路）：
   - 双方都带 range：bearing Mahalanobis² ≤ 9.0 且 range Mahalanobis² ≤ 9.0
     且**联合** Mahalanobis²（S = P_v+P_s 的 LDLT 白化）≤ 18.0（两阈值之和）；
   - 仅声呐带 range（视觉 bearing-only）：bearing Mahalanobis² ≤ 9.0；
5. **运动连续性** `kMotionContinuity`：|Δbearing| ≤ `max_motion_bearing_delta_rad
   + max_motion_rate_rad_s·Δt`（0.25 + 1.5·Δt——静态偏置 + 速率项）；
6. 全过 → **Fuse** 成一条融合测量，pair cost 上再加归一化的时间项 + 运动项。

### 7.3 Fuse 的数学（互协方差忽略的加权融合）

```
fused.t = max(t_v, t_s)；class = 特定标签优先于通用；confidence = 1−(1−c_v)(1−c_s)
双方有 range：gain = P_v(P_v+P_s)⁻¹（LDLT，非正定则弃）；
             state = x_v + gain·(x_s − x_v)；P = P_v − gain·P_v
仅声呐有 range：标量方位版的同一公式（以声呐为基准）
来源/观测 id 取并集去重排序
```

融合失败（数值原因）→ 该对按 kUncertainty 丢弃，不硬造。

### 7.4 分配与单源保留

- 全局唯一 observation_id：批内任何重复 → **整批原子拒绝**（来源集合是全局唯一
  id 的集合，保留无关测量会让重试语义含糊）。
- 贪心分配：eligible 对按 (cost, visual_id, sonar_id) 排序（确定性），逐个认领
  未占用的两侧；冲突对记 `kPairConflict` 并记录获胜对的 cost。
- **未配对的单源测量保留**（不是丢弃！）：未匹配视觉 → 丢掉 range、保留
  bearing-only 测量；未匹配声呐 → 原样保留。各自有 `kSingleSourceAccepted` 诊断。
  这是"声呐断供不停视觉航迹"在关联层的机制保障。
- 输出测量与诊断都做确定性排序（诊断的主键是模态类别，避免空字符串排序把
  sonar-only 排在 visual-only 前面的意外）。

---

## 8. 目标跟踪：`TargetTracker`

**`include/frontends/target_tracker.hpp` + `src/frontends/target_tracker.cpp`**。

### 8.1 状态与运动模型

每航迹 4 维状态 `[bearing, range, bearing_rate, range_rate]`：

- **CV（匀速）模型**，分步预测：`max_prediction_dt_s`（0.5 s）封顶的单步，步内
  分段常数加速度噪声（`add_cv_noise`：位置项 dt³/3·σ²、交叉 dt²/2·σ²、速度项
  dt·σ²）；
- bearing 每步 wrap 到 (−π, π]；带 range 的航迹 range 预测 ≤ 0 时钉回 1e-6、
  速率清零（物理约束）；
- 协方差传播后过 `SanitizeCovariance`（对称化 + 特征值下限 1e-12）——数值卫生
  贯穿始终（所有进入状态/协方差的值都有 finite/PSD 检查，坏输入在边界被拒）。

### 8.2 批次原子性（`Update`）

**预检不过 → 整批拒绝、零变更**：now_s 非法或倒退；任何检测不满足
`ValidMeasurement`（时间戳合法且不晚于 now、不早于已提交的 capture 水位、置信度
[0,1]、bearing [−π,π]、协方差对称 PSD、来源与观测 id 的组合约束——
visual-only ⇔ 无 range 且 1 个观测；sonar-only ⇔ 有 range 且 1 个观测；融合 ⇔
有 range 且 2 个观测）；任何 observation_id 已被接受或批内重复。

这条原子性就是 6.2/6.4 里两处"pending 管理"纪律存在的原因。

### 8.3 关联与更新（`Update` 非空批次）

1. 检测按 (corrected_time, bearing, FirstObservation) 确定性排序；
2. 候选对：航迹预测到检测时刻后算关联代价——带 range 的用 S = P+R 的联合
   Mahalanobis²（LDLT），bearing-only 的用归一化方位残差平方；门限
   `association_mahalanobis_sq` = 16；
3. 候选按 (cost, 航迹 numeric_id, 检测序号) 排序，贪心认领（确定性；
   floating-point 平局用 id 决出，不靠运气）；
4. **确定性合并**：一个观测同时门到多条近邻航迹 → 全部并进**最老 id**（即使
   某条年轻航迹的浮点 cost 略低）。`MergeClose` 判据：bearing 差 ≤ 0.03 rad、
   range 差 ≤ 0.30 m、类别兼容。合并时 `PromoteRangeFrom` 保留老航迹的方位
   子系统 + 有距航迹的距离子系统，未知跨子系统相关性置零（保守块对角）；
5. 配上的航迹：`UpdateTrack`——首次获得 range 的航迹把 range 状态对中到测量值
   但保留高不确定先验（1e6 方差），让完整相关的量测协方差参与 Joseph 更新；
   量测更新用 **Joseph 形式**（bearing-only 或 bearing+range 两版）——数值
   稳定性优于普通更新式；hits++、misses 清零；非通用标签覆盖、置信度取 max；
   sources/observation_ids 并集去重；`hits ≥ confirm_hits(2)` → CONFIRMED；
6. 没配上的航迹：misses++；≥ `degraded_misses`(3) → DEGRADED；
7. 没配上的检测：新建航迹（协方差从量测初始化；无 range 则 range 方差 =
   `kUnobservedRangeVariance` 1e6；`confirm_hits==1` 时直接 CONFIRMED）；
8. 全部航迹预测到批次的最新 capture 时刻（固定入口延迟得以保持，未来迟到的
   capture 仍可按时间序应用）；接受所有 observation_id；`PruneExpired`；
9. **淘汰**（`retention_after_s` = 5.0 s，code review finding C2）：超过 5 s 没
   有命中的航迹**整体删除**。刻意远大于 stale_after_s（0.5 s）：短暂消失又出现
   的目标应沿用原 id 而不是立刻拿新 id；但没有它，一条永不被确认的虚假检测
   航迹会在多小时运行中无限积累内存和每拍预测开销。

### 8.4 输出与状态语义

- `Tracks(now)`：复制-预测到 now；age > `stale_after_s`（0.5 s）→ 打 STALE
  （FUS-TRACK-003：500 ms → STALE，退出正常引导）。
- `ToProto` 有完整出口校验，其中**来源-可观测性配对约束**（`ValidTrackProvenance`）
  值得记住：visual-only 航迹必须无 range；sonar-only 必须有 range；融合航迹必须
  有 range 且 ≥2 条来源观测。违反则**拒绝发布**（返回 nullopt）——线上的
  TargetTrack 不可能携带自相矛盾的来源声明。

---

## 9. 输出侧

三层输出，全部 replace-latest、无积压（FUS-OUT-002）：

1. **`LatestAssistSink`**（`include/application/latest_assist_sink.hpp`）：互斥锁
   保护的单槽 `OperatorAssistState`。冒烟测试/单测消费这个。
2. **`RealtimeAssistOutputSink`**（`src/application/holoocean_realtime_sink.cpp`）：
   生产输出。Publish 时：取最新 PilotCamera 图像 + 最新声呐帧 →
   `OperatorOverlayRenderer.Render(...)` 合成 overlay → `PublishOverlay`；
   `BuildOnlineAssistStatusJson(state, source_.HealthReports())` → `PublishStatus`
   （**队列背压由此浮上 HMI**）；喂 `RuntimeMetricsCollector`。
3. **ROS2 发布器**：QoS(1)，见 §3。

**运行时报告**（`RuntimeMetricsCollector`，code review finding A2 的补齐）：
`run_report_path` 非空时，借 Publish 的节流节奏再叠加 ≥1 s 的独立门，把 JSON 报告
写到磁盘——结果/状态年龄分位数、deadline 未中率、队列背压、RTF、RSS 增长、CPU
余量、恢复时长、检测/融合航迹计数、guidance 过期标记。rt gate 每一两秒读一次就够，
不配专门写线程（同一注释里记录了"专门线程不值得"的判断）。`deadline_ms`（默认
250 ms）是"发布算准时"的预算口径——nominal 档的 FUS-RT-002 值。

---

## 10. 飞手与评分

- **`scripted_pilot.py`**：消费 `/uw/hmi/status`，按辅助状态（guidance_valid、
  bearing/range、path_lateral_offset）经简单策略发布 `/uw/pilot/thrusters`。
  它**只见 HMI，不见真值**——闭环里飞手是算法输出的第一个"消费者"。
- **`pilot_command_model.py`**：把离散飞手意图转成 8 推进器 BlueROV2 的推力分配
  （带执行器界限，SIM-ROV-002）。
- **`task_scorer.py`**：**全系统唯一**允许消费 `/uw/sim/ground_truth` 的地方
  （`observe_truth`；SIM-ARCH-002）。用 `AssistTrackObservation` 词表（与 pilot
  看到的同一词汇 + confidence）打分：任务成功 / 降级完成（`_DEGRADED_SOURCES =
  ("SONAR",)`——纯声呐引导的完成算降级）/完成时间/有效观测占比。
  `algorithm_topics` 显式镜像 `build_topic_map().algorithm_inputs`，测试可以断言
  评分器的真值消费没漏进算法输入列表。

真值隔离的三道闸（贯穿全链）：网关不订阅 ground_truth（§3）；LiveEventSource 对
reference-only 事件直接拒收（§5.2）；OnlineAssistPipeline 对 GT 事件
accept-and-ignore（§6.1）。

---

## 11. Gate 体系

**`realtime_gate.py`**：监督一次闭环 gate run 需要的**四个进程**——

- HoloOcean 会话：真实 OS 子进程（`subprocess.Popen`，Task 3 自己的 CLI）；
- C++ 网关 `holoocean_realtime_node`：真实独立可执行文件的子进程；
- ScriptedPilot、TaskScorer：`multiprocessing.Process`（fork——没有独立 CLI 的
  已提交文件，补 CLI 超出当时任务范围；仍是真 OS 进程隔离）。

**四个 profile**（`GateProfile`，由 YAML 加载）：`minimum` / `nominal`（20/10/50 Hz）
/ `disturbed`（故障 + 扰动矩阵开）/ `overload`（相机 1.25x、声呐 20 Hz、状态
100 Hz）。前两个是单次连续 soak；种子战役（nominal 10 seed ≥ 8/10 过、disturbed
10 seed ≥ 7/10 过）由 `_REQUIRED_SUCCESS_FRACTION` 控制。

**`run_report.py`**：把每个 run 的指标评成 GateFailure 列表——结果年龄 p95 vs
deadline（FUS-RT-002/RT-003/RT-004、SIM-ACC-006）、RSS 增长/CPU 余量
（SIM-ACC-005）、故障时间线与健康时间线的相关性（SIM-FAULT-002）、code/scenario/
task/config/calibration 哈希（SIM-CFG-003）等。seed/版本随报告落盘，可复现。

这些 gate 的共同点（CSV 里一大片 `gated`）：**本机跑不了**——需要真 HoloOcean/
GPU/ROS2 原生主机。这是"代码完成 ≠ 闭环验证"结论的直接来源。

---

## 12. 无仿真器的测试路径

C++ 侧、不需要仿真器就能实跑的两条冒烟链：

1. **`live_ingress_smoke`**（`apps/live_ingress_smoke.cpp`）：原始摄取链——合成
   事件流以标称频率 Submit 进 LiveEventSource，PumpEvents 到计数 port，验证
   车道/调度/丢弃语义。
2. **`online_assist_smoke`**（`apps/online_assist_smoke.cpp`，Task 8）：**真实**
   检测前端（OpenCvVisualAssistFrontend + SonarCfarFrontend）+ 真实
   OnlineAssistPipeline 挂在同一个 LiveEventSource → PumpEvents 接线上，以
   20/10/50 Hz 跑 5 s，验收**融合航迹非零**（FUS-ACC-001/SIM-ACC-002：验收跑必须
   显示非零声呐/视觉检测数与非零融合航迹，不是"进程活着"）。
   `--drop-visual-at-s` / `--drop-sonar-at-s` 两个开关在指定时刻掐断一个模态，
   专门演练 6.6 的掉线-恢复路径。

单测地图（对应 CSV 的 test 列）：`tests/runtime/live_event_source_test.cpp`（车道/
调度/陈旧丢/健康）、`tests/runtime/acoustic_optic_buffer_test.cpp`（同步窗口）、
`tests/frontends/target_associator_test.cpp`（门链/fail-closed rig/时间偏移）、
`tests/frontends/target_tracker_test.cpp`（原子批/确认/降级/淘汰/出处约束）、
`tests/frontends/sonar_target_extractor_test.cpp`、
`tests/application/online_assist_pipeline_test.cpp`（降级链/恢复/dense 门/节流）、
`tests/adapters/opencv_visual_assist_frontend_test.cpp`、
`tests/adapters/holoocean_live_conversion_test.cpp`、Python 侧
`adapters/holoocean/tests/test_*.py`。

---

## 13. v1 简化边界汇总

| # | 边界 | 出处 |
|---|---|---|
| 1 | **整条闭环未在真实 HoloOcean/UE5 上跑过**；native-host 验收全部 `gated` | CLAUDE.md、CSV、realtime_gate.py 文件头 |
| 2 | 视觉前端是 HSV 阈值 + Hough 占位级实现，参数无 YAML schema；对真实水下图像的适用性未决 | `opencv_visual_assist_frontend.hpp`、finding D1 |
| 3 | 网关默认 fallback 是 identity 外参占位 rig——投影全错，仅供 smoke；真验收必须配 `rig_config_path` | `ResolveRig`、FUS-CAL-001 |
| 4 | dense depth 默认关（无实测收益研究）；启用时的预算检查是事后判定（块匹配不可抢占） | FUS-DENSE-004、`StereoBlockMatchDenseDepthProvider` |
| 5 | AcousticOpticBuffer 的配对只看"当下已缓冲"，无水位/批末 API——未来到达不可知 | `acoustic_optic_buffer.hpp` 文件头 |
| 6 | 关联无多假设跟踪（MHT）/JPDA——贪心分配 + 确定性合并；遮挡/交叉目标的消歧靠 tracker 的合并规则 | `target_associator.cpp` |
| 7 | tracker 是 CV 模型——无 IMM/CT 模型切换，机动目标靠过程噪声与门限容忍 | `target_tracker.cpp` |
| 8 | 洋流故障未接入实时会话（跨 Task 2 文件边界的文档化 gap） | `realtime_ros_session.py` 文件头 |
| 9 | 飞手是脚本策略（非人在环）；评分降级语义目前只有 "SONAR" 一档 | `scripted_pilot.py`、`task_scorer.py` |
| 10 | VisualAssist 的真实标定链（非 y 轴基线 rectify 后 VO 崩溃的教训，见主线一 §4）对在线视觉检测路径同样适用但未验证 | CLAUDE.md |
| 11 | run_report 借发布节奏写盘（≥1 s 门）——不是独立遥测线程 | `MaybeWriteReport` |
| 12 | `opencv_adapters` 的 VisualAssistParams 不进 platform_config_path 的加载范围（无 schema 可加载） | `HoloOceanRealtimeSinkConfig` 文件头 |
