# Live/Replay 统一输入主链实施计划

> **执行要求：** 实施时使用 `superpowers:executing-plans`，按任务逐项执行和复核。涉及生产代码变更时使用 TDD；未经用户明确授权，不执行计划中的 Git 提交步骤。

**目标：** 在不改写现有前端、因子、估计和地图算法的前提下，建立一条按规范消息、单次有序读取、错误可审计的输入主链，使 MCAP replay 与未来供应商 SDK live gateway 能向同一应用入口注入数据。

**本计划定位：** 这是路线图 S1 的第一个软件实施包，预计 2–3 周。它解决“当前只有批量回放编排、没有可复用在线入口”的根因，但不在本包中实现供应商 SDK、完整实时调度器、HMI 或在线 SLAM。

**技术栈：** C++17、Protobuf、MCAP、GoogleTest、CMake/CTest；HoloOcean 录制修复使用 Python/pytest。

---

## 1. 设计边界

### 1.1 必须保持不变

- `schemas/proto/uw/domain/` 仍是跨进程、跨语言消息的唯一真相源；本计划不新建平行 DTO。
- 新增的 `CanonicalEvent` 只是现有 Protobuf 消息的进程内 `std::variant` 包装，不序列化、不跨进程。
- `runtime` 可以依赖 `domain/core/measurement_api`，不能依赖 `application`、算法实现或 vendor SDK。
- `application` 负责组装和调度；`apps/` 继续只保留 CLI。
- 现有 `ReadMcapMessages<T>` 暂时保留，迁移完成前不破坏 `bag_audit`、评测工具和测试。
- Ground truth 只能进入评测支路，不能进入在线算法输入。

### 1.2 本包完成后的主链

```text
MCAP ReplaySource ─┐
                  ├─> CanonicalEvent ─> PipelineInputPort ─> 现有处理/评测组件
SDK LiveSource ───┘                         ├─> Health / metrics
                                           └─> Recorder tap（下一实施包）
```

本包只实现 `MCAP ReplaySource`、事件契约、应用输入端口和现有回放迁移。`SDK LiveSource` 在下一实施包实现，但必须只依赖本包冻结的接口。

### 1.3 明确不做

- 不增加通用插件框架、依赖注入容器或动态组件注册表；
- 不把 `SurfelMap` 强行接入当前回放热路径；
- 不实现 VIO、回环、固定滞后优化或全自主控制；
- 不定义尚未和 HMI/机械臂确认的目标、抓取或任务结果 Protobuf；
- 不删除当前批量求解逻辑，只把输入读取和身份关联从中拆出。

---

## 2. 完成标准

本计划只有同时满足以下条件才算完成：

1. 一个 MCAP 文件只扫描一次即可得到当前回放所需的全部规范消息；
2. 事件顺序由 MCAP `logTime` 和稳定次序定义，调用者不再按 topic 重扫文件；
3. 未知 topic、schema 不匹配和 payload 解析失败都有显式统计，不能静默丢失；
4. `RunReplayPipeline` 不再根据 `capture_time / 0.2s` 推导 `kfN`；身份来自消息中的 `ObservationId`/`EvidenceId`；
5. 现有 replay 输出与门禁在同一输入下保持确定性；
6. HoloOcean 录制器在没有相机帧的 tick 上仍独立记录 IMU、DVL 和声呐；
7. 新增“同一事件序列经 MCAP source 和内存 source 注入，应用看到的消息与顺序一致”测试；
8. 全量构建、296 项现有测试及新增测试全部通过，分层 lint 继续通过。

---

## 3. Task 1：冻结规范 topic 与进程内事件契约

**文件：**

- 新建：`include/runtime/canonical_topics.hpp`
- 新建：`include/runtime/canonical_event.hpp`
- 新建：`tests/runtime/canonical_event_test.cpp`
- 修改：`cmake/Tests.cmake`

### Step 1：先写失败测试

覆盖以下行为：

- 已知 topic 能映射到唯一的 `CanonicalEventKind`；
- topic 与 Protobuf 类型不匹配时返回错误，而不是构造错误事件；
- `capture_time` 相同的两个事件仍保留各自的 MCAP `log_time_ns` 和输入序号；
- `/gt/state` 被标记为 reference-only，不能被误认为算法状态输入。

测试接口以如下契约为准：

```cpp
namespace uw::runtime {

enum class CanonicalEventKind {
  kImageFrame,
  kSonarFrame,
  kImuSample,
  kDvlSample,
  kMeasurementEvidence,
  kStateSnapshot,
  kHealthReport,
  kMapEvidence,
};

using CanonicalPayload = std::variant<
    uw::domain::ImageFrame,
    uw::domain::SonarFrame,
    uw::domain::ImuSample,
    uw::domain::DvlSample,
    uw::domain::MeasurementEvidence,
    uw::domain::StateSnapshot,
    uw::domain::HealthReport,
    uw::domain::MapEvidence>;

struct CanonicalEvent {
  std::string topic;
  uint64_t log_time_ns = 0;
  uint64_t source_sequence = 0;
  CanonicalPayload payload;
};

}  // namespace uw::runtime
```

`canonical_topics.hpp` 集中定义当前已使用的 topic 字符串和 topic→类型元数据。禁止在新增 reader/source 代码中重复硬编码 `/raw/...`。

### Step 2：运行测试，确认失败

```bash
cmake --build build -j2 --target runtime_tests
ctest --test-dir build -R 'unit.runtime.CanonicalEvent' --output-on-failure
```

预期：编译失败或测试失败，因为事件契约尚不存在。

### Step 3：最小实现并通过测试

实现头文件，不增加跨层依赖。`CanonicalEvent` 不提供 vendor 类型构造函数，也不携带算法对象。

### Step 4：验证分层

```bash
python3 tools/lint/check_layer_dependencies.py .
```

预期：输出 `OK`。

### Step 5：提交（仅用户授权时）

```bash
git add include/runtime/canonical_topics.hpp include/runtime/canonical_event.hpp tests/runtime/canonical_event_test.cpp cmake/Tests.cmake
git commit -m "feat(runtime): define canonical event contract"
```

---

## 4. Task 2：实现单次扫描、错误可审计的 MCAP EventSource

**文件：**

- 新建：`include/runtime/event_source.hpp`
- 新建：`include/runtime/mcap_event_source.hpp`
- 新建：`src/runtime/mcap_event_source.cpp`
- 新建：`tests/runtime/mcap_event_source_test.cpp`
- 修改：`cmake/Libraries.cmake`
- 修改：`cmake/Tests.cmake`

### Step 1：先写失败测试

测试创建一个包含交错相机、IMU、声呐和未知 topic 的临时 MCAP，并验证：

- reader 只打开并遍历一次文件；
- callback 收到的已知事件按 `(log_time_ns, source_sequence)` 稳定排序；
- 未知 topic 计入 `unknown_topic_count`；
- payload 与 channel schema 不匹配时计入 `parse_failure_count`；
- 文件无法打开时返回 `kOpenFailed`；
- callback 主动停止时返回 `kStoppedByConsumer`，reader 仍正确关闭。

排序测试必须让 `McapProtobufWriter::WriteMessage()` 的**调用顺序**与消息的 `log_time_ns` 顺序不一致（例如先调用写入一条时间戳更晚的声呐消息，再写入一条时间戳更早的图像消息）。如果测试夹具本身是按时间顺序调用写入的，MCAP C++ SDK `reader.readMessages()` 的默认 `ReadOrder::FileOrder`（写入顺序）会和 `LogTimeOrder` 碰巧一致，测试就测不出排序实现是否正确——这个坑必须在测试设计阶段堵住，不能留到实现阶段才发现。

接口以如下最小契约实现：

```cpp
namespace uw::runtime {

enum class EventSourceStatus {
  kCompleted,
  kOpenFailed,
  kStoppedByConsumer,
};

struct EventSourceReport {
  EventSourceStatus status = EventSourceStatus::kOpenFailed;
  uint64_t messages_seen = 0;
  uint64_t events_emitted = 0;
  uint64_t unknown_topic_count = 0;
  uint64_t parse_failure_count = 0;
};

using EventConsumer = std::function<bool(const CanonicalEvent&)>;

class EventSource {
 public:
  virtual ~EventSource() = default;
  virtual EventSourceReport Run(const EventConsumer& consumer) = 0;
};

class McapEventSource final : public EventSource {
 public:
  explicit McapEventSource(std::string path);
  EventSourceReport Run(const EventConsumer& consumer) override;
};

}  // namespace uw::runtime
```

### Step 2：运行测试，确认失败

```bash
cmake --build build -j2 --target runtime_tests
ctest --test-dir build -R 'unit.runtime.McapEventSource' --output-on-failure
```

### Step 3：实现 topic/type 校验与解析

实现要求：

- 用 channel topic 查找 `canonical_topics.hpp` 元数据；
- 同时校验 MCAP schema name 与目标 Protobuf descriptor full name；
- 解析失败必须计数；
- 不在 reader 内做图像转换、同步、关键帧生成或算法调用；
- 不用异常吞掉坏数据；错误通过 report 返回；
- 必须显式传入 `mcap::ReadMessageOptions{.readOrder = mcap::ReadMessageOptions::ReadOrder::LogTimeOrder}`——vendored `mcap/reader.hpp` 里 `readOrder` 默认值是 `FileOrder`（按写入顺序返回，不是按 `logTime`），直接调用 `reader.readMessages()` 不传 options 拿到的不是完成标准 #2 要求的顺序；同一 `log_time_ns` 的多条消息再按 `source_sequence`（本 reader 自己维护的读取序号，用于稳定 tie-break）排序。

现有 `ReadMcapMessages<T>` 暂不删除。给它补充 deprecated 迁移注释，但不要在本任务改变其返回签名，以免扩大影响面。

### Step 4：通过目标测试与现有 MCAP 测试

```bash
cmake --build build -j2 --target runtime_tests
ctest --test-dir build -R 'unit.runtime.(McapIo|McapEventSource)' --output-on-failure
```

### Step 5：提交（仅用户授权时）

```bash
git add include/runtime/event_source.hpp include/runtime/mcap_event_source.hpp src/runtime/mcap_event_source.cpp tests/runtime/mcap_event_source_test.cpp include/runtime/mcap_io.hpp cmake/Libraries.cmake cmake/Tests.cmake
git commit -m "feat(runtime): add audited single-pass MCAP event source"
```

---

## 5. Task 3：建立与来源无关的应用输入端口

**文件：**

- 新建：`include/application/pipeline_input_port.hpp`
- 新建：`include/application/event_pump.hpp`
- 新建：`src/application/event_pump.cpp`
- 新建：`tests/application/event_pump_test.cpp`
- 修改：`cmake/Libraries.cmake`
- 修改：`cmake/Tests.cmake`

### Step 1：先写失败测试

用一个内存 `EventSource` 和 spy input port 验证：

- image/sonar/IMU/DVL/evidence/reference state 被分派到正确方法；
- reference state 只调用 `OnReferenceState`；
- input port 返回 false 时 source 停止，report 保留已处理数量；
- 同一事件序列通过内存 source 与 MCAP source 时，spy 观察到的类型、topic、时间和顺序一致。

最小接口：

```cpp
namespace uw::application {

class PipelineInputPort {
 public:
  virtual ~PipelineInputPort() = default;
  virtual bool OnImageFrame(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnSonarFrame(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnImuSample(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnDvlSample(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnMeasurementEvidence(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnReferenceState(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnHealthReport(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnMapEvidence(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool Flush() = 0;
};

uw::runtime::EventSourceReport PumpEvents(
    uw::runtime::EventSource& source,
    PipelineInputPort& input);

}  // namespace uw::application
```

不要让 `PipelineInputPort` 暴露 MCAP、ROS2 或供应商 SDK 类型。

### Step 2：运行失败测试

```bash
cmake --build build -j2 --target application_tests
ctest --test-dir build -R 'unit.application.EventPump' --output-on-failure
```

### Step 3：实现 variant visitor 与停止语义

`PumpEvents` 只做来源与端口之间的分派，不做算法处理。`Flush()` 只在 source 正常完成时调用；消费者中止或打开失败时不伪装成正常结束。

### Step 4：验证

```bash
cmake --build build -j2 --target application_tests runtime_tests
ctest --test-dir build -R 'unit.(application.EventPump|runtime.McapEventSource)' --output-on-failure
```

### Step 5：提交（仅用户授权时）

```bash
git add include/application/pipeline_input_port.hpp include/application/event_pump.hpp src/application/event_pump.cpp tests/application/event_pump_test.cpp cmake/Libraries.cmake cmake/Tests.cmake
git commit -m "feat(application): add source-independent pipeline input port"
```

---

## 6. Task 4：迁移 replay 输入，消除多次扫描和 0.2 秒身份推导

**文件：**

- 新建：`include/application/replay_input_accumulator.hpp`
- 新建：`src/application/replay_input_accumulator.cpp`
- 新建：`tests/application/replay_input_accumulator_test.cpp`
- 修改：`src/application/replay_pipeline.cpp`
- 修改：`include/application/replay_pipeline.hpp`
- 修改：`tests/application/replay_pipeline_test.cpp`
- 修改：`apps/synth_bag_gen.cpp`
- 修改：`cmake/Libraries.cmake`
- 修改：`cmake/Tests.cmake`

### Step 0：前置修复——补全 `ImageFrame.header.observation_id`

本任务的完成标准（#4：不再用 `capture_time/0.2s` 推导 `kfN`，身份来自 `ObservationId`）目前在合成回放路径下无法达成，必须先修：

- `apps/synth_bag_gen.cpp` 的 `BuildStereoPair`（约第 273–289 行）给 `ImageFrame.header` 设置了 `sensor_frame`/`sensor_id`/`capture_time`/`receive_time`/`clock_domain`/`validity`/`provenance`，唯独没有设置 `observation_id`。这正是 `replay_pipeline.cpp` 里 `kKeyframeIntervalS` 这个时间推导 hack 存在的根本原因（该文件约第 414–423 行的注释原话："synth_bag_gen's camera ImageFrames don't carry a keyframe id via header.observation_id ... only a capture_time"）。
- `tests/application/replay_pipeline_test.cpp` 的 `MakeCameraImage` helper（约第 25–32 行）同样没有设置 `observation_id`。
- 声呐帧（`BuildStereoPair` 之外，`synth_bag_gen.cpp` 约第 436 行）和 depth evidence（约第 517–519 行）已经正确携带 `ObservationId`/`source_observations`，说明这只是相机帧这一条路径的遗留缺口，不是普遍问题。
- 真实 HoloOcean 录制路径（`record_session.py` 的 `_write_keyframe`）已经给相机帧设置了 `observation_id=kf_id`，可以直接照抄这个约定（`"kf" + str(index)`）应用到 `BuildStereoPair`。

在 Step 1 写失败测试之前，先给 `BuildStereoPair` 和 `MakeCameraImage` 补上 `observation_id`，并删除/更新 `replay_pipeline.cpp` 里描述"相机帧没有 observation_id"的过期注释（约第 39 行、第 414–423 行）。不这样做的话，Step 4 的确定性验证会因为现有 `replay_demo`/`integration.replay_determinism`/`application_tests` 用到的相机帧全部被判定为空 id 输入错误而失败。

### Step 1：为身份关联写回归测试

新增一个非 5 Hz、时间有抖动的 synthetic bag：

- observation id 使用 `frame-A`、`frame-B`，时间间隔分别为 173 ms 和 241 ms；
- evidence 通过 `MeasurementEvidence.source_observations`（`repeated ObservationId`，注意不是单数 `source_observation_id`）指向对应 frame；
- 验证 replay 不生成 `kf0/kf1`，且关联关系不依赖时间整除；
- 两条消息 capture time 相同但 observation id 不同时不能被覆盖；
- 缺失或重复身份返回明确输入错误并退出非零。

先运行：

```bash
cmake --build build -j2 --target application_tests
ctest --test-dir build -R 'unit.application.ReplayInputAccumulator' --output-on-failure
```

预期：失败，因为当前 `replay_pipeline.cpp` 在约第 339、421 行仍使用固定 `kKeyframeIntervalS = 0.2` 和时间反推 id。

### Step 2：实现 accumulator

`ReplayInputAccumulator` 实现 `PipelineInputPort`，只负责把有序事件整理为 replay 现有算法所需的明确索引：

```cpp
struct ReplayInputData {
  std::vector<uw::domain::ImageFrame> images;
  std::vector<uw::domain::SonarFrame> sonar_frames;
  std::vector<uw::domain::ImuSample> imu_samples;
  std::vector<uw::domain::DvlSample> dvl_samples;
  std::vector<uw::domain::MeasurementEvidence> evidence;
  std::vector<uw::domain::StateSnapshot> reference_states;
};
```

身份校验规则：

- raw observation 使用 `header.observation_id.value`；
- evidence 使用其 `source_observations`（`repeated ObservationId`），不得用时间生成 id；
- reference state 仅按 `state_id`/capture time 用于评测；
- 空 id、重复的 `(sensor_id, observation_id)`、引用不存在的 source observation 都进入 `ReplayInputDiagnostics`；
- IMU/DVL 保持采样级顺序，不强行归属相机关键帧。

### Step 3：将 `RunReplayPipeline` 的读取阶段替换为一次 Pump

在 `RunReplayPipeline` 开头构造：

```cpp
uw::runtime::McapEventSource source(opt.bag_path);
ReplayInputAccumulator accumulator;
const auto report = PumpEvents(source, accumulator);
```

随后现有整流、前端、求解、映射、评测逻辑从 `ReplayInputData` 获取数据。删除函数内所有 `ReadMcapMessages<T>` 调用和 `kKeyframeIntervalS` 身份推导。

这一步不要同时重写求解器、地图后端或输出格式。保持已有 manifest、gate 和 CLI 行为，从而把回归范围限制在输入阶段。

现有代码里 `/evidence/depth` 被至少 3 处独立的 `ReadMcapMessages<MeasurementEvidence>` 调用分别按不同 `oneof` payload 消费（kf0 锚点的 `PressureDepthMeasurement` 查找、常规 depth 因子构建、声光融合通路），`SonarFrame`/`ImageFrame` 也各有多处独立扫描。合并成单次 `ReplayInputData` 之后，这些消费点原有的按 payload 类型过滤逻辑都要在扁平化后的 vector 上正确复现——这不是简单删掉几个 `ReadMcapMessages<T>` 调用就能完成的机械替换，要按消费点逐一核对过滤条件迁移正确。

### Step 4：确定性验证

```bash
cmake --build build -j2 --target replay_demo application_tests
ctest --test-dir build -R 'unit.application|integration.replay_determinism' --output-on-failure
```

预期：全部通过，且同一 bag 的 replay 输出保持字节确定性。仅靠这个聚合确定性 gate 不足以覆盖上一段提到的多消费点迁移风险——在 `replay_pipeline_test.cpp` 里为每条 evidence 消费路径（kf0 锚点 z 值、常规 depth 因子计数、声光融合触发）各补一条针对性断言，而不是只信总体 ATE/gate 不变。

### Step 5：提交（仅用户授权时）

```bash
git add include/application/replay_input_accumulator.hpp src/application/replay_input_accumulator.cpp tests/application/replay_input_accumulator_test.cpp src/application/replay_pipeline.cpp include/application/replay_pipeline.hpp tests/application/replay_pipeline_test.cpp apps/synth_bag_gen.cpp cmake/Libraries.cmake cmake/Tests.cmake
git commit -m "refactor(application): feed replay through canonical event source"
```

---

## 7. Task 5：修复 HoloOcean 多速率传感器录制语义

**文件：**

- 修改：`adapters/holoocean/uw_holoocean_adapter/record_session.py`
- 修改：`adapters/holoocean/tests/test_record_session.py`
- 修改：`adapters/holoocean/README.md`

### Step 1：先写失败测试

新增四个场景：

- 只有 IMU 的 tick 仍写 `/raw/imu`；
- 只有 DVL 和声呐的 tick 分别写对应 topic；
- 只有单目相机时记录该相机，但不生成伪造的 stereo keyframe；
- 同一 tick 的不同传感器共享真实 capture time，但各自 observation id 不依赖相机 keyframe counter。

```bash
cd adapters/holoocean
.venv/bin/pytest tests/test_record_session.py -q
```

预期：前三类测试至少有一类失败，因为当前 `_write_keyframe()` 在缺少左右相机时提前返回。

### Step 2：拆分 tick 与相机对身份

将 `_write_keyframe` 改名为 `_write_sensor_tick`，返回写入消息数和是否形成 stereo pair。每个传感器 observation id 使用稳定的“sensor id + 原始 tick 序号”，不能让高频 IMU/DVL 继承低频相机关键帧 id。

`record_frames` 的成功返回值继续表示形成的 stereo frame 数，另通过测试确认无相机 tick 的消息没有被丢弃。更新模块注释，删除“只在 camera-bearing tick 写其他传感器”的表述。

### Step 3：运行 HoloOcean 测试

```bash
cd adapters/holoocean
.venv/bin/pytest -q
```

### Step 4：提交（仅用户授权时）

```bash
git add adapters/holoocean/uw_holoocean_adapter/record_session.py adapters/holoocean/tests/test_record_session.py adapters/holoocean/README.md
git commit -m "fix(holoocean): record sensors at independent rates"
```

---

## 8. Task 6：建立本实施包的回归门禁和迁移文档

**文件：**

- 新建：`tests/integration/event_source_parity_test.cpp`
- 修改：`cmake/Tests.cmake`
- 修改：`README.md`
- 修改：`docs/acoustic-optic-slam-platform-architecture-2026-08-17.md`
- 修改：`docs/uw-slam-newcomer-guide.md`

### Step 1：增加 parity 集成测试

测试在临时目录用 `McapProtobufWriter` 写入交错事件，分别通过：

1. `McapEventSource`；
2. 测试内存 `EventSource`；

送入同一个 spy `PipelineInputPort`，比较规范化后的事件摘要：

```text
source_sequence,log_time_ns,topic,protobuf_full_name,identity
```

两个摘要必须完全一致。在 `cmake/Tests.cmake` 中把该二进制注册为
`integration.event_source_parity`，label 为 `integration;replay;runtime`。该测试验证的是输入
主链等价性，不声称 live 线程调度已经完成。

### Step 2：更新架构状态

文档明确写出：

- 已完成：规范事件契约、单次 MCAP source、应用输入端口、replay 迁移；
- 尚未完成：SDK live source、有界调度、生命周期、异步 recorder、HMI bridge；
- 下一实施包入口：供应商 SDK adapter + runtime hardening；
- 禁止新代码绕过 `PipelineInputPort` 直接让算法读取 MCAP 或 vendor 消息。

### Step 3：全量验证

```bash
cmake --build build -j2
ctest --test-dir build --output-on-failure
cd adapters/holoocean
.venv/bin/pytest -q
cd ../..
python3 tools/lint/check_layer_dependencies.py .
git diff --check
```

完成判定：所有命令退出码为 0；不得只依据日志中局部的 `Passed`。

### Step 4：提交（仅用户授权时）

```bash
git add tests/integration/event_source_parity_test.cpp cmake/Tests.cmake README.md docs/acoustic-optic-slam-platform-architecture-2026-08-17.md docs/uw-slam-newcomer-guide.md
git commit -m "test: gate canonical live replay ingress parity"
```

---

## 9. 后续实施包接口（本计划不执行）

完成本计划后，按以下顺序继续；每一项单独写实施计划，不合并成一次大重构：

1. **Runtime hardening：** `BoundedQueue` 明确 push 结果，增加 close/wait/high-watermark，注入 monotonic clock，做缓冲式多速率同步和 Start/Stop/Drain 生命周期；
2. **供应商 SDK adapter：** vendor callback 只做校验、时间/坐标转换和有界入队，使用供应商真实样例建立 contract fixture；
3. **Recorder + HMI bridge：** recorder 作为统一事件/输出 tap，HMI 使用独立 presentation adapter，不进入算法层；
4. **首条任务垂直链：** 在规则和机械臂交接字段确认后再冻结 target/track/task result schema；
5. **在线定位增强：** 真实池测表明任务需要时，再引入 Ceres 后端接口、固定滞后或声光状态估计，不占第 7 周硬件闭环关键路径。

## 10. 风险与停止条件

- Task 4 的 Step 0（补全 `apps/synth_bag_gen.cpp` 的 `ImageFrame.header.observation_id`）是完成标准 #4 的硬前提，不是可选项——如果跳过它直接改 `RunReplayPipeline` 的读取阶段，`replay_demo`/`integration.replay_determinism` 会因为现有合成 bag 的相机帧被判定为空 id 输入错误而失败，不要通过放宽 `ReplayInputAccumulator` 的空 id 校验来让它"看起来通过"。
- 若供应商 SDK 不能提供原始时间戳、坐标约定或样例数据，停止 SDK adapter 开发并升级为 S1 阻塞项；不要在核心中猜测。
- 若迁移后的 replay 输出发生变化，先定位输入身份/顺序差异；不得通过放宽现有 gate 掩盖回归。
- 若 `CanonicalEvent` 开始承载 vendor 类型或算法对象，立即停止并回到分层设计审查。
- 若单次 MCAP source 需要缓存整个大文件才能工作，停止实现并改为流式消费；本计划的目的之一就是去除按 topic 重扫和无界加载。
- 若某类消息尚无真实 producer/consumer，不因为“未来可能需要”而新增 schema 字段。
