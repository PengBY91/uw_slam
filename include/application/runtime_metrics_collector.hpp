// 采集运行期遥测数据，供 realtime_gate.py 的 evaluate_gate()（run_report.py）判出
// 真正的 pass/fail 结论。
//
// 由来：此前 C++ 网关这边根本没人填这些字段，导致 run_gate() 只能返回一个 4 字段的
// 桩，evaluate_gate 第一次查表就崩（见 docs/archive/rov-realtime-closed-loop-code-
// review-2026-08-27.md 的 A2 条）。
//
// 这是**有意划定范围**的第一版，已覆盖：结果/状态时龄分位数、超期比例、队列背压统计
// （LiveEventSource 早就算好了，见 B2 条，只是此前没人按整轮聚合）、RTF、RSS 增长、
// CPU 余量、恢复耗时、检测/融合航迹计数，以及"超期时引导是否已被标记为陈旧"。
//
// **GPU 余量没有采集**——理由见 gpu_headroom_fraction_avg 的注释：这里宁可让字段缺席
// 也不编一个数出来，因为缺席会被 gate 判成一条有名有姓的失败，而编造的数字会静默地
// 让门禁变绿。
//
// 与操作系统绑定的资源采样（RSS、CPU jiffies）通过 std::function 注入，这样类里其余
// 部分——分位数统计、超期计数、JSON 拼装——不需要真实的 /proc 就能完整单测。
// ReadProcessRssMib / ReadSystemCpuJiffies（在这里声明、.cpp 里定义）是真实的 Linux
// 实现，作为默认值使用。
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include "application/online_assist_pipeline.hpp"
#include "domain/domain.hpp"
#include "runtime/rolling_latency.hpp"

namespace uw::application {

// 全系统的原始 CPU 时间计数（取 /proc/stat 汇总那行 "cpu "，单位是 USER_HZ jiffies）。
// 注意读取器返回的是**累计计数**而不是利用率：利用率只有在两次采样之差上才有意义，
// 单次快照算不出来。
struct CpuJiffies {
  uint64_t idle = 0;
  uint64_t total = 0;
};

// 真实的 Linux 读取实现。任何读取/解析失败（非 Linux 主机、/proc 没挂载、格式不符）
// 都返回 nullopt 而不是抛异常；RuntimeMetricsCollector 会把"一次样本都没拿到"处理成
// "报告里没有这个字段"，而**不是填 0**——填 0 会让门禁误判成"测过且合格"。
std::optional<double> ReadProcessRssMib();
std::optional<CpuJiffies> ReadSystemCpuJiffies();

struct RuntimeMetricsConfig {
  // 结果时龄预算：FUS-RT-002 的标称目标（250ms）。刻意在构造时定死、不在类内按
  // profile 分支——各档 profile 该用什么值由 realtime_gate.py 通过 ROS2 参数传进来
  // （见 adapters/ros2/src/holoocean_realtime_node.cpp）。
  double deadline_ms = 250.0;
  // SYS-HMI-002 / FUS-TRACK-003 规定的硬过期线：发布出去的结果一旦比这个还老，
  // 就必须显示 guidance_valid=false。否则 guidance_marked_stale_when_overdue 永远
  // 不可能为真——这个字段考的就是"该标陈旧的时候有没有真的标"。
  double stale_guidance_threshold_ms = 500.0;
  // 固定顺序，与 LiveEventSource::HealthReports() 文档承诺的返回顺序一致：
  // localization、correction、mapping、evidence。填 0 表示容量未知，该车道永远不会
  // 被判为容量越界。
  std::array<std::size_t, 4> queue_lane_capacities{};
};

// 线程安全。实际部署里有三类线程同时打交道：ROS2 回调线程调
// ObserveSimTime/ObserveVehicleState，泵线程调
// ObservePublish/ObserveQueueHealth/SampleResourceUsage/ObserveDiagnostics，
// 还有一个专门写报告的线程调 BuildReportJson()。
//
// 实现上每个 public 方法都锁同一把内部 mutex。粒度粗，但好推理，而且这里离热点路径
// 很远（overload 档最坏也就 ~145 次调用/秒），与 LiveEventSource 自己那把单 mutex
// 的做法保持一致。
class RuntimeMetricsCollector {
 public:
  explicit RuntimeMetricsCollector(
      RuntimeMetricsConfig config,
      std::function<std::optional<double>()> read_rss_mib = ReadProcessRssMib,
      std::function<std::optional<CpuJiffies>()> read_cpu_jiffies = ReadSystemCpuJiffies);

  // 每次**真正发生**的 HMI 发布调一次（即 RealtimeAssistOutputSink::Publish 中
  // 经过 C1 限流之后那一刻），传入该次发布的墙钟时刻。注意是"真发布"而不是"想发布"，
  // 被限流掉的不算。
  // 它喂给四项指标：result_age 分位数、deadline_miss_fraction、恢复耗时统计，以及
  // guidance_marked_stale_when_overdue。
  void ObservePublish(const uw::domain::OperatorAssistState& state, double wall_now_s);

  // 每收进一条 VehicleState 调一次，传它自己的 header 和收进来的墙钟时刻。
  // 它独立于 ObservePublish 喂 state_age 分位数：融合结果的时龄和车辆状态流本身的
  // 时龄是两笔不同的预算，FUS-RT-002 对二者分别设了目标，不能混为一谈。
  void ObserveVehicleState(const uw::domain::ObservationHeader& header, double wall_now_s);

  // 每收进一条带 CLOCK_DOMAIN_SIMULATION 头的消息调一次（不分模态——图像、声呐、
  // 车辆状态都一样锚定，做法与 SimWallClockEstimator 相同）。喂 rtf_p50/p95。
  void ObserveSimTime(double capture_sim_s, double wall_now_s);

  // 周期性调用（比如每次发布一次，或挂个定时器），传入 LiveEventSource::
  // HealthReports() 的实时快照。它累积三样东西：队列高水位（整轮取 max）、累计
  // 丢弃/拒绝数（来自各快照自身单调递增的计数器，所以调用频率要保证计数器不回绕
  // ——uint64 宽度下任何现实规模的运行都不用担心），以及容量越界（某车道水位触到
  // 了它配置的容量）。
  void ObserveQueueHealth(const std::array<uw::domain::HealthReport, 4>& queue_health);

  // 周期性调用（比如每秒一次），传入进程已运行时长。
  //
  // RSS 每次都采；但**基线**是在 process_uptime_s 第一次超过 warmup_s 时才建立的
  // ——预热期的内存增长是正常的加载行为，算进泄漏指标会误报。基线建立之后，跟踪
  // 相对它的最大增长量。
  //
  // 同时以滑动平均采 CPU 余量（1 - 利用率），算的是与**上一次** SampleResourceUsage()
  // 之间的差值。所以一轮里的第一次调用只负责建锚点，本身不贡献样本。
  void SampleResourceUsage(double process_uptime_s, double warmup_s = 600.0);

  // diagnostics 来自 OnlineAssistPipeline::Diagnostics()。可以放心反复调用（比如
  // 跟着每个 SampleResourceUsage tick 调），因为这里只保留最新一份——那些计数本身
  // 就已经是累计值，不需要在这里再累加（重复累加反而会翻倍）。
  void ObserveDiagnostics(const OnlineAssistPipelineDiagnostics& diagnostics);

  // 把 run_report.py 里 RunReport 的"运行期指标"那一部分拼成 JSON 对象返回。
  // profile / seed / task_id / duration_s **不归这个采集器管**——realtime_gate.py 的
  // run_gate() 本来就有这些值，它会把这里的字段并进自己的报告字典。
  //
  // 关键约定：一次样本都没拿到的字段会被**整个省略**，而不是填 0。evaluate_gate()
  // 遇到必需字段缺失会干净地报出一条具名 GateFailure（见 docs/archive/rov-realtime-
  // closed-loop-code-review-2026-08-27.md A2 条更早的那次修复），这对于一个确实没测到
  // 的指标才是诚实的结果——填 0 等于谎报"测过了且是 0"。
  std::string BuildReportJson() const;

 private:
  mutable std::mutex mutex_;
  RuntimeMetricsConfig config_;
  std::function<std::optional<double>()> read_rss_mib_;
  std::function<std::optional<CpuJiffies>()> read_cpu_jiffies_;

  uw::runtime::RollingLatency result_age_ms_{1024};
  uw::runtime::RollingLatency state_age_ms_{1024};
  // 这个不是毫秒。借用 RollingLatency 只是因为它的分位数算法对任何有界非负序列都
  // 成立；RTF 是量纲为一的比值（正常约等于 1.0）。
  uw::runtime::RollingLatency rtf_samples_{256};

  uint64_t published_count_ = 0;
  uint64_t deadline_miss_count_ = 0;
  bool guidance_marked_stale_when_overdue_ = false;

  bool currently_recovering_ = false;
  double recovering_since_wall_s_ = 0.0;
  double recovery_duration_s_max_ = 0.0;

  std::optional<double> last_sim_s_;
  std::optional<double> last_sim_wall_s_;

  uint32_t queue_high_watermark_max_ = 0;
  std::array<uint64_t, 4> queue_dropped_last_{};
  std::array<uint64_t, 4> queue_rejected_last_{};
  bool queue_health_observed_ = false;
  bool queue_capacity_violation_ = false;

  std::optional<double> rss_baseline_mib_;
  double rss_growth_max_mib_ = 0.0;
  bool rss_growth_observed_ = false;

  std::optional<CpuJiffies> last_cpu_jiffies_;
  double cpu_headroom_sum_ = 0.0;
  uint64_t cpu_headroom_sample_count_ = 0;

  OnlineAssistPipelineDiagnostics diagnostics_;
};

}  // namespace uw::application
