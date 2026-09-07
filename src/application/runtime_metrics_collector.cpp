// RuntimeMetricsCollector 的实现。指标各自的含义、线程模型、以及"没测到就省略而不是
// 填 0"的原则见头文件；这里只讲代码结构和几处容易踩错的地方。
//
// ============================ 本文件主要逻辑 ============================
//
// 结构上分三段：
//
//   1. 两个 /proc 读取器（ReadProcessRssMib / ReadSystemCpuJiffies）。纯 Linux 相关，
//      任何异常都退化成 nullopt。它们是构造函数的默认实参，单测里换成假的即可。
//
//   2. 五个 Observe*/Sample* 采集入口，各自维护一小撮增量状态，全部在同一把 mutex 下：
//        ObservePublish       -> 结果时龄分位数、超期计数、恢复区间、陈旧标记正确性
//        ObserveVehicleState  -> 状态时龄分位数
//        ObserveSimTime       -> RTF（两次采样的 仿真时间增量 / 墙钟增量）
//        ObserveQueueHealth   -> 队列水位取 max、丢弃/拒绝取最新累计值、容量越界置位
//        SampleResourceUsage  -> RSS 相对基线的最大增长、CPU 余量滑动平均
//        ObserveDiagnostics   -> 直接覆盖保存管线自己的累计诊断
//
//   3. BuildReportJson()：把上面攒的状态拼成一个 JSON 对象。分位数走 RollingLatency
//      的 Snapshot()，比率类字段在这里现算。
//
// 几个刻意为之的细节：
//   - 恢复耗时用"进入 RECOVERING 记起点、离开时结算 max"的边沿方式统计，所以一轮里
//     还没结束的那段恢复不会被计入——这是保守的方向（宁可少报也不虚报）。
//   - 丢弃/拒绝数取的是各车道**最新一次**快照的值直接相加，不做累加，因为上游那些
//     计数器本身就是单调递增的累计量，再累加会重复计数。
//   - RSS 基线只在过了预热期后才建立，且建立那一次不产出增长样本。
// =======================================================================
#include "application/runtime_metrics_collector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace uw::application {

std::optional<double> ReadProcessRssMib() {
  std::ifstream status("/proc/self/status");
  if (!status.is_open()) return std::nullopt;
  std::string line;
  while (std::getline(status, line)) {
    if (line.rfind("VmRSS:", 0) != 0) continue;
    std::istringstream iss(line.substr(6));
    double kib = 0.0;
    if (!(iss >> kib)) return std::nullopt;
    return kib / 1024.0;
  }
  return std::nullopt;
}

std::optional<CpuJiffies> ReadSystemCpuJiffies() {
  std::ifstream stat("/proc/stat");
  if (!stat.is_open()) return std::nullopt;
  std::string line;
  if (!std::getline(stat, line)) return std::nullopt;
  if (line.rfind("cpu", 0) != 0) return std::nullopt;
  std::istringstream iss(line.substr(3));
  std::vector<uint64_t> fields;
  uint64_t value = 0;
  while (iss >> value) fields.push_back(value);
  // /proc/stat 的 "cpu " 行字段顺序：
  // user nice system idle iowait irq softirq steal [guest guest_nice]
  // 这里把 idle 与 iowait 一起算作"空闲"（iowait 期间 CPU 同样没在干活）。
  if (fields.size() < 4) return std::nullopt;
  CpuJiffies jiffies;
  jiffies.idle = fields[3] + (fields.size() > 4 ? fields[4] : 0);
  jiffies.total = 0;
  for (uint64_t f : fields) jiffies.total += f;
  return jiffies;
}

RuntimeMetricsCollector::RuntimeMetricsCollector(
    RuntimeMetricsConfig config, std::function<std::optional<double>()> read_rss_mib,
    std::function<std::optional<CpuJiffies>()> read_cpu_jiffies)
    : config_(config), read_rss_mib_(std::move(read_rss_mib)),
      read_cpu_jiffies_(std::move(read_cpu_jiffies)) {}

void RuntimeMetricsCollector::ObservePublish(const uw::domain::OperatorAssistState& state,
                                             double wall_now_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++published_count_;
  const double age_ms = state.data_age_ms();
  result_age_ms_.ObserveMs(std::max(0.0, age_ms));
  if (age_ms > config_.deadline_ms) ++deadline_miss_count_;

  if (age_ms > config_.stale_guidance_threshold_ms) {
    if (!state.guidance_valid()) guidance_marked_stale_when_overdue_ = true;
  }

  const bool recovering = state.system_health().status() == uw::domain::HealthReport::STATUS_RECOVERING;
  if (recovering && !currently_recovering_) {
    currently_recovering_ = true;
    recovering_since_wall_s_ = wall_now_s;
  } else if (!recovering && currently_recovering_) {
    currently_recovering_ = false;
    recovery_duration_s_max_ =
        std::max(recovery_duration_s_max_, wall_now_s - recovering_since_wall_s_);
  }
}

void RuntimeMetricsCollector::ObserveVehicleState(const uw::domain::ObservationHeader& header,
                                                  double wall_now_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double capture_s = uw::domain::ToSeconds(header.capture_time());
  const double age_s = wall_now_s - capture_s;
  state_age_ms_.ObserveMs(std::max(0.0, age_s * 1000.0));
}

void RuntimeMetricsCollector::ObserveSimTime(double capture_sim_s, double wall_now_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (last_sim_s_.has_value() && last_sim_wall_s_.has_value()) {
    const double wall_delta = wall_now_s - *last_sim_wall_s_;
    // 必须要有一段有意义的墙钟间隔才算 RTF。两次观测挤在同一瞬间（或者墙钟不单调
    // ——理论上不该发生，但真发生了也不能拿接近 0 的数去做除数）会算出一个无界的
    // 荒唐比值，把分位数彻底污染掉。
    if (wall_delta > 1e-6) {
      const double sim_delta = capture_sim_s - *last_sim_s_;
      const double rtf = sim_delta / wall_delta;
      if (std::isfinite(rtf) && rtf >= 0.0) rtf_samples_.ObserveMs(rtf);
    }
  }
  last_sim_s_ = capture_sim_s;
  last_sim_wall_s_ = wall_now_s;
}

void RuntimeMetricsCollector::ObserveQueueHealth(
    const std::array<uw::domain::HealthReport, 4>& queue_health) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t i = 0; i < queue_health.size(); ++i) {
    const auto& report = queue_health[i];
    queue_high_watermark_max_ = std::max(queue_high_watermark_max_, report.queue_high_watermark());
    queue_dropped_last_[i] = report.dropped_frame_count();
    queue_rejected_last_[i] = report.rejected_frame_count();
    const std::size_t capacity = config_.queue_lane_capacities[i];
    if (capacity > 0 && report.queue_high_watermark() >= capacity) {
      queue_capacity_violation_ = true;
    }
  }
  queue_health_observed_ = true;
}

void RuntimeMetricsCollector::SampleResourceUsage(double process_uptime_s, double warmup_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (const auto rss_mib = read_rss_mib_ ? read_rss_mib_() : std::nullopt) {
    if (process_uptime_s >= warmup_s) {
      if (!rss_baseline_mib_.has_value()) {
        rss_baseline_mib_ = *rss_mib;
      } else {
        rss_growth_observed_ = true;
        rss_growth_max_mib_ = std::max(rss_growth_max_mib_, *rss_mib - *rss_baseline_mib_);
      }
    }
  }

  if (const auto jiffies = read_cpu_jiffies_ ? read_cpu_jiffies_() : std::nullopt) {
    if (last_cpu_jiffies_.has_value()) {
      const uint64_t total_delta = jiffies->total - last_cpu_jiffies_->total;
      const uint64_t idle_delta = jiffies->idle - last_cpu_jiffies_->idle;
      if (total_delta > 0) {
        const double utilization =
            static_cast<double>(total_delta - idle_delta) / static_cast<double>(total_delta);
        const double headroom = std::max(0.0, std::min(1.0, 1.0 - utilization));
        cpu_headroom_sum_ += headroom;
        ++cpu_headroom_sample_count_;
      }
    }
    last_cpu_jiffies_ = jiffies;
  }
}

void RuntimeMetricsCollector::ObserveDiagnostics(const OnlineAssistPipelineDiagnostics& diagnostics) {
  std::lock_guard<std::mutex> lock(mutex_);
  diagnostics_ = diagnostics;
}

std::string RuntimeMetricsCollector::BuildReportJson() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream oss;
  oss << "{";

  const auto rtf = rtf_samples_.Snapshot();
  oss << "\"rtf_p50\":" << rtf.p50_ms << ",\"rtf_p95\":" << rtf.p95_ms << ",";

  const auto result_age = result_age_ms_.Snapshot();
  oss << "\"result_age_p50_ms\":" << result_age.p50_ms << ",\"result_age_p95_ms\":" << result_age.p95_ms
      << ",\"result_age_p99_ms\":" << result_age.p99_ms << ",";

  const auto state_age = state_age_ms_.Snapshot();
  oss << "\"state_age_p50_ms\":" << state_age.p50_ms << ",\"state_age_p95_ms\":" << state_age.p95_ms
      << ",\"state_age_p99_ms\":" << state_age.p99_ms << ",";

  const double deadline_miss_fraction =
      published_count_ > 0 ? static_cast<double>(deadline_miss_count_) / static_cast<double>(published_count_)
                          : 0.0;
  oss << "\"deadline_miss_fraction\":" << deadline_miss_fraction << ",";

  oss << "\"queue_high_watermark\":" << queue_high_watermark_max_ << ",";
  uint64_t total_drops = 0, total_rejects = 0;
  for (std::size_t i = 0; i < queue_dropped_last_.size(); ++i) {
    total_drops += queue_dropped_last_[i];
    total_rejects += queue_rejected_last_[i];
  }
  oss << "\"queue_drops\":" << total_drops << ",\"queue_rejects\":" << total_rejects << ",";
  oss << "\"queue_capacity_violations\":" << (queue_capacity_violation_ ? 1 : 0) << ",";

  oss << "\"recovery_duration_s_max\":" << recovery_duration_s_max_ << ",";

  if (rss_growth_observed_) {
    oss << "\"rss_growth_after_warmup_mib\":" << rss_growth_max_mib_ << ",";
  }
  if (cpu_headroom_sample_count_ > 0) {
    oss << "\"cpu_headroom_fraction_avg\":" << (cpu_headroom_sum_ / static_cast<double>(cpu_headroom_sample_count_))
        << ",";
  }
  // gpu_headroom_fraction_avg 刻意永远不输出：这个采集器没有可靠且跨平台的办法去测
  // 它（要测就得接 nvidia-smi 之类，这里没做）。省略的后果是 evaluate_gate() 会让
  // nominal/disturbed 档的运行以一条具名的 "gpu_headroom_fraction_avg: missing"
  // GateFailure 失败——这正是想要的：**宁可显式失败，也不要靠一个编出来的数字静默
  // 通过**。详见头文件的说明。

  oss << "\"sonar_detection_count\":" << diagnostics_.sonar_detection_count << ",";
  oss << "\"visual_detection_count\":" << diagnostics_.visual_detection_count << ",";
  oss << "\"fused_track_count\":" << diagnostics_.fused_track_publish_count << ",";

  oss << "\"guidance_marked_stale_when_overdue\":"
      << (guidance_marked_stale_when_overdue_ ? "true" : "false");

  oss << "}";
  return oss.str();
}

}  // namespace uw::application
