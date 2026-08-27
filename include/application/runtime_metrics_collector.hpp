// Collects the runtime telemetry realtime_gate.py's evaluate_gate()
// (run_report.py) needs to render an actual pass/fail verdict --
// previously nothing in the C++ gateway populated any of this, so
// run_gate() returned a 4-field stub and evaluate_gate's first lookup
// crashed (see docs/rov-realtime-closed-loop-code-review-2026-08-27.md
// findings A2). This is a SCOPED first version: result/state age
// percentiles, deadline-miss fraction, queue backpressure stats (already
// computed by LiveEventSource per finding B2, just not aggregated over a
// run before), RTF, RSS growth, CPU headroom, recovery duration, detection/
// fused-track counts, and guidance-marked-stale-when-overdue are all
// covered. GPU headroom is NOT measured -- see gpu_headroom_fraction_avg's
// own doc comment for why, and why that is the honest choice rather than
// fabricating a number.
//
// OS-specific resource sampling (RSS, CPU jiffies) is injected via
// std::function so the rest of this class -- percentile tracking, deadline
// miss counting, JSON building -- stays fully unit-testable without a real
// /proc filesystem. ReadProcessRssMib/ReadSystemCpuJiffies (declared here,
// defined in the .cpp) are the real Linux implementations, used by
// default.
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

// Raw system-wide CPU time counters (aggregate "cpu " line of /proc/stat,
// in USER_HZ jiffies) -- utilization is only meaningful as a DELTA between
// two samples, so the reader returns cumulative counters, not a
// utilization fraction itself.
struct CpuJiffies {
  uint64_t idle = 0;
  uint64_t total = 0;
};

// Real Linux readers -- return nullopt on any read/parse failure
// (non-Linux host, /proc not mounted, unexpected format) rather than
// throwing; RuntimeMetricsCollector treats "never got a sample" as
// "field absent from the report", not zero.
std::optional<double> ReadProcessRssMib();
std::optional<CpuJiffies> ReadSystemCpuJiffies();

struct RuntimeMetricsConfig {
  // Result-age budget: FUS-RT-002's nominal target (250ms). Deliberately
  // fixed at construction, not per-profile -- realtime_gate.py passes the
  // right value in via a ROS2 parameter per profile (see
  // adapters/ros2/src/holoocean_realtime_node.cpp).
  double deadline_ms = 250.0;
  // SYS-HMI-002/FUS-TRACK-003 hard expire -- a published result older
  // than this must show guidance_valid=false, or
  // guidance_marked_stale_when_overdue can never become true.
  double stale_guidance_threshold_ms = 500.0;
  // Fixed order LiveEventSource::HealthReports() itself documents
  // returning: localization, correction, mapping, evidence. 0 = unknown
  // (never flags a capacity violation for that lane).
  std::array<std::size_t, 4> queue_lane_capacities{};
};

// Thread-safe: production use has the ROS2 callback thread calling
// ObserveSimTime/ObserveVehicleState, the pump thread calling
// ObservePublish/ObserveQueueHealth/SampleResourceUsage/ObserveDiagnostics,
// and a dedicated report-writer thread calling BuildReportJson() --
// every public method locks one internal mutex, coarse but simple to
// reason about and nowhere near a real hot path (worst case ~145 calls/
// sec at overload, matching LiveEventSource's own single-mutex precedent).
class RuntimeMetricsCollector {
 public:
  explicit RuntimeMetricsCollector(
      RuntimeMetricsConfig config,
      std::function<std::optional<double>()> read_rss_mib = ReadProcessRssMib,
      std::function<std::optional<CpuJiffies>()> read_cpu_jiffies = ReadSystemCpuJiffies);

  // Called once per ACTUAL HMI publish (RealtimeAssistOutputSink::Publish,
  // post-C1-throttle) with the wall-clock instant of that publish. Feeds
  // result_age percentiles, deadline_miss_fraction, recovery-duration
  // tracking, and guidance_marked_stale_when_overdue.
  void ObservePublish(const uw::domain::OperatorAssistState& state, double wall_now_s);

  // Called once per ingested VehicleState message with its own header and
  // the wall-clock instant it was ingested. Feeds state_age percentiles
  // independently of ObservePublish (a fused result's age and the
  // vehicle-state feed's own age are different budgets -- FUS-RT-002 sets
  // separate targets for each).
  void ObserveVehicleState(const uw::domain::ObservationHeader& header, double wall_now_s);

  // Called once per ingested message carrying a CLOCK_DOMAIN_SIMULATION
  // header (any modality -- image, sonar, or vehicle state all anchor
  // this the same way SimWallClockEstimator does). Feeds rtf_p50/p95.
  void ObserveSimTime(double capture_sim_s, double wall_now_s);

  // Called periodically (e.g. once per publish, or on a timer) with the
  // live LiveEventSource::HealthReports() snapshot. Accumulates queue
  // high-watermark (max over the run), cumulative drops/rejects (from
  // each snapshot's own monotonically-increasing counters, so this must
  // be called often enough that a lane's counter doesn't wrap -- not a
  // concern at uint64 width for any realistic run), and capacity
  // violations (a lane's watermark reaching its configured capacity).
  void ObserveQueueHealth(const std::array<uw::domain::HealthReport, 4>& queue_health);

  // Call periodically (e.g. once/sec) with the wall-clock instant and how
  // long the process has been running. Samples RSS unconditionally;
  // establishes the RSS baseline the first time process_uptime_s exceeds
  // warmup_s, and tracks the max growth above that baseline seen since.
  // Also samples CPU headroom (1 - utilization) as a running average,
  // relative to the PREVIOUS SampleResourceUsage() call, if any -- the
  // first call in a run only establishes the anchor, contributing no
  // sample yet.
  void SampleResourceUsage(double process_uptime_s, double warmup_s = 600.0);

  // diagnostics comes from OnlineAssistPipeline::Diagnostics() -- safe to
  // call repeatedly (e.g. every SampleResourceUsage tick); only the most
  // recent (already-cumulative) counts are kept.
  void ObserveDiagnostics(const OnlineAssistPipelineDiagnostics& diagnostics);

  // Builds the runtime-metrics subset of run_report.py's RunReport as a
  // JSON object -- profile/seed/task_id/duration_s are NOT this
  // collector's concern (realtime_gate.py's run_gate() already has them
  // and merges this object's fields into its own report dict). Fields
  // this collector never got a sample for are OMITTED (not zero-filled)
  // -- evaluate_gate() (see docs/rov-realtime-closed-loop-code-review-
  // 2026-08-27.md finding A2's earlier fix) already turns a missing
  // required field into a clean, named GateFailure rather than crashing,
  // which is the correct, honest outcome for a metric this collector
  // could not actually measure (see gpu_headroom_fraction_avg's doc
  // comment).
  std::string BuildReportJson() const;

 private:
  mutable std::mutex mutex_;
  RuntimeMetricsConfig config_;
  std::function<std::optional<double>()> read_rss_mib_;
  std::function<std::optional<CpuJiffies>()> read_cpu_jiffies_;

  uw::runtime::RollingLatency result_age_ms_{1024};
  uw::runtime::RollingLatency state_age_ms_{1024};
  // Not milliseconds -- RollingLatency's percentile math works on any
  // bounded, non-negative series; RTF is dimensionless (~1.0).
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
