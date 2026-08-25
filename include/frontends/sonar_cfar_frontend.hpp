// Ported from sonar_camera_reconstruction (MIT) — see NOTICE and
// cfar_detector.hpp/dbscan.hpp for what was and wasn't ported. This class
// wires CFAR detection + first-contact extraction + DBSCAN clustering into
// the uw::measurement_api::SonarFrontend contract, producing a
// HypothesisSet<SonarRangeBearing> per platform architecture section 7.5.
//
// Deliberate deviation from upstream (imaging_sonar.py get_sonar_scanline /
// merge.py rotate_cloud): this frontend NEVER remaps to Cartesian image
// space and NEVER bakes points into a world/map frame. It stays in
// sonar-local polar coordinates (range, bearing) the whole way — matching
// architecture invariant #6/#21 ("FLS 只在可观测维度约束状态，不虚构
// elevation") and #1 ("前端拥有测量证据...不拥有另一套轨迹/地图").
#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>

#include "frontends/cfar_detector.hpp"
#include "measurement_api/frontend.hpp"

namespace uw::frontends {

struct SonarCfarHealthThresholds {
  double max_background_noise_mean = 40.0;
  double max_false_alarm_density = 0.5;
  uint64_t min_valid_measurements = 1;
  double max_processing_latency_ms = 50.0;
};

struct SonarCfarFrontendParams {
  CfarParams cfar;
  CfarVariant cfar_variant = CfarVariant::kSOCA;  // matches upstream's actual default usage
  uint8_t detector_threshold = 0;  // additional intensity floor, mirrors imaging_sonar.py
  double dbscan_eps_m = 0.2;       // matches cluster_scanline's DBSCAN(eps=0.2, ...)
  int dbscan_min_samples = 2;      // matches cluster_scanline's DBSCAN(..., min_samples=2)
  double default_range_sigma_m = 0.05;
  double default_bearing_sigma_rad = 0.01;
  SonarCfarHealthThresholds health;
  // Optional deployment expectation. When non-empty, Health() compares it
  // against a stable hash of the active detection parameters.
  std::string expected_config_hash;
};

class SonarCfarFrontend : public uw::measurement_api::SonarFrontend {
 public:
  using SteadyClockNow = std::function<std::chrono::steady_clock::time_point()>;

  explicit SonarCfarFrontend(
      SonarCfarFrontendParams params,
      SteadyClockNow now = [] { return std::chrono::steady_clock::now(); });

  uw::domain::HypothesisSet ProcessSonarFrame(const uw::domain::SonarFrame& frame) override;
  uw::domain::HealthReport Health() const override;
  const std::string& ActiveConfigHash() const { return active_config_hash_; }

 private:
  static constexpr std::size_t kProcessingLatencyWindowSize = 32;
  void FinishProcessing(std::chrono::steady_clock::time_point start);

  SonarCfarFrontendParams params_;
  CfarDetector detector_;
  SteadyClockNow now_;
  std::string active_config_hash_;
  uint64_t next_evidence_id_ = 1;

  // Rolling health counters, updated by ProcessSonarFrame.
  uint64_t frames_processed_ = 0;
  uint64_t frames_rejected_ = 0;  // e.g. non-ascending azimuth_angles
  uint64_t frames_with_no_detections_ = 0;
  double last_background_noise_mean_ = 0.0;
  double last_false_alarm_density_ = 0.0;
  uint64_t last_valid_measurement_count_ = 0;
  std::deque<double> processing_latencies_ms_;
};

}  // namespace uw::frontends
