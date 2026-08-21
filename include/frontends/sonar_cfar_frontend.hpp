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

#include <cstdint>

#include "frontends/cfar_detector.hpp"
#include "measurement_api/frontend.hpp"

namespace uw::frontends {

struct SonarCfarFrontendParams {
  CfarParams cfar;
  CfarVariant cfar_variant = CfarVariant::kSOCA;  // matches upstream's actual default usage
  uint8_t detector_threshold = 0;  // additional intensity floor, mirrors imaging_sonar.py
  double dbscan_eps_m = 0.2;       // matches cluster_scanline's DBSCAN(eps=0.2, ...)
  int dbscan_min_samples = 2;      // matches cluster_scanline's DBSCAN(..., min_samples=2)
  double default_range_sigma_m = 0.05;
  double default_bearing_sigma_rad = 0.01;
};

class SonarCfarFrontend : public uw::measurement_api::SonarFrontend {
 public:
  explicit SonarCfarFrontend(SonarCfarFrontendParams params);

  uw::domain::HypothesisSet ProcessSonarFrame(const uw::domain::SonarFrame& frame) override;
  uw::domain::HealthReport Health() const override;

 private:
  SonarCfarFrontendParams params_;
  CfarDetector detector_;
  uint64_t next_evidence_id_ = 1;

  // Rolling health counters, updated by ProcessSonarFrame.
  uint64_t frames_processed_ = 0;
  uint64_t frames_rejected_ = 0;  // e.g. non-ascending azimuth_angles
  uint64_t frames_with_no_detections_ = 0;
};

}  // namespace uw::frontends
