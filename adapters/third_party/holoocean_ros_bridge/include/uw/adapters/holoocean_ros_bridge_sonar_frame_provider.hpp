// SonarFrameProvider wrapper for HoloOcean's official ROS 2 interface
// (byu-holoocean/holoocean-ros, external_repos/holoocean-ros/), specifically
// its `holoocean_interfaces/msg/ImagingSonar` topic (published by
// holoocean_main's node when a RaycastImagingSonar/GPUImagingSonar sensor is
// configured on the agent).
//
// KNOWN LIMITATION (production wiring): this class does NOT itself
// subscribe to a ROS2 topic — mirroring adapters/third_party/svin_bridge,
// the conversion logic lives here (so it is unit-testable without rclcpp)
// and PushImagingSonar() is the explicit injection point. The actual ROS2
// subscription — unpacking holoocean_interfaces::msg::ImagingSonar and
// calling this — is isolated to adapters/ros2 (built only when
// UW_BUILD_ROS2=ON with holoocean_interfaces on CMAKE_PREFIX_PATH).
//
// Field mapping (see external_repos/holoocean-ros/holoocean_interfaces/msg/ImagingSonar.msg
// and the reference conversion this mirrors, external_repos/holoocean_bridge/
// holoocean_bridge/sonar_adapter_node.py, which targets sonar_oculus/OculusPing
// instead of this platform's native SonarFrame but resolves the same two
// non-obvious points below against real HoloOcean output):
//   - ImagingSonar.image_range is the flattened [num_ranges, num_beams]
//     row-major intensity image, float32 in [0, 1] (NOT image_azimuth,
//     which the reference adapter leaves unused).
//   - HoloOcean's column order runs opposite this platform's ascending-
//     bearing convention (domain.hpp's IsAzimuthAscending), so each row is
//     mirrored (reversed) when built into SonarFrame — same correction the
//     reference adapter applies via np.fliplr, empirically verified against
//     real sensor output there.
// ImagingSonar carries no per-beam angle array or range calibration, so
// horizontal FOV and min/max range come from HoloOceanImagingSonarParams
// (populated from the same scenario JSON field the reference adapter reads:
// agents[0].sensors[].configuration.{Azimuth,RangeMin,RangeMax}), not the
// message itself.
#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "uw/measurement_api/providers.hpp"

namespace uw::adapters {

struct HoloOceanImagingSonarParams {
  float horizontal_fov_rad = 0.0f;
  float min_range_m = 0.0f;
  float max_range_m = 0.0f;
  std::string sensor_id = "holoocean_imaging_sonar";
  std::string sensor_frame = "sonar_link";
};

class HoloOceanRosBridgeSonarFrameProvider : public uw::measurement_api::SonarFrameProvider {
 public:
  explicit HoloOceanRosBridgeSonarFrameProvider(HoloOceanImagingSonarParams params);

  // timestamp_ns is HoloOcean sim time (ImagingSonar.timestamp), image_range
  // is the flattened row-major [num_ranges, num_beams] array described
  // above. Silently drops the frame (no push) if image_range.size() !=
  // num_ranges * num_beams or either dimension is zero — a malformed frame
  // must not enter the typed pipeline (mirrors ObservationHeader::Validity's
  // intent at this boundary).
  void PushImagingSonar(int64_t timestamp_ns, uint32_t num_ranges, uint32_t num_beams,
                         const std::vector<float>& image_range, std::string observation_id);

  std::optional<uw::domain::SonarFrame> PollSonarFrame() override;
  uw::domain::HealthReport Health() const override;

 private:
  HoloOceanImagingSonarParams params_;
  mutable std::mutex mutex_;
  std::deque<uw::domain::SonarFrame> pending_;
  uint64_t total_pushed_ = 0;
  uint64_t total_dropped_ = 0;
  uint64_t total_polled_ = 0;
};

}  // namespace uw::adapters
