#pragma once

#include <cstdint>
#include <string>

#include "domain/domain.hpp"

namespace uw::runtime {

struct SyntheticSonarFrameSpec {
  uint32_t num_ranges = 600;
  uint32_t num_beams = 300;
  double min_range_m = 0.0;
  double max_range_m = 15.0;
  double horizontal_fov_rad = 6.0;
  uint8_t background_intensity = 5;
  uint8_t target_intensity = 200;
  int target_half_width_beams = 1;
  std::string observation_id;
  std::string sensor_id;
  std::string sensor_frame = "sonar_link";
  std::string provenance;
  uint64_t timestamp_ns = 0;
};

struct SyntheticSonarFrameResult {
  uw::domain::SonarFrame frame;
  bool target_rendered = false;
};

SyntheticSonarFrameResult RenderSyntheticSonarFrame(
    const SyntheticSonarFrameSpec& spec, double target_range_m,
    double target_bearing_rad);

}  // namespace uw::runtime
