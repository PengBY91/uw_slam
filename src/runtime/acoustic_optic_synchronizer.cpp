#include "runtime/acoustic_optic_synchronizer.hpp"

#include <algorithm>
#include <vector>

namespace uw::runtime {

namespace {

double CorrectedTime(const uw::domain::ObservationHeader& header,
                     const uw::domain::RigCalibrationSnapshot& rig) {
  const double offset = rig.time_offset_seconds().count(header.sensor_id().value()) > 0
                            ? rig.time_offset_seconds().at(header.sensor_id().value())
                            : 0.0;
  return uw::domain::ToSeconds(header.capture_time()) + offset;
}

}  // namespace

std::optional<SynchronizedAcousticOpticBundle> SynchronizeAcousticOptic(
    const uw::domain::ImageFrame& primary, const std::optional<uw::domain::ImageFrame>& secondary,
    const uw::domain::SonarFrame& sonar, const uw::domain::RigCalibrationSnapshot& rig,
    const SynchronizerParams& params) {
  std::vector<double> times;
  times.push_back(CorrectedTime(primary.header(), rig));
  if (secondary.has_value()) times.push_back(CorrectedTime(secondary->header(), rig));
  times.push_back(CorrectedTime(sonar.header(), rig));

  const double max_time = *std::max_element(times.begin(), times.end());
  const double min_time = *std::min_element(times.begin(), times.end());
  const double delta = max_time - min_time;
  if (delta > params.max_time_delta_s) return std::nullopt;

  SynchronizedAcousticOpticBundle bundle;
  bundle.images.primary = primary;
  bundle.images.secondary = secondary;
  bundle.sonar = sonar;
  bundle.max_pairwise_time_delta_s = delta;
  return bundle;
}

}  // namespace uw::runtime
