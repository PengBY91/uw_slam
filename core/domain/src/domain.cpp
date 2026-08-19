#include "uw/domain/domain.hpp"

#include <cmath>
#include <cstdint>

namespace uw::domain {

Stamp ToStamp(std::chrono::system_clock::time_point tp) {
  const auto since_epoch = tp.time_since_epoch();
  const auto secs = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
  const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - secs);
  Stamp stamp;
  stamp.set_seconds(secs.count());
  stamp.set_nanos(static_cast<int32_t>(nanos.count()));
  return stamp;
}

std::chrono::system_clock::time_point ToTimePoint(const Stamp& stamp) {
  return std::chrono::system_clock::time_point{std::chrono::seconds(stamp.seconds()) +
                                                 std::chrono::nanoseconds(stamp.nanos())};
}

double ToSeconds(const Stamp& stamp) {
  return static_cast<double>(stamp.seconds()) + static_cast<double>(stamp.nanos()) * 1e-9;
}

Stamp FromSeconds(double seconds) {
  Stamp stamp;
  const int64_t whole = static_cast<int64_t>(std::floor(seconds));
  stamp.set_seconds(whole);
  stamp.set_nanos(static_cast<int32_t>((seconds - static_cast<double>(whole)) * 1e9));
  return stamp;
}

bool IsAzimuthAscending(const SonarFrame& frame) {
  for (int i = 1; i < frame.azimuth_angles_size(); ++i) {
    if (frame.azimuth_angles(i) <= frame.azimuth_angles(i - 1)) return false;
  }
  return true;
}

}  // namespace uw::domain
