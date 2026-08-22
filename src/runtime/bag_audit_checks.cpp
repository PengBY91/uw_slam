#include "runtime/bag_audit_checks.hpp"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace uw::runtime {

int64_t StampToNanos(const uw::domain::Stamp& s) { return s.seconds() * 1'000'000'000LL + s.nanos(); }

bool StampIsZero(const uw::domain::Stamp& s) { return s.seconds() == 0 && s.nanos() == 0; }

void AccumulateHeader(HeaderStats& stats, const uw::domain::ObservationHeader& header) {
  ++stats.count;
  if (!StampIsZero(header.capture_time())) {
    stats.capture_time_ever_populated = true;
    const int64_t ns = StampToNanos(header.capture_time());
    if (ns < stats.last_capture_ns) stats.capture_time_monotonic = false;
    stats.last_capture_ns = ns;
  }
  if (!StampIsZero(header.receive_time())) {
    stats.receive_time_ever_populated = true;
    const int64_t ns = StampToNanos(header.receive_time());
    if (ns < stats.last_receive_ns) stats.receive_time_monotonic = false;
    stats.last_receive_ns = ns;
  }
  stats.clock_domains.insert(static_cast<int>(header.clock_domain()));
  if (!header.sensor_frame().value().empty()) stats.sensor_frames.insert(header.sensor_frame().value());
}

bool FrameResolves(const uw::domain::RigCalibrationSnapshot& rig, const std::string& frame) {
  if (frame == "base_link") return true;
  std::unordered_map<std::string, std::vector<std::string>> adjacency;
  for (const auto& edge : rig.frame_tree()) {
    adjacency[edge.parent_frame().value()].push_back(edge.child_frame().value());
    adjacency[edge.child_frame().value()].push_back(edge.parent_frame().value());
  }
  std::unordered_set<std::string> visited{"base_link"};
  std::vector<std::string> queue{"base_link"};
  while (!queue.empty()) {
    const std::string current = queue.back();
    queue.pop_back();
    for (const auto& next : adjacency[current]) {
      if (visited.insert(next).second) queue.push_back(next);
    }
  }
  return visited.count(frame) > 0;
}

}  // namespace uw::runtime
