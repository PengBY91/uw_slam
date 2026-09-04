#include "runtime/bag_audit_checks.hpp"

#include <cmath>
#include <cstdio>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace uw::runtime {

int64_t StampToNanos(const uw::domain::Stamp& s) { return s.seconds() * 1'000'000'000LL + s.nanos(); }

bool StampIsZero(const uw::domain::Stamp& s) { return s.seconds() == 0 && s.nanos() == 0; }

void AccumulateHeader(HeaderStats& stats, const uw::domain::ObservationHeader& header) {
  ++stats.count;
  {
    // Every message contributes to the span and to the ordering checks,
    // including one stamped at exactly zero: in a simulation clock domain
    // t = 0 is a real instant (apps/synth_bag_gen.cpp writes the first IMU
    // sample of a stationary pre-roll there), and skipping it would report
    // the stream as starting one sample late. The all-zero Stamp is still
    // what distinguishes "populated" from "left at the proto default" --
    // that judgement is *_ever_populated's job, and it stays a whole-topic
    // property: a topic where no message ever carried a non-zero stamp is
    // the unpopulated case the audit reports.
    const int64_t ns = StampToNanos(header.capture_time());
    if (StampIsZero(header.capture_time())) {
      if (stats.capture_time_ever_populated) ++stats.unpopulated_capture_time_after_populated_count;
    } else {
      stats.capture_time_ever_populated = true;
    }
    if (stats.capture_time_count > 0) {
      if (ns < stats.last_capture_ns) stats.capture_time_monotonic = false;
      if (ns <= stats.last_capture_ns) stats.capture_time_strictly_increasing = false;
    } else {
      stats.first_capture_ns = ns;
    }
    ++stats.capture_time_count;
    stats.last_capture_ns = ns;
  }
  {
    const int64_t ns = StampToNanos(header.receive_time());
    if (StampIsZero(header.receive_time())) {
      if (stats.receive_time_ever_populated) ++stats.unpopulated_receive_time_after_populated_count;
    } else {
      stats.receive_time_ever_populated = true;
    }
    if (stats.receive_time_count > 0 && ns < stats.last_receive_ns) {
      stats.receive_time_monotonic = false;
    }
    ++stats.receive_time_count;
    stats.last_receive_ns = ns;
  }
  stats.clock_domains.insert(static_cast<int>(header.clock_domain()));
  if (!header.sensor_frame().value().empty()) stats.sensor_frames.insert(header.sensor_frame().value());
}

bool HasMixedStampPopulation(const HeaderStats& stats) {
  return stats.unpopulated_capture_time_after_populated_count > 0 ||
         stats.unpopulated_receive_time_after_populated_count > 0;
}

double MeanCaptureRateHz(const HeaderStats& stats) {
  if (stats.capture_time_count < 2) return 0.0;
  const int64_t span_ns = stats.last_capture_ns - stats.first_capture_ns;
  if (span_ns <= 0) return 0.0;
  return static_cast<double>(stats.capture_time_count - 1) * 1e9 / static_cast<double>(span_ns);
}

void AccumulateKeyframeBoundary(KeyframeBoundaryStats& stats,
                                const uw::domain::KeyframeBoundary& boundary) {
  AccumulateHeader(stats.header, boundary.header());
  const std::string& keyframe_id = boundary.keyframe_id().value();
  if (keyframe_id.empty()) {
    stats.keyframe_id_ever_empty = true;
  } else if (!stats.keyframe_ids.insert(keyframe_id).second) {
    stats.keyframe_ids_unique = false;
  }
  if (boundary.source().empty()) stats.source_ever_empty = true;
}

namespace {

bool IsWellFormedTriple(const google::protobuf::RepeatedField<double>& values) {
  if (values.size() != 3) return false;
  for (double value : values) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}

double VectorNorm(const double (&sum)[3], uint64_t count) {
  if (count == 0) return 0.0;
  const double scale = 1.0 / static_cast<double>(count);
  const double x = sum[0] * scale;
  const double y = sum[1] * scale;
  const double z = sum[2] * scale;
  return std::sqrt(x * x + y * y + z * z);
}

}  // namespace

void AccumulateImuSample(ImuWindowStats& stats, const uw::domain::ImuSample& sample) {
  if (!IsWellFormedTriple(sample.linear_acceleration_mps2()) ||
      !IsWellFormedTriple(sample.angular_velocity_radps())) {
    ++stats.malformed_sample_count;
    return;
  }
  ++stats.sample_count;
  for (int i = 0; i < 3; ++i) {
    stats.accel_sum_mps2[i] += sample.linear_acceleration_mps2(i);
    stats.gyro_sum_radps[i] += sample.angular_velocity_radps(i);
  }
}

double MeanGyroNormRadps(const ImuWindowStats& stats) {
  return VectorNorm(stats.gyro_sum_radps, stats.sample_count);
}

double MeanAccelNormMps2(const ImuWindowStats& stats) {
  return VectorNorm(stats.accel_sum_mps2, stats.sample_count);
}

uint64_t Fnv1a64Update(uint64_t hash, const std::string& bytes) {
  constexpr uint64_t kPrime = 1099511628211ULL;
  for (unsigned char byte : bytes) {
    hash ^= static_cast<uint64_t>(byte);
    hash *= kPrime;
  }
  return hash;
}

std::string ToHex64(uint64_t value) {
  char buffer[17];
  std::snprintf(buffer, sizeof(buffer), "%016llx", static_cast<unsigned long long>(value));
  return std::string(buffer);
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
