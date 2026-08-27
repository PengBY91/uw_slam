#include "application/holoocean_status_json.hpp"

#include <cstddef>
#include <iomanip>
#include <sstream>

namespace uw::application {
namespace {

std::string JsonEscape(const std::string& text) {
  std::string escaped;
  escaped.reserve(text.size());
  for (const char c : text) {
    switch (c) {
      case '"':
        escaped += "\\\"";
        break;
      case '\\':
        escaped += "\\\\";
        break;
      case '\n':
        escaped += "\\n";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          std::ostringstream oss;
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(static_cast<unsigned char>(c));
          escaped += oss.str();
        } else {
          escaped += c;
        }
    }
  }
  return escaped;
}

std::string HealthReportToJson(const uw::domain::HealthReport& health) {
  std::ostringstream oss;
  oss << "{\"component_id\":\"" << JsonEscape(health.component_id()) << "\",\"status\":"
      << static_cast<int>(health.status()) << ",\"reason_code\":\""
      << JsonEscape(health.reason_code()) << "\"}";
  return oss.str();
}

// LiveEventSource::HealthReports() computes queue depth/high-watermark/
// dropped/rejected/sequence-gap counts and oldest-message age per lane, but
// previously nothing consumed it -- an operator had no visible signal that
// a lane was backpressuring/dropping until data age eventually tripped
// downstream staleness, which per docs/rov-realtime-closed-loop-code-
// review-2026-08-27.md finding B2 violates FUS-Q-002/FUS-RT-001's
// observability requirement. This mirrors HealthReportToJson's shape but
// adds the queue-specific fields that struct doesn't carry.
std::string QueueHealthToJson(const uw::domain::HealthReport& health) {
  std::ostringstream oss;
  oss << "{\"component_id\":\"" << JsonEscape(health.component_id()) << "\",\"status\":"
      << static_cast<int>(health.status()) << ",\"queue_depth\":" << health.queue_depth()
      << ",\"queue_high_watermark\":" << health.queue_high_watermark()
      << ",\"dropped_frame_count\":" << health.dropped_frame_count()
      << ",\"rejected_frame_count\":" << health.rejected_frame_count()
      << ",\"sequence_gap_count\":" << health.sequence_gap_count()
      << ",\"oldest_message_age_ms\":" << health.oldest_message_age_ms()
      << ",\"latency_p50_ms\":" << health.latency_p50_ms()
      << ",\"latency_p95_ms\":" << health.latency_p95_ms()
      << ",\"latency_p99_ms\":" << health.latency_p99_ms() << "}";
  return oss.str();
}

// Fixed order LiveEventSource::HealthReports() itself documents returning:
// localization, correction, mapping, evidence.
constexpr std::array<const char*, 4> kQueueLaneNames{"localization", "correction", "mapping",
                                                     "evidence"};

}  // namespace

// Compact JSON status: target/path values, source, confidence, data age,
// discrete guidance state, every sensor's health and degradation reason --
// exactly the fields the plan's Task 4 text requires, drawn straight from
// uw.domain.OperatorAssistState (schemas/proto/uw/domain/target.proto) --
// plus a queue_health section (see QueueHealthToJson's doc comment).
std::string BuildOnlineAssistStatusJson(const uw::domain::OperatorAssistState& state,
                                        const std::array<uw::domain::HealthReport, 4>& queue_health) {
  std::ostringstream oss;
  oss << "{";
  oss << "\"guidance_valid\":" << (state.guidance_valid() ? "true" : "false") << ",";
  oss << "\"degradation_reason\":\"" << JsonEscape(state.degradation_reason()) << "\",";
  oss << "\"data_age_ms\":" << state.data_age_ms() << ",";
  oss << "\"system_health\":" << HealthReportToJson(state.system_health()) << ",";
  oss << "\"queue_health\":{";
  for (std::size_t i = 0; i < kQueueLaneNames.size(); ++i) {
    if (i > 0) oss << ",";
    oss << "\"" << kQueueLaneNames[i] << "\":" << QueueHealthToJson(queue_health[i]);
  }
  oss << "},";
  oss << "\"has_path_lateral_offset\":" << (state.has_path_lateral_offset() ? "true" : "false") << ",";
  oss << "\"path_lateral_offset_m\":" << state.path_lateral_offset_m() << ",";
  oss << "\"path_offset_sigma_m\":" << state.path_offset_sigma_m() << ",";
  oss << "\"sensor_health\":[";
  for (int i = 0; i < state.sensor_health_size(); ++i) {
    if (i > 0) oss << ",";
    oss << HealthReportToJson(state.sensor_health(i));
  }
  oss << "],";
  oss << "\"target_tracks\":[";
  for (int i = 0; i < state.target_tracks().tracks_size(); ++i) {
    if (i > 0) oss << ",";
    const auto& track = state.target_tracks().tracks(i);
    oss << "{\"track_id\":\"" << JsonEscape(track.track_id().value()) << "\",\"class_label\":\""
        << JsonEscape(track.class_label()) << "\",\"confidence\":" << track.class_confidence()
        << ",\"bearing_rad\":" << track.bearing_rad()
        << ",\"has_range\":" << (track.has_range_m() ? "true" : "false")
        << ",\"range_m\":" << (track.has_range_m() ? track.range_m() : 0.0)
        << ",\"status\":" << static_cast<int>(track.status()) << ",\"sources\":[";
    for (int s = 0; s < track.sources_size(); ++s) {
      if (s > 0) oss << ",";
      oss << static_cast<int>(track.sources(s));
    }
    oss << "]}";
  }
  oss << "]";
  oss << "}";
  return oss.str();
}

}  // namespace uw::application
