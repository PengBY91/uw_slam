// BuildOnlineAssistStatusJson 的实现：把 OperatorAssistState + 四条车道的队列健康度
// 拼成一行紧凑 JSON。文件里三个私有小函数各管一段——JsonEscape 负责转义、
// HealthReportToJson 负责通用健康度、QueueHealthToJson 负责队列专属字段。
// 为什么单独成文件、queue_health 为什么存在，见头文件。
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

// LiveEventSource::HealthReports() 会逐车道算出队列深度 / 高水位 / 丢弃数 / 拒绝数 /
// 序号缺口数，以及最老消息的滞留时长，但此前没有任何地方消费这些数字——飞手看不到
// 某条车道正在背压或丢帧，只能等数据陈旧到触发下游 staleness 才间接发现。按
// docs/archive/rov-realtime-closed-loop-code-review-2026-08-27.md 的 B2 条，这违反了
// FUS-Q-002 / FUS-RT-001 的可观测性要求。
//
// 这个函数的输出形状与 HealthReportToJson 一致，额外多出 HealthReport 那几个队列专属
// 字段（深度、水位、丢弃、延迟分位数等）。
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

// 固定顺序，与 LiveEventSource::HealthReports() 文档承诺的返回顺序一致：
// localization、correction、mapping、evidence。
constexpr std::array<const char*, 4> kQueueLaneNames{"localization", "correction", "mapping",
                                                     "evidence"};

}  // namespace

// 拼出紧凑的 JSON 状态串：目标 / 路径量、来源、置信度、数据时龄、离散引导状态、
// 每个传感器的健康度与降级原因——正好是计划 Task 4 要求的那几项，全部直接取自
// uw.domain.OperatorAssistState（schemas/proto/uw/domain/target.proto），不另造字段；
// 外加一节 queue_health（理由见 QueueHealthToJson 的注释）。
//
// 手写 JSON 而不引 nlohmann/json 之类的库，是因为这里的输出形状是固定且扁平的，
// 一个 ostringstream 就够，不值得为它给 application 层加一个第三方依赖。所有字符串
// 字段都过 JsonEscape，避免 reason_code / class_label 里的引号把 JSON 撑破。
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
