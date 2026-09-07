// PumpEvents 的实现：一条 std::visit 把 CanonicalEvent 的 payload 分派到
// PipelineInputPort 对应的 On* 上。整个函数就两步——遍历分派，读完再 Flush。
#include "application/event_pump.hpp"

#include <type_traits>
#include <variant>

#include "domain/domain.hpp"

namespace uw::application {

uw::runtime::EventSourceReport PumpEvents(uw::runtime::EventSource& source, PipelineInputPort& input) {
  const auto report = source.Run([&](const uw::runtime::CanonicalEvent& event) {
    return std::visit(
        [&](const auto& payload) -> bool {
          using T = std::decay_t<decltype(payload)>;
          if constexpr (std::is_same_v<T, uw::domain::ImageFrame>) {
            return input.OnImageFrame(event);
          } else if constexpr (std::is_same_v<T, uw::domain::SonarFrame>) {
            return input.OnSonarFrame(event);
          } else if constexpr (std::is_same_v<T, uw::domain::ImuSample>) {
            return input.OnImuSample(event);
          } else if constexpr (std::is_same_v<T, uw::domain::DvlSample>) {
            return input.OnDvlSample(event);
          } else if constexpr (std::is_same_v<T, uw::domain::VehicleState>) {
            return input.OnVehicleState(event);
          } else if constexpr (std::is_same_v<T, uw::domain::KeyframeBoundary>) {
            return input.OnKeyframeBoundary(event);
          } else if constexpr (std::is_same_v<T, uw::domain::MeasurementEvidence>) {
            return input.OnMeasurementEvidence(event);
          } else if constexpr (std::is_same_v<T, uw::domain::StateSnapshot>) {
            // 目前唯一会产出 StateSnapshot 的话题就是 /gt/state，而它是
            // "只作参考、不许当算法输入"的真值话题（见 canonical_topics.hpp）。
            // 这里把所有 StateSnapshot 无条件送去 OnReferenceState，是刻意让这条
            // 保证成为结构性的：不依赖每个 PipelineInputPort 实现各自再判一次
            // topic 名——漏判一次就是真值泄漏。
            return input.OnReferenceState(event);
          } else if constexpr (std::is_same_v<T, uw::domain::HealthReport>) {
            return input.OnHealthReport(event);
          } else if constexpr (std::is_same_v<T, uw::domain::MapEvidence>) {
            return input.OnMapEvidence(event);
          }
        },
        event.payload);
  });

  if (report.status == uw::runtime::EventSourceStatus::kCompleted) {
    input.Flush();
  }
  return report;
}

}  // namespace uw::application
