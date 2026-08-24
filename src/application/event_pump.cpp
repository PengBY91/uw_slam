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
          } else if constexpr (std::is_same_v<T, uw::domain::MeasurementEvidence>) {
            return input.OnMeasurementEvidence(event);
          } else if constexpr (std::is_same_v<T, uw::domain::StateSnapshot>) {
            // The only current producer of a StateSnapshot event is
            // /gt/state, a reference-only topic (see canonical_topics.hpp)
            // -- routing every StateSnapshot payload to OnReferenceState
            // keeps that guarantee structural rather than relying on each
            // PipelineInputPort implementation to re-check the topic.
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
