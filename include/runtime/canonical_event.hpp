// In-process event contract shared by every EventSource (MCAP replay today,
// a vendor SDK live source in a later implementation package) and consumed
// by application::PipelineInputPort. This is a std::variant wrapper around
// the existing schemas/proto/uw/domain/ messages -- it never serializes and
// never carries a vendor/algorithm type (see docs/superpowers/plans/
// 2026-08-24-live-replay-unified-ingress.md section 1.1/10 stop conditions).
#pragma once

#include <cstdint>
#include <string>
#include <variant>

#include "domain/domain.hpp"
#include "runtime/canonical_topics.hpp"

namespace uw::runtime {

using CanonicalPayload =
    std::variant<uw::domain::ImageFrame, uw::domain::SonarFrame, uw::domain::ImuSample,
                 uw::domain::DvlSample, uw::domain::MeasurementEvidence, uw::domain::StateSnapshot,
                 uw::domain::HealthReport, uw::domain::MapEvidence>;

struct CanonicalEvent {
  std::string topic;
  // MCAP log_time_ns for a replayed event; a live source's own monotonic
  // receive-time equivalent. This is what defines event ORDER (completion
  // standard #2) -- never re-derived from a payload's own capture_time.
  uint64_t log_time_ns = 0;
  // Stable per-source read/receive sequence, used only to break ties when
  // two events share the same log_time_ns (never as an identity key).
  uint64_t source_sequence = 0;
  CanonicalPayload payload;
};

}  // namespace uw::runtime
