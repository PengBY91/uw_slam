// Source-agnostic event contract: an EventSource emits CanonicalEvents to a
// consumer callback in a single pass, reporting counts for anything it could
// not translate rather than silently dropping it. McapEventSource
// (mcap_event_source.hpp) is the first implementation; a later
// implementation package adds a vendor SDK live source behind this same
// interface (see docs/superpowers/plans/2026-08-24-live-replay-unified-
// ingress.md section 1.2).
#pragma once

#include <cstdint>
#include <functional>

#include "runtime/canonical_event.hpp"

namespace uw::runtime {

enum class EventSourceStatus {
  kCompleted,
  kOpenFailed,
  kStoppedByConsumer,
};

struct EventSourceReport {
  EventSourceStatus status = EventSourceStatus::kOpenFailed;
  uint64_t messages_seen = 0;
  uint64_t events_emitted = 0;
  uint64_t unknown_topic_count = 0;
  uint64_t parse_failure_count = 0;
  // Reference-only inputs rejected before algorithm-consumer delivery.
  uint64_t reference_rejected_count = 0;
  // Control-output messages (/cmd/*, see canonical_topics.hpp) seen in a
  // recorded bag: legitimate content, never emitted as an input event, and
  // not an "unknown topic".
  uint64_t control_message_count = 0;
};

// Returning false stops the source early: Run() returns with status
// kStoppedByConsumer (not kCompleted) and messages_seen/events_emitted
// reflect only what was processed before the stop.
using EventConsumer = std::function<bool(const CanonicalEvent&)>;

class EventSource {
 public:
  virtual ~EventSource() = default;
  virtual EventSourceReport Run(const EventConsumer& consumer) = 0;
};

}  // namespace uw::runtime
