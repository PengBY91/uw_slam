// MCAP-backed EventSource: opens `path` and traverses it exactly once,
// yielding CanonicalEvents in (log_time_ns, source_sequence) order. Does no
// image/sonar transformation, synchronization, or keyframe/identity
// derivation itself -- that belongs to application::PipelineInputPort
// implementations downstream (see ReplayInputAccumulator).
#pragma once

#include <string>

#include "runtime/event_source.hpp"

namespace uw::runtime {

class McapEventSource final : public EventSource {
 public:
  explicit McapEventSource(std::string path);

  EventSourceReport Run(const EventConsumer& consumer) override;

 private:
  std::string path_;
};

}  // namespace uw::runtime
