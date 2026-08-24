// Dispatches an EventSource's CanonicalEvents to a PipelineInputPort. This
// is the only place that translates "which CanonicalPayload alternative is
// active" into "which PipelineInputPort method to call" -- callers never do
// that switch themselves.
#pragma once

#include "application/pipeline_input_port.hpp"
#include "runtime/event_source.hpp"

namespace uw::application {

uw::runtime::EventSourceReport PumpEvents(uw::runtime::EventSource& source, PipelineInputPort& input);

}  // namespace uw::application
