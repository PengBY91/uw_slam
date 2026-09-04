// Portable JSON formatting for the /uw/hmi/status payload
// (holoocean_realtime_sink.cpp's RealtimeAssistOutputSink is the ROS-facing
// caller). Split out so this pure string-building logic -- previously
// anonymous-namespace-private inside holoocean_realtime_sink.cpp with zero
// test coverage -- can be unit tested directly; only depends on domain
// types, no runtime/opencv/ROS2. See docs/archive/rov-realtime-closed-loop-code-
// review-2026-08-27.md finding B2 for why the queue_health section exists:
// LiveEventSource::HealthReports() already computed per-lane backpressure/
// drop stats, but nothing surfaced them to the operator before.
#pragma once

#include <array>
#include <string>

#include "domain/domain.hpp"

namespace uw::application {

// Fixed order LiveEventSource::HealthReports() itself documents returning:
// localization, correction, mapping, evidence.
std::string BuildOnlineAssistStatusJson(const uw::domain::OperatorAssistState& state,
                                        const std::array<uw::domain::HealthReport, 4>& queue_health);

}  // namespace uw::application
