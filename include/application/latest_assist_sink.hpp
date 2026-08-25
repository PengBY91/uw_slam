// Replace-latest AssistOutputSink for the online operator-assist pipeline.
// Publish always overwrites the stored state rather than queueing --
// consumers (an overlay renderer, a future ROS2 publisher) only ever care
// about the most recent guidance, never a backlog of stale ones.
#pragma once

#include <mutex>
#include <optional>

#include "application/assist_output_sink.hpp"
#include "domain/domain.hpp"

namespace uw::application {

class LatestAssistSink final : public AssistOutputSink {
 public:
  void Publish(const uw::domain::OperatorAssistState& state) override;
  std::optional<uw::domain::OperatorAssistState> Latest() const;

 private:
  mutable std::mutex mutex_;
  std::optional<uw::domain::OperatorAssistState> latest_;
};

}  // namespace uw::application
