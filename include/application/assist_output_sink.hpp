#pragma once

#include "domain/domain.hpp"

namespace uw::application {

class AssistOutputSink {
 public:
  virtual ~AssistOutputSink() = default;
  virtual void Publish(const uw::domain::OperatorAssistState& state) = 0;
};

}  // namespace uw::application
