#pragma once

#include <vector>

#include "domain/domain.hpp"

namespace uw::frontends {

class SonarTargetExtractor {
 public:
  std::vector<uw::domain::TargetDetection> Extract(
      const uw::domain::HypothesisSet& hypotheses,
      const uw::domain::SonarFrame& source_frame) const;
};

}  // namespace uw::frontends
