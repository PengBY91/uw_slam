#pragma once

#include <string>

#include "runtime/canonical_event.hpp"

namespace uw::runtime {

enum class CanonicalEventValidationCode {
  kOk,
  kUnknownTopic,
  kTopicPayloadMismatch,
  kHeaderInvalid,
  kKeyframeBoundaryInvalid,
  kImageInvalid,
  kSonarPayloadSizeMismatch,
  kSonarGeometryInvalid,
  kVehicleVectorSizeInvalid,
  kVehicleQuaternionInvalid,
  kVehicleValueInvalid,
};

struct CanonicalEventValidationResult {
  CanonicalEventValidationCode code = CanonicalEventValidationCode::kOk;
  std::string message;
  bool ok() const { return code == CanonicalEventValidationCode::kOk; }
};

CanonicalEventValidationResult ValidateCanonicalEvent(const CanonicalEvent& event);

}  // namespace uw::runtime
