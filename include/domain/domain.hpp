// domain: pure C++17, no ROS / simulator / third-party vendor
// dependency (platform architecture section 5, dependency invariant #1).
//
// The wire types themselves are generated from schemas/proto/uw/domain/ (see
// cmake/UwProtobuf.cmake). This header adds a thin, type-safe C++ layer on
// top of the protobuf `oneof`-based MeasurementEvidence/HypothesisSet so
// call sites get the generic MeasurementEvidence<T> / HypothesisSet<T>
// ergonomics described in the architecture doc (section 7.4/7.5) without a
// second hand-written schema that could drift from the wire format.
#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "uw/domain/calibration.pb.h"
#include "uw/domain/dvl.pb.h"
#include "uw/domain/factor.pb.h"
#include "uw/domain/health.pb.h"
#include "uw/domain/hypothesis.pb.h"
#include "uw/domain/ids.pb.h"
#include "uw/domain/image.pb.h"
#include "uw/domain/imu.pb.h"
#include "uw/domain/map.pb.h"
#include "uw/domain/measurement.pb.h"
#include "uw/domain/observation.pb.h"
#include "uw/domain/sonar.pb.h"
#include "uw/domain/state.pb.h"
#include "uw/domain/target.pb.h"
#include "uw/domain/time.pb.h"
#include "uw/domain/vehicle.pb.h"

namespace uw::domain {

// ---- Stamp <-> std::chrono -------------------------------------------
Stamp ToStamp(std::chrono::system_clock::time_point tp);
std::chrono::system_clock::time_point ToTimePoint(const Stamp& stamp);
double ToSeconds(const Stamp& stamp);
Stamp FromSeconds(double seconds);

// ---- Camera/depth grid validation --------------------------------------
enum class ValidationCode {
  kOk = 0,
  kMissingDimensions,
  kUnsupportedImageEncoding,
  kInvalidImageStride,
  kImagePayloadSizeMismatch,
  kDepthGridSizeMismatch,
  kDepthMaskSizeMismatch,
  kContributionMaskSizeMismatch,
  kInvalidScaleStatus,
  kInvalidDepthValue,
  kInvalidVarianceValue,
  kInvalidObservationHeader,
};

struct ValidationResult {
  ValidationCode code = ValidationCode::kOk;
  std::string message;
  bool ok() const { return code == ValidationCode::kOk; }
};

ValidationResult ValidateImageFrame(const ImageFrame& frame);
ValidationResult ValidateObservationHeader(const ObservationHeader& header);
ValidationResult ValidateOpticalDepthPrior(const OpticalDepthPriorMeasurement& prior);
ValidationResult ValidateFusedDepth(const FusedDepthMeasurement& fused);

// RGB8/BGR8 -> MONO8 grayscale conversion (ITU-R BT.601 luminance weights).
// Frontends that only need grayscale (stereo_landmark_vo_frontend,
// stereo_optical_depth_frontend) hard-require MONO8; this lets a caller
// convert a color camera frame (e.g. a real HoloOcean RGB8 capture) at the
// point of consumption instead of teaching recording adapters to discard
// color at capture time. Returns `frame` unchanged if already MONO8, and
// nullopt if `frame` fails ValidateImageFrame or uses an encoding other
// than MONO8/RGB8/BGR8.
std::optional<ImageFrame> ConvertToMono8(const ImageFrame& frame);

// ---- SonarFrame validation ---------------------------------------------
// sonar_camera_reconstruction's imaging_sonar.py assumes azimuth_angles is
// ascending (interp1d(..., assume_sorted=True)) and never checks it; a
// non-ascending input silently produces a wrong remap. This platform
// enforces the check explicitly at the boundary instead (architecture
// section 22.4). Returns false (frame should be rejected, not "fixed
// silently") if not strictly ascending.
bool IsAzimuthAscending(const SonarFrame& frame);

// ---- MeasurementEvidence<T> ergonomics ---------------------------------
// PayloadTraits<T> maps a typed measurement payload (SonarRangeBearing,
// RelativePoseMeasurement, PressureDepthMeasurement, ...) to its
// MeasurementEvidence oneof accessors. Adding a new frontend's payload type
// means adding one specialization here; everything else (HypothesisSet,
// factor builders) stays generic.
template <typename T>
struct PayloadTraits;  // no generic definition on purpose: every payload
                        // type must opt in explicitly.

#define UW_DOMAIN_DEFINE_PAYLOAD_TRAITS(Type, field)                        \
  template <>                                                               \
  struct PayloadTraits<Type> {                                              \
    static bool Has(const MeasurementEvidence& e) { return e.has_##field(); } \
    static const Type& Get(const MeasurementEvidence& e) { return e.field(); } \
    static void Set(MeasurementEvidence& e, Type value) {                   \
      *e.mutable_##field() = std::move(value);                             \
    }                                                                       \
  }

UW_DOMAIN_DEFINE_PAYLOAD_TRAITS(SonarRangeBearing, sonar_range_bearing);
UW_DOMAIN_DEFINE_PAYLOAD_TRAITS(RelativePoseMeasurement, relative_pose);
UW_DOMAIN_DEFINE_PAYLOAD_TRAITS(PressureDepthMeasurement, pressure_depth);
UW_DOMAIN_DEFINE_PAYLOAD_TRAITS(VisualTrackMeasurement, visual_track);
UW_DOMAIN_DEFINE_PAYLOAD_TRAITS(StereoDepthMeasurement, stereo_depth);
UW_DOMAIN_DEFINE_PAYLOAD_TRAITS(SonarRegistrationMeasurement, sonar_registration);
UW_DOMAIN_DEFINE_PAYLOAD_TRAITS(ImuPreintegrationMeasurement, imu_preintegration);
UW_DOMAIN_DEFINE_PAYLOAD_TRAITS(OpticalDepthPriorMeasurement, optical_depth_prior);
UW_DOMAIN_DEFINE_PAYLOAD_TRAITS(FusedDepthMeasurement, fused_depth);

#undef UW_DOMAIN_DEFINE_PAYLOAD_TRAITS

template <typename T>
bool HasPayload(const MeasurementEvidence& evidence) {
  return PayloadTraits<T>::Has(evidence);
}

template <typename T>
const T& GetPayload(const MeasurementEvidence& evidence) {
  return PayloadTraits<T>::Get(evidence);
}

template <typename T>
void SetPayload(MeasurementEvidence& evidence, T value) {
  PayloadTraits<T>::Set(evidence, std::move(value));
}

// Builds a MeasurementEvidence<T> envelope. `noise_scale` is a SUGGESTION
// (architecture section 7.4/8.4): the estimator, not the frontend, decides
// the final information after applying the AI/physical/calibration/
// cross-modal caps.
template <typename T>
MeasurementEvidence MakeEvidence(EvidenceId evidence_id,
                                  std::vector<ObservationId> source_observations,
                                  T payload,
                                  double noise_scale,
                                  std::string algorithm_version) {
  MeasurementEvidence evidence;
  *evidence.mutable_evidence_id() = std::move(evidence_id);
  for (auto& obs : source_observations) {
    *evidence.add_source_observations() = std::move(obs);
  }
  evidence.set_estimated_noise_scale(noise_scale);
  evidence.set_algorithm_version(std::move(algorithm_version));
  SetPayload<T>(evidence, std::move(payload));
  return evidence;
}

// ---- HypothesisSet<T> ergonomics ---------------------------------------
// v1 algorithms may only consume the top-1 candidate (architecture section
// 7.5); this returns nullopt if there is no candidate or it is not of the
// requested type, so a caller can never silently misinterpret the payload.
template <typename T>
std::optional<T> TopCandidate(const HypothesisSet& hypothesis_set) {
  if (hypothesis_set.candidates_size() == 0) return std::nullopt;
  const MeasurementEvidence& top = hypothesis_set.candidates(0);
  if (!HasPayload<T>(top)) return std::nullopt;
  return GetPayload<T>(top);
}

}  // namespace uw::domain
