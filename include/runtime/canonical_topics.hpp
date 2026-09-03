// Canonical MCAP topic strings and their event-kind/type metadata. This is
// the single place that maps a wire topic to what it means -- new
// reader/source code (McapEventSource, any future live SDK source) must
// look topics up here rather than re-hardcoding "/raw/..." strings, so a
// topic's meaning changes in exactly one place.
#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include <google/protobuf/descriptor.h>

#include "domain/domain.hpp"

namespace uw::runtime {

enum class CanonicalEventKind {
  kImageFrame,
  kSonarFrame,
  kImuSample,
  kDvlSample,
  kVehicleState,
  kMeasurementEvidence,
  kStateSnapshot,
  kHealthReport,
  kMapEvidence,
};

// kReferenceOnly topics (currently just /gt/state) carry ground truth or
// other evaluation-only data: per CLAUDE.md ("Ground truth 只能进入评测支路"),
// consumers must route these to evaluation, never to online algorithm input.
enum class CanonicalTopicRole {
  kAlgorithmInput,
  kReferenceOnly,
};

struct CanonicalTopicInfo {
  CanonicalEventKind kind;
  const google::protobuf::Descriptor* descriptor;
  CanonicalTopicRole role;
};

inline constexpr char kTopicCameraLeft[] = "/raw/camera/left";
inline constexpr char kTopicCameraRight[] = "/raw/camera/right";
// PREP-A-03: the contract vehicle's single gimbal camera (IMX462). Same
// ImageFrame payload as left/right; a bag carrying only this topic is a
// monocular recording (never a stereo keyframe source).
inline constexpr char kTopicCameraMain[] = "/raw/camera/main";
inline constexpr char kTopicSonarFrame[] = "/raw/sonar_frame";
inline constexpr char kTopicImu[] = "/raw/imu";
inline constexpr char kTopicDvl[] = "/raw/dvl";
inline constexpr char kTopicVehicleState[] = "/raw/vehicle_state";
inline constexpr char kTopicEvidenceDepth[] = "/evidence/depth";
inline constexpr char kTopicEvidenceRelativePose[] = "/evidence/relative_pose";
inline constexpr char kTopicHealth[] = "/health";
inline constexpr char kTopicEvidenceMap[] = "/evidence/map";
inline constexpr char kTopicGtState[] = "/gt/state";

namespace detail {

template <typename T>
inline CanonicalTopicInfo MakeCanonicalTopicInfo(CanonicalEventKind kind, CanonicalTopicRole role) {
  return CanonicalTopicInfo{kind, T::descriptor(), role};
}

inline const std::unordered_map<std::string, CanonicalTopicInfo>& CanonicalTopicRegistry() {
  static const std::unordered_map<std::string, CanonicalTopicInfo> registry = {
      {kTopicCameraLeft, MakeCanonicalTopicInfo<uw::domain::ImageFrame>(
                             CanonicalEventKind::kImageFrame, CanonicalTopicRole::kAlgorithmInput)},
      {kTopicCameraRight, MakeCanonicalTopicInfo<uw::domain::ImageFrame>(
                              CanonicalEventKind::kImageFrame, CanonicalTopicRole::kAlgorithmInput)},
      {kTopicCameraMain, MakeCanonicalTopicInfo<uw::domain::ImageFrame>(
                             CanonicalEventKind::kImageFrame, CanonicalTopicRole::kAlgorithmInput)},
      {kTopicSonarFrame, MakeCanonicalTopicInfo<uw::domain::SonarFrame>(
                             CanonicalEventKind::kSonarFrame, CanonicalTopicRole::kAlgorithmInput)},
      {kTopicImu, MakeCanonicalTopicInfo<uw::domain::ImuSample>(
                      CanonicalEventKind::kImuSample, CanonicalTopicRole::kAlgorithmInput)},
      {kTopicDvl, MakeCanonicalTopicInfo<uw::domain::DvlSample>(
                      CanonicalEventKind::kDvlSample, CanonicalTopicRole::kAlgorithmInput)},
      {kTopicVehicleState, MakeCanonicalTopicInfo<uw::domain::VehicleState>(
                               CanonicalEventKind::kVehicleState,
                               CanonicalTopicRole::kAlgorithmInput)},
      {kTopicEvidenceDepth, MakeCanonicalTopicInfo<uw::domain::MeasurementEvidence>(
                                CanonicalEventKind::kMeasurementEvidence, CanonicalTopicRole::kAlgorithmInput)},
      {kTopicEvidenceRelativePose,
       MakeCanonicalTopicInfo<uw::domain::MeasurementEvidence>(CanonicalEventKind::kMeasurementEvidence,
                                                                CanonicalTopicRole::kAlgorithmInput)},
      {kTopicHealth, MakeCanonicalTopicInfo<uw::domain::HealthReport>(
                         CanonicalEventKind::kHealthReport, CanonicalTopicRole::kAlgorithmInput)},
      {kTopicEvidenceMap, MakeCanonicalTopicInfo<uw::domain::MapEvidence>(
                              CanonicalEventKind::kMapEvidence, CanonicalTopicRole::kAlgorithmInput)},
      {kTopicGtState, MakeCanonicalTopicInfo<uw::domain::StateSnapshot>(
                          CanonicalEventKind::kStateSnapshot, CanonicalTopicRole::kReferenceOnly)},
  };
  return registry;
}

}  // namespace detail

// Returns metadata for `topic`, or nullptr if it is not a known canonical
// topic.
inline const CanonicalTopicInfo* LookupCanonicalTopic(const std::string& topic) {
  const auto& registry = detail::CanonicalTopicRegistry();
  const auto it = registry.find(topic);
  return it == registry.end() ? nullptr : &it->second;
}

// ---------------------------------------------------------------------------
// Control-output topics (PREP-C-02). These carry commands FROM the pilot /
// assist / autonomy layer TO the vehicle (MAVLink adapter on the real ROV,
// thrust allocation in the HoloOcean backend). They are deliberately NOT in
// CanonicalTopicRegistry and have no CanonicalEventKind: they never enter an
// algorithm lane, so a LiveEventSource cannot even be handed one (there is
// no CanonicalPayload alternative for them), and McapEventSource counts them
// separately from unknown topics when they show up in a recorded bag.
inline constexpr char kTopicCmdPilot[] = "/cmd/pilot";
inline constexpr char kTopicCmdSetpoint[] = "/cmd/setpoint";
inline constexpr char kTopicCmdMode[] = "/cmd/mode";

struct ControlTopicInfo {
  const google::protobuf::Descriptor* descriptor;
};

namespace detail {

inline const std::unordered_map<std::string, ControlTopicInfo>& ControlTopicRegistry() {
  static const std::unordered_map<std::string, ControlTopicInfo> registry = {
      {kTopicCmdPilot, ControlTopicInfo{uw::domain::PilotCommand::descriptor()}},
      {kTopicCmdSetpoint, ControlTopicInfo{uw::domain::MotionSetpoint::descriptor()}},
      {kTopicCmdMode, ControlTopicInfo{uw::domain::FlightModeRequest::descriptor()}},
  };
  return registry;
}

}  // namespace detail

// Returns metadata for a control-output `topic`, or nullptr if it is not one.
inline const ControlTopicInfo* LookupControlTopic(const std::string& topic) {
  const auto& registry = detail::ControlTopicRegistry();
  const auto it = registry.find(topic);
  return it == registry.end() ? nullptr : &it->second;
}

// Resolves `topic` to its CanonicalEventKind only if `descriptor` matches
// the Protobuf type registered for that topic. Returns std::nullopt for an
// unknown topic OR a topic/schema mismatch -- callers (e.g.
// McapEventSource's per-message validation) must treat both the same way:
// count it as an error, never construct an event from mismatched data.
inline std::optional<CanonicalEventKind> ResolveCanonicalTopic(
    const std::string& topic, const google::protobuf::Descriptor* descriptor) {
  const auto* info = LookupCanonicalTopic(topic);
  if (info == nullptr || info->descriptor != descriptor) return std::nullopt;
  return info->kind;
}

}  // namespace uw::runtime
