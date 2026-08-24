// Turns an ordered CanonicalEvent stream (from any EventSource -- MCAP
// replay today, a live SDK source later) into the flat, identity-validated
// ReplayInputData that RunReplayPipeline's existing solve/map/evaluate
// logic consumes. See docs/superpowers/plans/2026-08-24-live-replay-
// unified-ingress.md Task 4: this replaces RunReplayPipeline's old
// `capture_time / 0.2s` keyframe-id derivation -- every identity here comes
// from a wire field (ObservationHeader.observation_id, MeasurementEvidence.
// source_observations), never from time.
#pragma once

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#include "application/pipeline_input_port.hpp"
#include "domain/domain.hpp"

namespace uw::application {

struct ReplayInputData {
  std::vector<uw::domain::ImageFrame> images;
  std::vector<uw::domain::SonarFrame> sonar_frames;
  std::vector<uw::domain::ImuSample> imu_samples;
  std::vector<uw::domain::DvlSample> dvl_samples;
  std::vector<uw::domain::MeasurementEvidence> evidence;
  std::vector<uw::domain::StateSnapshot> reference_states;
};

// Never silently drops a bad identity (completion standard #3): every
// rejected record increments exactly one counter here and appends a
// human-readable message, so a caller can both gate on HasErrors() and
// print specifics.
struct ReplayInputDiagnostics {
  uint64_t empty_observation_id_count = 0;
  uint64_t duplicate_observation_count = 0;
  uint64_t dangling_evidence_reference_count = 0;
  std::vector<std::string> messages;

  bool HasErrors() const {
    return empty_observation_id_count > 0 || duplicate_observation_count > 0 ||
          dangling_evidence_reference_count > 0;
  }
};

class ReplayInputAccumulator final : public PipelineInputPort {
 public:
  bool OnImageFrame(const uw::runtime::CanonicalEvent& event) override;
  bool OnSonarFrame(const uw::runtime::CanonicalEvent& event) override;
  bool OnImuSample(const uw::runtime::CanonicalEvent& event) override;
  bool OnDvlSample(const uw::runtime::CanonicalEvent& event) override;
  bool OnMeasurementEvidence(const uw::runtime::CanonicalEvent& event) override;
  bool OnReferenceState(const uw::runtime::CanonicalEvent& event) override;
  // No current producer emits /health or /evidence/map (see canonical_
  // topics.hpp); ReplayInputData deliberately has no field for either, so
  // these are accept-and-ignore until a real consumer exists (per the
  // plan's stop condition: don't grow the struct for a hypothetical need).
  bool OnHealthReport(const uw::runtime::CanonicalEvent& event) override;
  bool OnMapEvidence(const uw::runtime::CanonicalEvent& event) override;

  // Validates every accumulated evidence's source_observations against the
  // full set of raw observation identities seen this run (deferred to here,
  // rather than checked inline per-event, because an evidence record can
  // legitimately arrive before the raw observation it references in
  // log_time_ns order).
  bool Flush() override;

  const ReplayInputData& Data() const { return data_; }
  const ReplayInputDiagnostics& Diagnostics() const { return diagnostics_; }

  // Index-aligned with Data().evidence: the CanonicalEvent::log_time_ns each
  // evidence record arrived with. Exists only because MeasurementEvidence
  // (unlike ImageFrame/SonarFrame) carries no header/timestamp of its own on
  // the wire -- RunReplayPipeline uses this as a last-resort keyframe
  // timestamp when neither camera capture_time nor ground truth covers a
  // keyframe (see its capture_time_by_keyframe priority-tier comment).
  const std::vector<uint64_t>& EvidenceLogTimeNs() const { return evidence_log_time_ns_; }

 private:
  // Returns true (and records the identity as known) for a well-formed
  // observation_id; returns false (and appends a diagnostic) for an empty
  // one. A repeat of an already-seen (sensor_id, observation_id) pair is
  // rejected UNLESS allow_duplicate_identity is set -- SonarFrame passes
  // true because synth_bag_gen/HoloOcean's sonar model legitimately emits
  // more than one ping (one per in-range target) under the same physical
  // observation identity (see replay_pipeline.cpp's documented v1 "keep the
  // first sonar frame seen per keyframe" rule); ImageFrame/ImuSample/
  // DvlSample pass false because a repeat there is a genuine data-integrity
  // problem, not an accepted modeling choice.
  bool ValidateRawIdentity(const std::string& sensor_id, const std::string& observation_id,
                           const std::string& kind_label, bool allow_duplicate_identity);

  ReplayInputData data_;
  ReplayInputDiagnostics diagnostics_;
  std::unordered_set<std::string> seen_raw_identities_;    // "sensor_id\x1Fobservation_id"
  std::unordered_set<std::string> known_observation_ids_;  // union across all raw kinds
  std::vector<uint64_t> evidence_log_time_ns_;              // parallel to data_.evidence
};

}  // namespace uw::application
