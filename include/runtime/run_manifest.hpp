#pragma once

#include <cstdint>
#include <string>

namespace uw::runtime {

// Per platform architecture section 14.2: every run produces one immutable
// RunManifest. Dynamic parameter changes must become timestamped events,
// never a silent overwrite of an already-written manifest — callers should
// treat a RunManifest as write-once.
struct RunManifest {
  std::string run_id;
  std::string git_commit;
  std::string config_hash;
  std::string calibration_hash;
  // Hash of the OpenCV-rectified derived rig (opencv_adapters::
  // StereoRectificationContext::DerivedRig()) actually fed to stereo
  // frontends -- independent of calibration_hash (the raw rig's hash), and
  // empty (not a copy of calibration_hash) when no rectification context
  // was constructed for this run.
  std::string derived_calibration_hash;
  std::string model_hash;
  // Canonical resolved target-association/tracker parameters actually used
  // by the run. Kept separate from config_hash because experiment YAML only
  // references the defaults file rather than embedding its values.
  std::string target_fusion_parameters;
  std::string dataset_or_scenario;
  std::string simulator;
  std::string os_info;
  std::string cpu_info;
  std::string gpu_info;
  uint64_t seed = 0;
  std::string start_time_iso8601;
  std::string end_time_iso8601;  // empty until the run finishes

  // Minimal hand-rolled JSON serialization (no external JSON dependency for
  // this small, fully-controlled field set). Field values are assumed to
  // not contain unescaped quotes/control characters — true for git
  // hashes/config hashes/run ids in practice; a general-purpose escaper is
  // unnecessary complexity for this v1 struct.
  std::string ToJson() const;
};

}  // namespace uw::runtime
