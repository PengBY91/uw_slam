// The design spec's 9-scenario matrix (section 10), each a distinct,
// honestly-described degeneration of ONE base synthetic scene: a single
// fronto-parallel textured plane at a known GT depth, visible to both
// cameras, with a sonar detection independently re-derived from the SAME
// 3D geometry via uw::sensor_models::UnprojectPixelToSonarRangeBearing (so
// "clean" scenarios are geometrically self-consistent by construction, not
// by coincidence). No photorealistic rendering — texture is a deterministic
// procedural function, degraded per scenario via block-quantization,
// additive per-pixel noise, or periodicity, all seeded from the trial's
// own RNG so results vary trial-to-trial but rerun identically for a fixed
// seed.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "uw/domain/domain.hpp"

namespace uw::scenario_matrix {

enum class ScenarioKind {
  kCleanTextured,
  kLowTextureSonarVisible,
  kTurbidSonarVisible,
  kRepeatedStructure,
  kElevationStress,
  kTimeOffsetFault,
  kExtrinsicPerturbation,
  kSonarDropout,
  kOpticalInvalidRegion,
};

struct ScenarioSpec {
  ScenarioKind kind;
  std::string name;
};

const std::vector<ScenarioSpec>& AllScenarios();

struct SyntheticTrial {
  uw::domain::ImageFrame left;
  uw::domain::ImageFrame right;
  std::optional<uw::domain::SonarFrame> sonar;  // nullopt for kSonarDropout
  uw::domain::RigCalibrationSnapshot pipeline_rig;  // fed to the frontends (may be perturbed)
  // Scene has genuine local depth structure — a target patch against a
  // different-depth background — NOT a single uniform-depth infinite
  // plane. A flat plane filling the whole image is geometrically
  // ambiguous for a wide-aperture sonar arc by construction (many arc
  // elevation samples land on equally-"consistent" optical pixels when
  // the whole image is the same depth), which is a scene-design flaw, not
  // a pipeline property worth measuring.
  double gt_target_depth_m = 0.0;
  double gt_background_depth_m = 0.0;
  int patch_center_u = 0;
  int patch_center_v = 0;
  int patch_half_size = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  // The analytically-exact sonar range/bearing this trial's target was
  // derived from (before CFAR range/beam quantization and detection),
  // exposed for diagnostics/debugging — compare against what
  // SonarCfarFrontend actually detects to separate "scene geometry is
  // wrong" from "CFAR detection/quantization drifted".
  double expected_sonar_range_m = 0.0;
  double expected_sonar_bearing_rad = 0.0;
};

// Builds one deterministic trial for `kind`, using `true_rig` (the real,
// unperturbed configs/rig/example_auv.yaml) as ground truth geometry and
// `trial_seed` for any stochastic degradation. `kExtrinsicPerturbation`
// returns a `pipeline_rig` that differs from `true_rig`; every other
// scenario returns `pipeline_rig == true_rig`.
SyntheticTrial BuildTrial(ScenarioKind kind, const uw::domain::RigCalibrationSnapshot& true_rig,
                          uint64_t trial_seed);

}  // namespace uw::scenario_matrix
