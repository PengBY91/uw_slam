# P3 Workstream D8 Decision: Surfel, Not TSDF/Occupancy

**Source:** `docs/superpowers/plans/2026-08-22-p3-online-reconstruction-and-
productionization.md`, Workstream D8 ("TSDF/surfel/occupancy backend
selection (time-boxed spike)"). This is that spike's output: a decision +
a minimal proof-of-concept, not a production implementation.

## Decision

**Surfel**, hand-rolled (no new external dependency). Implemented as
`include/mapping/surfel_map.hpp` + `src/mapping/surfel_map.cpp`, wired into
the existing `mapping` CMake target — the first non-point-cloud
`MapRepresentationType` this repo actually implements (`MAP_REPRESENTATION_
SURFEL` was, like `_TSDF` and `_OCCUPANCY`, a reserved enum value with zero
code behind it until now).

## Why surfel over TSDF and occupancy

Three things about this repo's existing architecture drove the choice, in
order of how much they mattered:

**1. `SubmapManager`'s local-frame, transform-only reintegration model is
already exactly a surfel map's natural fit, and NOT a natural fit for a
voxel grid.** Read `include/mapping/submap_manager.hpp` closely: `MapEvidence`
is kept in a keyframe's *local* frame, never baked into a global pose, and
`WorldPointsForKeyframe` re-applies the keyframe's *current* pose on every
call — deliberately, so a pose-graph correction changes what's returned
with no re-running of any frontend (see the file's own header comment,
which frames this as the counter-pattern to `sonar_camera_reconstruction`'s
`merge.py` baking points into a stale global frame). A surfel — just a
richer point (position + normal + confidence) — slots into this model with
*zero* architectural change: it's still a flat list per keyframe, still
transformed on demand, still `REINTEGRATION_POLICY_TRANSFORM_ONLY`-
compatible exactly as-is. A voxel-grid TSDF, by contrast, is normally
*globally* indexed (that's the point of a spatial grid — O(1) neighbor
lookup by coordinate) — making it local-frame-per-keyframe like this
repo's point clouds is possible (a "submap TSDF," which real systems do
use), but it is a design decision this repo hasn't made and would need a
new voxel-hashing/spatial-indexing layer to do properly. Surfel gets the
existing local-frame contract for free; TSDF would require deliberately
re-deriving it.

**2. Sonar's range/bearing-only, no-elevation evidence does not have a
well-defined sign convention, which TSDF fundamentally needs.** TSDF
fusion (Curless & Levoy 1996, and every KinectFusion-descended system
since) requires casting a ray from a known sensor origin through an
observed point and writing a signed distance *along that ray* into
voxels near the surface — the sign tells you which side of the surface
you're on. That's well-defined for a dense optical depth image (each
pixel is a known ray). It is NOT well-defined for a sparse sonar
range/bearing return with unknown elevation: this repo's own
`sonar_range_factor`'s documented v1 limitation (see `apps/replay_demo.cpp`'s
file header) already places a newly-discovered sonar landmark's z at "a
placeholder, level with the sensor" for exactly this reason — sonar alone
can't observe elevation, so there's no single unambiguous ray to sign a
voxel field against without inventing an elevation assumption that the
domain model doesn't actually support yet. A surfel, by contrast, is
honest about this: `SurfelMap::AddPoint` (position only, no normal) is a
first-class, fully-supported case — `normal_W` simply stays `Zero()` (see
`Surfel`'s doc comment) — rather than forcing every observation through a
machinery that assumes a normal/sign it doesn't have.

**3. `MapEvidence.uncertainty` already carries per-point variance, and a
surfel's confidence-weighted merge consumes it with ZERO protocol change.**
`src/mapping/acoustic_optic_map_bridge.cpp:83` already populates
`evidence.add_uncertainty(fused.variance_m2(i))` per point — this field
exists and is populated *today*, before this workstream touched anything.
`SurfelMap::AddPoint(point, confidence)` is designed so `confidence` is
exactly `1/variance_m2` (documented in `Surfel::confidence`'s comment) —
meaning a caller wiring real `MapEvidence` into a `SurfelMap` doesn't need
a new field, a new message, or a new convention; it reads a value that's
already there. This is the single strongest piece of evidence for surfel:
the domain model was already shaped for exactly this kind of
confidence-weighted fusion, just never had a consumer. It also directly
answers roadmap item 3 ("uncertainty-aware 融合") and sets up roadmap item
2's "visual-only 和 sonar-grounded 两条局部几何路径" — a
`SurfelMap::AddPoint` test in this PoC (`HigherConfidencePointDominatesMergedPosition`)
demonstrates exactly the scenario those two roadmap items describe: a
low-confidence sonar return and a high-confidence optical return of the
same physical point merge toward the confident one, not a naive 50/50
average.

**Occupancy grid** was the easiest to rule out: this platform's stated
evaluation targets (roadmap P3 acceptance: "accuracy/completeness、
Chamfer/F-score、outlier ratio") are surface-reconstruction-quality
metrics — the exact ones `include/evaluation/map_metrics.hpp` (P3
workstream A2) already computes against point/surfel-style data. Occupancy
grids are the right tool for collision-avoidance/free-space reasoning, not
for the reconstruction-quality metrics this platform has already committed
to measuring itself against; picking it now would mean building a second,
unrelated representation later anyway once reconstruction quality actually
needs measuring against something continuous.

## Hand-rolled vs. library

Considered Open3D (has surfel/point-cloud fusion utilities) and voxblib/
voxblox-style libraries (TSDF-focused, historically ROS/catkin-coupled).
Rejected both on dependency-footprint grounds, matching this repo's
existing, deliberate posture: `cmake/Dependencies.cmake` names exactly five
external dependencies (Eigen, Protobuf, yaml-cpp, GTest, MCAP via
FetchContent) and the estimator is a hand-rolled Eigen Gauss-Newton solver
specifically *not* swapped for Ceres/GTSAM yet (`CLAUDE.md`: "不要因为'手写
的不够好'就顺手去重构成别的库，除非用户明确要这么做"). A basic surfel
structure — position, normal, confidence, and a confidence-weighted merge
— is genuinely small (the entire implementation is ~60 lines); pulling in
Open3D (a multi-hundred-MB dependency with its own build system and dozens
of transitive deps) to get it would be a wildly disproportionate trade for
what this PoC needed. This is not a permanent rejection of a library for
the eventual production geometry backend — just the right call for a
time-boxed spike proving the seam works.

## What this PoC proved vs. what's still open

**Proved (with real, run verification — see below):**
- The surfel merge math (confidence-weighted position averaging, normal
  averaging + renormalization) behaves correctly on hand-computable cases.
- `SurfelMap` genuinely consumes `SubmapManager::WorldPointsForKeyframe`'s
  real output — not a synthetic stand-in — including composing a nontrivial
  keyframe pose correctly (`ConsumesSubmapManagerWorldPointsForKeyframeOutput`
  test).
- The confidence-weighted merge behaves exactly as roadmap items 2/3 will
  eventually need (`HigherConfidencePointDominatesMergedPosition`).

**Explicitly NOT done (left for D9/D10/D11, not forgotten):**
- **Scale.** `SurfelMap::AddPoint` is brute-force O(existing surfels) per
  insert (`SurfelMap::FindNearest` linearly scans). Ran the real pipeline
  (`synth_bag_gen` + `replay_demo` against `synthetic_smoke.yaml`) during
  this spike and confirmed it logs "3418897 map evidence points added" in
  a single synthetic run — brute-force merging at that scale is
  computationally infeasible (~10^12+ distance comparisons), so this PoC
  was deliberately NOT run against that real output end-to-end; doing so
  would have hung rather than proven anything. A spatial index (voxel
  hash or KD-tree) is a hard prerequisite before this can touch real
  pipeline output, not a nice-to-have optimization — this is the load-
  bearing scope boundary of this workstream, documented in
  `surfel_map.hpp`'s own header comment so nobody wires this into
  `replay_demo` before that exists.
- **Normal estimation from a point neighborhood.** `AddPoint` (no normal)
  is the common case for a lone fused point; `AddPointWithNormal` exists
  for a caller that already has one (e.g. from a locally-fit plane), but
  nothing in this PoC estimates normals from scratch. That's D9's
  "visual-only 和 sonar-grounded 两条局部几何路径" territory.
- **Confidence decay, radius growth/shrinkage, pruning of low-confidence
  or stale surfels, free-space/occlusion reasoning.** All roadmap item 3
  territory (D10), not touched here.
- **Wiring into `SubmapManager`/`apps/replay_demo` as a real
  `MAP_REPRESENTATION_SURFEL` producer/consumer.** This PoC proves the
  *math* and the *integration seam* (SubmapManager's real output really
  does flow into a `SurfelMap` correctly); it does not add
  `MAP_REPRESENTATION_SURFEL` encode/decode to `MapEvidence.
  geometry_or_occupancy` or change what `apps/replay_demo` stores. That's
  real pipeline-integration work for D9-D11, gated on the scale problem
  above being solved first.

## Verification (real, not simulated)

- `cmake --build build -j"$(nproc)"`: clean, no new warnings.
- `./build/bin/tests/mapping_tests --gtest_filter="SurfelMap.*"`: 6/6 pass
  (merge math, distant-points-stay-separate, confidence-weighted
  dominance, normal merge/renormalization, unknown-normal default, and the
  `SubmapManager` integration-seam test).
- `ctest --test-dir build --output-on-failure`: 142/142 pass (up from 136
  before this workstream — the 6 new `SurfelMap` tests; nothing else
  changed or regressed).
- `tools/lint/check_no_ros_in_core.sh`: OK.
- Ran `synth_bag_gen` + `replay_demo` against `synthetic_smoke.yaml` during
  this spike (not as a `SurfelMap` test, per the scale note above) —
  confirmed unaffected: solver still converges in 6 iterations, ATE still
  0.0665821m, matching the tracked baseline exactly. This workstream did
  not touch anything on the existing point-cloud path.
