# P3 Implementation Plan: Reconstruction Backend and Online Productionization

**Source:** `docs/uw-slam-production-readiness-and-roadmap-2026-08-21.md` section 7,
P3 ("重建后端与在线生产化"). That document owns the team-level roadmap and phase
acceptance criteria; per its own section 11 maintenance rule, this file owns the
concrete task breakdown for P3 specifically.

**Status of this file:** planning/sequencing only. No code has been written yet.

**Relationship to P1/P2:** the roadmap lists P3 as able to run "可与 P2 后半段
并行" (parallel with the second half of P2) — not parallel with P1, and not
fully independent of either. This plan takes that literally: it identifies
exactly which P3 work has zero P1/P2 dependency (small) versus which is
blocked on P1's real sensor data or P2's estimator/loop-closure landing
(most of it), instead of assuming "P3 can start" applies uniformly across
all eight of the roadmap's P3 bullets.

## Verified current state (read 2026-08-22, this session — file:line evidence, not the roadmap prose)

| P3 roadmap item | Current state | Evidence |
|---|---|---|
| (1) TSDF/surfel/occupancy backend | Zero implementation. `MAP_REPRESENTATION_TSDF` is a reserved enum literal nobody produces or consumes. | `schemas/proto/uw/domain/map.proto:19`; `include/mapping/submap_manager.hpp:37-39`'s own doc comment: "Other representation types return empty (not implemented in v1)" |
| (2) visual-only + sonar-grounded geometry paths | N/A — nothing to build on top of until (1) exists | — |
| (3) uncertainty-aware fusion / free-space / outlier suppression | N/A — same | — |
| (4) submap transform/reintegration + stale evidence | Partially exists for the point-cloud case: `REINTEGRATION_POLICY_TRANSFORM_ONLY`/`FULL_REFUSE` (binary, no partial/geometry-aware policy), `SubmapManager::StaleKeyframes()` correctly flags `FULL_REFUSE` evidence as stale — but **nothing in the repo calls `StaleKeyframes()` except its own test**; no regeneration is ever triggered | `schemas/proto/uw/domain/map.proto:38-40`; `include/mapping/submap_manager.hpp:35`, `src/mapping/submap_manager.cpp:18-19,32-37` |
| (5) bounded queue/lane/state machines → real scheduler | The primitives exist and are unit-tested in isolation, but have **zero non-test consumers** anywhere in the repo — no scheduler, no app constructs them | `include/runtime/bounded_queue.hpp` (`BoundedQueue<T>`, `enum class Lane` — 4 values, just an enum, no per-lane queue wiring), `include/runtime/state_machines.hpp` (`HysteresisStateMachine<T>` + 3 typedefs); only consumer found: `tests/runtime/runtime_test.cpp` |
| (6) ROS2 gateway → frontend → estimator → mapping → evaluator/recorder | Transport-only, by an explicit written invariant, not just an oversight | `adapters/ros2/include/adapters/ros2_holoocean_sonar_bridge.hpp:9-16`: "must not carry any estimation or signal-processing logic itself (section 21 invariant #4: 'ROS2 拥有传输，不拥有算法语义')"; `OnImagingSonar` only reformats and forwards to a provider — no frontend/estimation/mapping code is called anywhere under `adapters/ros2/` |
| (7) watchdog/backpressure/degradation | Nothing implemented — every grep hit is unrelated comment prose ("degrades gracefully" in a doc comment, "stochastic degradation" in scenario generation) except one design-intent comment in `bounded_queue.hpp` noting IMU "must handle failure/backpressure" — not code | grepped `watchdog\|Watchdog\|backpressure\|Backpressure\|degrad` repo-wide |
| (8) ASan/UBSan/TSan/coverage/static analysis/packaging/60min soak | None present. `.github/workflows/ci.yml` and `tools/verify_pipeline.sh` run build → ctest → pytest → lint → synth_bag_gen → replay_demo only; no CMake option exists to even opt into a sanitizer build | `.github/workflows/ci.yml` (46 lines, read in full); `tools/verify_pipeline.sh` (189 lines, read in full); grepped `cmake/*.cmake` for `SANITIZE\|ASAN\|UBSAN\|TSAN\|COVERAGE` — no hits |
| (acceptance-only) map quality metrics (Chamfer/F-score/outlier ratio/loop discontinuity) | Not implemented. Only trajectory ATE, per-pixel depth metrics, and single-value fusion accept/reject exist | `include/evaluation/{trajectory_metrics,depth_metrics,fusion_metrics}.hpp`; grepped `Chamfer\|F-score\|Fscore\|completeness\|outlier_ratio\|loop_discontinuity` — zero hits in `include/evaluation`/`src/evaluation` |
| (acceptance-only) Hz/P95 latency instrumentation | Exists in exactly one place, scoped to acoustic-optic association only — not the general pipeline | `apps/acoustic_optic_scenario_matrix.cpp` (P95 computed/printed/gated, `--max-p95-latency-ms`); `replay_demo` — the actual end-to-end batch pipeline — has no latency measurement at all |

**Cross-cutting finding:** every one of P3's 8 roadmap bullets requires either
(a) a real online/streaming data path that does not exist today (`replay_demo`
is a batch, multi-pass-over-a-bag tool, not a live scheduler loop), or (b) a
map-geometry backend with zero prior art in the repo. Only CI sanitizers/
coverage/static-analysis and offline map-quality-metric groundwork have no
P1/P2 dependency at all.

## Four tracks, by blocking dependency

**Track A — code-only, startable now, no scheduler/ROS2/TSDF dependency:**
1. CI sanitizers (ASan/UBSan) + coverage + static analysis, run against the
   existing batch test suite (not the soak test — that needs Track C)
2. Map-quality evaluation metrics (Chamfer distance, completeness, outlier
   ratio) computed against the *existing* point-cloud `SubmapManager` output
   vs. a synthetic ground-truth surface — doesn't need a TSDF backend to
   exist, since a point cloud can already be compared to a reference mesh
3. Extend P95 latency measurement from `acoustic_optic_scenario_matrix`-only
   into `replay_demo` itself (per-stage wall-clock timing) — groundwork
   before an online scheduler exists, not a substitute for real online
   latency once one does

**Track B — the scheduler itself (keystone; almost everything else in P3
hangs off this landing first). BLOCKED ON P1, not startable now — see the
2026-08-22 decision note under Workstream B4 below before picking this up:**
4. Wire `BoundedQueue`/`Lane`/the three state machines into an actual online
   scheduler loop (roadmap item 5)

**Track C — blocked on Track B, and needs this machine's local ROS2 dev
environment (colcon workspace, outside version control per `CLAUDE.md`) to
test against anything real:**
5. ROS2 gateway → frontend → estimator → mapping → evaluator/recorder
   integration (roadmap item 6)
6. Watchdog/backpressure/degradation (roadmap item 7) — needs Track B/C's
   real queues-under-load to have any actual signal to react to; building
   this against nothing produces untested code, not working degradation
7. 60-minute soak test + packaging (remainder of roadmap item 8) — needs
   something long-running (Track C.5) to soak-test

**Track D — the map geometry backend; a large standalone subsystem that
doesn't depend on Track B/C, but everything else in P3's geometry bullets
(roadmap items 1-4) depends on it landing first:**
8. TSDF/surfel/occupancy backend selection (roadmap item 1) — a **limited-
   time benchmark/spike**, per the roadmap's own risk-table entry "过早自研
   完整求解器" (the analogous risk here is over-investing in a hand-built
   geometry backend before confirming it's the right one — same mitigation
   pattern: time-box the comparison, don't let it become the backend)
9. Visual-only + sonar-grounded local geometry paths (roadmap item 2) —
   depends on D.8
10. Uncertainty-aware fusion + free-space/occlusion + outlier suppression
    (roadmap item 3) — depends on D.9
11. Extend submap transform/reintegration to the new backend + finally wire
    something to consume `StaleKeyframes()` (roadmap item 4) — depends on
    D.8; the "detect staleness" half already exists (see current-state
    table), only "act on staleness" is missing

Recommended order: **A1/A2/A3 in parallel (any time) → D8 (spike, time-boxed)
→ [P1's real sensor data pipeline lands] → B4 → D9/D10/D11 and C5 in
parallel once B4 lands → C6 → C7.** Track A does not block anything and
does not get blocked by anything; start it whenever convenient. Track B is
no longer "start whenever" — see Workstream B4's decision note.

---

## Workstream A1: CI sanitizers, coverage, static analysis — DONE (ASan+UBSan+cppcheck+gcov), TSan deliberately excluded

Implemented 2026-08-22: `UW_SANITIZER` (`OFF`/`address`/`thread`) and
`UW_COVERAGE` CMake options in `CMakeLists.txt`, applied globally (not
per-target — mixing sanitized/non-sanitized objects in one link graph fails
at link/runtime); `tools/run_quality_checks.sh` (modes `sanitizer`,
`coverage`, `static-analysis`, `all`) wraps them the same way
`tools/verify_pipeline.sh` wraps the main build so CI and local dev share
one source of truth; two new parallel CI jobs (`sanitizers`, `quality`) in
`.github/workflows/ci.yml`, independent of the existing `verify` job.

**ASan+UBSan: clean.** Full 130-test ctest suite passes with zero findings
(build+run ~190s vs. ~40s plain — acceptable for CI). This is real signal,
not an untested claim — this session actually ran it.

**TSan: excluded from CI, not just "not yet wired."** Ran it directly and
got two separate problems, both diagnosed to a confident root cause (not
guessed):
1. TSan needs ASLR disabled to even start in this sandbox
   (`setarch $(uname -m) -R`) — without it, `FATAL: ThreadSanitizer:
   unexpected memory mapping` before any test code runs.
2. Past that, TSan reports `heap-use-after-free` on `DomainContract.
   ImageFrameRoundTripsWithCanonicalHeader` and 8 other tests — all
   genuinely single-threaded test bodies. Root cause: `ldd` on any test
   binary shows `libprotobuf.so`/`libgtest.so`/`libgtest_main.so` are
   prebuilt conda-forge shared libraries, not rebuilt with
   `-fsanitize=thread` — TSan cannot see synchronization inside an
   uninstrumented library and flags its internal atomics/arena allocation
   as races. Corroborating evidence: the exact same code passes clean under
   ASan (at least as sensitive to a genuine use-after-free), and none of
   the 9 failing tests spawn a thread. `CMakeLists.txt`'s `UW_SANITIZER`
   option comment and `CLAUDE.md`'s "已经踩过的坑" both document this in
   full, so nobody re-diagnoses it from scratch. Fixing it for real needs a
   from-source protobuf+gtest rebuild under TSan — out of scope here;
   `UW_SANITIZER=thread` stays available for whoever eventually does that
   (most likely Track B's scheduler work, the first place real
   application-level threading will exist).

**cppcheck: wired, reported real (minor) findings, not gated.** 8 findings
across the codebase — 4 pass-by-value performance nits (constructor params
that should be `const&`), 3 strict-aliasing portability warnings
(`reinterpret_cast<float*>` on `signed char*` bytes in
`src/mapping/submap_manager.cpp`, `apps/replay_demo.cpp`,
`apps/synth_bag_gen.cpp` — all pre-existing binary-parsing code, not
introduced by this workstream), 1 `stlFindInsert` perf nit in
`apps/replay_demo.cpp`. Left as reported findings per this workstream's own
acceptance bar (no baseline existed to set a threshold against) — worth
someone triaging in a follow-up, not fixed here since that's cleanup outside
A1's actual scope (instrumentation, not fixing what it finds).

**Coverage: wired, real per-file numbers obtained.** No lcov/gcovr on this
toolchain (checked, neither installed nor trivially available), so
`tools/run_quality_checks.sh` uses raw `gcov` filtered (by absolute path)
to this repo's own `src/`/`include/` files, excluding protobuf-generated
code and third-party headers pulled in by inlining (Eigen headers otherwise
show up as "covered" files, which is noise, not signal). 49 files reported
with real percentages — e.g. `src/sensor_models/sonar_beam_model.cpp` at
0.00% (genuinely untested, a real gap this instrumentation was built to
surface) alongside plenty of files at 90-100%. Coverage build is
unoptimized (`-O0` + instrumentation) and noticeably slower — the
scenario-matrix integration test alone took ~310s under coverage vs. ~38s
plain, ~190s under ASan.

**Also fixed in passing (blocking, not optional):** enabling
`UW_COVERAGE=ON` changed link order enough to expose a real (if narrow)
gap in `cmake/UwProtobuf.cmake`'s explicit absl-component link list —
`absl::log_internal_check_op` was missing, causing "DSO missing from
command line" on `MakeCheckOpString`. Added it, matching the existing
pattern exactly (same list, same rationale) — documented in `CLAUDE.md`'s
existing bullet about this exact class of issue rather than a new one.
Verified the main `build/` still configures, builds, and passes 130/130
after this change (it's a shared file, not scoped to the new sanitizer/
coverage configs).

**Original scope (for reference below), superseded by the above where they
differ.**

**Goal:** Catch memory/UB/race bugs and measure coverage on the existing
batch test suite — this does not require any of P3's online infrastructure,
just a second CI configuration (or a local `tools/` script) that builds with
`-fsanitize=address,undefined` (and separately `-fsanitize=thread` — ASan
and TSan can't be combined in one build) and runs the same `ctest` suite.

**Files:**
- New: a CMake option, e.g. `UW_ENABLE_SANITIZERS` / `UW_SANITIZER` (string:
  `asan`, `ubsan`, `tsan`), added where other build options are defined —
  check `CMakeLists.txt`/`cmake/Dependencies.cmake` for the existing option
  pattern before inventing a new one
- Modify: `.github/workflows/ci.yml` — either a new job or a matrix entry
  building with the sanitizer flag and running `ctest`
- New (optional but recommended): a `tools/` script wrapping
  clang-tidy/cppcheck (whichever this environment already has via
  conda-forge — check before adding a new dependency) for static analysis,
  invoked from CI
- Coverage: `--coverage`/`gcov`/`llvm-cov` CMake flags + a CI step
  summarizing line/branch coverage; decide against gcc vs clang tooling
  based on what `tools/setup_dev_env.sh`'s conda-forge toolchain already
  provides

**Acceptance:** a CI job builds and runs the full `ctest` suite under ASan+UBSan
and separately under TSan, both passing clean on the current codebase (a
red sanitizer run against *existing* code would itself be a real finding,
worth surfacing before this workstream is called done — don't paper over it
by narrowing the enabled checks). Coverage numbers are reported, not
gated (no threshold yet — that's a judgment call for later once a baseline
number exists).

## Workstream A2: Map-quality evaluation metrics (Chamfer/completeness/outlier ratio) — DONE (metrics module), NOT WIRED (into replay_demo/synth_bag_gen)

Implemented 2026-08-22: `MapMetricsResult ComputeMapMetrics(estimated, reference,
distance_threshold_m)` in `include/evaluation/map_metrics.hpp` +
`src/evaluation/map_metrics.cpp`, matching the sibling
`trajectory_metrics`/`depth_metrics`/`fusion_metrics` pattern exactly
(same directory, same `evaluation` CMake target, same "avoid NaN on empty
input" philosophy `ComputeDepthMetrics` already established). Computes
Chamfer distance (symmetric sum of both nearest-neighbor directions),
completeness (recall), outlier ratio, and F-score (added beyond the
original scope below — the roadmap literally names "Chamfer/F-score" and
it's the harmonic mean of two quantities the module already computes, so
there was no reason to leave it for a later pass). 6 tests in
`tests/evaluation/map_metrics_test.cpp`: perfect overlap (every metric at
its ideal value), a hand-computed asymmetric case (checked against
independently-computed expected numbers, not just "doesn't crash"), a
no-overlap case, and — the design decision most worth locking in with a
test — the two different empty-input conventions (empty `estimated` →
outlier_ratio 0.0 since there's nothing to be an outlier; empty
`reference` → outlier_ratio 1.0, deliberately NOT 0.0, since nothing can
confirm an estimated point is correct). All pass; 136/136 ctest total (up
from 130).

**Explicit v1 scale limitation, load-bearing not cosmetic:** brute-force
O(|estimated|×|reference|) nearest-neighbor search, no spatial index.
Fine for the point counts this is tested against (tens of points); NOT
usable against real map output — `apps/replay_demo`'s acoustic-optic pass
routinely logs "3418897 map evidence points added" in a single run, and
brute-force against millions of points is computationally infeasible.
This is exactly why wiring below was descoped rather than attempted anyway.

**NOT wired into `apps/replay_demo`/`apps/synth_bag_gen`, and not because
it was forgotten:** the original scope note (below) already flagged this
as conditional — "once a synthetic ground-truth surface generator exists."
Checked: `apps/synth_bag_gen.cpp` has `sonar_targets_world` (a handful of
discrete 3D points used to place synthetic sonar targets), not a dense
reference surface — nowhere near enough structure to meaningfully compute
Chamfer/completeness against the millions of points the optical depth
pipeline produces. Building a real synthetic ground-truth surface
generator is its own piece of work, and doing it hastily just to have
*something* to wire this against would produce a metric nobody should
trust — worse than leaving the gap explicit. A KD-tree/octree (for scale)
and a real reference-surface generator (for something meaningful to
compare against) are both prerequisites for wiring this into the actual
pipeline; this workstream delivers the metric math, correctly and
verifiably, and stops there.

**Acceptance (metric math):** met — given known point sets with
hand-computable expected values, `ComputeMapMetrics` returns matching
Chamfer distance, completeness, outlier ratio (and F-score) within
numerical tolerance.

**Acceptance (wired into a real pipeline path):** NOT met — deliberately
out of scope, see above.

### Original scope (for reference)

**Goal:** Implement the map-quality metrics P3's own acceptance criteria
require ("accuracy/completeness、Chamfer/F-score、outlier ratio"), computed
against the point-cloud output that already exists today
(`SubmapManager`/`apps/replay_demo`'s map evidence) versus a synthetic
ground-truth surface — this does not need to wait for Track D's TSDF/surfel
backend. "Loop discontinuity" is explicitly excluded here (see below).

**Files:**
- New: `include/evaluation/map_metrics.hpp` + `src/evaluation/map_metrics.cpp`
  (matching the existing `trajectory_metrics`/`depth_metrics`/
  `fusion_metrics` sibling pattern in that directory) — Chamfer distance
  between an estimated point set and a reference surface/point set,
  completeness (fraction of reference covered within a distance threshold),
  outlier ratio (fraction of estimated points farther than a threshold from
  the reference)
- Test: `tests/evaluation/map_metrics_test.cpp` — known point sets with
  hand-computable Chamfer/completeness/outlier values, plus a
  perfect-overlap case (all metrics at their ideal value) and a
  no-overlap case (metrics at their worst value)
- Wire into `apps/synth_bag_gen`/`apps/replay_demo` or a new small eval app
  once a synthetic ground-truth surface generator exists — check whether
  `apps/synth_bag_gen.cpp`'s existing synthetic scene already has enough
  structure to derive a reference surface, or whether that needs its own
  small addition first

**Explicit non-goal:** "loop discontinuity" is NOT implemented in this
workstream — it requires loop closure to exist (P2 roadmap item, not yet
built: see P2's "实现回环、断图恢复和重定位的最小闭环"). Leave it as a
documented gap, not a stub metric that always reports zero (a metric that
can't fail yet is worse than an honestly-missing one — see the roadmap's
own section-10 risk "困难场景 gate 放空").

**Acceptance:** given two known point sets (or a point set + reference
mesh) with hand-computable expected values, `ComputeMapMetrics` (or
similar) returns matching Chamfer distance, completeness, and outlier
ratio within numerical tolerance.

## Workstream A3: P95 latency instrumentation in `replay_demo` — DONE

Implemented 2026-08-22: `apps/replay_demo.cpp`'s sonar-frame processing
loop (`/raw/sonar_frame` → `SonarCfarFrontend` → landmark association →
`SonarRangeFactorBuilder`) is timed per-frame via an RAII scope-exit
recorder (so early-`return` reject paths — warmup, unknown keyframe, no
detection — are timed too, not just full-processing frames; a reject
decision still costs real time online), collected into a
`std::vector<double>`, and reduced with the exact same P95 formula
`acoustic_optic_scenario_matrix.cpp` already uses (sorted, nearest-rank at
index `0.95*(n-1)`, no interpolation). Printed as
`sonar frame processing latency: p95_ms=...` alongside the existing
factor-count/ATE summary.

**Why this loop specifically** (not the `stereo_landmark_vo` per-keyframe
loop, and not some new unified per-keyframe wrapper): `replay_demo` is
structured as several independent multi-pass batch loops (relative-pose
pass, sonar pass, depth pass, then one global solve, then a separate
acoustic-optic pass) — there is no single "per keyframe, do everything"
loop to wrap. The sonar loop is the one genuinely per-keyframe frontend +
data-association + factor-building pass that runs in *every* experiment
config regardless of `estimator_mode` (the VO loop only runs in
`stereo_landmark_vo` mode), so it's the only candidate that gives a
comparable number across configs — matching this workstream's own
acceptance bar below.

**Not done (descoped, not forgotten):** RunManifest integration — the
original scope note below listed this as a "consider," not a requirement,
and the acceptance bar (below) only needed the number printed, not
persisted. Revisit if/when RunManifest's existing ATE/factor-count fields
get a general "we're adding an extra pipeline metric" pass — doing it once
for latency alone wasn't worth a schema change.

**Verified:** built and ran both `synthetic_smoke.yaml` (black_box_vio) and
`synthetic_smoke_vo.yaml` (stereo_landmark_vo) — both print a P95 line
(2.93ms and 4.07ms respectively — plausible orders of magnitude for CFAR +
DBSCAN + association on a synthetic scene), and both still report their
exact previously-tracked ATE (0.0666m/6 iterations, 0.0608m/7 iterations —
byte-identical to before this change). 130/130 ctest, lint clean.

**Acceptance:** met — `replay_demo` prints a P95 keyframe-processing-latency
number for both tracked synthetic experiments without changing either's
tracked ATE.

### Original scope (for reference)

**Goal:** Extend the per-stage wall-clock timing pattern
`acoustic_optic_scenario_matrix` already has to `replay_demo` itself, so
there's a P95 number for the actual end-to-end pipeline, not just the
acoustic-optic association sub-step. This is groundwork, not a substitute
for real online latency measurement — `replay_demo` is a batch tool that
processes a whole bag in a few passes, so its "latency" is a proxy
(per-keyframe processing time within those passes), not what Track C's
online scheduler will eventually need to report against the roadmap's
"capture-to-pose/map P95 ≤ 200ms" acceptance bar.

**Files:**
- Modify: `apps/replay_demo.cpp` — wrap each keyframe's frontend/factor-
  building/solve work with timing (mirror `acoustic_optic_scenario_matrix
  .cpp`'s existing pattern rather than inventing a new one), print P95/P50
  at the end alongside the existing ATE/factor-count summary
- Consider: whether this belongs in `RunManifest` as a new provenance
  field, matching how ATE/factor counts already surface there

**Acceptance:** `replay_demo` prints a P95 keyframe-processing-latency
number for both `synthetic_smoke.yaml` and `synthetic_smoke_vo.yaml`
without changing either experiment's tracked ATE (a pure instrumentation
addition, no behavior change).

## Workstream B4: Wire bounded queue/lane/state machines into a real scheduler

**2026-08-22 decision: NOT started, and not startable yet — discussed with
the user and deliberately deferred.** `BoundedQueue`'s entire design
(overflow policies, drop-oldest vs. reject) exists to answer "what happens
when a producer outpaces a consumer" — a question that only has a real
answer when a real, asynchronous producer exists. Today there is none:
the current-state table above already established the ROS2 adapter is
transport-only by design and drives nothing, so a scheduler built now
would only ever be exercised against a synthetic/fake event-injection
harness — validating against an imagined load profile, not a real one.
Compounding this: this sandbox's TSan is unreliable (see Workstream A1 —
uninstrumented prebuilt protobuf/gtest cause false positives), so writing
real multi-threaded code here also means writing it with a meaningfully
reduced ability to catch a genuine race condition. Both reasons together
match the roadmap's own risk-table pattern of "过早自研" (over-investing in
infrastructure before there's something real to validate it against — the
same shape as that table's Ceres/GTSAM-vs-hand-rolled-solver risk, applied
to the scheduler instead). **Revisit this workstream once P1's real
sensor-data pipeline (and ideally Track C.5's ROS2 integration) gives the
scheduler an actual producer to design and test against** — not on a fixed
calendar date, on that concrete landing.

**Original scope, kept for whoever picks this up once unblocked (not
re-litigated by the decision above — the "what" below is still believed
correct, only the "when" changed):**

**Goal:** The keystone P3 item. Give `BoundedQueue`/`Lane`/the three
`HysteresisStateMachine` typedefs an actual consumer — a scheduler loop
that owns per-lane queues, drains them, and drives the system-state and
modality-health state machines off real events, instead of existing only
as unit-tested-in-isolation primitives.

**Scope decision needed before starting (not resolved by this plan):**
what drives the scheduler in v1 — a synthetic/replay-fed harness (testable
in this sandbox, no ROS2 needed) or the real ROS2 gateway directly (needs
Track C.5 concurrently, and this machine's local ROS2 dev environment)?
Recommend starting with a replay-fed harness first (matches this repo's
existing pattern of proving a layer synthetically before the real-data
version — see `apps/replay_demo` itself, and P1's own Track A/B split) so
Track B doesn't silently become blocked on ROS2 environment availability.

**Files:**
- New: likely `include/runtime/scheduler.hpp` + `src/runtime/scheduler.cpp`
  (naming TBD — check for an existing "scheduler" mention in the
  architecture doc's section 8 before inventing a name) — the actual loop:
  pulls from each `Lane`'s `BoundedQueue`, dispatches to the right
  frontend/estimator/mapping call, updates state machines on
  success/failure/timeout
- New: `tests/runtime/scheduler_test.cpp` — feed synthetic events through
  bounded queues under controlled overflow conditions, assert the
  `OverflowPolicy` (kDropOldest/kDropNewest/kReject) behaves as configured
  and state machines transition correctly on hysteresis-gated events

**Acceptance:** a scheduler test drives real (not just unit-tested-in-
isolation) queue overflow and state-machine transitions from a sequence of
synthetic events, with assertions on which events were dropped (matching
the configured `OverflowPolicy`) and what state transitions resulted.

## Workstream C5: ROS2 gateway → frontend → estimator → mapping → evaluator/recorder

**Blocked on:** B4 landing (this is the first real consumer of the
scheduler), and requires this machine's local ROS2 Jazzy + colcon
workspace (`~/ros2_ws`, outside this repo's version control per
`CLAUDE.md`) to test against anything beyond a mock.

**Goal:** Drive the actual frontend/estimator/mapping/evaluator/recorder
chain from the ROS2 gateway's incoming messages via the Track B scheduler
— closing the gap the current-state table's item (6) documents (transport
only, by design, today).

**Files:** TBD — depends heavily on B4's scheduler interface shape; not
usefully specified further until B4 exists. Likely touches
`adapters/ros2/` (new nodes/wiring) without violating the "ROS2 owns
transport, not algorithm semantics" invariant — i.e. `adapters/ros2/`
should call into the scheduler, not reimplement dispatch logic itself.

**Acceptance:** TBD pending B4; at minimum, a live (or recorded-and-replayed
via `ros2 bag play`) ROS2 sonar/camera stream drives at least one full
frontend → estimator → mapping cycle through the scheduler, observable in
logs/manifest, without the ROS2 layer itself containing estimation logic.

## Workstream C6: Watchdog / backpressure / degradation

**Blocked on:** C5 (need a real pipeline under real or realistic load to
have any actual overflow/timeout/dropout signal to react to).

**Goal:** Roadmap acceptance: "camera/sonar dropout、时间延迟、packet drop
和队列过载均触发可观察降级" — observable, not just theoretically handled.

**Acceptance:** TBD pending C5; at minimum, each of the four named fault
types (camera dropout, sonar dropout, time delay, queue overflow) has a
test injecting it and asserting an observable degradation response (a
logged state transition, a manifest field, a metric) — not just "doesn't
crash."

## Workstream C7: 60-minute soak test + packaging

**Blocked on:** C5 (need something long-running to soak).

**Goal:** Roadmap acceptance: "60 分钟持续运行无崩溃、无无界内存增长，关
键队列和 dropped frame 可追踪."

**Acceptance:** TBD pending C5; at minimum, a CI (or manually-triggered,
given 60 minutes is long for standard CI budgets — check whether this
should be a scheduled job rather than per-PR) run exercises the online
path for 60 continuous minutes, tracking queue depths and dropped-frame
counts over time, with a memory-growth check (not just "didn't crash").

## Workstream D8: TSDF/surfel/occupancy backend selection (time-boxed spike) — DONE

Decided 2026-08-22: **surfel, hand-rolled** — full reasoning, alternatives
considered and rejected, and what's explicitly still open in
`docs/superpowers/plans/2026-08-22-p3-d8-geometry-backend-decision.md`.
Short version: `SubmapManager`'s existing local-frame/transform-only
reintegration model is already a surfel map's natural fit and NOT a
voxel grid's; sonar's no-elevation range/bearing evidence doesn't have a
well-defined sign convention that TSDF fundamentally needs; and
`MapEvidence.uncertainty` already carries per-point `variance_m2`
(populated today by `acoustic_optic_map_bridge.cpp`), which a surfel's
confidence-weighted merge consumes with zero protocol change.

Implemented: `include/mapping/surfel_map.hpp` + `src/mapping/surfel_map.cpp`
(`SurfelMap`/`Surfel`, ~60 lines, no new dependency), wired into the
existing `mapping` CMake target. 6 new tests in
`tests/mapping/surfel_map_test.cpp`, including one that feeds `SurfelMap`
real output from the pre-existing, unmodified `SubmapManager` (not a
synthetic stand-in) — the actual integration-seam proof this workstream
existed to produce. 142/142 ctest (up from 136), lint clean, verified via
a real `synth_bag_gen`+`replay_demo` run that the existing point-cloud
path is completely unaffected (ATE unchanged at 0.0665821m).

**Explicit scope boundary, load-bearing not cosmetic:** brute-force
O(existing surfels) nearest-neighbor search per insert. Ran the real
pipeline during this spike and confirmed `replay_demo` logs "3418897 map
evidence points added" in a single synthetic run — brute-force at that
scale is computationally infeasible, so this PoC was deliberately NOT run
against real pipeline output end-to-end (would have hung, not proven
anything). A spatial index (voxel hash/KD-tree) is a hard prerequisite
before D9-D11 wire this into `apps/replay_demo` for real, not a later
optimization — see the decision doc's "what's still open" section for the
complete list (scale, normal estimation, confidence decay/pruning, actual
`MAP_REPRESENTATION_SURFEL` wiring into `MapEvidence`).

### Original scope (for reference)

**Goal:** Pick the first real map-geometry backend, matching the roadmap's
own explicit instruction: "选择并接入...不继续扩展" pattern (used for the
solver library choice in P2) applies here too — evaluate via a time-boxed
benchmark, don't let evaluation become de facto implementation.

**Files:** none prescribed — this is a spike/evaluation, not an
implementation task. Output should be a short decision writeup (where the
P2 Ceres/GTSAM decision eventually gets written would be a reasonable
model to follow) covering: which of TSDF/surfel/occupancy was chosen, why,
what library or hand-rolled approach, and what it costs in dependencies
(this repo currently has almost no external dependencies beyond Eigen/
protobuf/yaml-cpp/MCAP — pulling in a full library like Open3D or
voxblox is a bigger dependency-surface decision than it might first
appear, worth flagging explicitly in the writeup).

**Acceptance:** a written decision + a minimal proof-of-concept (not a
full implementation) showing the chosen approach can consume this repo's
existing `MapEvidence`/point-cloud data and produce *some* geometric
output (a mesh, a voxel grid, a surfel set) — proving the integration
seam works before committing to the full roadmap items D9-D11.

## Workstreams D9/D10/D11: geometry paths, fusion, reintegration

Not specified further here — genuinely dependent on D8's outcome (a TSDF
backend, a surfel backend, and an occupancy grid backend imply different
concrete APIs for "visual-only path," "sonar-grounded path," and
"uncertainty-aware fusion"). Revisit this plan once D8 has a decision.

---

## Explicit non-goals for this plan

- Does not resolve P2 (Ceres/GTSAM selection, sliding window, IMU
  preintegration, loop closure) — several P3 acceptance criteria (loop
  discontinuity metric, reintegration triggered by real pose corrections)
  are honestly blocked on P2 landing first, not just P1.
- Does not decide Track B's scheduler design in detail (queue topology,
  dispatch policy, thread model) — that's real design work for whoever
  picks up B4, not something to pre-decide in a planning document.
- Does not commit to a TSDF vs. surfel vs. occupancy choice — that's
  D8's job, deliberately deferred to a time-boxed spike rather than
  guessed here.
- Does not attempt P4 (field validation) — out of scope per roadmap phase
  boundary.

Repository policy carried over from the roadmap doc and `CLAUDE.md`: do not
create git commits unless the user explicitly asks.
