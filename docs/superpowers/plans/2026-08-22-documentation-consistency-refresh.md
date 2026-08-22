# Documentation Consistency Refresh Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Do not dispatch subagents for this repository session.

**Goal:** Bring `README.md`, `CLAUDE.md`, and the user-facing documents directly under `docs/` into agreement with the 2026-08-22 working tree while preserving dated design history.

**Architecture:** Treat source, schema, tests, executable behavior, and fresh verification output as the facts layer. Update concise entry documents first, then current-reference documents, then add narrowly scoped status deltas to historical designs. Finish with a repository-wide stale-claim and link audit. Work in the existing tree because the facts being documented include user-owned uncommitted P1 changes that an isolated worktree would omit.

**Tech Stack:** Markdown, C++17/CMake/CTest, Python/pytest, Protobuf, MCAP, shell/`rg`, git read-only inspection.

**Commit policy:** Do not create commits. `CLAUDE.md` explicitly requires user authorization before `git commit`, and none was given. Preserve all pre-existing non-document changes.

---

### Task 1: Freeze the evidence baseline

**Files:**
- Read: `apps/replay_demo.cpp`
- Read: `apps/acoustic_optic_scenario_matrix.cpp`
- Read: `apps/acoustic_optic_scenarios.cpp`
- Read: `include/runtime/config.hpp`
- Read: `src/runtime/config.cpp`
- Read: `include/runtime/run_manifest.hpp`
- Read: `include/sensor_models/camera_rectifier.hpp`
- Read: `schemas/proto/uw/domain/image.proto`
- Read: `schemas/proto/uw/domain/measurement.proto`
- Read: `tests/integration/acoustic_optic_scenario_matrix_determinism_test.sh`
- Read: `cmake/Applications.cmake`
- Read: `cmake/Tests.cmake`

- [x] **Step 1: Record repository identity and user-owned changes**

Run:

```bash
git rev-parse --short HEAD
git status --short
git log -5 --oneline
```

Expected: HEAD is `8df083b`; status includes the existing P1/config/camera-rectifier changes and process documents. Save the list mentally and do not revert or overwrite those files.

- [x] **Step 2: Build the current working tree**

Run:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build -j2
```

Expected: configuration and build exit 0. If the user-owned P1 work is incomplete and fails, record the exact failure and document only facts independently supported by code/tests; do not repair code under this documentation task.

- [x] **Step 3: Measure the current verification surface**

Run:

```bash
ctest --test-dir build -N
ctest --test-dir build --output-on-failure
adapters/holoocean/.venv/bin/python -m pytest adapters/holoocean/tests -q
tools/lint/check_no_ros_in_core.sh
```

Expected at plan-writing time: 130 discovered CTest cases, all CTest cases pass, 35 Python tests pass, and lint exits 0. Use actual output if counts differ.

- [x] **Step 4: Run the documented end-to-end path**

Run:

```bash
tools/verify_pipeline.sh --out-dir /tmp/uw_slam_verify/docs_refresh
sed -n '1,240p' /tmp/uw_slam_verify/docs_refresh/summary.txt
```

Expected: verification exits 0 and the summary records successful build, tests, lint, synthetic MCAP generation, and replay. Capture actual CTest/pytest counts and output artifact names for the documentation edits.

- [x] **Step 5: Build a stale-claim inventory**

Run:

```bash
rg -n "106|114|25 个|35 个|没有任何提交|第一个提交|只在合成|未连接本机真实仿真器|退出码非 0 是正常|0/20 accepted|0 accepted|只被读取和打印|尚未真正切换|f285e0d|919e1f0|当前工作区" README.md CLAUDE.md docs/*.md
```

Expected: every match is classified as an obsolete current claim, a still-valid limitation, or an explicitly dated historical observation before editing.

### Task 2: Refresh the two root entry documents

**Files:**
- Modify: `README.md`
- Modify: `CLAUDE.md`

- [x] **Step 1: Correct README project status and capability matrix**

Update the opening and “核心能力” table so they state precisely:

- synthetic replay is a verified full vertical slice;
- recorded real HoloOcean stereo frames have been processed by the real-data VO path, but this is not a live ROS2/UE5 closed loop or a production accuracy claim;
- acoustic-optic fusion has an executable nine-scenario matrix, an enforced minimum-effective-coverage CTest gate, and fail-closed fault scenarios;
- `RunManifest` records git commit, config/calibration hashes, seed, OS/CPU/GPU description, and start/end time, while deeper dataset and dependency provenance remains future work;
- the working-tree camera rectifier only removes plumb-bob distortion for supported rigs and is not a general off-axis stereo rectifier.

Keep the entry paragraph concise and retain the explicit “not production ready” statement.

- [x] **Step 2: Correct README quick start, configuration, and boundaries**

Replace stale test counts with Task 1 results and “以实际命令输出为准”. In “配置与外部接入”, distinguish fields that dispatch (`estimator_mode`, `landmark_detector` where applicable), fields restricted to their sole recognized implementation, and fields still lacking alternative backends. Add a short real-data path/link without duplicating `adapters/holoocean/README.md`. Correct “已知边界” so scenario-matrix coverage and real-data VO are no longer described as absent.

- [x] **Step 3: Correct CLAUDE hard rules and verification instructions**

Delete “当前仓库只做过 git init、历史上没有任何提交” and retain only:

```markdown
- **除非用户明确要求，不要 `git commit`。** 保留用户已有改动；提交、变基、清理工作树都必须在授权范围内进行。
```

Update CTest/pytest counts from Task 1, add the enforced scenario-matrix gate command/meaning, and keep commands executable from repository root.

- [x] **Step 4: Add current semantic invariants to CLAUDE**

Document the exact split:

- `PressureDepthMeasurement.depth_m`: positive-down magnitude; world/body pose remains Z-up and consumers use `pose_z = -depth_m`;
- optical/fused `depth_m`: positive-forward camera optical-frame distance;
- expected-rejection scenarios (`time_offset_fault`, `extrinsic_perturbation`, sonar dropout, invalid optical region) must not be “fixed” by weakening gates;
- tied acoustic-optic candidates may be accepted only when their depth estimates agree within the combined-sigma gate;
- camera rectification support is plumb-bob undistortion plus parallel-rig assumptions, not arbitrary stereo rectification.

- [x] **Step 5: Check entry-document consistency**

Run:

```bash
rg -n "106|114|25 个|没有任何提交|第一个提交|退出码非 0 是正常|只在合成数据上验证" README.md CLAUDE.md
git diff --check -- README.md CLAUDE.md
```

Expected: no obsolete claim remains; any numerical match is an intentionally current value; diff check exits 0.

### Task 3: Refresh the documentation center and newcomer path

**Files:**
- Modify: `docs/README.md`
- Modify: `docs/uw-slam-newcomer-guide.md`

- [x] **Step 1: Update the documentation authority matrix**

Set current-fact documents’ last-check date to `2026-08-22`. Use `8df083b + 2026-08-22 working tree` where uncommitted P1 facts are included. Keep the architecture document as approved design and the Pipeline document as evolving/historical reference. Add the production-readiness roadmap to any missing task-routing rows.

- [x] **Step 2: Update the newcomer guide’s executable chains**

Describe three distinct paths:

1. synthetic pose-graph replay, including `black_box_vio` and `stereo_landmark_vo`;
2. acoustic-optic dense-depth association/fusion and its scenario matrix;
3. recorded real HoloOcean stereo data through offline VO, explicitly excluding live ROS2 closed loop.

Add `camera_rectifier` to the sensor-model map only if Task 1 build/tests pass. Update the “按任务快速定位” table with config selection validation, rectification, scenario-matrix tests, and RunManifest provenance.

- [x] **Step 3: Correct newcomer warnings and onboarding commands**

Replace stale test totals, clarify the scenario-matrix exit code, and distinguish “algorithm is not implemented” from “implemented but not validated on a live simulator”. Do not copy component installation procedures.

- [x] **Step 4: Verify routing and terminology**

Run:

```bash
rg -n "106|114|25 个|0 accepted|退出码非 0 是正常|只被读取和打印|只在合成" docs/README.md docs/uw-slam-newcomer-guide.md
git diff --check -- docs/README.md docs/uw-slam-newcomer-guide.md
```

Expected: remaining matches, if any, are explicitly dated historical observations or still-valid scoped limitations.

### Task 4: Perform the full codebase-reference audit

**Files:**
- Modify: `docs/uw-slam-codebase-reference-2026-08-18.md`

- [x] **Step 1: Update metadata, status overview, and directory map**

Change the last-check basis to `8df083b + 2026-08-22 working tree`, update test totals, add current applications and the camera rectifier files, and preserve the document’s “current facts only” authority statement.

- [x] **Step 2: Synchronize schema and sensor-model sections**

In `image.proto` and `measurement.proto` coverage, add `is_rectified`’s current enforcement boundary and the two different `depth_m` conventions. In sensor models, document `PlumbBobDistortion`, `ApplyPlumbBobDistortion`, and `UndistortImage`, including accepted encodings/coefficient counts, identity behavior, and parallel-rig limitation, but only if verified in Task 1.

- [x] **Step 3: Synchronize acoustic-optic frontend and scenario-matrix sections**

Replace the old “clean/elevation stress are 0/20 ambiguous” narrative with the post-fix behavior: tied geometric scores are accepted when depths agree, clean/elevation-stress regain effective output, deliberate fault-injection scenarios remain rejected, CTest enforces the matrix binary’s coverage exit code at fixed seed and eight trials, and fusion-improvement/latency gates remain opt-in.

Preserve the old result only inside a clearly labeled pre-fix investigation paragraph if it explains the bug.

- [x] **Step 4: Synchronize runtime, apps, and configuration sections**

Document current `RunManifest` population, `ValidateExperimentConfigSelections`, real-data experiment/recording paths, estimator and detector dispatch, and the precise fallback/error behavior. Do not claim a configurable alternative frontend or map backend exists when only one identifier is accepted.

- [x] **Step 5: Synchronize tests, build, and known boundaries**

Use Task 1 results, list the enforced scenario-matrix integration test, and rewrite boundaries around rectification, real-data VO, live ROS2, full VIO, landmark optimization, adaptive reliability, and production benchmarking.

- [x] **Step 6: Scan the long reference for internal contradictions**

Run:

```bash
rg -n "106|114|25 个|f285e0d|0/20|0 accepted|还没接进任何 app|退出码非 0 是正常|没有专门的确定性回归测试|只被读取|未实现" docs/uw-slam-codebase-reference-2026-08-18.md
git diff --check -- docs/uw-slam-codebase-reference-2026-08-18.md
```

Expected: each remaining match is verified against current source or clearly marked as historical/pre-fix.

### Task 5: Refresh verification guidance and production roadmap

**Files:**
- Modify: `docs/testing-and-verification-guide-2026-08-20.md`
- Modify: `docs/uw-slam-production-readiness-and-roadmap-2026-08-21.md`

- [x] **Step 1: Rewrite the verification matrix around current gates**

State that `integration.acoustic_optic_scenario_matrix_determinism` checks both deterministic output (excluding wall-clock latency) and a successful minimum-effective-coverage gate. A nonzero matrix exit is a failure unless the user intentionally enabled an additional opt-in threshold for an exploratory run. Add commands for config validation and camera-rectifier tests when verified.

- [x] **Step 2: Update complete and unavailable verification paths**

Use actual test counts and pipeline results. Separate offline real HoloOcean recording/VO evidence from live HoloOcean/ROS2/UE5 verification. Keep unavailable hardware/live tests explicitly unavailable.

- [x] **Step 3: Convert P0 roadmap items to completed status**

Mark complete with evidence:

- depth sign/frame comments;
- associator tied-candidate depth-agreement fix;
- minimum-effective-coverage gate and CTest wiring;
- pinned MCAP dependency and populated RunManifest fields added by recent commits.

Retain as future work the opt-in fusion-improvement threshold, calibrated latency gate, stronger dataset/dependency provenance, and all P1–P4 production milestones.

- [x] **Step 4: Reconcile gap and risk sections**

Qualify sections 5.5 and 5.6 as audit-time findings with current remediation status. Update configuration-risk language to reflect fail-fast validation for recognized selections while noting the lack of multiple frontend/map backend implementations. Reflect verified camera undistortion without claiming general rectification or P1 completion.

- [x] **Step 5: Verify guide/roadmap agreement**

Run:

```bash
rg -n "退出码非 0 是正常|0/20|0 accepted|106|114|25 个|尚未落地|可以在无有效输出时通过|只被读取和打印" docs/testing-and-verification-guide-2026-08-20.md docs/uw-slam-production-readiness-and-roadmap-2026-08-21.md
git diff --check -- docs/testing-and-verification-guide-2026-08-20.md docs/uw-slam-production-readiness-and-roadmap-2026-08-21.md
```

Expected: obsolete current claims are gone; audit-time findings remain only with explicit remediation annotations.

### Task 6: Add current-status deltas to dated design documents

**Files:**
- Modify: `docs/acoustic-optic-slam-platform-architecture-2026-08-17.md`
- Modify: `docs/holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md`

- [x] **Step 1: Update the architecture document’s current implementation map**

Add a concise `2026-08-22` status delta covering real-data offline VO, acoustic-optic scenario gates, provenance improvements, fail-fast config validation, and limited plumb-bob undistortion. Link to the codebase reference and roadmap for details. Do not rewrite target architecture sections 7–21 as present-tense implementation.

- [x] **Step 2: Update the Pipeline document’s current implementation status**

Correct the top status table so “光学/VO 前端” and “真实 HoloOcean 数据” are no longer wholly absent: offline recording and VO exist, while legacy baseline串联、live ROS2/UE5 closed loop, and production validation remain incomplete. Preserve the original Phase 0–3 plan as historical background.

- [x] **Step 3: Annotate only truly stale present-tense claims**

Search both documents for “当前/现在/尚未/未实现/已实现”. Where a claim describes the current repository, correct it or add a dated note. Where it expresses a target, historical audit, or milestone criterion, retain it unchanged.

- [x] **Step 4: Verify historical integrity**

Run:

```bash
git diff --word-diff=plain -- docs/acoustic-optic-slam-platform-architecture-2026-08-17.md docs/holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md
git diff --check -- docs/acoustic-optic-slam-platform-architecture-2026-08-17.md docs/holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md
```

Expected: changes are concentrated in status/implementation notes; original design rationale and milestone definitions remain intact.

### Task 7: Run the cross-document consistency and link audit

**Files:**
- Verify: `README.md`
- Verify: `CLAUDE.md`
- Verify: `docs/*.md`
- Verify: `docs/superpowers/specs/2026-08-22-documentation-consistency-refresh-design.md`
- Verify: `docs/superpowers/plans/2026-08-22-documentation-consistency-refresh.md`

- [x] **Step 1: Run the final stale-claim scan**

Run:

```bash
rg -n "106|114|25 个|没有任何提交|第一个提交|退出码非 0 是正常|只在合成数据上验证|0/20 accepted|0 accepted, 0 ambiguous|只被读取和打印|未连接本机真实仿真器|f285e0d|919e1f0" README.md CLAUDE.md docs/*.md
```

Expected: no unqualified obsolete current claim. Historical results must be labeled with date/commit and linked to their remediation.

- [x] **Step 2: Check exact code and path references**

Run:

```bash
rg -n "`[^`]+\.(cpp|hpp|proto|yaml|sh|md)`" README.md CLAUDE.md docs/*.md
rg --files apps include src schemas configs tests tools docs | sort
```

Expected: every mentioned repository path/target is present or explicitly described as planned/external.

- [x] **Step 3: Check Markdown relative links**

Run:

```bash
rg -n "\]\([^:)]+\.md(#[^)]+)?\)" README.md CLAUDE.md docs/*.md
rg -n "\[\[|\]\(file:|\]\(/home/" README.md CLAUDE.md docs/*.md
```

Expected: all repository links are relative Markdown links, no Obsidian/file/absolute local links appear, and manual target checks confirm linked files exist.

- [x] **Step 4: Re-run documentation-backed verification**

Run:

```bash
ctest --test-dir build --output-on-failure
adapters/holoocean/.venv/bin/python -m pytest adapters/holoocean/tests -q
tools/lint/check_no_ros_in_core.sh
git diff --check
```

Expected: tests/lint pass and diff check exits 0. If the pre-existing P1 work changes results during implementation, update documentation to the final observed output.

- [x] **Step 5: Review final scope and preserve user work**

Run:

```bash
git status --short
git diff --stat
git diff -- README.md CLAUDE.md docs/README.md docs/uw-slam-newcomer-guide.md docs/uw-slam-codebase-reference-2026-08-18.md docs/testing-and-verification-guide-2026-08-20.md docs/uw-slam-production-readiness-and-roadmap-2026-08-21.md docs/acoustic-optic-slam-platform-architecture-2026-08-17.md docs/holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md
```

Expected: task-authored product changes are limited to the approved nine Markdown files plus the new design/plan records; all pre-existing code/config changes remain present and untouched.
