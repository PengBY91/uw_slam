# `uw_slam` README Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite the root README as a truthful, executable onboarding guide for internal developers that remains approachable to first-time open-source visitors.

**Architecture:** Use progressive disclosure: project identity and the shortest runnable path come first, followed by the data flow, repository architecture, developer workflows, advanced integrations, and project constraints. Keep the root README authoritative for onboarding while linking low-frequency details to existing module documentation.

**Tech Stack:** GitHub-flavored Markdown, Mermaid, Shields.io static badges, Bash verification commands, CMake/CTest, Python/pytest, C++17, Protobuf, MCAP, ROS2 Jazzy.

---

## File map

- Modify: `README.md` — project landing page, quick start, architecture overview, developer guide, limitations, and documentation index.
- Reference: `docs/superpowers/specs/2026-08-19-readme-redesign.md` — approved content and acceptance criteria.
- Reference: `tools/setup_dev_env.sh` — supported C++ dependency setup paths.
- Reference: `tools/verify_pipeline.sh` — canonical local build, test, lint, and Demo verification path.
- Reference: `adapters/holoocean/README.md` and `adapters/ros2/README.md` — advanced integration details linked from the root README.
- Verify only: all other source, configuration, license, and documentation files used to substantiate README claims.

No source code or configuration file is changed. Git commit steps are intentionally omitted because `CLAUDE.md` forbids commits unless the user explicitly requests one.

### Task 1: Rewrite the landing page and runnable onboarding path

**Files:**

- Modify: `README.md`
- Reference: `docs/superpowers/specs/2026-08-19-readme-redesign.md`
- Reference: `tools/setup_dev_env.sh`
- Reference: `tools/verify_pipeline.sh`

- [ ] **Step 1: Replace the opening with project identity and verified status**

  Write a Chinese title and one-paragraph positioning statement. Add static badges for C++17,
  Python 3.10+, ROS2 Jazzy, and GPLv3. State that the repository contains a runnable vertical
  slice rather than a production system. Summarize the canonical path as HoloOcean or synthetic
  observations to MCAP/Protobuf, frontend evidence, factor construction, pose-graph estimation,
  mapping, and ATE/RPE evaluation.

- [ ] **Step 2: Add a compact navigation list, core capabilities, and technology table**

  List only major sections in the table of contents. Describe the implemented sonar CFAR frontend,
  relative-pose/depth/sonar-range factors, Eigen Gauss-Newton/LM solver, submap manager, layered
  configuration, deterministic replay, and adapters. Mark partially wired integrations explicitly.

- [ ] **Step 3: Add the five-minute quick-start path**

  Make `tools/setup_dev_env.sh` followed by `tools/verify_pipeline.sh` the primary entry point.
  Explain the default output directory convention and the `summary.txt`, MCAP, TUM trajectory,
  and RunManifest artifacts. Keep the verified command block exactly executable from the repository
  root:

  ```bash
  ./tools/setup_dev_env.sh
  tools/verify_pipeline.sh --out-dir /tmp/uw_slam_verify/readme_smoke
  cat /tmp/uw_slam_verify/readme_smoke/summary.txt
  ```

- [ ] **Step 4: Add the manual Demo path and expected behavior**

  Document configure, build, `synth_bag_gen`, and `replay_demo` commands with
  `configs/experiment/synthetic_smoke.yaml`. Describe convergence and ATE as typical observations,
  not contractual thresholds. Link the landmark-elevation explanation to
  `docs/uw-slam-codebase-reference-2026-08-18.md` rather than the nonexistent
  `CODEBASE_GUIDE.md`.

### Task 2: Add the architecture and internal developer guide

**Files:**

- Modify: `README.md`
- Reference: `CMakeLists.txt`
- Reference: `configs/README.md`
- Reference: `CLAUDE.md`
- Reference: `NOTICE`

- [ ] **Step 1: Add the Mermaid data-flow and dependency diagram**

  Show synthetic/HoloOcean/ROS2 inputs, canonical MCAP plus Protobuf contracts, frontend and factor
  stages, estimator and mapping stages, and evaluation outputs. Show the permitted dependency order
  `core → algorithms → runtime → adapters → apps`, while explaining that schemas are the
  cross-language source of truth.

- [ ] **Step 2: Add the repository map**

  Use a compact table for `schemas`, `core`, `algorithms`, `runtime`, `adapters`, `apps`, `configs`,
  `evaluation`, `tests`, and `tools`. Keep component descriptions aligned with actual CMake targets
  and avoid duplicating the detailed codebase reference.

- [ ] **Step 3: Document manual build and test commands**

  Include the apt/default CMake path and the conda fallback path from `tools/setup_dev_env.sh`.
  Include CTest, Python venv installation, pytest, and the dependency lint. Use the verified Python
  path rather than bare `uv sync`:

  ```bash
  python3 -m venv adapters/holoocean/.venv
  adapters/holoocean/.venv/bin/pip install -e "./adapters/holoocean[dev]"
  adapters/holoocean/.venv/bin/python -m pytest adapters/holoocean/tests
  ```

- [ ] **Step 4: Document configuration and advanced integrations**

  Explain `defaults → rig → scenario → experiment`, CLI override precedence, and immutable
  RunManifest output. Summarize HoloOcean and ROS2 requirements and link to their adapter READMEs;
  do not repeat the full colcon setup in the root README.

- [ ] **Step 5: Add contribution rules, known boundaries, licensing, and reading order**

  Surface the repository's critical rules: never modify `external_repos` children; preserve the
  one-way dependency; change Protobuf schemas instead of creating parallel cross-language types;
  read `NOTICE` before porting; use deterministic seeded RNG; and run the verification pipeline.
  Distinguish verified components from unwired integrations, link GPLv3 and provenance, and order
  the architecture, codebase reference, configuration, adapter, NOTICE, and CLAUDE documents by use.

### Task 3: Validate document structure, links, and factual claims

**Files:**

- Verify: `README.md`
- Verify: every local path referenced from `README.md`

- [ ] **Step 1: Inspect the final heading outline**

  Run:

  ```bash
  rg -n '^#{1,4} ' README.md
  ```

  Expected: one H1, sequential major sections, and no duplicate section names that would produce
  ambiguous GitHub anchors.

- [ ] **Step 2: Check all relative file links**

  Run:

  ```bash
  rg -o '\]\(\./[^)#]+' README.md \
    | sed 's/^.*](\.\///' \
    | sort -u \
    | while IFS= read -r path; do
        test -e "$path" || { echo "missing: $path"; exit 1; }
      done
  ```

  Expected: exit status `0` and no `missing:` output.

- [ ] **Step 3: Scan for known stale or unsupported claims**

  Run:

  ```bash
  if rg -n '13/13|CODEBASE_GUIDE|uv sync|生产可用|完整接入真实.*HoloOcean' README.md; then
    exit 1
  fi
  ```

  Expected: exit status `0` and no matches.

- [ ] **Step 4: Check whitespace and the final diff**

  Run:

  ```bash
  git diff --check
  git diff -- README.md docs/superpowers/specs/2026-08-19-readme-redesign.md \
    docs/superpowers/plans/2026-08-19-readme-redesign.md
  ```

  Expected: no whitespace errors; the diff contains documentation changes only.

### Task 4: Run the documented workflow and align status claims

**Files:**

- Verify: `README.md`
- Verify only: build and test outputs under existing `build/` and `/tmp/uw_slam_verify/readme_smoke`

- [ ] **Step 1: Run the complete non-ROS2 verification pipeline**

  Run:

  ```bash
  tools/verify_pipeline.sh --out-dir /tmp/uw_slam_verify/readme_smoke
  ```

  Expected: exit status `0`, `RESULT: all steps passed`, 14 CTest cases passing, 9 Python tests
  passing, lint passing, and both Demo applications completing.

- [ ] **Step 2: Inspect the generated summary and artifacts**

  Run:

  ```bash
  sed -n '1,220p' /tmp/uw_slam_verify/readme_smoke/summary.txt
  test -s /tmp/uw_slam_verify/readme_smoke/synthetic.mcap
  test -s /tmp/uw_slam_verify/readme_smoke/demo_trajectory.tum
  test -s /tmp/uw_slam_verify/readme_smoke/demo_run_manifest.json
  ```

  Expected: every recorded step is `PASS`, an ATE line is present, and all three artifacts are
  non-empty.

- [ ] **Step 3: Reconcile README status text with actual results**

  If counts or output values differ from the expected results above, update only the corresponding
  current-status sentence and typical-output example in `README.md`; keep the reproducible commands
  unchanged. Re-run `git diff --check` after any correction.

- [ ] **Step 4: Confirm scope preservation**

  Run:

  ```bash
  git status --short
  ```

  Expected: this work adds the approved design and plan documents and modifies `README.md`; the
  pre-existing untracked `external_repos/` remains untouched.
