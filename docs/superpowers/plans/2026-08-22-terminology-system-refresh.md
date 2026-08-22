# uw_slam Terminology System Refresh Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace enterprise-flavored or misleading architecture language with a precise robotics/SLAM terminology hierarchy while preserving every public identifier and executable behavior.

**Architecture:** User-facing documentation will use “核心消息与接口” as the umbrella concept and “跨语言规范化消息模型” for Protobuf specifically. Historical code identifiers remain unchanged, but authoritative documentation and narrowly scoped source comments will state their real current semantics.

**Tech Stack:** Markdown, C++17 comments, proto3 comments, YAML comments, ripgrep, repository layer lint

---

## Constraints

- Work in the existing dirty worktree because the target documentation already contains user changes.
- Preserve unrelated edits and apply only small contextual patches.
- Do not rename directories, namespaces, targets, classes, functions, configuration keys, Protobuf fields, test labels, or files.
- Do not modify `docs/superpowers/plans/` or earlier `docs/superpowers/specs/`, except this plan and its approved design document.
- Do not create a Git commit unless the user separately authorizes one.

### Task 1: Establish the terminology hierarchy in project entry documents

**Files:**

- Modify: `README.md`
- Modify: `CLAUDE.md`
- Modify: `docs/README.md`

- [ ] **Step 1: Capture the current scoped occurrences**

Run:

```bash
rg -n '领域契约|契约测试|垂直切片|canonical MCAP|evidence handoff' \
  README.md CLAUDE.md docs/README.md
```

Expected: the command reports the existing project-status, architecture diagram, repository table,
test description, and documentation-routing wording without touching historical plan files.

- [ ] **Step 2: Update the root README terminology**

Apply these exact concepts while preserving the surrounding current-status facts:

```text
“架构骨架 + 可运行垂直切片”
→ “架构骨架 + 可运行端到端链路”

能力表“领域契约”
→ “核心消息与接口”
说明：Protobuf 提供跨语言规范化消息模型，measurement_api 提供算法接口。

“点云 evidence handoff”
→ “局部点云数据交接”

“canonical MCAP 录制”
→ “统一 MCAP 格式录制”

架构图“Protobuf 领域契约”
→ “Protobuf 规范化消息模型”

依赖图：
schemas / 跨语言消息模型
→ domain / 生成消息辅助与校验
→ core / 传感器模型 + 算法接口

schemas/proto 目录职责
→ “跨语言规范化消息模型和 C++/Python 代码生成输入”

“契约测试”
→ “消息格式与接口一致性测试（CTest 标签仍为 contract.*）”
```

Do not change `contract.*`, `tests/contracts/`, `domain`, or any code identifier shown in backticks.

- [ ] **Step 3: Update contributor guidance and documentation routing**

Use these exact descriptions:

```text
CLAUDE.md:
- “骨架 + 每层至少一条真实可跑的端到端链路”
- “Protobuf 是跨语言规范化消息模型的唯一来源”
- 目录速查：“规范化 Protobuf 消息定义”

docs/README.md:
- “理解核心消息与接口、依赖 DAG、状态机与 Gate”
```

Keep the existing rule that new cross-language fields must be added to `.proto`; only its name and
explanation change.

- [ ] **Step 4: Review only the entry-document diff**

Run:

```bash
git diff -- README.md CLAUDE.md docs/README.md
```

Expected: only terminology and clarification lines from this task appear in addition to the user's
pre-existing changes; no existing status numbers, commands, or capability claims are removed.

### Task 2: Update authoritative architecture headings and cross-references

**Files:**

- Modify: `docs/acoustic-optic-slam-platform-architecture-2026-08-17.md`
- Modify: `docs/uw-slam-codebase-reference-2026-08-18.md`
- Modify: `docs/uw-slam-newcomer-guide.md`
- Modify: `docs/testing-and-verification-guide-2026-08-20.md`

- [ ] **Step 1: Rename the long-form headings and anchors together**

Apply these exact heading and link changes:

```text
Architecture document:
7. [领域契约](#7-领域契约)
→ 7. [核心消息与接口](#7-核心消息与接口)
## 7. 领域契约
→ ## 7. 核心消息与接口

Codebase reference:
[第 4 节](#4-领域契约层schemasproto)
→ [第 4 节](#4-跨语言规范化消息模型schemasproto)
4. [领域契约层：schemas/proto/](#4-领域契约层schemasproto)
→ 4. [跨语言规范化消息模型：schemas/proto/](#4-跨语言规范化消息模型schemasproto)
## 4. 领域契约层：schemas/proto/
→ ## 4. 跨语言规范化消息模型：schemas/proto/
```

Update the section-4 self-reference near the mapping discussion to the same new anchor.

- [ ] **Step 2: Replace the remaining conceptual wording in these documents**

Use this mapping consistently:

```text
领域契约 / Protobuf 契约 → 核心消息与接口 / Protobuf 规范化消息模型
领域类型 → 核心消息类型
契约测试 → 消息格式与接口一致性测试
垂直切片 → 端到端链路
canonical MCAP → 统一 MCAP 录制格式
mapping handoff → 局部地图数据交接
```

For `MeasurementEvidence`, introduce it once as
“带来源、有效域和不确定度描述的量测结果（`MeasurementEvidence`）”. For `MapEvidence`,
introduce it once as “保存在局部坐标系中的局部地图数据（`MapEvidence`）”. Preserve the
English type names wherever the text discusses exact fields or APIs.

- [ ] **Step 3: Rename the learning-model information section**

Change:

```text
### 8.4 AI information cap
→ ### 8.4 学习模型信息量上限
```

Keep the formula and its physical/calibration/cross-modal caps unchanged. Rewrite only the lead-in
so it says learned confidence is not automatically a calibrated covariance or information value.

- [ ] **Step 4: Verify changed Markdown anchors**

Run:

```bash
rg -n '#(4|7)-领域契约|#4-跨语言规范化消息模型schemasproto|#7-核心消息与接口' \
  docs README.md CLAUDE.md -g '!docs/superpowers/**'
```

Expected: no old `#4-领域契约...` or `#7-领域契约` anchor remains; the new table-of-contents
links and section-4 self-reference are reported.

### Task 3: Clarify historical identifiers without renaming them

**Files:**

- Modify: `configs/README.md`
- Modify: `configs/experiment/synthetic_smoke.yaml`
- Modify: `configs/experiment/synthetic_smoke_vo.yaml`
- Modify: `configs/experiment/acoustic_optic_demo.yaml`
- Modify: `configs/experiment/real_holoocean_vo.yaml`
- Modify: `docs/uw-slam-codebase-reference-2026-08-18.md`
- Modify: `docs/uw-slam-newcomer-guide.md`
- Modify: `docs/uw-slam-production-readiness-and-roadmap-2026-08-21.md`

- [ ] **Step 1: Correct the configuration vocabulary**

Add this statement near the first `estimator_mode` explanation in `configs/README.md`:

```text
`estimator_mode` 是保留兼容性的历史字段名。当前它只选择相对位姿输入来源：
`black_box_vio` 读取 bag 中外部或预生成的相对位姿量测，`stereo_landmark_vo` 从双目
图像在线计算相对位姿；两条路径最终使用同一个 `GaussNewtonSolver`，并不切换估计求解器。
```

Describe `map_backend` as a reserved map-implementation selector with one supported value. Rename
the heading “声光前端契约字段” to “声光消息与接口字段”. Replace general prose such as
“estimator mode” with “相对位姿输入模式”, while retaining the literal YAML key in backticks.

- [ ] **Step 2: Correct experiment YAML comments**

Use comments with these meanings without changing any YAML key or value:

```yaml
# estimator_mode 是历史字段名，当前选择相对位姿输入来源，不切换求解器。
# black_box_vio 读取 bag 中外部/预生成量测；stereo_landmark_vo 从双目图像计算量测。
# map_backend 是预留的地图实现选择，目前只有 submap_point_cloud_v1。
```

Preserve every existing scenario, rig, frontend, seed, threshold, and gate value exactly.

- [ ] **Step 3: Add concise responsibility warnings to current guides**

Ensure the codebase reference, newcomer guide, and production-readiness roadmap each make the
following distinctions at their first relevant discussion:

```text
estimator_mode: relative-pose source selector, not solver selector.
SubmapManager: keyframe-indexed local map-data store in v1, not a complete submap lifecycle manager.
runtime: runtime-support primitives today, not a composed online scheduler.
AcousticOpticDepthFusionFrontend: a fusion module retained under the historical frontends name.
map_backend: a reserved selector with one supported implementation.
```

Use natural Chinese prose rather than copying these English labels verbatim. Do not repeat the full
warning in every occurrence; establish it once, then use the code identifier normally.

- [ ] **Step 4: Normalize the production-readiness terminology**

Apply these exact conceptual changes while preserving all maturity scores and evidence:

```text
“架构与领域契约” → “架构、核心消息与接口”
“Protobuf 领域契约” → “Protobuf 规范化消息模型”
“点云 evidence handoff” → “局部点云数据交接”
“canonical writer / canonical MCAP” in Chinese prose
  → “统一格式写入器 / 统一 MCAP 录制格式”
“合成垂直切片” → “合成端到端链路”
```

Keep exact code filenames such as `canonical_writer.py` unchanged.

### Task 4: Document misleading wire and C++ identifiers at their definitions

**Files:**

- Modify: `schemas/proto/uw/domain/factor.proto`
- Modify: `include/runtime/config.hpp`
- Modify: `include/mapping/submap_manager.hpp`
- Modify: `include/frontends/acoustic_optic_depth_fusion_frontend.hpp`
- Modify: `apps/replay_demo.cpp`
- Modify: `cmake/UwProtobuf.cmake`

- [ ] **Step 1: Document `proposed_noise` at the wire definition**

Replace the bare field with this comment and unchanged field declaration:

```proto
// Historical field name. Current v1 FactorBuilders interpret a positive
// value as one isotropic sqrt-information scalar (larger means stronger
// weight); zero or a negative value falls back to 1.0. Renaming this wire
// field requires a separate schema-compatibility migration.
double proposed_noise = 4;
```

- [ ] **Step 2: Document configuration-field semantics at the C++ definition**

Add immediately above `estimator_mode`:

```cpp
// Historical name: this selects the relative-pose evidence source, not the
// optimizer. Both supported values feed the same GaussNewtonSolver.
```

Add immediately above `map_backend`:

```cpp
// Reserved map-implementation selector; v1 accepts only
// "submap_point_cloud_v1".
```

- [ ] **Step 3: State the real responsibilities of the two historically named classes**

Add to the `SubmapManager` class comment:

```cpp
// Despite the historical name, v1 is a keyframe-indexed local map-evidence
// store. It does not create, switch, merge, or retire full submaps.
```

Add before `AcousticOpticDepthFusionFrontend`:

```cpp
// This is a fusion module. The Frontend suffix and placement in the
// frontends target are retained for source compatibility.
```

- [ ] **Step 4: Remove misleading terminology from nearby implementation comments**

In `apps/replay_demo.cpp`, replace “AI information cap” with “learned-model information cap” and
clarify that the current fixed sqrt-information constants are not a calibrated reliability policy.
In `cmake/UwProtobuf.cmake`, replace “source of truth for domain contracts” with “single source for
the cross-language normalized message model”. Do not change executable statements.

- [ ] **Step 5: Confirm source changes are comments only**

Run:

```bash
git diff --word-diff=plain -- \
  schemas/proto/uw/domain/factor.proto include/runtime/config.hpp \
  include/mapping/submap_manager.hpp \
  include/frontends/acoustic_optic_depth_fusion_frontend.hpp \
  apps/replay_demo.cpp cmake/UwProtobuf.cmake
```

Expected: the only source-file changes from this plan are comments; field declarations, default
values, class signatures, executable statements, and target definitions are unchanged.

### Task 5: Verify terminology consistency and repository safety

**Files:**

- Verify all files modified in Tasks 1–4
- Do not modify historical files under `docs/superpowers/`

- [ ] **Step 1: Check removed terminology in the authoritative scope**

Run:

```bash
rg -n '领域契约|领域类型|契约测试|垂直切片|canonical MCAP|evidence handoff|mapping handoff|AI information cap' \
  README.md CLAUDE.md docs/README.md \
  docs/acoustic-optic-slam-platform-architecture-2026-08-17.md \
  docs/testing-and-verification-guide-2026-08-20.md \
  docs/uw-slam-codebase-reference-2026-08-18.md \
  docs/uw-slam-newcomer-guide.md \
  docs/uw-slam-production-readiness-and-roadmap-2026-08-21.md \
  configs/README.md configs/experiment
```

Expected: no match. Literal filenames or API identifiers containing `canonical` are outside this
search and remain unchanged.

- [ ] **Step 2: Check that historical identifiers remain present and explained**

Run:

```bash
rg -n 'estimator_mode|proposed_noise|SubmapManager|AcousticOpticDepthFusionFrontend|map_backend' \
  README.md CLAUDE.md docs configs include schemas apps \
  -g '!docs/superpowers/**'
```

Expected: identifiers still exist; their first authoritative explanations match the approved
design and no text claims that `estimator_mode` switches solvers or that v1 manages full submaps.

- [ ] **Step 3: Run structural checks**

Run:

```bash
python3 tools/lint/check_layer_dependencies.py .
git diff --check
```

Expected:

```text
OK: C++ layer dependencies and ROS/vendor boundaries are valid
```

`git diff --check` prints nothing and exits successfully.

- [ ] **Step 4: Review the final scoped diff without altering unrelated work**

Run:

```bash
git diff --stat
git diff -- README.md CLAUDE.md docs configs cmake/UwProtobuf.cmake \
  schemas/proto/uw/domain/factor.proto include/runtime/config.hpp \
  include/mapping/submap_manager.hpp \
  include/frontends/acoustic_optic_depth_fusion_frontend.hpp apps/replay_demo.cpp
git status --short
```

Expected: all pre-existing user modifications remain present, terminology edits are limited to the
approved scope, historical plan/spec files are untouched except for the newly added approved design
and this implementation plan, and no commit has been created.

## Execution verification record (2026-08-22)

This record identifies task ownership at hunk level in the intentionally dirty worktree. Ownership
is limited to the terminology, anchor, explanatory-prose, and source-comment changes listed below;
all other hunks in these files remain pre-existing user work.

### Task-owned hunk inventory

- **Task 1 — entry-document terminology and clarification prose:** `README.md`, `CLAUDE.md`, and
  `docs/README.md`. Owned hunks replace the approved project-status, core-message/interface,
  normalized-Protobuf-message, end-to-end-chain, unified-MCAP, local-map-data-handoff, repository
  role, and message/interface-consistency-test wording. Existing commands, test counts, capability
  updates, and other status prose in the same files are not owned by this task.
- **Task 2 — authoritative terminology, headings, and anchors:**
  `docs/acoustic-optic-slam-platform-architecture-2026-08-17.md`,
  `docs/uw-slam-codebase-reference-2026-08-18.md`, `docs/uw-slam-newcomer-guide.md`, and
  `docs/testing-and-verification-guide-2026-08-20.md`. Owned hunks rename the section 7 and section
  4 headings, TOC links, and self-links; replace the approved conceptual terms; introduce precise
  `MeasurementEvidence`/`MapEvidence` descriptions; and rename/explain the learned-model information
  limit. Other factual, maturity, test-count, implementation-status, and roadmap updates in these
  overlapping documents are not owned by this task.
- **Task 3 — historical-identifier explanations and YAML comments:** `configs/README.md`,
  `configs/experiment/synthetic_smoke.yaml`, `configs/experiment/synthetic_smoke_vo.yaml`,
  `configs/experiment/acoustic_optic_demo.yaml`, `configs/experiment/real_holoocean_vo.yaml`,
  `docs/uw-slam-codebase-reference-2026-08-18.md`, `docs/uw-slam-newcomer-guide.md`, and
  `docs/uw-slam-production-readiness-and-roadmap-2026-08-21.md`. Owned prose explains
  `estimator_mode`, `map_backend`, `SubmapManager`, runtime support, and
  `AcousticOpticDepthFusionFrontend` without renaming them. In the four experiment YAML files, only
  explanatory comments are owned; every non-comment key and value is unchanged from `HEAD`.
- **Task 4 — definition-site comments only:** `schemas/proto/uw/domain/factor.proto`,
  `include/runtime/config.hpp`, `include/mapping/submap_manager.hpp`,
  `include/frontends/acoustic_optic_depth_fusion_frontend.hpp`, `apps/replay_demo.cpp`, and
  `cmake/UwProtobuf.cmake`. The owned source hunks are uniquely identifiable by these phrases and
  meanings: the `proposed_noise` historical-name, isotropic sqrt-information, tag-4, and generated
  API/Protobuf JSON/text-name migration warning; the `estimator_mode` relative-pose-source and
  camera-rig fallback comment plus the one-supported-value `map_backend` comment; the
  `SubmapManager` “does not create, switch, merge, or retire full submaps” warning; the fusion
  module's `Frontend` source-compatibility and frontends-target build/link-compatibility warning;
  the replay “learned-model information cap” and fixed-constants-not-calibrated-policy comment; and
  the CMake “single source for the cross-language normalized message model” comment. No declaration,
  signature, target statement, configuration value, or executable statement in these files is
  task-owned.
- **New task records:**
  `docs/superpowers/specs/2026-08-22-terminology-system-refresh-design.md` and this file,
  `docs/superpowers/plans/2026-08-22-terminology-system-refresh.md`.

### Explicitly non-owned dirty-worktree work

The task preserved and neither created nor reverted the executable hunks already visible in
overlapping files: `include/runtime/config.hpp` adding `<optional>` and the
`ValidateExperimentConfigSelections` API; `apps/replay_demo.cpp` calling configuration validation
and adding sonar-frame latency collection, P95 calculation, and output; and
`cmake/UwProtobuf.cmake` adding the `absl::log_internal_check_op` link dependency. Any other hunk in
an overlapping task file that is not described in the owned inventory above is likewise
pre-existing/non-owned.

Other pre-existing tracked dirty paths outside the owned scope were preserved:
`.github/workflows/ci.yml`, `CMakeLists.txt`, `adapters/datasets/README.md`,
`docs/holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md`, `cmake/Libraries.cmake`,
`cmake/Tests.cmake`, `src/runtime/config.cpp`, `tests/runtime/config_test.cpp`, and
`tools/codegen/gen_py.sh`. Pre-existing untracked additions were also preserved: `.superpowers/`,
the `adapters/datasets/` Python package/tests, `configs/experiment/euroc_mh01_vo.yaml`,
`configs/rig/euroc_mh01.yaml`, the non-terminology 2026-08-21/22 plan/spec additions under
`docs/superpowers/`, the new camera-rectifier/map-metrics/surfel-map headers, sources, and tests, and
`tools/run_quality_checks.sh`.

### Verification evidence and limitation

- The authoritative-scope old-term search returned no matches. Old section-4/section-7 anchors are
  absent, the new TOC/self-link anchors are present, and the scoped link/anchor check passed.
- After stripping full-line and inline comments, all four task-edited experiment YAML files are
  byte-identical to their `HEAD` non-comment content.
- `python3 tools/lint/check_layer_dependencies.py .` exited 0 with
  `OK: C++ layer dependencies and ROS/vendor boundaries are valid`; `git diff --check` exited 0 with
  no output.
- Historical identifiers remain in place and their declarations, configuration keys, and values
  were not renamed. `HEAD` remained `8df083b58886fbfc65d6656f6a4227c04f455fb7`; this task created
  no commit. A full build and full CTest/pytest suites were not run because task-owned changes are
  documentation and comments only.

No saved byte-for-byte snapshot of the worktree immediately before this task exists. Attribution
therefore rests on this explicit hunk inventory together with the pre-task dirty status captured
during the session; it is not checksum proof and must not be represented as such.
