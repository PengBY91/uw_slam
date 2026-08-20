# `uw_slam` Documentation Information Architecture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reorganize the project’s core technical documentation into a navigable, status-aware system that clearly separates current code facts, approved architecture, evolving plans, and external references.

**Architecture:** Add `docs/README.md` as the routing layer between the project README and specialist documents. Give each long document a concise entry section and table of contents, preserve detailed technical bodies, migrate non-GitHub wiki links, and keep `external_repos/README.md` focused on repository recovery while its overview owns analysis and risk context.

**Tech Stack:** GitHub-flavored Markdown, YAML front matter, relative Markdown links, Mermaid/text diagrams, Bash and Python read-only document validation.

---

## File map

- Create: `docs/README.md` — documentation index, authority order, reading paths, and maintenance rules.
- Modify: `README.md` — add the documentation center to extended reading without removing direct links.
- Modify: `docs/holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md` — evolving-plan entry, status matrix, navigation, GitHub links, and labeled evolution/audit sections.
- Modify: `docs/acoustic-optic-slam-platform-architecture-2026-08-17.md` — approved-design entry, implementation map, navigation, and corrected link/assumption presentation.
- Modify: `docs/uw-slam-codebase-reference-2026-08-18.md` — verification metadata, task-oriented navigation, document authority clarification, and stale status corrections.
- Modify: `external_repos/external-repos-overview.md` — scope, read-only boundary, navigation, provenance/status matrix, and consistent detailed-section framing.
- Modify: `external_repos/README.md` — concise recovery table and special access notes.
- Modify: `.gitignore` — allow exactly the top-level external repository overview to be versioned.
- Verify only: third-party children below `external_repos/`, source code, schemas, configuration, adapter READMEs, `NOTICE`, and `CLAUDE.md`.

Git commit steps are omitted because `CLAUDE.md` prohibits commits unless the user explicitly asks for one. Preserve the existing uncommitted root README redesign and process documents.

### Task 1: Create the documentation routing layer

**Files:**

- Create: `docs/README.md`
- Modify: `README.md:323`
- Reference: `docs/superpowers/specs/2026-08-19-documentation-information-architecture-design.md`

- [ ] **Step 1: Capture the implementation baseline**

  Run:

  ```bash
  git rev-parse --short HEAD
  git status --short
  rg --files docs external_repos -g '*.md' | sort
  ```

  Expected: HEAD is `919e1f0` unless the user changed it; `README.md` and `docs/superpowers/`
  remain existing work, and no third-party child is selected for modification.

- [ ] **Step 2: Create `docs/README.md` with task-oriented reading paths**

  Use these major sections in this order:

  ```markdown
  # uw_slam 文档中心

  ## 从这里开始
  ## 按任务选择文档
  ## 文档状态与权威范围
  ## 信息冲突时以谁为准
  ## 文档维护约定
  ```

  Route first-time readers to the root README, implementation/debugging readers to the codebase
  reference, architecture changes to the approved architecture, experiment work to `configs/README.md`,
  integration work to adapter and external-repository docs, and provenance work to `NOTICE`.

- [ ] **Step 3: Add the status and authority matrix**

  Include every core document with these explicit classifications:

  - root README: current project entry;
  - codebase reference: current implementation facts, verified at `919e1f0` on `2026-08-19`;
  - architecture: approved target design;
  - Pipeline plan: evolving/historical engineering plan;
  - configuration and adapter READMEs: component-specific current instructions;
  - external overview: audited reference context;
  - `NOTICE`: authoritative porting provenance and licensing record.

- [ ] **Step 4: Define conflict resolution and maintenance rules**

  State this authority order: source code and Protobuf schema → codebase/config/adapter reference →
  root README → approved architecture for target decisions → evolving Pipeline history. Require
  relative links, verification date/commit for code-fact documents, and explicit current/design/history
  labels.

- [ ] **Step 5: Link the documentation center from the root README**

  Add the following first row to the `延伸阅读` table in `README.md`:

  ```markdown
  | 不确定应该先读哪份文档 | [文档中心](./docs/README.md) |
  ```

### Task 2: Reframe the HoloOcean Pipeline plan

**Files:**

- Modify: `docs/holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md:1-47`
- Modify: GitHub-incompatible links throughout the same file
- Reference: `README.md`
- Reference: `docs/uw-slam-codebase-reference-2026-08-18.md`
- Reference: `docs/acoustic-optic-slam-platform-architecture-2026-08-17.md`

- [ ] **Step 1: Update front matter and document authority**

  Change `status: draft` to `status: evolving-plan`, set `updated: 2026-08-19`, and add
  `verified_against: 919e1f0`. Immediately below H1, state that this document records the first-stage
  baseline and its evolution; current code facts come from the codebase reference, and long-term
  decisions come from the approved architecture.

- [ ] **Step 2: Add a thirty-second summary and implementation matrix**

  Summarize four states without changing technical claims:

  | State | Content |
  |---|---|
  | Current vertical slice | synthetic MCAP → CFAR → factors → pose graph → evaluation |
  | Partially implemented | Python HoloOcean gateway and ROS2 ImagingSonar transport |
  | Not connected | live HoloOcean/UE5 flow, optical/VIO frontend, historical baseline end-to-end |
  | Historical baseline | SVIn pose feeding sonar-camera reconstruction; retained for comparison only |

- [ ] **Step 3: Add a table of contents covering sections 1–11**

  Preserve existing numbering. Label section 10 as the target-architecture evolution record and
  section 11 as the code-audit correction appendix in both the table of contents and headings, so
  readers understand that later corrections supersede historical assumptions.

- [ ] **Step 4: Convert repository-local wiki links**

  Replace architecture links with relative Markdown links:

  ```markdown
  [水下声光融合 SLAM 平台长期架构设计](./acoustic-optic-slam-platform-architecture-2026-08-17.md)
  [平台架构设计第 22 节](./acoustic-optic-slam-platform-architecture-2026-08-17.md#22-2026-08-18-三方代码库审计与架构细化)
  ```

- [ ] **Step 5: Convert external knowledge-base wiki links**

  For every remaining `[[path|label]]`, preserve `label` as plain text followed by
  `（外部知识库资料）`. Do not create local links for absent files. Confirm removal with:

  ```bash
  if rg -n '\[\[' docs/holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md; then
    exit 1
  fi
  ```

  Expected: exit status `0` and no matches.

### Task 3: Add an entry layer to the approved architecture

**Files:**

- Modify: `docs/acoustic-optic-slam-platform-architecture-2026-08-17.md:1-45`
- Modify: repository-local and external knowledge-base links throughout the same file
- Reference: `README.md`
- Reference: `docs/uw-slam-codebase-reference-2026-08-18.md`

- [ ] **Step 1: Refresh metadata and define authority**

  Keep `status: approved-design`, set `updated: 2026-08-19`, and add
  `implementation_reference: ./uw-slam-codebase-reference-2026-08-18.md`. State that this document
  defines target decisions, while the codebase reference defines what exists today.

- [ ] **Step 2: Add a core-decision summary and current implementation map**

  Summarize the ROS-independent algorithm core, Protobuf contract authority, evidence/factor boundary,
  hybrid probabilistic estimator, versioned submaps, deterministic replay, and staged learning-model
  admission. Add a matrix distinguishing landed vertical-slice pieces, partial adapters, and future
  optical/native-tight-coupling/neural-map work.

- [ ] **Step 3: Add a table of contents for sections 1–22**

  Keep existing section numbering and include all H2 sections. Mark section 22 as a code-audit appendix
  whose corrections take precedence over older factual assumptions in the body.

- [ ] **Step 4: Correct the known branch-version statement**

  In the “团队已经完成” list, change the statement that SVIn was adapted to ROS2 Humble to the audited
  ROS2 Jazzy result. Keep the general C++17/Humble compatibility design goal in section 6 because that
  is a platform compatibility decision, not the audited upstream-branch claim.

- [ ] **Step 5: Convert wiki links using the same repository/external distinction**

  Link the Pipeline plan with a relative Markdown link. Convert the three absent knowledge-base pages
  to labeled plain text with `（外部知识库资料）`. Verify:

  ```bash
  if rg -n '\[\[' docs/acoustic-optic-slam-platform-architecture-2026-08-17.md; then
    exit 1
  fi
  ```

  Expected: exit status `0` and no matches.

### Task 4: Refresh the current-code reference entry

**Files:**

- Modify: `docs/uw-slam-codebase-reference-2026-08-18.md:1-56`
- Modify: stale test and Demo status at existing lines near 1077 and 1224
- Reference: `/tmp/uw_slam_verify/readme_final/summary.txt` when available
- Reference: `tools/verify_pipeline.sh`

- [ ] **Step 1: Add verification metadata and task shortcuts**

  Set `updated: 2026-08-19`, add `verified_commit: 919e1f0`, and state that the document records code
  facts rather than target design. Before the existing full table of contents, add links for finding
  domain types, algorithm implementations, end-to-end flow, configuration, tests/tooling, and known
  boundaries.

- [ ] **Step 2: Update the document relationship table**

  Classify the Pipeline document as an evolving/historical first-stage plan, add `docs/README.md` as
  the documentation router, and keep the root README, architecture, and current reference roles distinct.

- [ ] **Step 3: Correct stale verification results**

  Replace all `13/13` C++ results with `14/14`. Replace the early `~3cm` ATE statements with the current
  typical `0.15–0.22 m` range and explain that online landmark discovery plus fixed-landmark elevation
  assumptions changed the result. Do not present this range as a pass threshold.

- [ ] **Step 4: Confirm stale claims are gone**

  Run:

  ```bash
  if rg -n '13/13|~3cm|第一阶段工程方案（草稿）' \
    docs/uw-slam-codebase-reference-2026-08-18.md; then
    exit 1
  fi
  ```

  Expected: exit status `0` and no matches.

### Task 5: Separate external repository recovery from analysis

**Files:**

- Modify: `external_repos/README.md`
- Modify: `external_repos/external-repos-overview.md:1-66`
- Reference: `NOTICE`
- Reference: `CLAUDE.md`

- [ ] **Step 1: Rewrite `external_repos/README.md` as a recovery guide**

  Keep the read-only warning at the top. Replace repetitive sections with a table containing directory,
  source/access method, license, role, and known snapshot. Put clone commands in a single shell block,
  and keep separate notes for the internal-only `ocean_t` URL and unavailable `holoocean_bridge` source.
  Link detailed analysis to `external-repos-overview.md` and provenance to `../NOTICE`.

- [ ] **Step 2: Add overview metadata, purpose, and depth boundary**

  Add front matter with `type: external-code-audit`, `status: current-reference`,
  `updated: 2026-08-19`, and `verified_against: 919e1f0`. State that all four repositories receive a
  role summary, while only `holoocean-ros` and `holoocean_bridge` receive detailed code walkthroughs.

- [ ] **Step 3: Add navigation and a unified external-source matrix**

  Provide links to the summary, `holoocean-ros` detail, `holoocean_bridge` detail, and recovery guide.
  The matrix must include role, upstream/access, license, known snapshot or availability, `uw_slam`
  usage, and verification boundary for all six directories named by the recovery guide.

- [ ] **Step 4: Normalize detailed-section framing**

  Preserve existing detailed technical content. Ensure both detailed sections visibly contain or link
  to: positioning, key modules, interfaces/data flow, launch reference, relation to `uw_slam`, and known
  limitations. Do not add unsupported launch validation claims.

- [ ] **Step 5: Verify no third-party child changed**

  Run:

  ```bash
  git check-ignore -q external_repos/external-repos-overview.md && exit 1 || true
  git status --short --untracked-files=all -- external_repos
  ```

  Expected: the overview is not ignored, only the two approved top-level Markdown files appear, and
  ignored third-party children do not appear as modified tracked content.

### Task 6: Validate the documentation system

**Files:**

- Verify: `README.md`
- Verify: `docs/README.md`
- Verify: `docs/*.md`
- Verify: `external_repos/README.md`
- Verify: `external_repos/external-repos-overview.md`

- [ ] **Step 1: Inspect all major heading outlines**

  Run:

  ```bash
  for file in README.md docs/README.md docs/*.md external_repos/*.md; do
    printf '\n%s\n' "$file"
    rg -n '^#{1,3} ' "$file"
  done
  ```

  Expected: one H1 per file, an entry/status layer before deep technical sections, and tables of contents
  in the three documents longer than 400 lines.

- [ ] **Step 2: Validate relative Markdown file and heading targets**

  Run this read-only checker from the repository root:

  ```bash
  python3 - <<'PY'
  import pathlib
  import re
  import sys
  from collections import defaultdict

  files = [pathlib.Path("README.md"), *pathlib.Path("docs").glob("*.md"),
           *pathlib.Path("external_repos").glob("*.md")]
  failures = []
  pattern = re.compile(r"\[[^]]*\]\(([^)]+)\)")
  headings = {}
  texts = {}

  def outside_fences(source):
      kept = []
      in_fence = False
      marker = None
      for line in source.read_text(encoding="utf-8").splitlines():
          fence = re.match(r"^\s*(`{3,}|~{3,})", line)
          if fence:
              token = fence.group(1)[0]
              if not in_fence:
                  in_fence, marker = True, token
              elif token == marker:
                  in_fence, marker = False, None
              continue
          if not in_fence:
              kept.append(line)
      return "\n".join(kept)

  def slug(text):
      text = re.sub(r"<[^>]+>", "", text).lower()
      text = re.sub(r"[^\w\- ]", "", text)
      return text.strip().replace(" ", "-")

  for source in files:
      text = outside_fences(source)
      texts[source] = text
      counts = defaultdict(int)
      anchors = set()
      for heading in re.findall(r"^#{1,6}\s+(.+?)\s*$", text, re.M):
          base = slug(heading)
          suffix = counts[base]
          counts[base] += 1
          anchors.add(base if suffix == 0 else f"{base}-{suffix}")
      headings[source.resolve()] = anchors

  for source, text in texts.items():
      for raw in pattern.findall(text):
          target, separator, anchor = raw.partition("#")
          target = target.strip()
          if "://" in target or target.startswith("mailto:"):
              continue
          resolved = (source.parent / target).resolve() if target else source.resolve()
          if not resolved.exists():
              failures.append(f"{source}: missing {raw}")
          elif separator and resolved in headings and anchor not in headings[resolved]:
              failures.append(f"{source}: missing heading {raw}")
  if failures:
      print("\n".join(failures))
      sys.exit(1)
  PY
  ```

  Expected: exit status `0` and no missing targets.

- [ ] **Step 3: Verify link style and status boundaries**

  Run:

  ```bash
  if rg -n '\[\[' docs/*.md; then
    exit 1
  fi
  rg -n 'status: (current|approved-design|evolving-plan|current-reference)' docs/*.md external_repos/*.md
  rg -n '当前|目标|历史|权威' docs/README.md \
    docs/holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md \
    docs/acoustic-optic-slam-platform-architecture-2026-08-17.md
  ```

  Expected: no wiki links; each routed long document exposes its status and authority boundary.

- [ ] **Step 4: Run Markdown whitespace and scope checks**

  Run:

  ```bash
  git diff --check
  git status --short
  git diff --stat
  ```

  Expected: no whitespace errors; changes are limited to the approved Markdown files, existing process
  artifacts, and the exact `.gitignore` exception; no source, license, or third-party child changed.

- [ ] **Step 5: Re-run the repository’s non-ROS2 verification pipeline**

  Run:

  ```bash
  tools/verify_pipeline.sh --out-dir /tmp/uw_slam_verify/docs_reorganization
  ```

  Expected: build, 14 CTest cases, 9 Python tests, architecture lint, synthetic generation, and replay
  all pass. This confirms that documentation work did not disturb the executable workspace.
