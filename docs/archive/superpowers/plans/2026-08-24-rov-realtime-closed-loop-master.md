# ROV Realtime Closed-Loop Master Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a bounded, observable, realtime acoustic-optic driver-assistance chain for BlueROV2 Heavy + SV1213 + AI-D, first with an in-process live source and then through HoloOcean/ROS2.

**Architecture:** Build the realtime ingestion and scheduling substrate first, then add target/track-level fusion and a minimal operator overlay, and finally attach the versioned HoloOcean scenario and ROS2 gateway. Each child plan ends in a runnable vertical slice and a hard gate; real AI-D/SV1213 adapters begin only after vendor artifacts pass the documented acceptance gate.

**Tech Stack:** C++17, Protobuf, Eigen, OpenCV, yaml-cpp, GoogleTest/CTest, Python 3.10+, pytest, HoloOcean 2.3.x, ROS2 Jazzy

---

## 1. Authoritative inputs

- `docs/specifications/rov-competition-online-system-requirements.md`
- `docs/specifications/rov-acoustic-optic-online-fusion-spec.md`
- `docs/specifications/holoocean-realtime-closed-loop-simulation-spec.md`

This plan intentionally excludes DVL, manipulator work, algorithm-issued thruster control, loop closure, global dense SLAM,
and production task-model training. MCAP remains a diagnostic side path and is not a stage gate.

## 2. Execution isolation

The current main worktree contains unrelated user edits. Execute the plans in a dedicated worktree created from
commit `4634cc1` or its descendant. Do not copy or stage the existing `CLAUDE.md`, ROV roadmap, or DOCX changes.

Verification baseline before the first code task:

```bash
git status --short
cmake --build build -j2
ctest --test-dir build --output-on-failure
adapters/holoocean/.venv/bin/python -m pytest -q adapters/holoocean/tests
```

Expected: the worktree is clean; the build succeeds; the existing CTest and Python adapter suites pass.

## 3. Child plans and dependency order

| Order | Plan | Working deliverable | Hard exit gate |
|---:|---|---|---|
| 1 | [Online runtime foundation](./2026-08-24-rov-online-runtime-foundation.md) | A live canonical source with semantic validation, priority queues, bounded overflow, health and shutdown | Mixed-rate events run for 30 minutes with bounded queue depth; reference truth never reaches the algorithm port |
| 2 | [Acoustic-optic online tracking](./2026-08-24-acoustic-optic-online-tracking.md) | Multi-target sonar/visual candidates, nearest-time association, target tracks, degradation and operator overlay | In-process 20/10/50 Hz stream publishes fresh target tracks and overlay; sensor loss produces the specified degradation |
| 3 | [HoloOcean realtime closed loop](./2026-08-24-holoocean-realtime-closed-loop.md) | Versioned BlueROV2/AI-D/SV1213 scene, deterministic randomization/faults, ROS2 gateway and scenario scoring | RTF ≥1.0; nominal 2-hour run and 30-minute overload/fault run satisfy queue, age, recovery and resource gates |

Do not start child plan 2 until child plan 1's complete verification command passes. Do not start child plan 3's
ROS2 integration tasks until child plan 2 publishes `TargetTrackSet` and `OperatorAssistState` in process.

## 4. Hardware SDK acceptance gate

Real-device adapter implementation is a follow-on plan, triggered only when all rows below have evidence. This is
not an invitation to invent APIs while waiting.

| Device | Required evidence before adapter coding |
|---|---|
| AI-D | Native resolution/rate table; left/right hardware synchronization measurement ≤2 ms; timestamp origin; image encoding; calibration file; buildable C++ or ROS sample |
| SV1213 | Purchased frequency/range configuration; raw-frame layout; per-ping timestamp; FOV/bins/range/gain/sound-speed metadata; SDK license; buildable sample |
| BlueROV2 | Attitude, angular velocity, depth, leak, power and link-health messages; timestamp origin; buildable MAVLink/ROS sample |

Store accepted artifacts outside core source and record hashes/versions in a new adapter-specific design before
implementation. Until this gate passes, HoloOcean and fixture adapters exercise the canonical interfaces.

## 5. Stage gates

### Gate G1 — realtime substrate

- `VehicleState` is an algorithm input distinct from `/gt/state`.
- Invalid headers/images/sonar/state are rejected before typed processing.
- Camera/sonar queues drop oldest; localization state rejects overflow and reports it.
- Queue capacity, high-water mark, drops, rejects, oldest age and latency percentiles are observable.
- Shutdown unblocks all waiters and calls `Flush()` once.

### Gate G2 — acoustic-optic assistance

- Sonar output contains every accepted cluster, not only the frame's top candidate.
- A sonar frame selects the nearest corrected-time stereo pair; observation IDs never establish synchronization.
- Corrected delta above 50 ms rejects only cross-modal fusion.
- Track output carries source, age, covariance, observation IDs and discrete health.
- Dense depth is opportunistic and cannot block tracks/health/HMI.
- At 500 ms age, a track leaves normal guidance.

### Gate G3 — simulator closed loop

- Scenario config includes BlueROV2, two 720p AI-D cameras, an independent pilot camera, imaging sonar,
  IMU/orientation and depth at independent rates.
- Pilot or scripted eight-thruster commands pass saturation, deadzone and response-delay models; no command sets pose.
- Same seed repeats layout/noise/fault events; a different seed changes at least one.
- Truth exists only in the scorer.
- Independent pilot video remains available when algorithm overlay/recording is unavailable; the HMI also shows sonar,
  source, confidence, age, health and degradation.
- RTF, end-to-end age, queues, resources, faults and recoveries are reported from the live run.
- Search and structure scenarios produce nonzero sonar, visual and fused tracks under visible conditions.

### Gate G4 — real hardware readiness

- Vendor artifact gate is complete.
- A dedicated adapter plan maps actual vendor fields to canonical contracts with fixture-based tests.
- Dry-bench and pool gates reuse G1/G2 metrics and never route algorithm video as the sole pilot video.

## 6. Requirement coverage map

| Requirement group | Child plan |
|---|---|
| `SYS-HW-*`, `SYS-ARCH-*` | Master hardware gate + runtime foundation + HoloOcean closed loop |
| `SYS-IN-*`, `SYS-RT-*` | Runtime foundation |
| `SYS-PER-*` | Online tracking |
| `SYS-TASK-*` | Online tracking + HoloOcean task/scorer/scripted-pilot loop |
| `FUS-ARCH-*` | Runtime foundation + online tracking |
| `FUS-IN-*`, `FUS-CAM-*`, `FUS-SON-*`, `FUS-STATE-*` | Runtime foundation + online tracking |
| `FUS-CAL-*`, `FUS-SYNC-*`, `FUS-Q-*` | Runtime foundation + online tracking |
| `FUS-VIS-*`, `FUS-AC-*`, `FUS-ASSOC-*`, `FUS-TRACK-*` | Online tracking |
| `FUS-DENSE-*`, `FUS-OUT-*`, `FUS-HEALTH-*`, `FUS-RT-*` | Online tracking |
| `FUS-ACC-*` | Online tracking + HoloOcean closed loop |
| `SIM-ARCH-*`, `SIM-CFG-*`, `SIM-ROV-*`, `SIM-CAM-*`, `SIM-SON-*`, `SIM-STATE-*` | HoloOcean closed loop |
| `SIM-TIME-*`, `SIM-SCENE-*`, `SIM-FAULT-*`, `SIM-ACC-*` | HoloOcean closed loop |
| `SYS-HMI-*`, `SYS-DEG-*`, `SYS-ACC-*` | Online tracking + HoloOcean closed loop |
| `SYS-PROC-*` | Hardware SDK gate and subsequent adapter plan |
| `SIM-S2R-001` | Triggered after first valid pool dataset; not claimable from simulation-only execution |

## 7. Program-level verification

After all three child plans:

```bash
cmake --build build -j2
ctest --test-dir build --output-on-failure
adapters/holoocean/.venv/bin/python -m pytest -q adapters/holoocean/tests
tools/lint/check_layer_dependencies.py .
```

On the Windows/ROS2 simulator host, run the child plan's nominal and fault scripts. Attach the produced manifest,
metrics JSON, task score, health timeline and HMI recording to the release evidence. Do not state that G3 passes
from Linux unit tests alone.
