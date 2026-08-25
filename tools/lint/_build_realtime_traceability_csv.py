"""One-off generator for docs/traceability/rov-realtime-closed-loop.csv.

Not part of the shipped tool surface (check_realtime_traceability.py is the
thing that actually gets run/tested going forward) -- this script exists so
the 125-row mapping used to populate the CSV is reviewable as code rather
than opaque hand-typed rows, and can be regenerated if new requirement IDs
are added to the three spec files later. Delete-safe: nothing else imports
this module.
"""
from __future__ import annotations

import csv
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

SPEC_FILES = {
    "SYS": ROOT / "docs/specifications/rov-competition-online-system-requirements.md",
    "FUS": ROOT / "docs/specifications/rov-acoustic-optic-online-fusion-spec.md",
    "SIM": ROOT / "docs/specifications/holoocean-realtime-closed-loop-simulation-spec.md",
}

ID_RE = re.compile(r"`((?:SYS|FUS|SIM)-[A-Z0-9]+-\d{3})`")

BOTH_TASKS = "aquaculture_search+structure_inspection"
ARCH = "n/a (architecture)"
NOMINAL = "rov_realtime_nominal"
MINIMUM = "rov_realtime_minimum"
OVERLOAD = "rov_realtime_overload"
DISTURBED = "rov_realtime_disturbed"
ALL_PROFILES = "rov_realtime_minimum+nominal+disturbed+overload"

# (scenario, implementation_module, test, evidence_path, status)
ROWS: dict[str, tuple[str, str, str, str, str]] = {
    # ---- SYS-HW: hardware framing constraints ----
    "SYS-HW-001": (
        ARCH,
        "adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json",
        "adapters/holoocean/tests/test_scenario_manifest.py",
        "adapters/holoocean/tests/test_scenario_manifest.py",
        "implemented",
    ),
    "SYS-HW-002": (
        ARCH,
        "adapters/ros2/src/holoocean_realtime_node.cpp (independent PilotCamera subscription)",
        "n/a (native ROS2 host needed to observe the raw pilot feed surviving an algorithm-process failure)",
        "adapters/ros2/README.md",
        "gated",
    ),
    # ---- SYS-TASK ----
    "SYS-TASK-001": (
        BOTH_TASKS,
        "adapters/holoocean/scenarios/{aquaculture_search,structure_inspection}.yaml",
        "adapters/holoocean/tests/test_scenario_manifest.py",
        "adapters/holoocean/tests/test_scenario_manifest.py",
        "verified",
    ),
    "SYS-TASK-002": (
        ARCH,
        "adapters/holoocean/uw_holoocean_adapter/scenario_manifest.py (TaskSpec is extensible per-task config, not hardcoded to these two)",
        "adapters/holoocean/tests/test_scenario_manifest.py",
        "adapters/holoocean/tests/test_scenario_manifest.py",
        "implemented",
    ),
    # ---- SYS-ARCH ----
    "SYS-ARCH-001": (
        ARCH,
        "adapters/holoocean/uw_holoocean_adapter/scripted_pilot.py (explicitly a sim-test driver, not production autonomy)",
        "adapters/holoocean/tests/test_scripted_pilot.py",
        "adapters/holoocean/uw_holoocean_adapter/scripted_pilot.py",
        "implemented",
    ),
    "SYS-ARCH-002": (
        ARCH,
        "tools/lint/check_layer_dependencies.py (ros2 role ALLOWED set)",
        "tests/lint/check_layer_dependencies_test.py",
        "tools/lint/check_layer_dependencies.py",
        "verified",
    ),
    "SYS-ARCH-003": (
        ARCH,
        "adapters/holoocean/uw_holoocean_adapter/task_scorer.py (TaskScorer is the only truth consumer)",
        "adapters/holoocean/tests/test_task_scorer.py::test_scorer_consumes_truth_but_algorithm_topic_list_does_not",
        "adapters/holoocean/uw_holoocean_adapter/task_scorer.py",
        "verified",
    ),
    "SYS-ARCH-004": (
        ARCH,
        "adapters/holoocean/uw_holoocean_adapter/async_diagnostic_recorder.py",
        "adapters/holoocean/tests/test_async_diagnostic_recorder.py::test_blocked_or_failed_recorder_never_blocks_live_submit",
        "adapters/holoocean/uw_holoocean_adapter/async_diagnostic_recorder.py",
        "verified",
    ),
    # ---- SYS-IN ----
    "SYS-IN-001": (
        ARCH,
        "adapters/ros2/src/holoocean_realtime_node.cpp (subscribes AI-D L/R + SV1213 sonar + VehicleState concurrently)",
        "n/a (native host needed to observe concurrent live ingestion)",
        "adapters/ros2/README.md",
        "gated",
    ),
    "SYS-IN-002": (
        ARCH,
        "include/adapters/holoocean_live_conversion.hpp (ConvertHoloImage/ConvertHoloSonar/ConvertHoloVehicleState header population)",
        "tests/adapters/holoocean_live_conversion_test.cpp",
        "tests/adapters/holoocean_live_conversion_test.cpp",
        "verified",
    ),
    "SYS-IN-003": (
        ARCH,
        "include/runtime/canonical_event_validation.hpp",
        "tests/runtime/canonical_event_validation_test.cpp",
        "tests/runtime/canonical_event_validation_test.cpp",
        "verified",
    ),
    # ---- SYS-PER ----
    "SYS-PER-001": (
        ARCH,
        "include/frontends/target_tracker.hpp, schemas/proto/uw/domain/target.proto (TargetTrack)",
        "tests/frontends/target_tracker_test.cpp",
        "tests/frontends/target_tracker_test.cpp",
        "verified",
    ),
    "SYS-PER-002": (
        ARCH,
        "include/application/online_assist_pipeline.hpp (DenseDepthProvider, dense.enabled gate)",
        "tests/application/online_assist_pipeline_test.cpp",
        "tests/application/online_assist_pipeline_test.cpp",
        "verified",
    ),
    "SYS-PER-003": (
        ARCH,
        "adapters/opencv/include/adapters/operator_overlay_renderer.hpp (SOURCE label: ACOUSTIC_OPTIC/VISUAL/SONAR)",
        "tests/adapters/operator_overlay_renderer_test.cpp",
        "tests/adapters/operator_overlay_renderer_test.cpp",
        "verified",
    ),
    # ---- SYS-HMI ----
    "SYS-HMI-001": (
        ARCH,
        "adapters/opencv/include/adapters/operator_overlay_renderer.hpp; src/application/holoocean_realtime_sink.cpp (BuildStatusJson)",
        "tests/adapters/operator_overlay_renderer_test.cpp",
        "tests/adapters/operator_overlay_renderer_test.cpp",
        "verified",
    ),
    "SYS-HMI-002": (
        ARCH,
        "include/frontends/target_tracker.hpp (500ms staleness -> TARGET_TRACK_STATUS_STALE)",
        "tests/frontends/target_tracker_test.cpp",
        "tests/frontends/target_tracker_test.cpp",
        "verified",
    ),
    # ---- SYS-DEG ----
    "SYS-DEG-001": (
        ARCH,
        "include/application/online_assist_pipeline.hpp (ComputeDegradation: visual_unavailable/sonar_unavailable)",
        "tests/application/online_assist_pipeline_test.cpp",
        "tests/application/online_assist_pipeline_test.cpp",
        "verified",
    ),
    "SYS-DEG-002": (
        ARCH,
        "include/application/online_assist_pipeline.hpp (vehicle_state_stale_after_s gate)",
        "tests/application/online_assist_pipeline_test.cpp",
        "tests/application/online_assist_pipeline_test.cpp",
        "verified",
    ),
    "SYS-DEG-003": (
        ARCH,
        "adapters/ros2/src/holoocean_realtime_node.cpp (independent PilotCamera path survives assist-pipeline failure); src/application/holoocean_realtime_sink.cpp (pump-thread exception no longer silent)",
        "n/a (native host needed to prove pure-teleop fallback end to end)",
        "adapters/ros2/README.md",
        "gated",
    ),
    # ---- SYS-RT ----
    "SYS-RT-001": (
        ARCH,
        "include/runtime/bounded_queue.hpp, include/runtime/live_event_source.hpp",
        "tests/runtime/live_event_source_test.cpp",
        "tests/runtime/live_event_source_test.cpp",
        "verified",
    ),
    "SYS-RT-002": (
        NOMINAL,
        "include/runtime/live_event_source.hpp (drop-oldest overflow policy)",
        "tests/runtime/live_event_source_test.cpp",
        "tests/runtime/live_event_source_test.cpp",
        "verified",
    ),
    "SYS-RT-003": (
        NOMINAL,
        "adapters/holoocean/uw_holoocean_adapter/run_report.py (rss_growth_after_warmup_mib gate)",
        "adapters/holoocean/tests/test_run_report.py",
        "adapters/holoocean/tests/test_run_report.py",
        "implemented",
    ),
    "SYS-RT-004": (
        NOMINAL,
        "adapters/holoocean/uw_holoocean_adapter/run_report.py (cpu/gpu headroom fields)",
        "n/a (native host needed for a real CPU/GPU sample)",
        "adapters/holoocean/uw_holoocean_adapter/run_report.py",
        "gated",
    ),
    "SYS-RT-005": (
        ARCH,
        "include/application/online_assist_pipeline.hpp (3-period activity timeout -> degraded; recovery resets cache)",
        "tests/application/online_assist_pipeline_test.cpp",
        "tests/application/online_assist_pipeline_test.cpp",
        "verified",
    ),
    # ---- SYS-ACC ----
    "SYS-ACC-001": (
        BOTH_TASKS,
        "adapters/holoocean/uw_holoocean_adapter/task_scorer.py (precision/recall)",
        "n/a (native host, real frozen acceptance scenes needed for a real 90% number)",
        "adapters/holoocean/uw_holoocean_adapter/task_scorer.py",
        "gated",
    ),
    "SYS-ACC-002": (
        BOTH_TASKS,
        "adapters/holoocean/uw_holoocean_adapter/task_scorer.py (bearing_error_p95_rad/range_error_p95_m)",
        "adapters/holoocean/tests/test_task_scorer.py",
        "adapters/holoocean/uw_holoocean_adapter/task_scorer.py",
        "implemented",
    ),
    "SYS-ACC-003": (
        "structure_inspection",
        "adapters/holoocean/uw_holoocean_adapter/task_scorer.py (lateral_offset_p95_m)",
        "adapters/holoocean/tests/test_task_scorer.py",
        "adapters/holoocean/uw_holoocean_adapter/task_scorer.py",
        "implemented",
    ),
    "SYS-ACC-004": (
        BOTH_TASKS,
        "adapters/holoocean/uw_holoocean_adapter/realtime_gate.py",
        "n/a (native host, 10-seed campaign)",
        "adapters/holoocean/uw_holoocean_adapter/realtime_gate.py",
        "gated",
    ),
    "SYS-ACC-005": (
        DISTURBED,
        "adapters/holoocean/uw_holoocean_adapter/realtime_gate.py",
        "n/a (native host, 10-seed disturbed campaign)",
        "adapters/holoocean/uw_holoocean_adapter/realtime_gate.py",
        "gated",
    ),
    # ---- SYS-PROC: hardware SDK gate, explicitly out of this plan's reach ----
    "SYS-PROC-001": ("n/a (hardware)", "AI-D hardware SDK gate", "n/a", "docs/superpowers/plans/2026-08-24-rov-realtime-closed-loop-master.md", "gated"),
    "SYS-PROC-002": ("n/a (hardware)", "SV1213 hardware SDK gate", "n/a", "docs/superpowers/plans/2026-08-24-rov-realtime-closed-loop-master.md", "gated"),
    "SYS-PROC-003": ("n/a (hardware)", "BlueROV2 hardware interface gate", "n/a", "docs/superpowers/plans/2026-08-24-rov-realtime-closed-loop-master.md", "gated"),
    "SYS-PROC-004": ("n/a (hardware)", "hardware device acceptance policy", "n/a", "docs/superpowers/plans/2026-08-24-rov-realtime-closed-loop-master.md", "gated"),
}


def _fus_state(pfx: str, scenario: str, module: str, test: str, evidence: str, status: str):
    ROWS[pfx] = (scenario, module, test, evidence, status)


# ---- FUS-ARCH ----
_fus_state("FUS-ARCH-001", ARCH, "measurement_api/{frontend,providers}.hpp interfaces", "tests/frontends/*, tests/application/online_assist_pipeline_test.cpp", "include/measurement_api/frontend.hpp", "verified")
_fus_state("FUS-ARCH-002", ARCH, "tools/lint/check_layer_dependencies.py", "tests/lint/check_layer_dependencies_test.py", "tools/lint/check_layer_dependencies.py", "verified")
_fus_state("FUS-ARCH-003", ARCH, "include/application/online_assist_pipeline.hpp consumes CanonicalEvent regardless of source", "tests/application/online_assist_pipeline_test.cpp, apps/online_assist_smoke.cpp", "include/runtime/event_source.hpp", "verified")

# ---- FUS-IN ----
_fus_state("FUS-IN-001", ARCH, "schemas/proto/uw/domain/observation.proto (ObservationHeader)", "tests/runtime/canonical_event_validation_test.cpp", "schemas/proto/uw/domain/observation.proto", "verified")
_fus_state("FUS-IN-002", ARCH, "schemas/proto/uw/domain/ids.proto (ObservationId is identity-only)", "tests/frontends/target_associator_test.cpp (provenance-uniqueness rejection)", "tests/frontends/target_associator_test.cpp", "verified")
_fus_state("FUS-IN-003", ARCH, "schemas/proto/uw/domain/time.proto (capture_time vs receive_time)", "tests/runtime/canonical_event_validation_test.cpp", "schemas/proto/uw/domain/time.proto", "verified")

# ---- FUS-CAM ----
_fus_state("FUS-CAM-001", ARCH, "schemas/proto/uw/domain/image.proto, calibration.proto", "tests/contracts/*", "schemas/proto/uw/domain/image.proto", "implemented")
_fus_state("FUS-CAM-002", ARCH, "50ms/2ms sync gates -- FUS-SYNC-002 owns the runtime check", "tests/runtime/acoustic_optic_buffer_test.cpp", "include/runtime/acoustic_optic_buffer.hpp", "verified")
_fus_state("FUS-CAM-003", ARCH, "adapters/opencv (camera_rectifier is a distinct opt-in stage from raw ImageFrame)", "tests/adapters/opencv_stereo_rectifier_test.cpp", "tests/adapters/opencv_stereo_rectifier_test.cpp", "implemented")

# ---- FUS-SON ----
_fus_state("FUS-SON-001", ARCH, "schemas/proto/uw/domain/sonar.proto (SonarFrame geometry fields)", "tests/runtime/canonical_event_validation_test.cpp", "schemas/proto/uw/domain/sonar.proto", "verified")
_fus_state("FUS-SON-002", ARCH, "include/runtime/canonical_event_validation.hpp (finite/monotonic geometry checks)", "tests/runtime/canonical_event_validation_test.cpp", "tests/runtime/canonical_event_validation_test.cpp", "verified")

# ---- FUS-STATE ----
_fus_state("FUS-STATE-001", ARCH, "schemas/proto/uw/domain/state.proto (VehicleState: attitude/angular velocity/depth + valid flags)", "tests/contracts/*", "schemas/proto/uw/domain/state.proto", "implemented")
_fus_state("FUS-STATE-002", ARCH, "include/adapters/holoocean_live_conversion.hpp (ConvertHoloVehicleState: attitude_valid/depth_valid only, device_health_valid left false rather than fabricated)", "tests/adapters/holoocean_live_conversion_test.cpp", "src/adapters/holoocean_live_conversion.cpp", "verified")

# ---- FUS-CAL ----
_fus_state("FUS-CAL-001", ARCH, "schemas/proto/uw/domain/calibration.proto (RigCalibrationSnapshot frame_tree)", "tests/contracts/*, apps/online_assist_smoke.cpp (BuildRig)", "schemas/proto/uw/domain/calibration.proto", "implemented")
_fus_state("FUS-CAL-002", ARCH, "RigCalibrationSnapshot.calibration_version + time_offset_provenance", "apps/online_assist_smoke.cpp, adapters/ros2/src/holoocean_realtime_node.cpp (BuildIdentityRig)", "adapters/ros2/src/holoocean_realtime_node.cpp", "implemented")
_fus_state("FUS-CAL-003", ARCH, "include/application/online_assist_pipeline.hpp (calibration-version-change reset+recovering)", "tests/application/online_assist_pipeline_test.cpp", "tests/application/online_assist_pipeline_test.cpp", "verified")

# ---- FUS-SYNC ----
_fus_state("FUS-SYNC-001", ARCH, "include/runtime/acoustic_optic_buffer.hpp (nearest corrected capture-time match)", "tests/runtime/acoustic_optic_buffer_test.cpp", "tests/runtime/acoustic_optic_buffer_test.cpp", "verified")
_fus_state("FUS-SYNC-002", ARCH, "include/runtime/acoustic_optic_buffer.hpp (sync window rejection)", "tests/runtime/acoustic_optic_buffer_test.cpp", "tests/runtime/acoustic_optic_buffer_test.cpp", "verified")
_fus_state("FUS-SYNC-003", ARCH, "include/frontends/target_associator.hpp (state interpolation to sensor capture time)", "tests/frontends/target_associator_test.cpp", "tests/frontends/target_associator_test.cpp", "verified")
_fus_state("FUS-SYNC-004", ARCH, "include/runtime/acoustic_optic_buffer.hpp (candidate/accepted/no-match/out-of-window/invalid-timestamp counters)", "tests/runtime/acoustic_optic_buffer_test.cpp", "tests/runtime/acoustic_optic_buffer_test.cpp", "verified")

# ---- FUS-Q ----
_fus_state("FUS-Q-001", ARCH, "include/runtime/live_event_source.hpp (LaneQueueConfig capacity/overflow_policy)", "tests/runtime/live_event_source_test.cpp", "tests/runtime/live_event_source_test.cpp", "verified")
_fus_state("FUS-Q-002", ARCH, "include/runtime/live_event_source.hpp (kDropOldest lanes for correction/mapping)", "tests/runtime/live_event_source_test.cpp", "tests/runtime/live_event_source_test.cpp", "verified")
_fus_state("FUS-Q-003", ARCH, "include/runtime/live_event_source.hpp (localization lane kReject, never silently dropped)", "tests/runtime/live_event_source_test.cpp", "tests/runtime/live_event_source_test.cpp", "verified")
_fus_state("FUS-Q-004", ARCH, "include/runtime/rolling_latency.hpp + LiveSourceStats age accounting", "tests/runtime/live_event_source_test.cpp", "tests/runtime/live_event_source_test.cpp", "implemented")
_fus_state("FUS-Q-005", ARCH, "include/runtime/live_event_source.hpp (weighted lane scheduler: localization/correction/mapping/evidence)", "tests/runtime/live_event_source_test.cpp", "tests/runtime/live_event_source_test.cpp", "verified")

# ---- FUS-VIS ----
_fus_state("FUS-VIS-001", ARCH, "adapters/opencv/include/adapters/opencv_visual_assist_frontend.hpp", "tests/adapters/opencv_visual_assist_frontend_test.cpp", "tests/adapters/opencv_visual_assist_frontend_test.cpp", "verified")
_fus_state("FUS-VIS-002", ARCH, "include/application/online_assist_pipeline.hpp (dense-depth gating: texture/uniqueness/lr-consistency/disparity/occlusion/calibration)", "tests/application/online_assist_pipeline_test.cpp", "tests/application/online_assist_pipeline_test.cpp", "verified")
_fus_state("FUS-VIS-003", ARCH, "adapters/opencv/include/adapters/opencv_visual_assist_frontend.hpp (degradation reason codes)", "tests/adapters/opencv_visual_assist_frontend_test.cpp", "tests/adapters/opencv_visual_assist_frontend_test.cpp", "implemented")

# ---- FUS-AC ----
_fus_state("FUS-AC-001", ARCH, "include/frontends/sonar_target_extractor.hpp (keeps every cluster, not just the strongest)", "tests/frontends/sonar_target_extractor_test.cpp", "tests/frontends/sonar_target_extractor_test.cpp", "verified")
_fus_state("FUS-AC-002", ARCH, "include/frontends/cfar_detector.hpp, dbscan.hpp, sonar_cfar_frontend.hpp (versioned config structs)", "tests/frontends/*", "include/frontends/sonar_cfar_frontend.hpp", "verified")
_fus_state("FUS-AC-003", ARCH, "include/frontends/sonar_cfar_frontend.hpp (HealthReport: background noise, measurement count, latency)", "tests/frontends/*", "include/frontends/sonar_cfar_frontend.hpp", "implemented")

# ---- FUS-ASSOC ----
_fus_state("FUS-ASSOC-001", ARCH, "include/frontends/target_associator.hpp (time+geometry+class+continuity+depth-agreement gating)", "tests/frontends/target_associator_test.cpp", "tests/frontends/target_associator_test.cpp", "verified")
_fus_state("FUS-ASSOC-002", ARCH, "include/frontends/target_associator.hpp (multi-candidate greedy assignment; new/merge/unmatched handling)", "tests/frontends/target_associator_test.cpp", "tests/frontends/target_associator_test.cpp", "verified")
_fus_state("FUS-ASSOC-003", ARCH, "include/frontends/target_associator.hpp (AssociationDecision reason_code + threshold echo)", "tests/frontends/target_associator_test.cpp", "tests/frontends/target_associator_test.cpp", "verified")

# ---- FUS-TRACK ----
_fus_state("FUS-TRACK-001", ARCH, "schemas/proto/uw/domain/target.proto (TargetTrack fields)", "tests/frontends/target_tracker_test.cpp", "schemas/proto/uw/domain/target.proto", "verified")
_fus_state("FUS-TRACK-002", ARCH, "include/frontends/target_tracker.hpp (sources-aware confirm/degrade)", "tests/frontends/target_tracker_test.cpp", "tests/frontends/target_tracker_test.cpp", "verified")
_fus_state("FUS-TRACK-003", ARCH, "include/frontends/target_tracker.hpp (500ms -> STALE, exits normal guidance)", "tests/frontends/target_tracker_test.cpp", "tests/frontends/target_tracker_test.cpp", "verified")

# ---- FUS-DENSE ----
_fus_state("FUS-DENSE-001", ARCH, "include/application/online_assist_pipeline.hpp (dense.enabled + all gates)", "tests/application/online_assist_pipeline_test.cpp", "tests/application/online_assist_pipeline_test.cpp", "verified")
_fus_state("FUS-DENSE-002", ARCH, "include/application/online_assist_pipeline.hpp (dense refines region, does not replace visual/sonar identity)", "tests/application/online_assist_pipeline_test.cpp", "tests/application/online_assist_pipeline_test.cpp", "implemented")
_fus_state("FUS-DENSE-003", ARCH, "include/application/online_assist_pipeline.hpp (DenseCurrentlyFresh -- deadline-missed never blocks track/state/health)", "tests/application/online_assist_pipeline_test.cpp", "tests/application/online_assist_pipeline_test.cpp", "verified")
_fus_state("FUS-DENSE-004", ARCH, "dense stays default-disabled repo-wide (configs/defaults/platform.yaml online_assist.dense.enabled=false)", "n/a (no measured improvement study exists yet)", "configs/defaults/platform.yaml", "gated")

# ---- FUS-OUT ----
_fus_state("FUS-OUT-001", ARCH, "schemas/proto/uw/domain/target.proto (OperatorAssistState)", "tests/application/online_assist_pipeline_test.cpp", "schemas/proto/uw/domain/target.proto", "verified")
_fus_state("FUS-OUT-002", ARCH, "include/application/latest_assist_sink.hpp (replace-latest, mutex-guarded)", "tests/application/*", "include/application/latest_assist_sink.hpp", "implemented")
_fus_state("FUS-OUT-003", ARCH, "schemas/proto/uw/domain/target.proto (TargetTrackStatus/HealthReport.Status enums, not raw confidence)", "tests/frontends/target_tracker_test.cpp", "schemas/proto/uw/domain/target.proto", "verified")
_fus_state("FUS-OUT-004", ARCH, "adapters/ros2/src/holoocean_realtime_node.cpp (PilotCamera subscription independent of overlay compositor)", "n/a (native host needed to prove decoupled failure behavior)", "adapters/ros2/README.md", "gated")

# ---- FUS-HEALTH ----
_fus_state("FUS-HEALTH-001", ARCH, "include/application/online_assist_pipeline.hpp (3-period timeout -> degraded)", "tests/application/online_assist_pipeline_test.cpp", "tests/application/online_assist_pipeline_test.cpp", "verified")
_fus_state("FUS-HEALTH-002", ARCH, "include/application/online_assist_pipeline.hpp (recovering clears sync cache, re-associates)", "tests/application/online_assist_pipeline_test.cpp", "tests/application/online_assist_pipeline_test.cpp", "verified")
_fus_state("FUS-HEALTH-003", ARCH, "schemas/proto/uw/domain/health.proto (HealthReport fields)", "tests/application/online_assist_pipeline_test.cpp", "schemas/proto/uw/domain/health.proto", "verified")

# ---- FUS-RT ----
_fus_state("FUS-RT-001", ALL_PROFILES, "adapters/holoocean/uw_holoocean_adapter/run_report.py (minimum/nominal/overload gate specs)", "adapters/holoocean/tests/test_run_report.py", "adapters/holoocean/uw_holoocean_adapter/run_report.py", "implemented")
_fus_state("FUS-RT-002", ALL_PROFILES, "adapters/holoocean/uw_holoocean_adapter/run_report.py", "adapters/holoocean/tests/test_run_report.py", "adapters/holoocean/uw_holoocean_adapter/run_report.py", "verified")
_fus_state("FUS-RT-003", ALL_PROFILES, "adapters/holoocean/uw_holoocean_adapter/run_report.py", "adapters/holoocean/tests/test_run_report.py", "adapters/holoocean/uw_holoocean_adapter/run_report.py", "verified")
_fus_state("FUS-RT-004", ALL_PROFILES, "adapters/holoocean/uw_holoocean_adapter/run_report.py", "adapters/holoocean/tests/test_run_report.py", "adapters/holoocean/uw_holoocean_adapter/run_report.py", "verified")

# ---- FUS-ACC ----
_fus_state("FUS-ACC-001", NOMINAL, "apps/online_assist_smoke.cpp (fused_tracks>0, real frontends not a counting stub)", "tests/integration/online_assist_smoke_test.sh", "tests/integration/online_assist_smoke_test.sh", "verified")
_fus_state("FUS-ACC-002", ALL_PROFILES, "adapters/holoocean/uw_holoocean_adapter/run_report.py", "adapters/holoocean/tests/test_run_report.py", "adapters/holoocean/uw_holoocean_adapter/run_report.py", "implemented")
_fus_state("FUS-ACC-003", NOMINAL + "+" + OVERLOAD, "adapters/holoocean/uw_holoocean_adapter/realtime_gate.py", "n/a (native host, 2h/30min campaigns)", "adapters/holoocean/uw_holoocean_adapter/realtime_gate.py", "gated")
_fus_state("FUS-ACC-004", ARCH, "docs/testing-and-verification-guide-2026-08-20.md (MCAP replay does not substitute for online gates)", "tests/integration/*", "tests/integration/live_ingress_smoke_test.sh", "implemented")

# ---- SIM-ARCH ----
_fus_state("SIM-ARCH-001", ARCH, "include/adapters/holoocean_live_conversion.hpp (same CanonicalEvent contract as real hardware)", "tests/adapters/holoocean_live_conversion_test.cpp", "tests/adapters/holoocean_live_conversion_test.cpp", "verified")
_fus_state("SIM-ARCH-002", ARCH, "adapters/holoocean/uw_holoocean_adapter/task_scorer.py (only scorer sees truth)", "adapters/holoocean/tests/test_task_scorer.py", "adapters/holoocean/uw_holoocean_adapter/task_scorer.py", "verified")
_fus_state("SIM-ARCH-003", ARCH, "adapters/holoocean/uw_holoocean_adapter/async_diagnostic_recorder.py", "adapters/holoocean/tests/test_async_diagnostic_recorder.py", "adapters/holoocean/uw_holoocean_adapter/async_diagnostic_recorder.py", "verified")

# ---- SIM-CFG ----
_fus_state("SIM-CFG-001", BOTH_TASKS, "adapters/holoocean/uw_holoocean_adapter/scenario_manifest.py", "adapters/holoocean/tests/test_scenario_manifest.py", "adapters/holoocean/uw_holoocean_adapter/scenario_manifest.py", "verified")
_fus_state("SIM-CFG-002", BOTH_TASKS, "adapters/holoocean/uw_holoocean_adapter/scenario_manifest.py (load_realtime_manifest never accepts a bare name)", "adapters/holoocean/tests/test_scenario_manifest.py", "adapters/holoocean/uw_holoocean_adapter/scenario_manifest.py", "verified")
_fus_state("SIM-CFG-003", ALL_PROFILES, "adapters/holoocean/uw_holoocean_adapter/run_report.py (code/scenario/task/config/calibration hashes)", "adapters/holoocean/tests/test_run_report.py", "adapters/holoocean/uw_holoocean_adapter/run_report.py", "implemented")

# ---- SIM-ROV ----
_fus_state("SIM-ROV-001", BOTH_TASKS, "adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json (8-thruster, actuator bounds)", "adapters/holoocean/tests/test_scenario_manifest.py", "adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json", "verified")
_fus_state("SIM-ROV-002", BOTH_TASKS, "adapters/holoocean/uw_holoocean_adapter/pilot_command_model.py", "adapters/holoocean/tests/test_pilot_command_model.py", "adapters/holoocean/uw_holoocean_adapter/pilot_command_model.py", "verified")
_fus_state("SIM-ROV-003", DISTURBED, "adapters/holoocean/uw_holoocean_adapter/fault_injector.py (current baseline) + sensor_perturbation.py", "adapters/holoocean/tests/test_fault_injector.py, test_sensor_perturbation.py", "adapters/holoocean/uw_holoocean_adapter/fault_injector.py", "implemented")

# ---- SIM-CAM ----
_fus_state("SIM-CAM-001", MINIMUM, "adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json + configs/experiment/rov_realtime_minimum.yaml", "adapters/holoocean/tests/test_scenario_manifest.py", "adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json", "implemented")
_fus_state("SIM-CAM-002", MINIMUM, "adapters/holoocean/uw_holoocean_adapter/fault_injector.py (camera-sonar desync via per-topic clock_offset_s)", "adapters/holoocean/tests/test_fault_injector.py", "adapters/holoocean/uw_holoocean_adapter/fault_injector.py", "implemented")
_fus_state("SIM-CAM-003", DISTURBED, "adapters/holoocean/uw_holoocean_adapter/sensor_perturbation.py", "adapters/holoocean/tests/test_sensor_perturbation.py", "adapters/holoocean/uw_holoocean_adapter/sensor_perturbation.py", "verified")

# ---- SIM-SON ----
_fus_state("SIM-SON-001", BOTH_TASKS, "include/adapters/holoocean_live_conversion.hpp (HoloOceanSonarCalibration, manifest-driven not hardcoded)", "tests/adapters/holoocean_live_conversion_test.cpp", "tests/adapters/holoocean_live_conversion_test.cpp", "verified")
_fus_state("SIM-SON-002", ALL_PROFILES, "adapters/holoocean/uw_holoocean_adapter/realtime_ros_session.py (per-sensor Hz)", "adapters/holoocean/tests/test_realtime_ros_session.py", "adapters/holoocean/uw_holoocean_adapter/realtime_ros_session.py", "implemented")
_fus_state("SIM-SON-003", DISTURBED, "adapters/holoocean/uw_holoocean_adapter/sensor_perturbation.py (perturb_sonar)", "adapters/holoocean/tests/test_sensor_perturbation.py", "adapters/holoocean/uw_holoocean_adapter/sensor_perturbation.py", "verified")

# ---- SIM-STATE ----
_fus_state("SIM-STATE-001", ALL_PROFILES, "adapters/holoocean/uw_holoocean_adapter/realtime_ros_session.py (VehicleOrientation/IMUSensor/DepthSensor at manifest Hz)", "adapters/holoocean/tests/test_realtime_ros_session.py", "adapters/holoocean/uw_holoocean_adapter/realtime_ros_session.py", "implemented")
_fus_state("SIM-STATE-002", BOTH_TASKS, "adapters/holoocean/uw_holoocean_adapter/ros_message_conversion.py (vehicle_state_to_odometry: noisy, never PoseSensor)", "adapters/holoocean/tests/test_ros_message_conversion.py", "adapters/holoocean/uw_holoocean_adapter/ros_message_conversion.py", "verified")

# ---- SIM-TIME ----
_fus_state("SIM-TIME-001", ARCH, "include/adapters/holoocean_live_conversion.hpp (ObservationHeader population)", "tests/adapters/holoocean_live_conversion_test.cpp", "tests/adapters/holoocean_live_conversion_test.cpp", "verified")
_fus_state("SIM-TIME-002", ALL_PROFILES, "adapters/holoocean/uw_holoocean_adapter/realtime_ros_session.py (independent per-sensor rates, no cross-sensor tick alignment requirement)", "adapters/holoocean/tests/test_realtime_ros_session.py", "adapters/holoocean/uw_holoocean_adapter/realtime_ros_session.py", "implemented")
_fus_state("SIM-TIME-003", BOTH_TASKS, "adapters/holoocean/uw_holoocean_adapter/holoocean_driver.py (owned RNG + global random seed/restore) + fault_injector.build_fault_schedule", "adapters/holoocean/tests/test_holoocean_driver.py, test_fault_injector.py", "adapters/holoocean/uw_holoocean_adapter/holoocean_driver.py", "verified")
_fus_state("SIM-TIME-004", BOTH_TASKS, "adapters/holoocean/uw_holoocean_adapter/scenario_randomization.py + fault_injector.py", "adapters/holoocean/tests/test_scenario_randomization.py::test_same_seed_samples_same_layout_noise_and_faults, test_fault_injector.py", "adapters/holoocean/uw_holoocean_adapter/scenario_randomization.py", "verified")
_fus_state("SIM-TIME-005", ALL_PROFILES, "adapters/holoocean/uw_holoocean_adapter/run_report.py (rtf_p50/rtf_p95 fields + gate)", "n/a (native host needed for a real RTF sample)", "adapters/holoocean/uw_holoocean_adapter/run_report.py", "gated")

# ---- SIM-SCENE ----
_fus_state("SIM-SCENE-001", "aquaculture_search", "adapters/holoocean/scenarios/aquaculture_search.yaml", "adapters/holoocean/tests/test_scenario_manifest.py", "adapters/holoocean/scenarios/aquaculture_search.yaml", "verified")
_fus_state("SIM-SCENE-002", "aquaculture_search", "adapters/holoocean/uw_holoocean_adapter/task_scorer.py (_kind == 'point' success path)", "adapters/holoocean/tests/test_task_scorer.py", "adapters/holoocean/uw_holoocean_adapter/task_scorer.py", "implemented")
_fus_state("SIM-SCENE-003", "structure_inspection", "adapters/holoocean/scenarios/structure_inspection.yaml", "adapters/holoocean/tests/test_scenario_manifest.py", "adapters/holoocean/scenarios/structure_inspection.yaml", "verified")
_fus_state("SIM-SCENE-004", "structure_inspection", "adapters/holoocean/uw_holoocean_adapter/task_scorer.py (_kind == 'path' success path)", "adapters/holoocean/tests/test_task_scorer.py", "adapters/holoocean/uw_holoocean_adapter/task_scorer.py", "implemented")
_fus_state("SIM-SCENE-005", BOTH_TASKS, "adapters/holoocean/scenarios/*.yaml (nominal) + configs/experiment/rov_realtime_disturbed.yaml (disturbed)", "adapters/holoocean/tests/test_scenario_manifest.py", "configs/experiment/rov_realtime_disturbed.yaml", "implemented")

# ---- SIM-FAULT ----
_fus_state("SIM-FAULT-001", DISTURBED, "adapters/holoocean/uw_holoocean_adapter/fault_injector.py (ScheduledFault: release_time_s/duration_s/kind)", "adapters/holoocean/tests/test_fault_injector.py", "adapters/holoocean/uw_holoocean_adapter/fault_injector.py", "implemented")
_fus_state("SIM-FAULT-002", DISTURBED, "adapters/holoocean/uw_holoocean_adapter/run_report.py (fault timeline + health timeline correlation)", "n/a (native host needed for a real correlated run)", "adapters/holoocean/uw_holoocean_adapter/run_report.py", "gated")

# ---- SIM-ACC ----
_fus_state("SIM-ACC-001", BOTH_TASKS, "adapters/holoocean/uw_holoocean_adapter/realtime_gate.py (drives real HoloOcean session -> algorithm -> HMI, never offline-only)", "n/a (native host)", "adapters/holoocean/uw_holoocean_adapter/realtime_gate.py", "gated")
_fus_state("SIM-ACC-002", BOTH_TASKS, "apps/online_assist_smoke.cpp (synthetic proof of the same wiring shape)", "tests/integration/online_assist_smoke_test.sh", "tests/integration/online_assist_smoke_test.sh", "implemented")
_fus_state("SIM-ACC-003", BOTH_TASKS, "adapters/holoocean/uw_holoocean_adapter/realtime_gate.py", "n/a (native host, 10-seed campaigns)", "adapters/holoocean/uw_holoocean_adapter/realtime_gate.py", "gated")
_fus_state("SIM-ACC-004", NOMINAL + "+" + OVERLOAD, "adapters/holoocean/uw_holoocean_adapter/realtime_gate.py --soak-duration-s", "n/a (native host, 2h/30min)", "adapters/holoocean/uw_holoocean_adapter/realtime_gate.py", "gated")
_fus_state("SIM-ACC-005", NOMINAL, "adapters/holoocean/uw_holoocean_adapter/run_report.py (rss_growth_after_warmup_mib, cpu/gpu headroom)", "adapters/holoocean/tests/test_run_report.py", "adapters/holoocean/uw_holoocean_adapter/run_report.py", "implemented")
_fus_state("SIM-ACC-006", ALL_PROFILES, "adapters/holoocean/uw_holoocean_adapter/run_report.py (result_age_p95_ms gate)", "adapters/holoocean/tests/test_run_report.py", "adapters/holoocean/uw_holoocean_adapter/run_report.py", "verified")

# ---- SIM-S2R ----
_fus_state("SIM-S2R-001", "n/a (pool)", "randomization ranges must come from measured pool-test distributions", "no automated evidence possible from simulation execution alone", "n/a", "gated")


def main() -> None:
    ids = []
    for prefix, path in SPEC_FILES.items():
        text = path.read_text(encoding="utf-8")
        ids.extend(sorted(set(ID_RE.findall(text))))
    ids = sorted(set(ids))
    missing = [i for i in ids if i not in ROWS]
    if missing:
        raise SystemExit(f"missing mapping for: {missing}")
    extra = [i for i in ROWS if i not in ids]
    if extra:
        raise SystemExit(f"mapping has IDs not present in any spec file: {extra}")

    out_path = ROOT / "docs/traceability/rov-realtime-closed-loop.csv"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["requirement_id", "scenario", "implementation_module", "test", "evidence_path", "status"])
        for rid in ids:
            scenario, module, test, evidence, status = ROWS[rid]
            writer.writerow([rid, scenario, module, test, evidence, status])
    print(f"wrote {len(ids)} rows to {out_path}")


if __name__ == "__main__":
    main()
