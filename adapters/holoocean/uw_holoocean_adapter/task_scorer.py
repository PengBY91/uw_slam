"""Truth-isolated task scoring for the realtime closed loop.

`TaskScorer` is the ONE place in the entire realtime system allowed to
consume `/uw/sim/ground_truth` (`observe_truth`) — nothing else in this
package (the ROS gateway, `ScriptedPilot`, `OnlineAssistPipeline`) ever
sees it. `algorithm_topics` mirrors `ros_message_conversion.build_topic_map()
.algorithm_inputs` so a test/caller can assert the scorer's own truth
consumption never leaks into that list.
"""
from __future__ import annotations

import dataclasses
import math
from typing import Any, Dict, List, Optional

import numpy as np

from uw_holoocean_adapter.coordinates import Pose
from uw_holoocean_adapter.ros_message_conversion import build_topic_map
from uw_holoocean_adapter.scenario_manifest import TaskSpec

_DEGRADED_SOURCES = ("SONAR",)


@dataclasses.dataclass(frozen=True)
class AssistTrackObservation:
    """One assist-guidance observation fed to the scorer — same vocabulary
    as `scripted_pilot.AssistGuidanceStatus`, plus `confidence`: the pilot
    doesn't need a confidence value to drive thrusters, but
    `min_track_confidence` success conditions do."""

    guidance_valid: bool
    source: str  # "ACOUSTIC_OPTIC" | "VISUAL" | "SONAR" | ""
    confidence: float = 0.0
    bearing_rad: Optional[float] = None
    range_m: Optional[float] = None
    path_lateral_offset_m: Optional[float] = None


@dataclasses.dataclass
class TaskScoreReport:
    task_id: str
    task_version: int
    seed: Optional[int]
    task_success: bool
    degraded_completion: bool
    completion_time_s: Optional[float]
    observation_count: int
    valid_observation_count: int
    track_valid_fraction: float
    true_positive_count: int
    false_positive_count: int
    false_negative_count: int
    precision: Optional[float]
    recall: Optional[float]
    false_positives_per_minute: float
    bearing_error_p95_rad: Optional[float]
    range_error_p95_m: Optional[float]
    lateral_offset_p95_m: Optional[float]

    def as_dict(self) -> Dict[str, Any]:
        return dataclasses.asdict(self)


def _point_target_geometry(pose: Pose, target_location_m) -> tuple[float, float]:
    """True (bearing_rad, range_m) from `pose` to a static point target,
    in `pose`'s own body frame (x-forward, y-left, matching this package's
    body-frame convention elsewhere -- see coordinates.py)."""
    to_target_world = np.asarray(target_location_m, dtype=float) - pose.translation
    to_target_body = pose.inverse().rotation_matrix() @ to_target_world
    range_m = float(np.linalg.norm(to_target_body))
    bearing_rad = float(math.atan2(to_target_body[1], to_target_body[0]))
    return bearing_rad, range_m


def _path_lateral_offset_m(pose: Pose, waypoints_m) -> float:
    """Perpendicular distance from `pose.translation` to the nearest
    segment of the waypoint polyline (2D, x/y only)."""
    points = np.asarray(waypoints_m, dtype=float)[:, :2]
    position = pose.translation[:2]
    if len(points) == 1:
        return float(np.linalg.norm(position - points[0]))
    best = math.inf
    for start, end in zip(points[:-1], points[1:]):
        segment = end - start
        length_sq = float(np.dot(segment, segment))
        if length_sq == 0.0:
            distance = float(np.linalg.norm(position - start))
        else:
            t = np.clip(np.dot(position - start, segment) / length_sq, 0.0, 1.0)
            projection = start + t * segment
            distance = float(np.linalg.norm(position - projection))
        best = min(best, distance)
    return best


class TaskScorer:
    algorithm_topics = build_topic_map().algorithm_inputs

    def __init__(self, task: TaskSpec, seed: Optional[int] = None):
        self._task = task
        self._seed = seed
        self._kind = task.target.get("kind")
        self._min_confidence = float(task.success_conditions.get("min_track_confidence", 0.0))

        self._latest_truth_pose: Optional[Pose] = None
        self._first_time_s: Optional[float] = None
        self._first_qualifying_time_s: Optional[float] = None
        self._last_qualifying_time_s: Optional[float] = None
        self._completion_time_s: Optional[float] = None
        self._degraded_completion = False

        self._observation_count = 0
        self._valid_observation_count = 0
        self._true_positives = 0
        self._false_positives = 0
        self._false_negatives = 0
        self._bearing_errors: List[float] = []
        self._range_errors: List[float] = []
        self._lateral_offset_errors: List[float] = []

    def observe_truth(self, pose: Pose, capture_time_s: float) -> None:
        self._latest_truth_pose = pose
        self._note_time(capture_time_s)

    def observe_assist(self, track: AssistTrackObservation, receive_time_s: float) -> None:
        self._note_time(receive_time_s)
        self._observation_count += 1
        if not track.guidance_valid:
            self._false_negatives += self._truth_expects_detection()
            return

        self._valid_observation_count += 1
        associated = self._score_association(track)
        if associated:
            self._true_positives += 1
            if track.confidence >= self._min_confidence:
                if self._first_qualifying_time_s is None:
                    self._first_qualifying_time_s = receive_time_s
                self._last_qualifying_time_s = receive_time_s
                if track.source in _DEGRADED_SOURCES:
                    self._degraded_completion = True
        else:
            self._false_positives += 1

    def report(self) -> TaskScoreReport:
        task_success = self._is_success()
        if task_success and self._completion_time_s is None:
            self._completion_time_s = self._last_qualifying_time_s

        precision = None
        recall = None
        denom_p = self._true_positives + self._false_positives
        denom_r = self._true_positives + self._false_negatives
        if denom_p > 0:
            precision = self._true_positives / denom_p
        if denom_r > 0:
            recall = self._true_positives / denom_r

        elapsed_minutes = None
        if self._first_time_s is not None and self._last_qualifying_time_s is not None:
            elapsed_minutes = max(1e-9, (self._last_qualifying_time_s - self._first_time_s) / 60.0)
        fp_per_minute = (self._false_positives / elapsed_minutes) if elapsed_minutes else 0.0

        track_valid_fraction = (
            self._valid_observation_count / self._observation_count if self._observation_count else 0.0
        )

        return TaskScoreReport(
            task_id=self._task.task_id,
            task_version=self._task.version,
            seed=self._seed,
            task_success=task_success,
            degraded_completion=task_success and self._degraded_completion,
            completion_time_s=self._completion_time_s,
            observation_count=self._observation_count,
            valid_observation_count=self._valid_observation_count,
            track_valid_fraction=track_valid_fraction,
            true_positive_count=self._true_positives,
            false_positive_count=self._false_positives,
            false_negative_count=self._false_negatives,
            precision=precision,
            recall=recall,
            false_positives_per_minute=fp_per_minute,
            bearing_error_p95_rad=_p95(self._bearing_errors),
            range_error_p95_m=_p95(self._range_errors),
            lateral_offset_p95_m=_p95(self._lateral_offset_errors),
        )

    def _note_time(self, time_s: float) -> None:
        if self._first_time_s is None:
            self._first_time_s = time_s

    def _truth_expects_detection(self) -> int:
        # Without a per-tick truth stream keyed to each assist observation's
        # exact timestamp, this scorer can't reliably tell "target was
        # within detectable range at this precise instant" from a single
        # latest-truth-pose snapshot -- conservatively count every invalid
        # observation as a miss only once truth has been observed at all
        # (before that, there's no basis to expect detection yet).
        return 1 if self._latest_truth_pose is not None else 0

    def _score_association(self, track: AssistTrackObservation) -> bool:
        if self._latest_truth_pose is None:
            return False
        if self._kind == "point":
            target_location_m = self._task.target.get("location_m")
            if target_location_m is None or track.bearing_rad is None or track.range_m is None:
                return False
            true_bearing_rad, true_range_m = _point_target_geometry(self._latest_truth_pose, target_location_m)
            bearing_error = abs(_wrap_angle(track.bearing_rad - true_bearing_rad))
            range_error = abs(track.range_m - true_range_m)
            self._bearing_errors.append(bearing_error)
            self._range_errors.append(range_error)
            return bearing_error <= _ASSOCIATION_BEARING_TOLERANCE_RAD and range_error <= _ASSOCIATION_RANGE_TOLERANCE_M
        if self._kind == "path":
            waypoints_m = self._task.target.get("waypoints_m")
            if waypoints_m is None or track.path_lateral_offset_m is None:
                return False
            true_offset_m = _path_lateral_offset_m(self._latest_truth_pose, waypoints_m)
            offset_error = abs(track.path_lateral_offset_m - true_offset_m)
            self._lateral_offset_errors.append(offset_error)
            tolerance_m = float(self._task.target.get("lateral_tolerance_m", _ASSOCIATION_RANGE_TOLERANCE_M))
            return offset_error <= tolerance_m
        return False

    def _is_success(self) -> bool:
        if self._first_qualifying_time_s is None:
            return False
        if self._kind == "point":
            deadline_s = self._task.success_conditions.get("detection_confirmed_within_s")
            if deadline_s is not None and self._first_time_s is not None:
                if (self._first_qualifying_time_s - self._first_time_s) > deadline_s:
                    return False
            return True
        if self._kind == "path":
            max_offset_p95_m = self._task.success_conditions.get("max_lateral_offset_p95_m")
            offset_p95 = _p95(self._lateral_offset_errors)
            if max_offset_p95_m is not None and offset_p95 is not None and offset_p95 > max_offset_p95_m:
                return False
            return True
        return False


_ASSOCIATION_BEARING_TOLERANCE_RAD = math.radians(15.0)
_ASSOCIATION_RANGE_TOLERANCE_M = 3.0


def _wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _p95(samples: List[float]) -> Optional[float]:
    if not samples:
        return None
    return float(np.percentile(np.asarray(samples), 95))
