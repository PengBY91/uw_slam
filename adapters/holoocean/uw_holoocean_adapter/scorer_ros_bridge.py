"""rclpy wrapper closing the realtime loop's scoring leg: subscribes BOTH
`/uw/sim/ground_truth` and `/uw/hmi/status`, drives one `TaskScorer`, and
periodically writes its report to disk -- the piece `realtime_gate.py`'s
`_run_scorer_process` was missing (see docs/archive/rov-realtime-closed-loop-code-
review-2026-08-27.md finding A3). This is the ONE process in the realtime
closed loop allowed to see ground truth (SIM-ARCH-002/SYS-ARCH-003) -- see
`task_scorer.py`'s own docstring.

Thin and intentionally NOT unit tested directly here, same status as
`realtime_ros_session.py`'s `main()` -- this machine has no rclpy/ROS2
install. `hmi_status_bridge.parse_track_observation` and `TaskScorer` itself
are the portable, tested pieces this module drives.

Clock-domain note (mirrors docs/archive/rov-realtime-closed-loop-code-review-2026-
08-27.md finding A1, same root cause, same fix shape, reimplemented in
Python since this module cannot depend on the C++ `adapters` library):
`TaskScorer.observe_truth`/`observe_assist` both feed one internal "when did
this happen" clock (`_note_time`), used for completion-time and false-
positive-rate bookkeeping. Ground truth arrives stamped in HoloOcean
simulation time; `/uw/hmi/status` carries no absolute timestamp at all (only
a relative `data_age_ms`). Passing `time.monotonic()` (wall time) for
`observe_assist` while `observe_truth` uses sim time would silently mix two
clock domains into the same running "first/last observed" bookkeeping.
`_SimTimeTracker` below extrapolates a current-simulation-time estimate from
the most recently observed truth pose's timestamp, the same anchor-and-
extrapolate approach `include/adapters/sim_wall_clock_estimator.hpp` uses on
the C++ side.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Optional

from uw_holoocean_adapter.hmi_status_bridge import parse_track_observation
from uw_holoocean_adapter.scenario_manifest import TaskSpec
from uw_holoocean_adapter.task_scorer import TaskScorer

_STATUS_TOPIC = "/uw/hmi/status"
_TRUTH_TOPIC = "/uw/sim/ground_truth"
_WRITE_INTERVAL_S = 1.0


class _SimTimeTracker:
    """Anchors on each observed ground-truth capture time, extrapolates
    "now" in that same simulation clock domain from wall-clock elapsed time
    since the anchor. Returns None before the first truth observation --
    callers should not score an assist observation without at least one
    truth sample to associate it against anyway (TaskScorer itself treats
    "no truth yet" as unscoreable, see `_score_association`)."""

    def __init__(self):
        self._anchor_sim_s: Optional[float] = None
        self._anchor_wall_s: Optional[float] = None

    def observe_truth(self, capture_s: float) -> None:
        self._anchor_sim_s = capture_s
        self._anchor_wall_s = time.monotonic()

    def estimate_now(self) -> Optional[float]:
        if self._anchor_sim_s is None or self._anchor_wall_s is None:
            return None
        return self._anchor_sim_s + (time.monotonic() - self._anchor_wall_s)


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _pose_from_odometry(msg):
    import numpy as np

    from uw_holoocean_adapter.coordinates import Pose

    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    return Pose(
        translation=np.array([position.x, position.y, position.z]),
        quaternion_xyzw=np.array([orientation.x, orientation.y, orientation.z, orientation.w]),
    )


def run_scorer_bridge(task: TaskSpec, seed: Optional[int], out_path: str) -> None:
    """Blocks forever (rclpy.spin), writing `scorer.report()` to `out_path`
    every `_WRITE_INTERVAL_S` seconds and once more on shutdown. Called as a
    `multiprocessing.Process` target from `realtime_gate.py`'s
    `_run_scorer_process`."""
    import rclpy  # noqa: E402  (lazy: no rclpy install outside a sourced ROS2 distro)
    from rclpy.node import Node  # noqa: E402
    from rclpy.qos import qos_profile_sensor_data  # noqa: E402
    from nav_msgs.msg import Odometry  # noqa: E402
    from std_msgs.msg import String  # noqa: E402

    scorer = TaskScorer(task, seed=seed)
    sim_clock = _SimTimeTracker()

    rclpy.init()
    node = Node("uw_task_scorer_bridge")

    def _on_truth(msg: "Odometry") -> None:
        capture_s = _stamp_to_seconds(msg.header.stamp)
        sim_clock.observe_truth(capture_s)
        scorer.observe_truth(_pose_from_odometry(msg), capture_s)

    def _on_status(msg: "String") -> None:
        now_s = sim_clock.estimate_now()
        if now_s is None:
            return  # no truth observed yet -- nothing to associate against
        try:
            observation = parse_track_observation(msg.data)
        except (ValueError, KeyError) as error:
            node.get_logger().warning(f"malformed /uw/hmi/status payload, skipping: {error}")
            return
        scorer.observe_assist(observation, now_s)

    node.create_subscription(Odometry, _TRUTH_TOPIC, _on_truth, qos_profile_sensor_data)
    node.create_subscription(String, _STATUS_TOPIC, _on_status, qos_profile_sensor_data)

    def _write_report() -> None:
        Path(out_path).write_text(json.dumps(scorer.report().as_dict()), encoding="utf-8")

    node.create_timer(_WRITE_INTERVAL_S, _write_report)
    try:
        rclpy.spin(node)
    finally:
        _write_report()
        node.destroy_node()
        rclpy.shutdown()
