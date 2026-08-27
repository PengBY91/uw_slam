"""rclpy wrapper closing the realtime loop's pilot leg: subscribes
`/uw/hmi/status`, drives one `ScriptedPilot`, publishes `/uw/pilot/thrusters`
-- the piece `realtime_gate.py`'s `_run_scripted_pilot_process` was missing
(see docs/rov-realtime-closed-loop-code-review-2026-08-27.md finding A3).

Thin and intentionally NOT unit tested directly here, same status as
`realtime_ros_session.py`'s `main()` -- this machine has no rclpy/ROS2
install (see adapters/holoocean/README.md's "what's real vs not tested
here"). `hmi_status_bridge.parse_guidance_status` is the portable, tested
core this module drives; `scripted_pilot.ScriptedPilot.command` is a pure
function already covered by `tests/test_scripted_pilot.py`.
"""
from __future__ import annotations

from uw_holoocean_adapter.hmi_status_bridge import parse_guidance_status
from uw_holoocean_adapter.scenario_manifest import TaskSpec
from uw_holoocean_adapter.scripted_pilot import ScriptedPilot

_STATUS_TOPIC = "/uw/hmi/status"
_THRUSTER_TOPIC = "/uw/pilot/thrusters"
_THRUSTER_COUNT = 8


def run_scripted_pilot_bridge(task: TaskSpec) -> None:
    """Blocks forever (rclpy.spin), publishing one command on every
    `/uw/hmi/status` update. Called as a `multiprocessing.Process` target
    from `realtime_gate.py`'s `_run_scripted_pilot_process` -- that module's
    own docstring explains why this has no separate CLI entrypoint."""
    import rclpy  # noqa: E402  (lazy: no rclpy install outside a sourced ROS2 distro)
    from rclpy.node import Node  # noqa: E402
    from rclpy.qos import qos_profile_sensor_data  # noqa: E402
    from std_msgs.msg import Float32MultiArray, String  # noqa: E402

    pilot = ScriptedPilot(task)

    rclpy.init()
    node = Node("uw_scripted_pilot_bridge")
    publisher = node.create_publisher(Float32MultiArray, _THRUSTER_TOPIC, qos_profile_sensor_data)

    def _on_status(msg: "String") -> None:
        try:
            status = parse_guidance_status(msg.data)
        except (ValueError, KeyError) as error:
            # A malformed payload must not crash the bridge or leave the
            # last (possibly stale) command latched forever -- hold station
            # instead, same fail-safe ScriptedPilot itself applies to a
            # stale/invalid status.
            node.get_logger().warning(f"malformed /uw/hmi/status payload, holding station: {error}")
            command = [0.0] * _THRUSTER_COUNT
        else:
            command = pilot.command(status)
        out = Float32MultiArray()
        out.data = [float(v) for v in command]
        publisher.publish(out)

    node.create_subscription(String, _STATUS_TOPIC, _on_status, qos_profile_sensor_data)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
