"""PREP-E-02: the realtime session's bandwidth profile is derived from the
scenario manifest (message sizes) and TopicMap (priorities), so the shaper
sees the same traffic shape the ROS publishers produce."""
import pathlib

import numpy as np

from uw_holoocean_adapter.fault_injector import BandwidthShaper, build_bandwidth_schedule
from uw_holoocean_adapter.realtime_ros_session import _build_bandwidth_profile
from uw_holoocean_adapter.ros_message_conversion import build_topic_map
from uw_holoocean_adapter.scenario_manifest import load_realtime_manifest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
BASE_SCENARIO = REPO_ROOT / "adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json"
SEARCH_TASK = REPO_ROOT / "adapters/holoocean/scenarios/aquaculture_search.yaml"


def test_bandwidth_profile_sizes_and_priorities_follow_the_manifest():
    manifest = load_realtime_manifest(BASE_SCENARIO, SEARCH_TASK)
    topics = build_topic_map(manifest.agent_name)
    profile = _build_bandwidth_profile(
        topics, manifest, nominal_mbps=20.0, min_mbps=10.0, max_mbps=40.0, walk_sigma_mbps_per_s=2.0
    )
    assert profile.topic_bytes[topics.left_camera] == 1280 * 720 * 3
    assert profile.topic_bytes[topics.right_camera] == 1280 * 720 * 3
    assert profile.topic_bytes[topics.pilot_camera] == 1280 * 720 * 3
    assert profile.topic_bytes[topics.imaging_sonar] == 512 * 768 * 4
    # PREP-E-01 degradation order: cameras give way first, then pilot video,
    # then sonar; ~1 kB telemetry outranks everything (it always fits).
    assert profile.topic_priority[topics.vehicle_state] < profile.topic_priority[topics.imaging_sonar]
    assert profile.topic_priority[topics.imaging_sonar] < profile.topic_priority[topics.pilot_camera]
    assert profile.topic_priority[topics.pilot_camera] < profile.topic_priority[topics.left_camera]
    assert profile.topic_priority[topics.left_camera] == profile.topic_priority[topics.right_camera]
    # The simulation clock and the scorer's truth channel are never tether traffic.
    assert topics.clock in profile.bypass_topics
    assert topics.scoring_truth in profile.bypass_topics


def test_nominal_stereo_plus_sonar_does_not_fit_a_20_mbps_tether():
    # The whole point of the profile: 20/20/10 Hz stereo+sonar at the base
    # scenario's resolutions is ~140 Mbps raw, so the shaper must drop most
    # camera frames while letting sonar through.
    manifest = load_realtime_manifest(BASE_SCENARIO, SEARCH_TASK)
    topics = build_topic_map(manifest.agent_name)
    profile = _build_bandwidth_profile(
        topics, manifest, nominal_mbps=20.0, min_mbps=10.0, max_mbps=40.0, walk_sigma_mbps_per_s=0.0
    )
    shaper = BandwidthShaper(build_bandwidth_schedule(1, profile, 10.0), profile)
    for tick in range(100):  # 10 s at 10 Hz (the base scenario's sonar rate)
        now_s = tick * 0.1
        messages = [(topics.imaging_sonar, tick), (topics.vehicle_state, tick), (topics.clock, tick)]
        messages += [(topics.left_camera, tick), (topics.right_camera, tick)] * 2  # 20 Hz stereo
        shaper.apply(now_s, messages)
    stats = shaper.stats()["per_topic"]
    assert stats[topics.imaging_sonar]["sent_count"] >= 10
    assert stats[topics.left_camera]["dropped_count"] > stats[topics.left_camera]["sent_count"]
    assert stats[topics.vehicle_state]["dropped_count"] == 0
    assert np.isclose(shaper.current_mbps, 20.0)
