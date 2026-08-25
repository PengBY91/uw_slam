import numpy as np
import pytest

from uw_holoocean_adapter.coordinates import Pose
from uw_holoocean_adapter.task_scorer import AssistTrackObservation, TaskScorer

from test_scripted_pilot import search_task_spec, structure_task_spec  # noqa: E402  (shared fixtures, same dir)

_IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])


def make_truth_pose(translation=(10.0, 6.0, -4.0)):
    return Pose(translation=np.array(translation, dtype=float), quaternion_xyzw=_IDENTITY_QUAT.copy())


def make_correct_assist_track(**overrides):
    defaults = dict(
        guidance_valid=True, source="ACOUSTIC_OPTIC", confidence=0.9, bearing_rad=0.0, range_m=5.0,
    )
    defaults.update(overrides)
    return AssistTrackObservation(**defaults)


def test_scorer_consumes_truth_but_algorithm_topic_list_does_not():
    scorer = TaskScorer(search_task_spec())
    scorer.observe_truth(make_truth_pose(), capture_time_s=1.0)
    scorer.observe_assist(make_correct_assist_track(), receive_time_s=1.2)

    assert scorer.report().task_success
    assert "/uw/sim/ground_truth" not in scorer.algorithm_topics


def test_report_is_json_serializable():
    import json

    scorer = TaskScorer(search_task_spec(), seed=42)
    scorer.observe_truth(make_truth_pose(), capture_time_s=1.0)
    scorer.observe_assist(make_correct_assist_track(), receive_time_s=1.2)

    payload = json.dumps(scorer.report().as_dict())
    decoded = json.loads(payload)
    assert decoded["task_id"] == "aquaculture_search_v1"
    assert decoded["seed"] == 42


def test_invalid_guidance_never_counts_as_a_true_positive():
    scorer = TaskScorer(search_task_spec())
    scorer.observe_truth(make_truth_pose(), capture_time_s=1.0)
    scorer.observe_assist(make_correct_assist_track(guidance_valid=False), receive_time_s=1.2)

    report = scorer.report()
    assert report.task_success is False
    assert report.true_positive_count == 0
    assert report.false_negative_count == 1


def test_mismatched_bearing_counts_as_a_false_positive_not_a_success():
    scorer = TaskScorer(search_task_spec())
    scorer.observe_truth(make_truth_pose(), capture_time_s=1.0)
    # True bearing to the target from this pose is 0 rad; 90 degrees off
    # is far outside the association tolerance.
    scorer.observe_assist(
        make_correct_assist_track(bearing_rad=1.5707963267948966), receive_time_s=1.2
    )

    report = scorer.report()
    assert report.task_success is False
    assert report.false_positive_count == 1
    assert report.true_positive_count == 0


def test_low_confidence_observation_associates_but_does_not_confirm_success():
    scorer = TaskScorer(search_task_spec())
    scorer.observe_truth(make_truth_pose(), capture_time_s=1.0)
    scorer.observe_assist(make_correct_assist_track(confidence=0.1), receive_time_s=1.2)

    report = scorer.report()
    assert report.true_positive_count == 1
    assert report.task_success is False


def test_track_valid_fraction_reflects_the_ratio_of_valid_observations():
    scorer = TaskScorer(search_task_spec())
    scorer.observe_truth(make_truth_pose(), capture_time_s=1.0)
    scorer.observe_assist(make_correct_assist_track(), receive_time_s=1.1)
    scorer.observe_assist(make_correct_assist_track(guidance_valid=False), receive_time_s=1.2)
    scorer.observe_assist(make_correct_assist_track(), receive_time_s=1.3)

    report = scorer.report()
    assert report.observation_count == 3
    assert report.valid_observation_count == 2
    assert report.track_valid_fraction == pytest.approx(2.0 / 3.0)


def test_degraded_completion_flags_success_reached_only_via_sonar_only_source():
    scorer = TaskScorer(search_task_spec())
    scorer.observe_truth(make_truth_pose(), capture_time_s=1.0)
    scorer.observe_assist(make_correct_assist_track(source="SONAR"), receive_time_s=1.2)

    report = scorer.report()
    assert report.task_success is True
    assert report.degraded_completion is True


def test_detection_confirmed_after_the_deadline_is_not_success():
    scorer = TaskScorer(search_task_spec())
    scorer.observe_truth(make_truth_pose(), capture_time_s=0.0)
    # aquaculture_search.yaml's detection_confirmed_within_s is 180.0.
    scorer.observe_assist(make_correct_assist_track(), receive_time_s=200.0)

    assert scorer.report().task_success is False


def test_path_task_uses_lateral_offset_association_and_p95_gate():
    scorer = TaskScorer(structure_task_spec())
    # On the [10,0]-[20,0] segment (x/y only), offset by 0.5m in y.
    scorer.observe_truth(Pose(translation=np.array([15.0, 0.5, -5.0]), quaternion_xyzw=_IDENTITY_QUAT.copy()),
                          capture_time_s=1.0)
    scorer.observe_assist(
        AssistTrackObservation(
            guidance_valid=True, source="VISUAL", confidence=0.9, path_lateral_offset_m=0.5
        ),
        receive_time_s=1.2,
    )

    report = scorer.report()
    assert report.task_success is True
    assert report.lateral_offset_p95_m == pytest.approx(0.0, abs=1e-9)


def test_bearing_and_range_p95_are_reported_for_point_tasks():
    scorer = TaskScorer(search_task_spec())
    scorer.observe_truth(make_truth_pose(), capture_time_s=1.0)
    for range_m in (4.0, 5.0, 5.0, 6.0):
        scorer.observe_assist(make_correct_assist_track(range_m=range_m), receive_time_s=1.2)

    report = scorer.report()
    assert report.range_error_p95_m is not None
    assert report.range_error_p95_m > 0.0
    assert report.bearing_error_p95_rad == pytest.approx(0.0, abs=1e-9)
