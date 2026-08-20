import tempfile
from pathlib import Path

from uw.domain import health_pb2, image_pb2, measurement_pb2  # noqa: E402

from uw_holoocean_adapter.canonical_writer import CanonicalMcapWriter, read_canonical_messages


def test_round_trips_protobuf_messages_through_mcap():
    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "test.mcap")

        with CanonicalMcapWriter(path) as writer:
            for i in range(3):
                report = health_pb2.HealthReport()
                report.component_id = "python_writer_test"
                report.status = health_pb2.HealthReport.STATUS_HEALTHY
                report.queue_depth = i
                writer.write_message("/health", i * 1000, report)

        queue_depths = []
        for log_time_ns, report in read_canonical_messages(path, "/health", health_pb2.HealthReport):
            assert report.component_id == "python_writer_test"
            queue_depths.append(report.queue_depth)
            del log_time_ns

        assert queue_depths == [0, 1, 2]


def test_round_trips_image_frame_through_canonical_mcap():
    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "camera.mcap")
        frame = image_pb2.ImageFrame()
        frame.header.observation_id.value = "left_0001"
        frame.header.sensor_id.value = "camera_left"
        frame.header.sensor_frame.value = "camera_left_link"
        frame.width = 2
        frame.height = 1
        frame.row_stride_bytes = 2
        frame.encoding = image_pb2.ImageFrame.IMAGE_ENCODING_MONO8
        frame.pixel_data = bytes([16, 32])
        frame.is_rectified = True

        with CanonicalMcapWriter(path) as writer:
            writer.write_message("/raw/camera/left", 1_000_000, frame)

        messages = list(
            read_canonical_messages(path, "/raw/camera/left", image_pb2.ImageFrame)
        )
        assert len(messages) == 1
        assert messages[0][1].pixel_data == bytes([16, 32])


def test_round_trips_optical_depth_evidence_through_canonical_mcap():
    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "depth.mcap")
        evidence = measurement_pb2.MeasurementEvidence()
        evidence.evidence_id.value = "optical_depth_1"
        evidence.algorithm_version = "stereo_depth_frontend_v1"
        evidence.optical_depth_prior.reference_camera_frame.value = "camera_left_link"
        evidence.optical_depth_prior.width = 1
        evidence.optical_depth_prior.height = 1
        evidence.optical_depth_prior.depth_m.append(2.0)
        evidence.optical_depth_prior.variance_m2.append(0.04)
        evidence.optical_depth_prior.valid_mask = bytes([1])
        evidence.optical_depth_prior.scale_status = (
            measurement_pb2.OPTICAL_DEPTH_SCALE_STATUS_METRIC
        )
        evidence.optical_depth_prior.producer_type = "stereo"

        with CanonicalMcapWriter(path) as writer:
            writer.write_message("/evidence/optical_depth", 1_000_000, evidence)

        messages = list(
            read_canonical_messages(
                path, "/evidence/optical_depth", measurement_pb2.MeasurementEvidence
            )
        )
        assert len(messages) == 1
        assert messages[0][1].WhichOneof("payload") == "optical_depth_prior"
        assert messages[0][1].optical_depth_prior.depth_m[0] == 2.0
