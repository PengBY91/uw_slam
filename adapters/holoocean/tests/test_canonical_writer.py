import tempfile
from pathlib import Path

from uw.domain import health_pb2  # noqa: E402  (path set up by conftest.py)

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
