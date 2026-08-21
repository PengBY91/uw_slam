import numpy as np
from uw.domain import image_pb2, observation_pb2, time_pb2  # noqa: E402

from uw_holoocean_adapter.camera_conversion import holoocean_camera_to_image_frame


def _bgra(height, width, fill):
    """`fill` is [B, G, R] — matches the real sensor's on-the-wire order
    (see camera_conversion.py's module docstring for how that was
    confirmed against a real HoloOcean install)."""
    pixels = np.zeros((height, width, 4), dtype=np.uint8)
    pixels[:, :, :3] = fill
    pixels[:, :, 3] = 255
    return pixels


def test_drops_alpha_and_reverses_bgr_to_rgb8():
    pixels = _bgra(2, 3, fill=[10, 20, 30])  # B=10, G=20, R=30

    frame = holoocean_camera_to_image_frame(
        image_pb2,
        observation_pb2,
        time_pb2,
        pixels,
        sensor_id="camera_left",
        sensor_frame="camera_left_link",
        observation_id="kf0",
        capture_time_s=1.5,
    )

    assert frame.encoding == image_pb2.ImageFrame.IMAGE_ENCODING_RGB8
    assert frame.width == 3
    assert frame.height == 2
    assert frame.row_stride_bytes == 9
    assert frame.pixel_data == bytes([30, 20, 10] * (2 * 3))  # R=30, G=20, B=10


def test_accepts_bgr_array_without_alpha():
    pixels = np.zeros((1, 1, 3), dtype=np.uint8)
    pixels[0, 0] = [1, 2, 3]  # B=1, G=2, R=3

    frame = holoocean_camera_to_image_frame(
        image_pb2,
        observation_pb2,
        time_pb2,
        pixels,
        sensor_id="camera_right",
        sensor_frame="camera_right_link",
        observation_id="kf1",
        capture_time_s=0.0,
    )

    assert frame.pixel_data == bytes([3, 2, 1])  # R=3, G=2, B=1


def test_sets_header_fields_from_arguments():
    pixels = _bgra(1, 1, fill=[0, 0, 0])

    frame = holoocean_camera_to_image_frame(
        image_pb2,
        observation_pb2,
        time_pb2,
        pixels,
        sensor_id="camera_left",
        sensor_frame="camera_left_link",
        observation_id="kf7",
        capture_time_s=2.25,
        is_rectified=True,
        provenance="test_provenance",
    )

    assert frame.header.observation_id.value == "kf7"
    assert frame.header.sensor_id.value == "camera_left"
    assert frame.header.sensor_frame.value == "camera_left_link"
    assert frame.header.clock_domain == time_pb2.CLOCK_DOMAIN_SIMULATION
    assert frame.header.validity == observation_pb2.ObservationHeader.VALIDITY_OK
    assert frame.header.provenance == "test_provenance"
    assert frame.header.capture_time.seconds == 2
    assert frame.header.capture_time.nanos == 250_000_000
    assert frame.is_rectified is True


def test_rejects_wrong_shape():
    pixels = np.zeros((4, 4), dtype=np.uint8)
    try:
        holoocean_camera_to_image_frame(
            image_pb2,
            observation_pb2,
            time_pb2,
            pixels,
            sensor_id="camera_left",
            sensor_frame="camera_left_link",
            observation_id="kf0",
            capture_time_s=0.0,
        )
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_rejects_wrong_dtype():
    pixels = np.zeros((2, 2, 3), dtype=np.float32)
    try:
        holoocean_camera_to_image_frame(
            image_pb2,
            observation_pb2,
            time_pb2,
            pixels,
            sensor_id="camera_left",
            sensor_frame="camera_left_link",
            observation_id="kf0",
            capture_time_s=0.0,
        )
        assert False, "expected ValueError"
    except ValueError:
        pass
