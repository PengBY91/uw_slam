"""HoloOcean RGBCamera sensor frame -> canonical uw.domain.ImageFrame conversion.

This is the piece that was missing between HoloOceanSession.step() (raw
HoloOcean sensor dict, see holoocean_driver.py) and CanonicalMcapWriter
(generic protobuf/MCAP writer, see canonical_writer.py): nothing previously
turned a camera reading into a schema message. apps/tools/synth_bag_gen's
BuildStereoPair (C++) writes the same ImageFrame shape from painted
synthetic textures; this module is the real-sensor counterpart so a bag
recorded from an actual HoloOcean session is byte-for-byte consumable by
apps/replay_demo without translation, same as the synthetic path.

Channel order: HoloOcean's docs describe RGBCamera's output as RGBA
(byu-holoocean.github.io/holoocean-docs), but that's wrong in practice —
confirmed by capturing a real frame from a real HoloOcean 2.3.0 install
(`OpenWater-HoveringCamera` scenario, native Windows, since WSL2 can't
render — see holoocean_driver.py's module docstring) and comparing it
untouched vs. with the first three channels reversed: the untouched frame
was a wrong-looking amber/orange underwater scene, while the
channel-reversed one showed a physically correct blue-water underwater
scene (sand, coral, a wreck). So the real runtime order is BGR(A), matching
what external_repos/ocean_t/src/main.py and svin2_pipeline.py (read-only
reference, the code this replaces) implicitly assumed by feeding the array
straight into cv2.imshow/cv2.imwrite with no channel swap — OpenCV's native
order is BGR too.
"""
from __future__ import annotations

import numpy as np

from uw_holoocean_adapter.time_utils import make_stamp


def holoocean_camera_to_image_frame(
    image_pb2_module,
    observation_pb2_module,
    time_pb2_module,
    pixels: np.ndarray,
    *,
    sensor_id: str,
    sensor_frame: str,
    observation_id: str,
    capture_time_s: float,
    is_rectified: bool = False,
    provenance: str = "uw_holoocean_adapter_v1",
):
    """Converts one HoloOcean RGBCamera tick into a uw.domain.ImageFrame.

    `pixels` must be a (height, width, 3-or-4) uint8 array as returned in
    HoloOcean's state dict under the sensor's configured name (e.g.
    state["LeftCamera"]) — real captures are (H, W, 4) in BGRA order (see
    module docstring); a trailing alpha channel, if present, is dropped and
    the remaining three channels are reversed into RGB. Takes the generated
    schema_pb2 modules as parameters rather than importing them, matching
    time_utils.make_stamp's approach — this module has no hard dependency
    on the generated-code output directory layout.
    """
    if pixels.ndim != 3 or pixels.shape[2] not in (3, 4):
        raise ValueError(f"expected a (H, W, 3-or-4) camera array, got shape {pixels.shape}")
    if pixels.dtype != np.uint8:
        raise ValueError(f"expected uint8 pixel data, got dtype {pixels.dtype}")

    height, width = pixels.shape[:2]
    bgr = pixels[:, :, :3]
    rgb = np.ascontiguousarray(bgr[:, :, ::-1])

    frame = image_pb2_module.ImageFrame()
    frame.header.observation_id.value = observation_id
    frame.header.sensor_id.value = sensor_id
    frame.header.sensor_frame.value = sensor_frame
    frame.header.capture_time.CopyFrom(make_stamp(time_pb2_module, capture_time_s))
    frame.header.clock_domain = time_pb2_module.CLOCK_DOMAIN_SIMULATION
    frame.header.validity = observation_pb2_module.ObservationHeader.VALIDITY_OK
    frame.header.provenance = provenance
    frame.width = width
    frame.height = height
    frame.row_stride_bytes = width * 3
    frame.encoding = image_pb2_module.ImageFrame.IMAGE_ENCODING_RGB8
    frame.pixel_data = rgb.tobytes()
    frame.is_rectified = is_rectified
    return frame
