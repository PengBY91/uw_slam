"""Applies `scenario_randomization.py`'s `VisualDegradation`/
`SonarDegradation` axes to real captured image/sonar arrays. Task 2's
`holoocean_driver.py`/`_prepare_scenario_cfg` already applies the axes that
are construction-time HoloOcean sensor config (sonar speckle/range-noise/
sound-speed); this module is for the axes that only make sense as a
post-capture transform on the actual pixel/intensity data HoloOcean
returned (haze, motion blur, particles, exposure, blind zone, false echoes,
range-scale bias).

Every function here takes the run's own owned `numpy.random.Generator` —
never constructs or reseeds one internally — matching this repo's
determinism rule (CLAUDE.md: explicit seeded RNG threaded through, never a
mid-run reseed).
"""
from __future__ import annotations

from typing import List, Tuple

import numpy as np

from uw_holoocean_adapter.scenario_randomization import SonarDegradation, VisualDegradation


def perturb_image(
    rng: np.random.Generator,
    degradation: VisualDegradation,
    image: np.ndarray,
    *,
    exposure_bias_ev: float = 0.0,
) -> Tuple[np.ndarray, List[str]]:
    """Applies color attenuation, haze/backscatter, motion blur, particles,
    local overexposure and exposure to one (H, W, 3-or-4) uint8 image.
    Returns the perturbed image (same shape/dtype as input) plus a list
    naming every axis that was actually non-trivial this call."""
    if image.ndim != 3 or image.shape[2] not in (3, 4):
        raise ValueError(f"expected a (H, W, 3-or-4) image array, got shape {image.shape}")
    if image.dtype != np.uint8:
        raise ValueError(f"expected uint8 image data, got dtype {image.dtype}")

    active: List[str] = []
    channels = image[:, :, :3].astype(np.float32)
    height, width = channels.shape[:2]

    if degradation.attenuation_per_m > 0.0 or degradation.turbidity > 0.0:
        # Coarse depth-independent attenuation: darken/desaturate toward a
        # blue-green haze color as turbidity/attenuation rise. Not a real
        # per-pixel range-dependent model (this module has no per-pixel
        # depth map to work from) -- a deliberately simple, documented
        # placeholder, same status as holoocean_driver.py's water_color
        # placeholder.
        attenuation = np.clip(degradation.attenuation_per_m * (0.3 + degradation.turbidity), 0.0, 1.0)
        haze_color = np.array([40.0, 90.0, 100.0], dtype=np.float32)
        channels = channels * (1.0 - attenuation) + haze_color * attenuation
        active.append("attenuation")

    if degradation.backscatter_gain > 0.0:
        backscatter = degradation.backscatter_gain * rng.normal(0.0, 12.0, size=channels.shape[:2])
        channels[:, :, 1] += backscatter  # backscatter skews green in this platform's water model
        active.append("backscatter")

    if degradation.motion_blur_px > 1.0:
        radius = max(1, int(round(degradation.motion_blur_px)))
        kernel = np.ones(radius, dtype=np.float32) / radius
        blurred = np.apply_along_axis(lambda row: np.convolve(row, kernel, mode="same"), axis=1, arr=channels)
        channels = blurred
        active.append("motion_blur")

    if degradation.particle_density > 0.0:
        particle_mask = rng.random(size=(height, width)) < (degradation.particle_density * 0.01)
        particle_brightness = rng.uniform(150.0, 255.0, size=int(particle_mask.sum()))
        channels[particle_mask] = particle_brightness[:, None]
        active.append("particles")

    if degradation.local_overexposure_probability > 0.0 and rng.random() < degradation.local_overexposure_probability:
        cy = int(rng.integers(0, height))
        cx = int(rng.integers(0, width))
        patch_radius = max(1, min(height, width) // 8)
        y0, y1 = max(0, cy - patch_radius), min(height, cy + patch_radius)
        x0, x1 = max(0, cx - patch_radius), min(width, cx + patch_radius)
        channels[y0:y1, x0:x1, :] = 255.0
        active.append("local_overexposure")

    total_exposure_ev = degradation.exposure_ev + exposure_bias_ev
    if total_exposure_ev != 0.0:
        channels *= 2.0**total_exposure_ev
        active.append("exposure")

    if degradation.illumination_scale != 1.0:
        channels *= degradation.illumination_scale
        active.append("illumination")

    out = image.copy()
    out[:, :, :3] = np.clip(channels, 0.0, 255.0).astype(np.uint8)
    return out, active


def perturb_stereo_pair(
    rng: np.random.Generator, degradation: VisualDegradation, left: np.ndarray, right: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, List[str]]:
    """Perturbs a stereo pair together so shared-scene axes (attenuation,
    backscatter, particles) use the same `rng` draws' *pattern* but each
    side keeps its own independent noise realization -- except exposure,
    where `stereo_exposure_mismatch_ev` deliberately biases only the right
    camera relative to the left, modeling a real exposure-control mismatch
    between two physically separate camera sensors."""
    left_out, left_active = perturb_image(rng, degradation, left, exposure_bias_ev=0.0)
    right_out, right_active = perturb_image(
        rng, degradation, right, exposure_bias_ev=degradation.stereo_exposure_mismatch_ev
    )
    active = sorted(set(left_active) | set(right_active))
    if degradation.stereo_exposure_mismatch_ev != 0.0:
        active.append("stereo_exposure_mismatch")
    return left_out, right_out, active


def perturb_sonar(
    rng: np.random.Generator,
    degradation: SonarDegradation,
    intensity_array: np.ndarray,
    *,
    min_range_m: float,
    max_range_m: float,
) -> Tuple[np.ndarray, List[str]]:
    """Applies speckle, additive/multiplicative noise, gain, blind zone,
    false/sidelobe echoes and range-scale bias to a raw HoloOcean
    ImagingSonar (num_ranges, num_beams) float32 intensity array in [0, 1].
    `range_scale_bias`/`min_range_m`/`max_range_m` only affect which ROWS
    the blind-zone mask covers -- the returned array's shape is unchanged
    (this module perturbs intensities, not the reported range-axis
    metadata; a caller applying `range_scale_bias` to the published
    `SonarFrame.range_bins`/`min_range`/`max_range` themselves is a
    separate, downstream concern)."""
    if intensity_array.ndim != 2:
        raise ValueError(f"expected a 2D (num_ranges, num_beams) array, got shape {intensity_array.shape}")
    if max_range_m <= min_range_m:
        raise ValueError(f"max_range_m ({max_range_m}) must exceed min_range_m ({min_range_m})")

    active: List[str] = []
    num_ranges, num_beams = intensity_array.shape
    out = intensity_array.astype(np.float32).copy()

    if degradation.gain_db != 0.0:
        out *= 10.0 ** (degradation.gain_db / 20.0)
        active.append("gain")

    if degradation.speckle_sigma > 0.0:
        out *= 1.0 + rng.normal(0.0, degradation.speckle_sigma, size=out.shape)
        active.append("speckle")

    if degradation.range_noise_sigma_m > 0.0:
        range_resolution_m = (max_range_m - min_range_m) / num_ranges
        shift_bins = int(round(rng.normal(0.0, degradation.range_noise_sigma_m) / range_resolution_m))
        if shift_bins != 0:
            out = np.roll(out, shift_bins, axis=0)
            if shift_bins > 0:
                out[:shift_bins, :] = 0.0
            else:
                out[shift_bins:, :] = 0.0
        active.append("range_noise")

    if degradation.blind_zone_m > 0.0:
        range_resolution_m = (max_range_m - min_range_m) / num_ranges
        blind_bins = min(num_ranges, int(round(degradation.blind_zone_m / range_resolution_m)))
        if blind_bins > 0:
            out[:blind_bins, :] = 0.0
            active.append("blind_zone")

    if degradation.false_echo_rate > 0.0:
        false_echo_mask = rng.random(size=(num_ranges, num_beams)) < degradation.false_echo_rate
        false_echo_intensity = rng.uniform(0.3, 1.0, size=int(false_echo_mask.sum()))
        out[false_echo_mask] = np.maximum(out[false_echo_mask], false_echo_intensity)
        active.append("false_echo")

    if degradation.range_scale_bias != 1.0:
        # Reported vs. actual range distortion: approximated here by
        # stretching/compressing the intensity image along the range axis
        # (a real range-scale bias would also need the caller to adjust the
        # published range_bins/min_range/max_range -- this only distorts
        # the intensity pattern, see docstring).
        source_rows = np.clip(
            (np.arange(num_ranges) / degradation.range_scale_bias).astype(int), 0, num_ranges - 1
        )
        out = out[source_rows, :]
        active.append("range_scale_bias")

    return np.clip(out, 0.0, 1.0).astype(np.float32), active
