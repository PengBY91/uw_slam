import numpy as np
import pytest

from uw_holoocean_adapter.scenario_randomization import PRESET_CLEAR, PRESET_CRITICAL_DEGRADED
from uw_holoocean_adapter.sensor_perturbation import perturb_image, perturb_sonar, perturb_stereo_pair


def _image(seed=0):
    rng = np.random.default_rng(seed)
    return rng.integers(0, 256, size=(16, 20, 3), dtype=np.uint8)


def _sonar(seed=0):
    rng = np.random.default_rng(seed)
    return rng.uniform(0.0, 1.0, size=(24, 18)).astype(np.float32)


def test_perturb_image_same_seed_is_byte_identical():
    image = _image()
    out_a, _ = perturb_image(np.random.default_rng(7), PRESET_CRITICAL_DEGRADED.visual, image)
    out_b, _ = perturb_image(np.random.default_rng(7), PRESET_CRITICAL_DEGRADED.visual, image)
    assert np.array_equal(out_a, out_b)


def test_perturb_image_different_seed_differs():
    image = _image()
    out_a, _ = perturb_image(np.random.default_rng(7), PRESET_CRITICAL_DEGRADED.visual, image)
    out_b, _ = perturb_image(np.random.default_rng(8), PRESET_CRITICAL_DEGRADED.visual, image)
    assert not np.array_equal(out_a, out_b)


def test_perturb_image_preserves_shape_dtype_and_valid_range():
    image = _image()
    out, _ = perturb_image(np.random.default_rng(1), PRESET_CRITICAL_DEGRADED.visual, image)
    assert out.shape == image.shape
    assert out.dtype == np.uint8
    assert out.min() >= 0 and out.max() <= 255


def test_perturb_image_clean_preset_visibly_differs_from_critical_preset():
    image = _image()
    clean, clean_active = perturb_image(np.random.default_rng(1), PRESET_CLEAR.visual, image)
    critical, critical_active = perturb_image(np.random.default_rng(1), PRESET_CRITICAL_DEGRADED.visual, image)
    assert not np.array_equal(clean, critical)
    assert critical_active
    # PRESET_CLEAR still has VisualDegradation's baseline attenuation_per_m
    # default (0.05, not a "clear" no-op -- only turbidity=0 means clear
    # water), so "attenuation" is expected active; nothing else should be
    # (backscatter_gain/exposure_ev/motion_blur_px/particle_density/
    # local_overexposure_probability/illumination_scale are all genuine
    # no-op defaults).
    assert clean_active == ["attenuation"]


def test_perturb_image_rejects_wrong_dtype():
    with pytest.raises(ValueError):
        perturb_image(np.random.default_rng(1), PRESET_CLEAR.visual, _image().astype(np.float32))


def test_perturb_stereo_pair_applies_exposure_mismatch_only_to_right():
    from dataclasses import replace

    from uw_holoocean_adapter.scenario_randomization import VisualDegradation

    degradation = replace(VisualDegradation(), stereo_exposure_mismatch_ev=2.0)
    left = np.full((8, 8, 3), 50, dtype=np.uint8)
    right = np.full((8, 8, 3), 50, dtype=np.uint8)

    left_out, right_out, active = perturb_stereo_pair(np.random.default_rng(3), degradation, left, right)

    assert left_out.mean() < right_out.mean()
    assert "stereo_exposure_mismatch" in active


def test_perturb_sonar_same_seed_is_byte_identical():
    sonar = _sonar()
    out_a, _ = perturb_sonar(
        np.random.default_rng(4), PRESET_CRITICAL_DEGRADED.sonar, sonar, min_range_m=0.3, max_range_m=30.0
    )
    out_b, _ = perturb_sonar(
        np.random.default_rng(4), PRESET_CRITICAL_DEGRADED.sonar, sonar, min_range_m=0.3, max_range_m=30.0
    )
    assert np.array_equal(out_a, out_b)


def test_perturb_sonar_different_seed_differs():
    sonar = _sonar()
    out_a, _ = perturb_sonar(
        np.random.default_rng(4), PRESET_CRITICAL_DEGRADED.sonar, sonar, min_range_m=0.3, max_range_m=30.0
    )
    out_b, _ = perturb_sonar(
        np.random.default_rng(5), PRESET_CRITICAL_DEGRADED.sonar, sonar, min_range_m=0.3, max_range_m=30.0
    )
    assert not np.array_equal(out_a, out_b)


def test_perturb_sonar_preserves_shape_dtype_and_valid_range():
    sonar = _sonar()
    out, _ = perturb_sonar(
        np.random.default_rng(2), PRESET_CRITICAL_DEGRADED.sonar, sonar, min_range_m=0.3, max_range_m=30.0
    )
    assert out.shape == sonar.shape
    assert out.dtype == np.float32
    assert out.min() >= 0.0 and out.max() <= 1.0


def test_perturb_sonar_blind_zone_zeros_the_near_range_rows():
    from dataclasses import replace

    sonar = np.ones((10, 6), dtype=np.float32)
    degradation = replace(PRESET_CLEAR.sonar, speckle_sigma=0.0, range_noise_sigma_m=0.0, blind_zone_m=3.0)

    out, active = perturb_sonar(np.random.default_rng(1), degradation, sonar, min_range_m=0.0, max_range_m=10.0)

    assert "blind_zone" in active
    assert np.array_equal(out[:3, :], np.zeros((3, 6), dtype=np.float32))
    assert np.array_equal(out[3:, :], np.ones((7, 6), dtype=np.float32))


def test_perturb_sonar_rejects_invalid_range_bounds():
    with pytest.raises(ValueError):
        perturb_sonar(np.random.default_rng(1), PRESET_CLEAR.sonar, _sonar(), min_range_m=10.0, max_range_m=5.0)
