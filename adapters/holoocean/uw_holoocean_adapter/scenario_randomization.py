"""Programmatic scenario randomization, replacing ocean_t's
water_control_panel.py.

Audit finding (platform architecture section 22.3): water_control_panel.py
only exposed two randomization axes (water_color, water_fog) through GUI
sliders with four hardcoded presets, and could not be driven
programmatically for batch dataset generation. Architecture section 11
requires covering turbidity/attenuation/backscatter/illumination/exposure,
sonar speckle/range noise/beam-FOV/gain, sound speed/range scale, time
offset/extrinsic perturbation/packet loss/latency, and camera-sonar
desync — this module defines that full parameter space as a typed,
sampleable dataclass instead.

Determinism (the other audit finding — svin2_pipeline.py called
`np.random.seed()` with no argument on every frame, breaking L2
determinism): every sampling function here takes an explicit
`numpy.random.Generator`, constructed ONCE per run from a scenario seed by
the caller. Nothing in this module touches global numpy random state.
"""
from __future__ import annotations

import dataclasses

import numpy as np


@dataclasses.dataclass(frozen=True)
class VisualDegradation:
    turbidity: float = 0.0  # 0 = clear, 1 = maximum modeled turbidity
    attenuation_per_m: float = 0.05
    backscatter_gain: float = 0.0
    illumination_scale: float = 1.0
    exposure_ev: float = 0.0
    # Task 5 additions (fault_injector.py/sensor_perturbation.py) — kept as
    # plain floats with numeric (never None) defaults so
    # sample_uniform_sweep's recursive `sorted((low, high))` field sampler
    # below keeps working: a None-sentinel field would make that sort raise
    # (`'<' not supported between instances of 'NoneType' and 'NoneType'`).
    motion_blur_px: float = 0.0
    particle_density: float = 0.0  # 0 = none, 1 = maximum modeled particulate matter
    local_overexposure_probability: float = 0.0
    stereo_exposure_mismatch_ev: float = 0.0  # extra exposure bias applied to the right camera only


@dataclasses.dataclass(frozen=True)
class SonarDegradation:
    speckle_sigma: float = 0.02  # multiplicative speckle noise std, fraction of intensity
    range_noise_sigma_m: float = 0.02
    beam_fov_perturbation_rad: float = 0.0
    gain_db: float = 0.0
    sound_speed_scale: float = 1.0  # multiplies the nominal speed of sound
    # Task 5 additions — see VisualDegradation's comment on why these stay
    # plain floats/never None.
    blind_zone_m: float = 0.0  # near-range band forced to zero intensity
    false_echo_rate: float = 0.0  # probability per beam of a synthetic sidelobe/false echo
    range_scale_bias: float = 1.0  # multiplicative bias on the reported range axis, 1.0 = none


@dataclasses.dataclass(frozen=True)
class TimingAndCalibrationDegradation:
    time_offset_s: float = 0.0  # fixed clock skew applied to one sensor stream
    extrinsic_translation_perturbation_m: float = 0.0
    extrinsic_rotation_perturbation_rad: float = 0.0
    packet_loss_probability: float = 0.0
    latency_jitter_s: float = 0.0
    camera_sonar_desync_s: float = 0.0  # additional desync beyond time_offset_s
    # Task 5 additions (fault_injector.py's per-topic TopicFaultConfig is
    # the actual runtime mechanism these numbers parameterize) — see
    # VisualDegradation's comment on why these stay plain floats/never None.
    duplicate_probability: float = 0.0
    reorder_probability: float = 0.0
    reorder_max_distance: float = 3.0
    outage_probability_per_s: float = 0.0
    outage_duration_s: float = 0.0


@dataclasses.dataclass(frozen=True)
class ScenarioRandomization:
    visual: VisualDegradation = dataclasses.field(default_factory=VisualDegradation)
    sonar: SonarDegradation = dataclasses.field(default_factory=SonarDegradation)
    timing: TimingAndCalibrationDegradation = dataclasses.field(
        default_factory=TimingAndCalibrationDegradation
    )


# Named presets replace water_control_panel.py's four hardcoded GUI presets
# with the same intent (quick manual testing) but as plain data, so they
# can also be used as one point in a scripted sweep rather than only via a
# slider.
PRESET_CLEAR = ScenarioRandomization()
PRESET_TURBID = ScenarioRandomization(
    visual=VisualDegradation(turbidity=0.6, attenuation_per_m=0.35, backscatter_gain=0.4),
    sonar=SonarDegradation(speckle_sigma=0.05),
)
PRESET_DEEP = ScenarioRandomization(
    visual=VisualDegradation(illumination_scale=0.2, exposure_ev=1.5),
)
PRESET_CRITICAL_DEGRADED = ScenarioRandomization(
    visual=VisualDegradation(turbidity=0.9, attenuation_per_m=0.6, backscatter_gain=0.7,
                              illumination_scale=0.1),
    sonar=SonarDegradation(speckle_sigma=0.08, range_noise_sigma_m=0.05),
    timing=TimingAndCalibrationDegradation(packet_loss_probability=0.05, latency_jitter_s=0.02),
)


def sample_uniform_sweep(
    rng: np.random.Generator,
    low: ScenarioRandomization,
    high: ScenarioRandomization,
) -> ScenarioRandomization:
    """Samples one ScenarioRandomization with each numeric field drawn
    uniformly between the corresponding fields of `low` and `high`. Used for
    batch dataset generation sweeps; `rng` must be a Generator the caller
    owns and seeds explicitly (see module docstring)."""

    def sample_dataclass(low_obj, high_obj):
        kwargs = {}
        for field in dataclasses.fields(low_obj):
            low_value = getattr(low_obj, field.name)
            high_value = getattr(high_obj, field.name)
            if dataclasses.is_dataclass(low_value):
                kwargs[field.name] = sample_dataclass(low_value, high_value)
            else:
                # Not every field increases from `low` to `high` in the same
                # direction (e.g. illumination_scale DECREASES as a scenario
                # gets more degraded, while turbidity increases) — sort the
                # bound instead of assuming low_value <= high_value.
                lo, hi = sorted((low_value, high_value))
                kwargs[field.name] = float(rng.uniform(lo, hi))
        return type(low_obj)(**kwargs)

    return sample_dataclass(low, high)


def sample_run_randomization(
    seed: int,
    low: ScenarioRandomization,
    high: ScenarioRandomization,
) -> ScenarioRandomization:
    """Same as `sample_uniform_sweep`, but owns its `Generator` construction
    from a plain integer `seed` instead of taking a caller-owned `Generator`
    — for run-level callers (HoloOcean session setup, the realtime gate)
    that have a single seed and want one deterministic sample, not a batch
    sweep where reusing one `Generator` across many samples matters."""
    return sample_uniform_sweep(np.random.default_rng(seed), low, high)
