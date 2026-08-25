import numpy as np

from uw_holoocean_adapter.scenario_randomization import (
    PRESET_CLEAR,
    PRESET_CRITICAL_DEGRADED,
    ScenarioRandomization,
    sample_run_randomization,
    sample_uniform_sweep,
)


def test_sampling_is_deterministic_given_the_same_seed():
    rng_a = np.random.default_rng(123)
    rng_b = np.random.default_rng(123)

    sample_a = sample_uniform_sweep(rng_a, PRESET_CLEAR, PRESET_CRITICAL_DEGRADED)
    sample_b = sample_uniform_sweep(rng_b, PRESET_CLEAR, PRESET_CRITICAL_DEGRADED)

    assert sample_a == sample_b


def test_sampled_values_stay_within_bounds():
    rng = np.random.default_rng(7)
    sample = sample_uniform_sweep(rng, PRESET_CLEAR, PRESET_CRITICAL_DEGRADED)

    assert PRESET_CLEAR.visual.turbidity <= sample.visual.turbidity <= PRESET_CRITICAL_DEGRADED.visual.turbidity
    assert (
        PRESET_CLEAR.sonar.speckle_sigma
        <= sample.sonar.speckle_sigma
        <= PRESET_CRITICAL_DEGRADED.sonar.speckle_sigma
    )


def test_default_scenario_randomization_is_clear_preset_equivalent():
    assert ScenarioRandomization() == PRESET_CLEAR


def test_same_seed_samples_same_layout_noise_and_faults():
    a = sample_run_randomization(42, PRESET_CLEAR, PRESET_CRITICAL_DEGRADED)
    b = sample_run_randomization(42, PRESET_CLEAR, PRESET_CRITICAL_DEGRADED)
    assert a == b
    assert sample_run_randomization(43, PRESET_CLEAR, PRESET_CRITICAL_DEGRADED) != a
