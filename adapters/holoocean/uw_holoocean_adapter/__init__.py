"""uw_holoocean_adapter: the real replacement for the legacy ocean_t scripts.

Per acoustic-optic-slam-platform-architecture section 22.3, ocean_t's
svin2_pipeline.py/main.py/water_control_panel.py had several concrete,
audited problems this package deliberately does not repeat:

  - fabricated FLS elevation (np.random.uniform per return, treated as a
    real 3D point) -> this package never assigns elevation at all; that is
    algorithms/frontends/sonar_cfar_frontend's job, downstream of the
    canonical bag this package writes, and it stays in range-bearing form.
  - non-deterministic replay (np.random.seed() called with no argument,
    every frame) -> every random draw in this package goes through an
    explicitly-seeded RNG instance threaded through the call sites; nothing
    here reseeds a global RNG.
  - no capture/receive time distinction -> see uw_holoocean_adapter.time.
  - module-level global state / hardcoded paths -> this package has no
    module-level mutable state; scenario parameters are passed explicitly
    (see scenario_randomization.py).
"""
