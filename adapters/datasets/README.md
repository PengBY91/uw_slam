# Public dataset adapters (stub)

Converts public SLAM/underwater datasets (e.g. EuRoC-style stereo+IMU sets, SVIn's own public
datasets, RUSSO/Tank/SonarSweep if/when available — see the platform architecture's sim-to-real
section) into the canonical MCAP schema defined in `schemas/proto/uw/domain/` — the same wire format
`adapters/holoocean` and `apps/synth_bag_gen` produce, so `apps/replay_demo` and the rest of the
pipeline don't need to know or care where a bag came from.

Not implemented in this pass: no public dataset has been converted or tested against this repo yet.
When adding one, follow the pattern in `apps/synth_bag_gen.cpp` (topics, message types,
keyframe/observation-id conventions) rather than inventing a new one per dataset.
