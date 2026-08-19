# `sonar_camera_reconstruction` external baseline runner

Per platform architecture section 19: `sonar_camera_reconstruction` is compared as an **unmodified**
external baseline, on the same bag/calibration/metrics as this platform's native
`sonar_cfar_frontend` — it is never linked into `core`/`algorithms` (see
`algorithms/frontends/sonar_cfar_frontend`'s `NOTICE` entry for what geometry *was* ported natively,
which is a separate, deliberate exception limited to CFAR/DBSCAN/polar math, not the ROS node itself).

## Known blockers (from the code audit, not yet resolved here)

1. `sonar_camera_reconstruction_pkg/package.xml` depends on `bruce_slam`, a sibling package from the
   same lab that is **not vendored in this repo** — the original repository does not build standalone.
   Resolving this (vendor a minimal subset, or a maintained fork/patch per the platform architecture's
   fork/patch policy) is a prerequisite, not done in this pass.
2. The repository's main branch is ROS1 Noetic; this machine has neither ROS1 nor `bruce_slam`
   installed, so `run_baseline.sh` below is a documented skeleton, not a verified script.

### A working reference for both blockers

A colleague's separate deployment package (`workfiles_02/src/sonar_camera_reconstruction_pkg/`,
outside this repo) already carries a `bruce_slam`-free `package.xml` and a ROS2 Humble
`launch/merge.launch.py` alongside the upstream ROS1 `merge.launch` — i.e. both blockers above have
already been resolved once, just not in this repository. On a machine with ROS2 available, that
package (plus its sibling `holoocean_bridge` for the HoloOcean-topic-to-`sonar_oculus/OculusPing`
conversion this baseline also needs) is worth reading before re-deriving the same fix here — it is
**not** vendored into this repo (different license/provenance chain, not audited for this platform's
GPLv3 NOTICE requirements), so treat it as a reference to consult, not code to copy in wholesale.

## Intended usage (once the blockers above are resolved)

```bash
./run_baseline.sh --bag <canonical.mcap-derived ROS1 bag> --out outputs/baseline/
```

`run_baseline.sh` is expected to:
1. Convert the canonical MCAP bag's `/evidence/*` and raw sensor topics into the ROS1 topics
   `sonar_camera_reconstruction`'s `merge.launch` expects (`sonar_oculus/OculusPing`,
   `sensor_msgs/CompressedImage`, `nav_msgs/Odometry` — see the platform architecture's field mapping
   in section 22.4 for the concrete OculusPing conversion).
2. Run `roslaunch sonar_camera_reconstruction_pkg merge.launch` unmodified.
3. Record `/sonar_camera_reconstruction/cloud` and hand it to `evaluation/` alongside this platform's
   native map output, for an apples-to-apples comparison.
