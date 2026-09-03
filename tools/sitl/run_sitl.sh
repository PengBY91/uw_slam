#!/usr/bin/env bash
# PREP-C-01: start ArduSub SITL (BlueROV2 Heavy = vectored_6dof frame) headless.
#   tools/sitl/run_sitl.sh        # MAVLink served by the SITL binary itself on tcp:127.0.0.1:5760 (no MAVProxy)
# Any MAVLink client (pymavlink `tcp:127.0.0.1:5760`, QGroundControl on the
# Windows host via TCP to the WSL2 IP, port 5760) connects there directly;
# MAVProxy is deliberately not started because it needs an interactive
# terminal and would own the only UDP fan-out. The JSON physics backend for
# the HoloOcean bridge (PREP-A-05) is selected with ARDUSUB_MODEL=JSON.
#
# SITL_RATE_HZ sets SIM_RATE_HZ, the physics timestep SITL asks the external
# backend for. It MUST equal the bridge's --step-hz and (with HoloOcean
# behind it) the scenario's ticks_per_sec, or the two simulators' clocks
# diverge while both believe they are in sync. ArduSub's own default is
# 1200 Hz on SITL builds (SITL.cpp SIM_RATE_HZ_DEFAULT -- 1200, not the 400
# the JSON backend's readme quotes for copter), which no external physics
# backend can serve, so ARDUSUB_MODEL=JSON without SITL_RATE_HZ is a
# misconfiguration the bridge warns about.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
MODEL="${ARDUSUB_MODEL:-vectored_6dof}"
export PATH="/usr/bin:/bin:/usr/local/bin:$HOME/.local/bin:$PATH"
unset CONDA_PREFIX CONDA_PYTHON_EXE || true
VENV="${SITL_VENV:-$SCRIPT_DIR/.venv}"

# ArduSub's SITL barometer is a WATER baro: AP_Baro_SITL feeds
# SimpleUnderWaterAtmosphere(-altitude_amsl), so the vehicle's depth is
# taken to be MINUS its AMSL altitude. sim_vehicle.py's default home (CMAC,
# 584 m) therefore makes ArduSub believe it is 581 m ABOVE the water and
# report press_abs = -55925 hPa, which is what PREP-C-01's smoke script was
# printing without checking. Home must sit at sea level for depth,
# DEPTH_HOLD or any depth-dependent behaviour to mean anything; override
# with SITL_HOME if a different site is wanted, but keep the altitude at 0.
SITL_HOME="${SITL_HOME:--35.363261,149.165230,0,0}"
EXTRA_ARGS=(--custom-location="$SITL_HOME")
if [[ -n "${SITL_RATE_HZ:-}" ]]; then
  PARAM_FILE="$(mktemp -t uw_sitl_rate_XXXXXX.parm)"
  trap 'rm -f "$PARAM_FILE"' EXIT
  printf 'SIM_RATE_HZ %s\n' "$SITL_RATE_HZ" > "$PARAM_FILE"
  EXTRA_ARGS+=(--add-param-file="$PARAM_FILE")
fi
if [[ -n "${SITL_PARAM_FILE:-}" ]]; then
  EXTRA_ARGS+=(--add-param-file="$SITL_PARAM_FILE")
fi

cd "$ARDUPILOT_DIR"
exec "$VENV/bin/python" Tools/autotest/sim_vehicle.py -v ArduSub -f vectored_6dof --model "$MODEL" --no-mavproxy \
  --no-rebuild --speedup 1 "${EXTRA_ARGS[@]}" "$@"
