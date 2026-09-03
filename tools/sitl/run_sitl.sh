#!/usr/bin/env bash
# PREP-C-01: start ArduSub SITL (BlueROV2 Heavy = vectored_6dof frame) headless.
#   tools/sitl/run_sitl.sh        # MAVLink served by the SITL binary itself on tcp:127.0.0.1:5760 (no MAVProxy)
# Any MAVLink client (pymavlink `tcp:127.0.0.1:5760`, QGroundControl on the
# Windows host via TCP to the WSL2 IP, port 5760) connects there directly;
# MAVProxy is deliberately not started because it needs an interactive
# terminal and would own the only UDP fan-out. The JSON physics backend for
# the HoloOcean bridge (PREP-A-05) is selected with ARDUSUB_MODEL=JSON.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
MODEL="${ARDUSUB_MODEL:-vectored_6dof}"
export PATH="/usr/bin:/bin:/usr/local/bin:$HOME/.local/bin:$PATH"
unset CONDA_PREFIX CONDA_PYTHON_EXE || true
VENV="${SITL_VENV:-$SCRIPT_DIR/.venv}"
cd "$ARDUPILOT_DIR"
exec "$VENV/bin/python" Tools/autotest/sim_vehicle.py -v ArduSub -f vectored_6dof --model "$MODEL" --no-mavproxy \
  --no-rebuild --speedup 1 "$@"
