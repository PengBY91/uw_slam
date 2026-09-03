#!/usr/bin/env bash
# PREP-C-01: one-shot ArduSub SITL setup for WSL2 (docs/ROV平台到货前准备工作规格-2026-09-02.md).
# Clones ArduPilot (Sub-4.5 branch, shallow + shallow submodules), installs
# the upstream prerequisites, and builds the SITL binary once so that
# run_sitl.sh starts in seconds afterwards.
#
# Deliberately uses the SYSTEM python (CLAUDE.md: conda's python must not be
# first on PATH for ArduPilot/colcon-style tooling) and the apt proxy already
# configured in /etc/apt/apt.conf.d/95proxy.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
BRANCH="${ARDUPILOT_BRANCH:-Sub-4.5}"
export PATH="/usr/bin:/bin:/usr/local/bin:$PATH"
unset CONDA_PREFIX CONDA_PYTHON_EXE || true

if [ ! -d "$ARDUPILOT_DIR/.git" ]; then
  git clone --depth 1 -b "$BRANCH" --recurse-submodules --shallow-submodules \
    https://github.com/ArduPilot/ardupilot "$ARDUPILOT_DIR"
fi
cd "$ARDUPILOT_DIR"
git submodule update --init --recursive --depth 1
# NOT the upstream Tools/environment_install/install-prereqs-ubuntu.sh: on
# Ubuntu 24.04 (noble) the Sub-4.5 copy of that script still apt-installs the
# long-gone `python-argparse` package and aborts. The apt list below is the
# subset SITL actually needs; the Python build tools go into the repo's
# tools/sitl/.venv (system python, never conda) instead of `pip --user`.
sudo apt-get --assume-yes install build-essential ccache g++ gawk git make wget \
  libtool libtool-bin libxml2-dev libxslt1-dev python3-dev python3-venv pkg-config
# ArduPilot Sub-4.5 ships a waf that still imports `imp` (removed in Python
# 3.12), so the venv must be built on Python <= 3.11. Ubuntu 24.04's system
# python is 3.12; this machine has a uv-managed 3.11 at ~/.local/bin/python3.11.
SITL_PYTHON="${SITL_PYTHON:-$(command -v python3.11 || command -v "$HOME/.local/bin/python3.11" || true)}"
if [ -z "$SITL_PYTHON" ]; then
  echo "need a Python 3.11 interpreter for ArduPilot's bundled waf (set SITL_PYTHON or: uv python install 3.11)" >&2
  exit 1
fi
VENV="${SITL_VENV:-$SCRIPT_DIR/.venv}"
if [ ! -x "$VENV/bin/python" ] || ! "$VENV/bin/python" -c 'import sys; sys.exit(0 if sys.version_info < (3, 12) else 1)'; then
  rm -rf "$VENV"
  "$SITL_PYTHON" -m venv "$VENV"
fi
"$VENV/bin/pip" install -q --upgrade pip
"$VENV/bin/pip" install -q "empy==3.3.4" pexpect future pymavlink MAVProxy dronecan setuptools wheel
"$VENV/bin/python" ./waf configure --board sitl
"$VENV/bin/python" ./waf sub
echo "SITL_SETUP_DONE $(git rev-parse --short HEAD)"
