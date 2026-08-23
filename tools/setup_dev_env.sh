#!/usr/bin/env bash
# Installs the C++ toolchain dependencies (Eigen3, Protobuf+protoc, GTest,
# OpenCV, CMake) needed to build this repo. Tries apt first (works on a normal
# Ubuntu machine with unrestricted network); falls back to a dedicated
# conda-forge environment if apt's package mirrors are unreachable/throttled
# — that fallback is what this repo was actually built and tested with, in
# a sandboxed environment where apt's HTTP(80) mirrors stalled for many
# minutes but HTTPS(443) (used by conda-forge and PyPI) worked fine.
#
# Either path leaves you with a working toolchain; the apt path installs
# system-wide, the conda path creates an isolated `uw_slam_build`
# environment you activate before building (see README.md).
set -euo pipefail

APT_PACKAGES=(protobuf-compiler libprotobuf-dev libeigen3-dev libgtest-dev libopencv-dev cmake build-essential)
CONDA_ENV_NAME="uw_slam_build"
CONDA_PACKAGES=(eigen libprotobuf protobuf gtest "opencv=4.*" cmake)

try_apt() {
  command -v apt-get >/dev/null 2>&1 || return 1
  echo "Trying apt-get (timeout 60s to detect a stalled/throttled mirror)..."
  if timeout 60 sudo apt-get update -qq && timeout 300 sudo apt-get install -y -qq "${APT_PACKAGES[@]}"; then
    echo "OK: installed via apt-get."
    return 0
  fi
  echo "apt-get did not complete in time; falling back to conda-forge."
  return 1
}

try_conda() {
  local conda_bin=""
  if command -v conda >/dev/null 2>&1; then
    conda_bin="$(command -v conda)"
  elif [ -x "$HOME/miniconda3/bin/conda" ]; then
    conda_bin="$HOME/miniconda3/bin/conda"
  else
    echo "No conda installation found; cannot fall back. Install Miniconda or fix apt network access."
    return 1
  fi

  echo "Creating/using conda env '$CONDA_ENV_NAME' via $conda_bin ..."
  "$conda_bin" create -y -n "$CONDA_ENV_NAME" -c conda-forge --override-channels "${CONDA_PACKAGES[@]}"
  local env_prefix
  env_prefix="$("$conda_bin" info --base)/envs/$CONDA_ENV_NAME"
  echo "OK: environment ready at $env_prefix"
  echo
  echo "Build with:"
  echo "  export PATH=$env_prefix/bin:\$PATH"
  echo "  cmake -S . -B build -DCMAKE_PREFIX_PATH=$env_prefix"
  echo "  cmake --build build -j\$(nproc)"
}

if ! try_apt; then
  try_conda
fi
