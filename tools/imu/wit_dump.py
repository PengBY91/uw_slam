#!/usr/bin/env python3
"""Thin entry point for adapters/wit_imu's packet counter, under the path
PREP-D-01's acceptance criterion names (`tools/imu/wit_dump.py`)."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "adapters" / "wit_imu"))

from uw_wit_imu.dump import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
