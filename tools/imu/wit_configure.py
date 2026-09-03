#!/usr/bin/env python3
"""Thin entry point for adapters/wit_imu's HWT9053-485 commissioning tool,
under the path PREP-D-01 names (`tools/imu/wit_configure.py`). All the
logic — and every constant that needs checking against the manual — lives
in adapters/wit_imu/uw_wit_imu/{configure,registers,protocol}.py.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "adapters" / "wit_imu"))

from uw_wit_imu.configure import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
