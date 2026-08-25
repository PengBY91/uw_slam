#!/usr/bin/env python3
"""Checks docs/traceability/rov-realtime-closed-loop.csv against the SYS-*/
FUS-*/SIM-* requirement IDs defined in the three approved specification
files, and validates the CSV's own shape.

Usage: python tools/lint/check_realtime_traceability.py <csv-path> [repo-root]
"""
import csv
import re
import sys
from pathlib import Path

REQUIRED_COLUMNS = ("requirement_id", "scenario", "implementation_module", "test", "evidence_path", "status")
ALLOWED_STATUSES = {"implemented", "verified", "gated", "failed"}
ID_RE = re.compile(r"`((?:SYS|FUS|SIM)-[A-Z0-9]+-\d{3})`")

SPEC_FILES = (
    "docs/specifications/rov-competition-online-system-requirements.md",
    "docs/specifications/rov-acoustic-optic-online-fusion-spec.md",
    "docs/specifications/holoocean-realtime-closed-loop-simulation-spec.md",
)


def extract_required_ids(root: Path):
    ids = set()
    for relative in SPEC_FILES:
        path = root / relative
        if not path.is_file():
            raise FileNotFoundError(f"missing specification file: {relative}")
        ids.update(ID_RE.findall(path.read_text(encoding="utf-8")))
    return ids


def load_rows(csv_path: Path):
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or list(reader.fieldnames) != list(REQUIRED_COLUMNS):
            raise ValueError(
                f"CSV header must be exactly {list(REQUIRED_COLUMNS)}, got {reader.fieldnames}"
            )
        return list(reader)


def check(csv_path: Path, root: Path):
    errors = []
    try:
        required_ids = extract_required_ids(root)
    except FileNotFoundError as exc:
        return [str(exc)]

    try:
        rows = load_rows(csv_path)
    except ValueError as exc:
        return [str(exc)]

    seen_ids = {}
    for line_number, row in enumerate(rows, start=2):
        rid = row.get("requirement_id", "")
        location = f"{csv_path}:{line_number}"
        if rid in seen_ids:
            errors.append(f"{location}: duplicate requirement_id {rid!r} (first seen line {seen_ids[rid]})")
        else:
            seen_ids[rid] = line_number

        for column in REQUIRED_COLUMNS:
            if not row.get(column, "").strip():
                errors.append(f"{location}: {rid or '<missing id>'} has an empty {column!r} column")

        status = row.get("status", "")
        if status not in ALLOWED_STATUSES:
            errors.append(
                f"{location}: {rid!r} has status {status!r}, must be one of {sorted(ALLOWED_STATUSES)}"
            )

        if status == "verified":
            evidence = row.get("evidence_path", "")
            if evidence and not (root / evidence).exists():
                errors.append(
                    f"{location}: {rid!r} is 'verified' but evidence_path {evidence!r} does not exist"
                )

        if rid.startswith("SYS-PROC-") or rid == "SIM-S2R-001":
            if status != "gated":
                errors.append(
                    f"{location}: {rid!r} must use status 'gated' until its external evidence exists, got {status!r}"
                )

    missing_ids = required_ids - set(seen_ids)
    for rid in sorted(missing_ids):
        errors.append(f"{csv_path}: missing a row for requirement {rid}")

    extra_ids = set(seen_ids) - required_ids
    for rid in sorted(extra_ids):
        errors.append(f"{csv_path}: row for {rid!r} does not match any requirement ID in the three spec files")

    return errors


def main(argv):
    if len(argv) < 2:
        print("usage: check_realtime_traceability.py <csv-path> [repo-root]", file=sys.stderr)
        return 2
    csv_path = Path(argv[1])
    root = Path(argv[2]) if len(argv) > 2 else Path(__file__).resolve().parents[2]

    errors = check(csv_path, root)
    if errors:
        print("Realtime traceability check failed:", file=sys.stderr)
        for error in errors:
            print(f"  {error}", file=sys.stderr)
        return 1
    print("OK: realtime traceability CSV covers every SYS-*/FUS-*/SIM-* requirement")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
