import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools" / "lint" / "check_realtime_traceability.py"
REAL_CSV = ROOT / "docs" / "traceability" / "rov-realtime-closed-loop.csv"

HEADER = ["requirement_id", "scenario", "implementation_module", "test", "evidence_path", "status"]


def load_checker():
    spec = importlib.util.spec_from_file_location("realtime_traceability_checker", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_csv(path: Path, rows) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(HEADER)
        writer.writerows(rows)


def write_spec(path: Path, ids) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(f"`{rid}` some requirement text." for rid in ids), encoding="utf-8")


class RealtimeTraceabilityCheckerTest(unittest.TestCase):
    def _synthetic_root(self, tmp: str) -> Path:
        root = Path(tmp)
        write_spec(root / "docs/specifications/rov-competition-online-system-requirements.md", ["SYS-X-001"])
        write_spec(root / "docs/specifications/rov-acoustic-optic-online-fusion-spec.md", ["FUS-Y-001"])
        write_spec(
            root / "docs/specifications/holoocean-realtime-closed-loop-simulation-spec.md", ["SIM-Z-001"]
        )
        return root

    def _complete_rows(self):
        return [
            ["SYS-X-001", "n/a (architecture)", "module", "test_a.py", "test_a.py", "implemented"],
            ["FUS-Y-001", "n/a (architecture)", "module", "test_b.py", "test_b.py", "implemented"],
            ["SIM-Z-001", "n/a (architecture)", "module", "test_c.py", "test_c.py", "implemented"],
        ]

    def test_accepts_a_complete_csv_covering_every_requirement(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = self._synthetic_root(tmp)
            csv_path = root / "traceability.csv"
            (root / "test_a.py").write_text("", encoding="utf-8")
            (root / "test_b.py").write_text("", encoding="utf-8")
            (root / "test_c.py").write_text("", encoding="utf-8")
            write_csv(csv_path, self._complete_rows())
            self.assertEqual(load_checker().check(csv_path, root), [])

    def test_rejects_a_missing_requirement_row(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = self._synthetic_root(tmp)
            csv_path = root / "traceability.csv"
            rows = self._complete_rows()[:-1]  # drop SIM-Z-001
            write_csv(csv_path, rows)
            errors = load_checker().check(csv_path, root)
            self.assertTrue(any("SIM-Z-001" in e for e in errors))

    def test_rejects_an_empty_required_column(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = self._synthetic_root(tmp)
            csv_path = root / "traceability.csv"
            rows = self._complete_rows()
            rows[0][2] = ""  # empty implementation_module
            write_csv(csv_path, rows)
            errors = load_checker().check(csv_path, root)
            self.assertTrue(any("implementation_module" in e for e in errors))

    def test_rejects_an_invalid_status_enum_value(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = self._synthetic_root(tmp)
            csv_path = root / "traceability.csv"
            rows = self._complete_rows()
            rows[0][5] = "in_progress"
            write_csv(csv_path, rows)
            errors = load_checker().check(csv_path, root)
            self.assertTrue(any("status" in e for e in errors))

    def test_rejects_verified_row_whose_evidence_path_does_not_exist(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = self._synthetic_root(tmp)
            csv_path = root / "traceability.csv"
            rows = self._complete_rows()
            rows[0][5] = "verified"
            rows[0][4] = "does_not_exist.py"
            write_csv(csv_path, rows)
            errors = load_checker().check(csv_path, root)
            self.assertTrue(any("does_not_exist.py" in e for e in errors))

    def test_accepts_verified_row_whose_evidence_path_exists(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = self._synthetic_root(tmp)
            csv_path = root / "traceability.csv"
            (root / "test_a.py").write_text("", encoding="utf-8")
            (root / "test_b.py").write_text("", encoding="utf-8")
            (root / "test_c.py").write_text("", encoding="utf-8")
            rows = self._complete_rows()
            rows[0][5] = "verified"
            write_csv(csv_path, rows)
            self.assertEqual(load_checker().check(csv_path, root), [])

    def test_sys_proc_and_sim_s2r_requirements_must_be_gated(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_spec(root / "docs/specifications/rov-competition-online-system-requirements.md", ["SYS-PROC-001"])
            write_spec(root / "docs/specifications/rov-acoustic-optic-online-fusion-spec.md", ["FUS-Y-001"])
            write_spec(
                root / "docs/specifications/holoocean-realtime-closed-loop-simulation-spec.md", ["SIM-S2R-001"]
            )
            csv_path = root / "traceability.csv"
            write_csv(
                csv_path,
                [
                    ["SYS-PROC-001", "n/a (hardware)", "m", "t", "e", "implemented"],
                    ["FUS-Y-001", "n/a (architecture)", "m", "t", "e", "implemented"],
                    ["SIM-S2R-001", "n/a (pool)", "m", "t", "e", "gated"],
                ],
            )
            errors = load_checker().check(csv_path, root)
            self.assertTrue(any("SYS-PROC-001" in e and "gated" in e for e in errors))

    def test_rejects_duplicate_requirement_id_rows(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = self._synthetic_root(tmp)
            csv_path = root / "traceability.csv"
            rows = self._complete_rows()
            rows.append(rows[0])
            write_csv(csv_path, rows)
            errors = load_checker().check(csv_path, root)
            self.assertTrue(any("duplicate" in e for e in errors))

    def test_the_real_csv_covers_every_real_spec_requirement_with_no_gaps(self):
        checker = load_checker()
        errors = checker.check(REAL_CSV, ROOT)
        self.assertEqual(errors, [], msg="\n".join(errors))


if __name__ == "__main__":
    unittest.main()
