import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[2] / "tools" / "lint" / "check_layer_dependencies.py"


def load_checker():
    spec = importlib.util.spec_from_file_location("layer_checker", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class LayerDependencyTest(unittest.TestCase):
    def write(self, root: Path, relative: str, contents: str) -> None:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding="utf-8")

    def test_allows_frontend_to_use_core_and_generated_domain_headers(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "src/frontends/example.cpp",
                '#include "measurement_api/frontend.hpp"\n'
                '#include "sensor_models/geometry.hpp"\n'
                '#include "uw/domain/measurement.pb.h"\n',
            )
            self.assertEqual(load_checker().check(root), [])

    def test_rejects_estimation_to_frontend_dependency(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(root, "src/estimation/example.cpp", '#include "frontends/example.hpp"\n')
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("estimation must not include frontends", errors[0])

    def test_rejects_ros_header_outside_ros2_adapter(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(root, "src/frontends/example.cpp", "#include <rclcpp/rclcpp.hpp>\n")
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("ROS/vendor header", errors[0])

    def test_rejects_old_handwritten_uw_include(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(root, "src/runtime/example.cpp", '#include "uw/runtime/config.hpp"\n')
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("legacy hand-written include", errors[0])


if __name__ == "__main__":
    unittest.main()
