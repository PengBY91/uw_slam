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

    def test_allows_opencv_adapter_to_use_opencv_and_core_headers(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "adapters/opencv/src/stereo_rectifier.cpp",
                "#include <opencv2/calib3d.hpp>\n"
                '#include "measurement_api/frontend.hpp"\n'
                '#include "sensor_models/geometry.hpp"\n'
                "cv::Mat private_image;\n",
            )
            self.assertEqual(load_checker().check(root), [])

    def test_rejects_opencv_header_from_fake_include_src_role(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "src/opencv_adapters/probe.cpp",
                "#include <opencv2/core.hpp>\n",
            )
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("OpenCV header", errors[0])

    def test_rejects_opencv_header_and_type_from_public_adapter_header(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "adapters/opencv/include/opencv_adapters/leaky_api.hpp",
                "#include <opencv2/core.hpp>\n"
                "cv::Mat* LeakyImage();\n",
            )
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 2)
            self.assertTrue(any("OpenCV header" in error for error in errors))
            self.assertTrue(any("OpenCV type" in error for error in errors))

    def test_rejects_opencv_type_outside_adapter_while_ignoring_comments(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "include/frontends/leaky_api.hpp",
                "// cv::Mat in a line comment is documentation only.\n"
                "/* cv::Mat in a block comment is documentation only. */\n"
                "namespace cv { class Mat; }\n"
                "cv::Mat* LeakyImage();\n",
            )
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("OpenCV type", errors[0])

    def test_ignores_strings_but_detects_real_opencv_type_after_string(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "include/frontends/string_literals.hpp",
                'const char* type_name = "cv::Mat is text";\n'
                'const char* url = "https://example.test/path";\n'
                'const char* open = "/*";\n'
                'const char* close = "*/"; cv::Mat* real_image;\n',
            )
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("string_literals.hpp:4", errors[0])
            self.assertIn("OpenCV type", errors[0])

    def test_char_literals_and_escapes_do_not_hide_real_opencv_types(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "include/frontends/char_literals.hpp",
                "const int line_marker = '//'; cv::Mat* first;\n"
                "const int block_open = '/*';\n"
                "const int block_close = '*/'; cv::Mat* second;\n"
                "const char quote = '\\''; cv::Mat* third;\n"
                "const char backslash = '\\\\'; cv::Mat* fourth;\n",
            )
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 4)
            self.assertTrue(all("OpenCV type" in error for error in errors))

    def test_ignores_raw_string_contents_but_detects_real_opencv_type(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "include/frontends/raw_string.hpp",
                'const char* raw = R"tag(cv::Mat\n'
                'https://example.test/* markers */\n'
                ')tag";\n'
                'cv::Mat* real_image;\n',
            )
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("raw_string.hpp:4", errors[0])
            self.assertIn("OpenCV type", errors[0])

    def test_rejects_opencv_type_tokens_split_across_lines_and_comments(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "include/frontends/split_type.hpp",
                "cv\n"
                "::Mat* first;\n"
                "cv/* block\n"
                "comment */::Mat* second;\n",
            )
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 2)
            self.assertIn("split_type.hpp:1", errors[0])
            self.assertIn("split_type.hpp:3", errors[1])
            self.assertTrue(all("OpenCV type" in error for error in errors))

    def test_line_comment_backslash_newline_continues_to_next_physical_line(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "include/frontends/continued_comment.hpp",
                "// cv::Mat comment " + "\\\n" +
                "cv::Mat* still_comment;\n"
                "cv::Mat* real_image;\n",
            )
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("continued_comment.hpp:3", errors[0])
            self.assertIn("OpenCV type", errors[0])

    def test_ignores_opencv_include_inside_block_comment(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "include/frontends/commented_include.hpp",
                "/*\n"
                "#include <opencv2/core.hpp>\n"
                "*/\n",
            )
            self.assertEqual(load_checker().check(root), [])

    def test_ignores_opencv_include_inside_raw_string(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "include/frontends/raw_include.hpp",
                'const char* raw = R"tag(\n'
                '#include <opencv2/core.hpp>\n'
                ')tag";\n',
            )
            self.assertEqual(load_checker().check(root), [])

    def test_ignores_include_on_continued_line_comment(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "include/frontends/continued_include_comment.hpp",
                "// explanation " + "\\\n" +
                "#include <opencv2/core.hpp>\n",
            )
            self.assertEqual(load_checker().check(root), [])

    def test_rejects_opencv_include_after_same_line_block_comment(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "include/frontends/prefixed_include.hpp",
                "/* explanation */ #include <opencv2/core.hpp>\n",
            )
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("prefixed_include.hpp:1", errors[0])
            self.assertIn("OpenCV header", errors[0])

    def test_rejects_opencv_include_after_multiline_block_comment(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "include/frontends/multiline_prefixed_include.hpp",
                "/* explanation\n"
                "continued */ #include <opencv2/core.hpp>\n",
            )
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("multiline_prefixed_include.hpp:2", errors[0])
            self.assertIn("OpenCV header", errors[0])

    def test_quoted_project_include_after_comment_still_checks_role(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "src/estimation/comment_prefixed_include.cpp",
                '/* explanation */ #include "frontends/example.hpp"\n',
            )
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("estimation must not include frontends", errors[0])

    def test_rejects_opencv_header_outside_opencv_adapter(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(root, "src/frontends/example.cpp", "#include <opencv2/core.hpp>\n")
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("OpenCV header", errors[0])

    def test_rejects_opencv_adapter_to_frontend_dependency(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "adapters/opencv/src/stereo_rectifier.cpp",
                '#include "frontends/stereo_optical_depth_frontend.hpp"\n',
            )
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("opencv_adapters must not include frontends", errors[0])

    def test_rejects_estimation_to_frontend_dependency(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(root, "src/estimation/example.cpp", '#include "frontends/example.hpp"\n')
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("estimation must not include frontends", errors[0])

    def test_rejects_algorithm_layer_to_application_dependency(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "src/frontends/example.cpp",
                '#include "application/replay_pipeline.hpp"\n',
            )
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("frontends must not include application", errors[0])

    def test_allows_application_layer_to_compose_project_roles(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "src/application/replay_pipeline.cpp",
                '#include "frontends/example.hpp"\n'
                '#include "estimation/example.hpp"\n'
                '#include "mapping/example.hpp"\n'
                '#include "runtime/example.hpp"\n'
                '#include "opencv_adapters/stereo_rectifier.hpp"\n',
            )
            self.assertEqual(load_checker().check(root), [])

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
