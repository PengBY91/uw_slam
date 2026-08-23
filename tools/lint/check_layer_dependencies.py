#!/usr/bin/env python3
import re
import sys
from pathlib import Path


INCLUDE_RE = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]')
INCLUDE_DIRECTIVE_RE = re.compile(r"^\s*#\s*include\b")
CV_TYPE_RE = re.compile(r"\bcv\s*::")
GENERATED_PROTO_RE = re.compile(r"^uw/domain/[^/]+\.pb\.h$")
ROS_VENDOR_PREFIXES = (
    "rclcpp/",
    "ros/",
    "rmw/",
    "nav_msgs/",
    "sensor_msgs/",
    "geometry_msgs/",
    "holoocean_interfaces/",
    "okvis/",
    "sonar_oculus/",
)
PROJECT_ROLES = {
    "domain",
    "sensor_models",
    "measurement_api",
    "frontends",
    "factor_builders",
    "estimation",
    "mapping",
    "runtime",
    "evaluation",
    "adapters",
    "opencv_adapters",
    "application",
}
INCLUDE_SRC_OWNERS = PROJECT_ROLES - {"opencv_adapters"}
ALLOWED = {
    "domain": {"domain", "domain_proto"},
    "sensor_models": {"sensor_models", "domain", "domain_proto"},
    "measurement_api": {"measurement_api", "sensor_models", "domain", "domain_proto"},
    "frontends": {"frontends", "measurement_api", "sensor_models", "domain", "domain_proto"},
    "factor_builders": {
        "factor_builders", "measurement_api", "sensor_models", "domain", "domain_proto"
    },
    "estimation": {"estimation", "measurement_api", "sensor_models", "domain", "domain_proto"},
    "mapping": {"mapping", "measurement_api", "sensor_models", "domain", "domain_proto"},
    "runtime": {"runtime", "measurement_api", "sensor_models", "domain", "domain_proto"},
    "evaluation": {"evaluation", "sensor_models", "domain", "domain_proto"},
    "adapters": {"adapters", "measurement_api", "sensor_models", "domain", "domain_proto"},
    "opencv_adapters": {
        "opencv_adapters", "measurement_api", "sensor_models", "domain", "domain_proto"
    },
    "application": PROJECT_ROLES | {"domain_proto"},
    "ros2": {"adapters", "measurement_api", "sensor_models", "domain", "domain_proto"},
    "apps": PROJECT_ROLES | {"domain_proto"},
}


def owner(root: Path, path: Path):
    parts = path.relative_to(root).parts
    if not parts:
        return None
    if parts[0] in {"include", "src"} and len(parts) > 1:
        return parts[1] if parts[1] in INCLUDE_SRC_OWNERS else None
    if parts[0:2] == ("adapters", "opencv"):
        return "opencv_adapters"
    if parts[0] == "adapters" and len(parts) > 1 and parts[1] == "ros2":
        return "ros2"
    if parts[0] == "apps":
        return "apps"
    return None


def is_opencv_private_source(root: Path, path: Path):
    return path.relative_to(root).parts[:3] == ("adapters", "opencv", "src")


def _sanitize_cpp_source(contents: str):
    sanitized = list(contents)
    state = "NORMAL"
    raw_terminator = ""
    index = 0

    def mask(position):
        if contents[position] not in "\r\n":
            sanitized[position] = " "

    while index < len(contents):
        char = contents[index]
        next_char = contents[index + 1] if index + 1 < len(contents) else ""

        if state == "NORMAL":
            if char == "/" and next_char == "/":
                mask(index)
                mask(index + 1)
                index += 2
                state = "LINE_COMMENT"
                continue
            if char == "/" and next_char == "*":
                mask(index)
                mask(index + 1)
                index += 2
                state = "BLOCK_COMMENT"
                continue
            if char == "R" and next_char == '"':
                delimiter_start = index + 2
                open_paren = contents.find("(", delimiter_start, delimiter_start + 17)
                if open_paren != -1:
                    delimiter = contents[delimiter_start:open_paren]
                    if not any(c.isspace() or c in "()\\" for c in delimiter):
                        for position in range(index, open_paren + 1):
                            mask(position)
                        raw_terminator = f'){delimiter}\"'
                        index = open_paren + 1
                        state = "RAW_STRING"
                        continue
            if char == '"':
                mask(index)
                index += 1
                state = "STRING"
                continue
            if char == "'":
                mask(index)
                index += 1
                state = "CHAR"
                continue
            index += 1
            continue

        if state == "LINE_COMMENT":
            if char == "\\" and next_char == "\n":
                mask(index)
                index += 2
                continue
            if char == "\\" and next_char == "\r" and contents[index + 2:index + 3] == "\n":
                mask(index)
                index += 3
                continue
            if char in "\r\n":
                index += 1
                state = "NORMAL"
                continue
            mask(index)
            index += 1
            continue

        if state == "BLOCK_COMMENT":
            if char == "*" and next_char == "/":
                mask(index)
                mask(index + 1)
                index += 2
                state = "NORMAL"
                continue
            mask(index)
            index += 1
            continue

        if state == "RAW_STRING":
            if contents.startswith(raw_terminator, index):
                for position in range(index, index + len(raw_terminator)):
                    mask(position)
                index += len(raw_terminator)
                state = "NORMAL"
                continue
            mask(index)
            index += 1
            continue

        if state in {"STRING", "CHAR"}:
            quote = '"' if state == "STRING" else "'"
            if char == "\\":
                mask(index)
                if index + 1 < len(contents):
                    mask(index + 1)
                    index += 2
                else:
                    index += 1
                continue
            if char == quote:
                mask(index)
                index += 1
                state = "NORMAL"
                continue
            if char in "\r\n":
                index += 1
                state = "NORMAL"
                continue
            mask(index)
            index += 1

    return "".join(sanitized)


def included_role(header: str):
    if GENERATED_PROTO_RE.match(header):
        return "domain_proto"
    if header.startswith("uw/"):
        return "legacy"
    first = header.split("/", 1)[0]
    return first if first in PROJECT_ROLES else None


def source_files(root: Path):
    for relative in ("include", "src", "adapters/ros2", "adapters/opencv", "apps"):
        base = root / relative
        if not base.exists():
            continue
        for suffix in ("*.hpp", "*.h", "*.cpp", "*.cc"):
            yield from base.rglob(suffix)


def check(root: Path):
    root = root.resolve()
    errors = []
    for path in sorted(set(source_files(root))):
        source_owner = owner(root, path)
        opencv_private_source = is_opencv_private_source(root, path)
        contents = path.read_text(encoding="utf-8")
        sanitized = _sanitize_cpp_source(contents)
        opencv_header_lines = set()
        original_lines = contents.splitlines()
        sanitized_lines = sanitized.splitlines()
        for line_number, (original_line, sanitized_line) in enumerate(
            zip(original_lines, sanitized_lines), 1
        ):
            directive_match = INCLUDE_DIRECTIVE_RE.match(sanitized_line)
            if not directive_match:
                continue
            hash_column = sanitized_line.find("#", 0, directive_match.end())
            match = INCLUDE_RE.match(original_line[hash_column:])
            if not match:
                continue
            location = f"{path.relative_to(root)}:{line_number}"
            header = match.group(1)
            if header.startswith(ROS_VENDOR_PREFIXES) and source_owner != "ros2":
                errors.append(
                    f"{location}: ROS/vendor header {header} is only allowed in adapters/ros2"
                )
                continue
            if header.startswith("opencv2/") and not opencv_private_source:
                errors.append(
                    f"{location}: OpenCV header {header} is only allowed in adapters/opencv/src"
                )
                opencv_header_lines.add(line_number)
                continue
            dependency = included_role(header)
            if dependency == "legacy":
                errors.append(f"{location}: legacy hand-written include {header}")
                continue
            if source_owner and dependency and dependency not in ALLOWED[source_owner]:
                errors.append(
                    f"{location}: {source_owner} must not include {dependency} ({header})"
                )
        if not opencv_private_source:
            reported_type_lines = set()
            for match in CV_TYPE_RE.finditer(sanitized):
                line_number = contents.count("\n", 0, match.start()) + 1
                if line_number in opencv_header_lines or line_number in reported_type_lines:
                    continue
                reported_type_lines.add(line_number)
                location = f"{path.relative_to(root)}:{line_number}"
                errors.append(
                    f"{location}: OpenCV type cv:: is only allowed in adapters/opencv/src"
                )
    return errors


def main(argv):
    root = Path(argv[1]) if len(argv) > 1 else Path(__file__).resolve().parents[2]
    errors = check(root)
    if errors:
        print("Layer dependency check failed:", file=sys.stderr)
        for error in errors:
            print(f"  {error}", file=sys.stderr)
        return 1
    print("OK: C++ layer dependencies and ROS/vendor boundaries are valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
