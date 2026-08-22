#!/usr/bin/env python3
import re
import sys
from pathlib import Path


INCLUDE_RE = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]')
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
    "application",
}
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
    "application": PROJECT_ROLES | {"domain_proto"},
    "ros2": {"adapters", "measurement_api", "sensor_models", "domain", "domain_proto"},
    "apps": PROJECT_ROLES | {"domain_proto"},
}


def owner(root: Path, path: Path):
    parts = path.relative_to(root).parts
    if not parts:
        return None
    if parts[0] in {"include", "src"} and len(parts) > 1:
        return parts[1] if parts[1] in PROJECT_ROLES else None
    if parts[0] == "adapters" and len(parts) > 1 and parts[1] == "ros2":
        return "ros2"
    if parts[0] == "apps":
        return "apps"
    return None


def included_role(header: str):
    if GENERATED_PROTO_RE.match(header):
        return "domain_proto"
    if header.startswith("uw/"):
        return "legacy"
    first = header.split("/", 1)[0]
    return first if first in PROJECT_ROLES else None


def source_files(root: Path):
    for relative in ("include", "src", "adapters/ros2", "apps"):
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
        if source_owner is None:
            continue
        for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            match = INCLUDE_RE.match(line)
            if not match:
                continue
            header = match.group(1)
            location = f"{path.relative_to(root)}:{line_number}"
            if header.startswith(ROS_VENDOR_PREFIXES) and source_owner != "ros2":
                errors.append(f"{location}: ROS/vendor header {header} is only allowed in adapters/ros2")
                continue
            dependency = included_role(header)
            if dependency == "legacy":
                errors.append(f"{location}: legacy hand-written include {header}")
                continue
            if dependency and dependency not in ALLOWED[source_owner]:
                errors.append(f"{location}: {source_owner} must not include {dependency} ({header})")
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
