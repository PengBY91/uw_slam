import pathlib
import sys

# tests/test_forwarder.py reuses test_protocol.py's packet builders.
sys.path.insert(0, str(pathlib.Path(__file__).parent))
