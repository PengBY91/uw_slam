"""HWT9053-485 IMU data chain (PREP-D-01 / PREP-D-02).

See adapters/wit_imu/README.md. The package deliberately has no top-level
imports beyond the standard library so that ``protocol``, ``registers``
and ``timebase`` can be imported and tested on a machine with neither
pyserial nor the generated protobuf bindings present.
"""
