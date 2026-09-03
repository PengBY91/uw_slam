# BlueOS extension image for the HWT9053-485 forwarder (PREP-D-02 step 4).
#
# arm64 (Raspberry Pi 4/5 under BlueOS). Slim base plus two pure-Python
# wheels — no compiler, no numpy, no ROS. The whole image is pulled over
# the tether the first time it is installed, so every megabyte here is
# minutes of a pre-dive checklist.
FROM python:3.11-slim-bookworm

# The generated protobuf bindings are NOT in git (tools/codegen/gen_py.sh
# writes them, **/*_pb2.py is gitignored) so that schemas/proto stays the
# single source of truth. Run
#   tools/codegen/gen_py.sh "$PWD/adapters/wit_imu/uw_wit_imu/schema_pb2"
# before building this image, from the repository root, with
# adapters/wit_imu as the build context.
WORKDIR /app
COPY pyproject.toml /app/
COPY uw_wit_imu /app/uw_wit_imu
RUN pip install --no-cache-dir .

# BlueOS passes the USB-RS485 converter through as /dev/ttyUSB0 and the
# topside host is reachable at 192.168.2.1; both are overridable at run
# time so the same image works on a bench PC.
ENV WIT_PORT=/dev/ttyUSB0 \
    WIT_BAUD=230400 \
    WIT_RATE=200 \
    WIT_DEST_HOST=192.168.2.1 \
    WIT_DEST_PORT=27500 \
    WIT_SENSOR_ID=hwt9053 \
    WIT_SENSOR_FRAME=imu_link

# exec form + a shell so the env vars above expand; PYTHONUNBUFFERED keeps
# the log usable through `docker logs` when something goes wrong at depth.
ENV PYTHONUNBUFFERED=1
CMD ["sh", "-c", "python -m uw_wit_imu.service --port \"$WIT_PORT\" --baud \"$WIT_BAUD\" --rate \"$WIT_RATE\" --dest-host \"$WIT_DEST_HOST\" --dest-port \"$WIT_DEST_PORT\" --sensor-id \"$WIT_SENSOR_ID\" --sensor-frame \"$WIT_SENSOR_FRAME\""]
