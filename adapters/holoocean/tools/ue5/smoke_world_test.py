"""PREP-A-02 acceptance: load our own packaged world (UwWorlds/UwSmokeLevel)
through the stock HoloOcean 2.3.0 client with a BlueROV2 + ImagingSonar and
check that a non-zero sonar frame comes back."""
import json, sys, time
import numpy as np
import holoocean
from holoocean import packagemanager as pm

print("installed packages:", pm.installed_packages(), flush=True)
cfg = {
    "name": "uw_smoke",
    "package_name": "UwWorlds",
    "world": "UwSmokeLevel",
    "main_agent": "auv0",
    "ticks_per_sec": 50,
    "frames_per_sec": False,
    "env_min": [-25, -25, -12],
    "env_max": [25, 25, 2],
    "octree_min": 0.1,
    "octree_max": 5.0,
    "agents": [{
        "agent_name": "auv0",
        "agent_type": "BlueROV2",
        "control_scheme": 0,
        "location": [0.0, 0.0, -3.0],
        "rotation": [0.0, 0.0, 0.0],
        "sensors": [
            {"sensor_type": "ImagingSonar", "sensor_name": "ImagingSonar", "socket": "CameraSocket",
             "Hz": 10, "configuration": {"Azimuth": 140, "Elevation": 20, "RangeMin": 0.3, "RangeMax": 30.0,
                                          "RangeBins": 512, "AzimuthBins": 768}},
            {"sensor_type": "RGBCamera", "sensor_name": "MainCamera", "socket": "CameraSocket", "Hz": 10,
             "configuration": {"CaptureWidth": 640, "CaptureHeight": 480}},
            {"sensor_type": "IMUSensor", "sensor_name": "IMUSensor", "socket": "COM", "Hz": 50},
            {"sensor_type": "PoseSensor", "sensor_name": "PoseSensor", "socket": "COM", "Hz": 10},
        ],
    }],
}
t0 = time.time()
env = holoocean.make(scenario_cfg=cfg, show_viewport=True, verbose=False)
print("env up in %.1fs" % (time.time() - t0), flush=True)
best = 0.0
cam_seen = False
pose = None
try:
    for i in range(150):
        state = env.step(np.array([0, 0, 0, 0, 5, 5, 5, 5], dtype=float))
        if "ImagingSonar" in state:
            s = np.asarray(state["ImagingSonar"]); best = max(best, float(s.max()))
            if i % 50 == 0: print("tick", i, "sonar shape", s.shape, "max", float(s.max()), "nonzero%", round(100 * float((s > 0).mean()), 2), flush=True)
        if "MainCamera" in state: cam_seen = True
        if "PoseSensor" in state: pose = np.asarray(state["PoseSensor"])[:3, 3]
finally:
    env.__exit__(None, None, None) if hasattr(env, "__exit__") else None
print("pose (m):", pose, "camera seen:", cam_seen, "sonar max:", best, flush=True)
print("RESULT:", "OK" if best > 0.0 and cam_seen else "FAIL", flush=True)
