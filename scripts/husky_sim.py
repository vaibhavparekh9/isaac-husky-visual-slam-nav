#!/usr/bin/env python3
"""Main entry: load the scene, build drive + camera graphs, run the sim."""
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import omni.usd
from husky_scene import load_scene
from drive_graph import build_drive_graph, set_wheel_drive_gains
from camera_graph import attach_camera, build_camera_graph, camera_topics, CAMERAS

world, husky = load_scene()
stage = omni.usd.get_context().get_stage()

# Rear camera stays defined in the URDF but is left off here to keep FPS up.
ACTIVE_CAMERAS = ["front_camera", "left_camera", "right_camera"]

attached = []
for name in ACTIVE_CAMERAS:
    cfg = CAMERAS[name]
    cam_path = attach_camera(stage, cfg["optical_path"], sensor_name="sensor")
    attached.append((name, cfg, cam_path))

build_drive_graph(extra_tf_prims=[cfg["optical_path"] for _, cfg, _ in attached])
set_wheel_drive_gains(stage)

for name, cfg, cam_path in attached:
    build_camera_graph(
        camera_path=cam_path,
        frame_id=cfg["frame_id"],
        graph_path=f"/Husky/CameraGraph_{name}",
        **camera_topics(cfg["ns"]),
    )

world.reset()

print(f"\n[husky] drive + {len(attached)} cameras running "
      f"({', '.join(n for n, _, _ in attached)}).")
print("[husky] topics: /cmd_vel /odom /tf /clock + "
      "/{front,left,right}_camera/{rgb,depth}/*")
print("[husky] Ctrl-C to stop.\n")
try:
    while simulation_app.is_running():
        world.step(render=not args.headless)
except KeyboardInterrupt:
    pass
simulation_app.close()
