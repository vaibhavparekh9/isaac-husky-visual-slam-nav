#!/usr/bin/env python3
"""Scene loader: reference the Husky into the warehouse stage at runtime.

A SimulationApp must already be created by the caller before importing this
module (Isaac requires the app up before core.api imports work).
"""
import os

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation, XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage, open_stage

# Resolve <repo>/assets relative to this file so any clone location works.
ASSETS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")
WAREHOUSE_USD = f"{ASSETS}/warehouse.usd"
HUSKY_USD = f"{ASSETS}/husky/husky.usd"

HUSKY_PRIM = "/Husky"
HUSKY_ART_ROOT = "/Husky/base_link"
WHEEL_JOINTS = [
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
]

# Spawn just above the floor and let physics settle; face away from the wall.
DEFAULT_SPAWN = (0.0, 0.0, 0.2)
DEFAULT_YAW_DEG = 180.0


def _yaw_quat(yaw_deg):
    half = np.radians(yaw_deg) / 2.0
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)])


def load_scene(spawn=DEFAULT_SPAWN, yaw_deg=DEFAULT_YAW_DEG, physics_dt=1.0 / 60.0):
    open_stage(WAREHOUSE_USD)
    add_reference_to_stage(usd_path=HUSKY_USD, prim_path=HUSKY_PRIM)

    XFormPrim(prim_paths_expr=HUSKY_PRIM,
              positions=np.array([list(spawn)]),
              orientations=np.array([_yaw_quat(yaw_deg)]))

    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt)
    husky = Articulation(prim_paths_expr=HUSKY_ART_ROOT, name="husky")
    world.scene.add(husky)
    return world, husky
