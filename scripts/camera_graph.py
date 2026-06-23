#!/usr/bin/env python3
"""D435 RGB-D cameras for the Husky: attach a sensor onto each URDF optical frame and publish rgb/depth/camera_info over ROS 2.

A SimulationApp must already be running before importing this module.
"""
import omni.graph.core as og
from isaacsim.core.nodes.scripts.utils import set_target_prims
from pxr import Gf, UsdGeom

_Q_CAM_IN_OPTICAL = Gf.Quatf(0.0, 1.0, 0.0, 0.0)  # (w, x, y, z)

# D435-like optics at 320x240 (to keep multi-camera FPS up).
# H_FOV = 2*atan(horizAperture/(2*focal)) ~69 deg.
CAM_WIDTH = 320
CAM_HEIGHT = 240
FOCAL_LENGTH = 24.0
HORIZ_APERTURE = 33.0
VERT_APERTURE = HORIZ_APERTURE * CAM_HEIGHT / CAM_WIDTH   # 24.75, square pixels

ROBOT_PRIM = "/Husky"
_BASE = f"{ROBOT_PRIM}/base_link"


def _optical_path(name):
    return (f"{_BASE}/{name}_bottom_screw_frame/{name}_link/"
            f"{name}_color_frame/{name}_color_optical_frame")

CAMERAS = {
    name: {
        "optical_path": _optical_path(name),
        "frame_id": f"{name}_color_optical_frame",
        "ns": name,
    }
    for name in ("front_camera", "rear_camera", "left_camera", "right_camera")
}


def attach_camera(stage, optical_path, sensor_name="sensor"):
    """Create a Camera prim under the given URDF optical frame. Returns its path."""
    camera_path = f"{optical_path}/{sensor_name}"
    cam = UsdGeom.Camera.Define(stage, camera_path)
    cprim = cam.GetPrim()
    UsdGeom.Xformable(cprim).ClearXformOpOrder()
    UsdGeom.Xformable(cprim).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    UsdGeom.Xformable(cprim).AddOrientOp().Set(_Q_CAM_IN_OPTICAL)
    cam.GetProjectionAttr().Set("perspective")
    cam.GetFocalLengthAttr().Set(FOCAL_LENGTH)
    cam.GetHorizontalApertureAttr().Set(HORIZ_APERTURE)
    cam.GetVerticalApertureAttr().Set(VERT_APERTURE)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 1000.0))
    return camera_path


def camera_topics(ns):
    return dict(
        rgb_topic=f"/{ns}/rgb/image_raw",
        depth_topic=f"/{ns}/depth/image_raw",
        info_topic=f"/{ns}/rgb/camera_info",
    )


def build_camera_graph(camera_path, frame_id,
                       rgb_topic, depth_topic, info_topic,
                       width=CAM_WIDTH, height=CAM_HEIGHT,
                       frame_skip=1, domain_id=0,
                       graph_path="/Husky/CameraGraph"):
    """OmniGraph: render `camera_path` and publish rgb/depth/camera_info."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("HelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("HelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("HelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("HelperInfoDepth", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "HelperRgb.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "HelperDepth.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "HelperInfo.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "HelperInfoDepth.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "HelperRgb.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "HelperDepth.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "HelperInfo.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "HelperInfoDepth.inputs:renderProductPath"),
                ("Context.outputs:context", "HelperRgb.inputs:context"),
                ("Context.outputs:context", "HelperDepth.inputs:context"),
                ("Context.outputs:context", "HelperInfo.inputs:context"),
                ("Context.outputs:context", "HelperInfoDepth.inputs:context"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:domain_id", domain_id),
                ("Context.inputs:useDomainIDEnvVar", False),
                ("RenderProduct.inputs:width", width),
                ("RenderProduct.inputs:height", height),
                ("HelperRgb.inputs:type", "rgb"),
                ("HelperRgb.inputs:topicName", rgb_topic),
                ("HelperRgb.inputs:frameId", frame_id),
                ("HelperRgb.inputs:frameSkipCount", frame_skip),
                ("HelperDepth.inputs:type", "depth"),
                ("HelperDepth.inputs:topicName", depth_topic),
                ("HelperDepth.inputs:frameId", frame_id),
                ("HelperDepth.inputs:frameSkipCount", frame_skip),
                ("HelperInfo.inputs:topicName", info_topic),
                ("HelperInfo.inputs:frameId", frame_id),
                ("HelperInfo.inputs:frameSkipCount", frame_skip),
                # depth camera_info next to the depth image, so RViz DepthCloud / RTAB-Map can find it (same intrinsics as rgb in sim)
                ("HelperInfoDepth.inputs:topicName",
                 depth_topic.rsplit("/", 1)[0] + "/camera_info"),
                ("HelperInfoDepth.inputs:frameId", frame_id),
                ("HelperInfoDepth.inputs:frameSkipCount", frame_skip),
            ],
        },
    )
    set_target_prims(primPath=f"{graph_path}/RenderProduct",
                     targetPrimPaths=[camera_path],
                     inputName="inputs:cameraPrim")
    return graph_path
