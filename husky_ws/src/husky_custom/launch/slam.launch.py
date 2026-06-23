#!/usr/bin/env python3
"""RTAB-Map RGB-D SLAM / localization for the Husky (3 cameras + wheel odom).
  localization_only:=false (default)  SLAM: build/extend the map (-d wipes the db).
  localization_only:=true             loads database_path and localize, no mapping.

Uses the Isaac wheel /odom as the motion prior (no visual odometry) and publishes the map->odom TF.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CAMERAS = ["front_camera", "left_camera", "right_camera"]


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    database_path = LaunchConfiguration("database_path").perform(context)
    localization_only = (
        LaunchConfiguration("localization_only").perform(context).lower() == "true"
    )

    actions = []

    # One rgbd_sync per camera: rgb + depth + camera_info -> /<cam>/rgbd_image.
    rtabmap_rgbd_remaps = []
    for i, cam in enumerate(CAMERAS):
        actions.append(Node(
            package="rtabmap_sync", executable="rgbd_sync",
            name=f"{cam}_rgbd_sync", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "approx_sync": True,
            }],
            remappings=[
                ("rgb/image", f"/{cam}/rgb/image_raw"),
                ("rgb/camera_info", f"/{cam}/rgb/camera_info"),
                ("depth/image", f"/{cam}/depth/image_raw"),
                ("rgbd_image", f"/{cam}/rgbd_image"),
            ],
        ))
        rtabmap_rgbd_remaps.append((f"rgbd_image{i}", f"/{cam}/rgbd_image"))

    # The only difference between the two modes:
    if localization_only:
        mode_params = {
            "Mem/IncrementalMemory": "false",
            "Mem/InitWMWithAllNodes": "true",   # load the whole saved map into memory
        }
        rtabmap_args = []                       # no -d: keep the saved db
    else:
        mode_params = {
            "Mem/IncrementalMemory": "true",
        }
        rtabmap_args = ["-d"]                   # wipe previous db for a fresh map

    rtabmap_params = {
        "use_sim_time": use_sim_time,
        "frame_id": "base_link",
        "odom_frame_id": "odom",
        "subscribe_rgbd": True,
        "subscribe_depth": False,
        "rgbd_cameras": len(CAMERAS),
        "approx_sync": True,
        "database_path": database_path,
        # 3 cameras + odom at mismatched rates: default queue of 10 is too small for the approx synchronizer to ever fire, so enlarge both.
        "topic_queue_size": 30,
        "sync_queue_size": 50,
        "Vis/MaxDepth": "10.0",
        "Grid/FromDepth": "true",
        # Height-based ground segmentation
        "Grid/NormalsSegmentation": "false",
        "Grid/MaxGroundHeight": "0.05",
        "Grid/MaxObstacleHeight": "2.0",
        # 3D ray tracing so one camera's averaged viewpoint doesn't clear obstacle cells marked by another.
        "Grid/3D": "true",
        "Grid/RayTracing": "true",
        "Grid/RangeMax": "3.5",
        "Grid/DepthDecimation": "2",
        # PnP registration; works because rtabmap is built WITH OpenGV. Enables the non-central multi-camera solvers for robust loop closures.
        "Vis/EstimationType": "1",
        "RGBD/NeighborLinkRefining": "true",
        "Mem/DepthCompressionFormat": ".png",
    }
    rtabmap_params.update(mode_params)

    actions.append(Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap",
        output="screen",
        parameters=[rtabmap_params],
        remappings=rtabmap_rgbd_remaps + [("odom", "/odom")],
        arguments=rtabmap_args,
    ))

    actions.append(Node(
        package="rviz2", executable="rviz2", name="rviz2",
        output="screen",
        condition=IfCondition(rviz),
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("husky_custom"), "config", "mapping.rviz"])],
        parameters=[{"use_sim_time": use_sim_time}],
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz with the mapping config."),
        DeclareLaunchArgument(
            "localization_only", default_value="false",
            description="false = SLAM (build/extend the map); "
                        "true = localize against the saved database_path without mapping."),
        DeclareLaunchArgument(
            "database_path",
            default_value=os.path.expanduser("~/.ros/rtabmap.db"),
            description="RTAB-Map database: written/wiped in SLAM mode, "
                        "loaded read-only in localization_only mode."),
        OpaqueFunction(function=launch_setup),
    ])
