#!/usr/bin/env python3
"""Nav2 stack for the Husky's planning/control only, on top of slam.launch.py.
No map_server; rtabmap owns /map (the static_layer subscribes directly) and obstacle avoidance uses rtabmap's /cloud_obstacles + /cloud_ground.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("husky_custom")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="Full path to the Nav2 params YAML."),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz with the navigation config."),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        ),

        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            output="screen",
            condition=IfCondition(rviz),
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("husky_custom"), "config", "navigation.rviz"])],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
