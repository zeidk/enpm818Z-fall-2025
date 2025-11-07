#!/usr/bin/env python3
"""
Launch file for SLAM frontend with KITTI full dataset (4541 frames).
This launch file starts:
1. Bag playback of full dataset
2. Odometry estimator node
3. Local map manager node
4. RViz visualization

Usage:
    ros2 launch slam_frontend rwa2_full.launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    pkg_share = FindPackageShare("slam_frontend").find("slam_frontend")

    # Paths
    params_file = PathJoinSubstitution([pkg_share, "config", "params.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "rwa2.rviz"])

    # Bag file path - assuming workspace structure
    # Adjust this path if your workspace structure is different
    bag_path = os.path.join(
        os.path.expanduser("~"),
        "workspace",
        "ros2_ws",
        "src",
        "rwa2_starter",
        "data",
        "kitti_00_full",
    )

    # Declare launch arguments
    rate_arg = DeclareLaunchArgument(
        "rate", default_value="1.0", description="Bag playback rate (1.0 = real-time)"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Start RViz visualization"
    )

    # Bag playback
    bag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--rate",
            LaunchConfiguration("rate"),
            "--clock",  # Publish simulation time
        ],
        output="screen",
        name="bag_play",
    )

    # Odometry estimator node
    odometry_estimator = Node(
        package="slam_frontend",
        executable="odometry_estimator_node",
        name="odometry_estimator_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": True}],
    )

    # Local map manager node
    local_map_manager = Node(
        package="slam_frontend",
        executable="local_map_manager_node",
        name="local_map_manager_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": True}],
    )

    # RViz visualization
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    static_tf_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_odom",
        arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
    )

    return LaunchDescription(
        [
            rate_arg,
            use_rviz_arg,
            bag_play,
            odometry_estimator,
            local_map_manager,
            static_tf_odom,
            rviz,
        ]
    )
