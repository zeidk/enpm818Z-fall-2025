#!/usr/bin/env python3
"""
Launch file for YOLO detector demo with KITTI rosbag playback.
Uses parameter file for configuration.
Compatible with ROS 2 Jazzy.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Setup function to handle conditional launches based on parameters."""
    
    # Get launch configurations
    bag_file = LaunchConfiguration('bag_file').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    play_rate = LaunchConfiguration('play_rate').perform(context)
    
    nodes_to_launch = []
    
    # YOLO detector node - always launch
    yolo_detector = Node(
        package='yolo_detector',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[params_file],
        emulate_tty=True
    )
    nodes_to_launch.append(yolo_detector)
    
    # RViz2 node - only if config provided
    if rviz_config and rviz_config != '':
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
        nodes_to_launch.append(rviz_node)
    
    # ROS bag playback - only if bag file provided
    if bag_file and bag_file != '':
        bag_play = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play',
                bag_file,
                '--rate', play_rate,
                '--loop'
            ],
            output='screen'
        )
        nodes_to_launch.append(bag_play)
    
    return nodes_to_launch


def generate_launch_description():
    """Generate launch description with YOLO detector and optional bag playback."""
    
    # Get package directory
    pkg_share = FindPackageShare('yolo_detector')
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'yolo_params.yaml']),
        description='Path to YAML parameter file'
    )
    
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='',
        description='Path to rosbag file (leave empty to not autoplay)'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'demo.rviz']),
        description='Path to RViz config file'
    )
    
    play_rate_arg = DeclareLaunchArgument(
        'play_rate',
        default_value='1.0',
        description='Playback rate for rosbag'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time from bag'
    )
    
    return LaunchDescription([
        params_file_arg,
        bag_file_arg,
        rviz_config_arg,
        play_rate_arg,
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])