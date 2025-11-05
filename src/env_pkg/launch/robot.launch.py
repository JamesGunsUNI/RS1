#!/usr/bin/env python3
"""
ROS 2 launch file to start mission, laserScan and imageRecogniation nodes.

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    pkg_name_arg = DeclareLaunchArgument(
        'ignition_bringup',
        default_value='ignition_bringup',
        description='Name of the ROS2 package containing the node executables'
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ros2 logging level'
    )

    package_name = LaunchConfiguration('ignition_bringup')
    log_level = LaunchConfiguration('log_level')

    # Nodes (assumes installed executables named mission, laserScan, imageRecogniation)
    mission_node = Node(
        package=package_name,
        executable='mission',                # executable name (entry point)
        name='mission_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        respawn=False
    )

    laser_node = Node(
        package=package_name,
        executable='laserScan',
        name='laser_scan_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        respawn=False
    )

    image_node = Node(
        package=package_name,
        executable='imageRecogniation',
        name='image_recognition_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        respawn=False
    )

    return LaunchDescription([
        pkg_name_arg,
        log_level_arg,
        mission_node,
        laser_node,
        image_node,
    ])
