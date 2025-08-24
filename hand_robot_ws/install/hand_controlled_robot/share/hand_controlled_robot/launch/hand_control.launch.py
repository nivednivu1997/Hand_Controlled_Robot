#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('hand_controlled_robot')
    
    # Declare launch arguments
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Camera index to use (default: 0)'
    )
    
    # Create the hand controlled robot node
    hand_control_node = Node(
        package='hand_controlled_robot',
        executable='hand_controlled_robot_node',
        name='hand_controlled_robot_node',
        output='screen',
        parameters=[{
            'camera_index': LaunchConfiguration('camera_index')
        }]
    )
    
    return LaunchDescription([
        camera_index_arg,
        hand_control_node
    ])
