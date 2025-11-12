#!/usr/bin/env python3
"""ROS2 Launch file for MAM Robot"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mam_robot_node = Node(
        package='mam_python',
        executable='ros2_node.py',
        name='mam_robot_node',
        output='screen',
    )
    
    return LaunchDescription([mam_robot_node])
