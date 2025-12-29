#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for bag replay tool"""

    return LaunchDescription([
        Node(
            package='bag_replay_tool_ros2',
            executable='bag_replay_tool_node',
            name='bag_replay_tool',
            output='screen',
            parameters=[],
        ),
    ])
