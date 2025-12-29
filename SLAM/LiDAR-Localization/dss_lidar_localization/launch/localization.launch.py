#!/usr/bin/env python3
"""
DSS Platform LiDAR Localization Launch File

This launch file runs lidar_localization_ros2 with DSS platform configuration.
It uses a pre-built PCD map to estimate the robot's current position.

Usage:
    ros2 launch dss_lidar_localization localization.launch.py map_path:=/path/to/map.pcd

Requirements:
    - A pre-built PCD map file (create using RTAB-Map, etc.)
    - LiDAR sensor publishing to /dss/sensor/lidar
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg


def generate_launch_description():
    # Package share directory
    pkg_share = FindPackageShare('dss_lidar_localization')

    # RViz config file path - use source directory for easy editing
    rviz_config_file = str(Path.home() / 'ros2_ws/src/SLAM/LiDAR-Localization/dss_lidar_localization/rviz/localization.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_path = LaunchConfiguration('map_path')
    rviz = LaunchConfiguration('rviz')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_z = LaunchConfiguration('initial_z')

    # Config file path
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'dss_localization.yaml'
    ])

    # Static TF: base_link -> lidar (DSS LiDAR at z=1.5m)
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_lidar',
        arguments=['0', '0', '1.5', '0', '0', '0', 'base_link', 'lidar'],
        output='screen'
    )

    # LiDAR Localization Node (Lifecycle)
    lidar_localization = LifecycleNode(
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        name='lidar_localization',
        namespace='',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'map_path': map_path,
                'initial_pose_x': initial_x,
                'initial_pose_y': initial_y,
                'initial_pose_z': initial_z,
            }
        ],
        remappings=[
            ('/cloud', '/dss/sensor/lidar'),
        ]
    )

    # Lifecycle state transitions
    to_inactive = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_unconfigured_to_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar_localization,
            goal_state='unconfigured',
            entities=[
                LogInfo(msg="-- Unconfigured --"),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg="-- Inactive -> Active --"),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'map_path',
            default_value='',
            description='Path to PCD map file (required)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        DeclareLaunchArgument(
            'initial_x',
            default_value='0.0',
            description='Initial X position'
        ),
        DeclareLaunchArgument(
            'initial_y',
            default_value='0.0',
            description='Initial Y position'
        ),
        DeclareLaunchArgument(
            'initial_z',
            default_value='0.0',
            description='Initial Z position'
        ),

        # Lifecycle handlers
        from_unconfigured_to_inactive,
        from_inactive_to_active,

        # Nodes
        static_tf_base_to_lidar,
        lidar_localization,
        to_inactive,
        rviz_node,
    ])
