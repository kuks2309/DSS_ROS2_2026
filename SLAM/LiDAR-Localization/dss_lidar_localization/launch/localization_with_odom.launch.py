#!/usr/bin/env python3
"""
DSS Platform LiDAR Localization with Odometry Launch File

This launch file runs lidar_localization_ros2 with KISS-ICP odometry
for improved localization accuracy. The odometry provides initial guess
for the NDT/GICP registration.

Usage:
    ros2 launch dss_lidar_localization localization_with_odom.launch.py map_path:=/path/to/map.pcd

Requirements:
    - A pre-built PCD map file
    - LiDAR sensor publishing to /dss/sensor/lidar
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg


def generate_launch_description():
    # Package share directories
    pkg_share = FindPackageShare('dss_lidar_localization')
    kiss_icp_share = FindPackageShare('dss_kiss_icp')

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

    # RViz config file path
    rviz_config_file = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'localization.rviz'
    ])

    # Static TF: base_link -> lidar (DSS LiDAR at z=1.5m)
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_lidar',
        arguments=['0', '0', '1.5', '0', '0', '0', 'base_link', 'lidar'],
        output='screen'
    )

    # KISS-ICP Odometry Node (provides /odom topic)
    kiss_icp_config = PathJoinSubstitution([
        kiss_icp_share,
        'config',
        'params.yaml'
    ])

    kiss_icp_node = Node(
        package='kiss_icp',
        executable='odometry_node',
        name='kiss_icp_odometry',
        output='screen',
        parameters=[
            kiss_icp_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('pointcloud_topic', '/dss/sensor/lidar'),
        ]
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
                'use_odom': True,  # Enable odometry fusion
            }
        ],
        remappings=[
            ('/cloud', '/dss/sensor/lidar'),
            ('/odom', '/kiss_icp/odometry'),
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
        kiss_icp_node,
        lidar_localization,
        to_inactive,
        rviz_node,
    ])
