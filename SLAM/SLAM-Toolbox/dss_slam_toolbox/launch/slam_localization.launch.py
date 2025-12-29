#!/usr/bin/env python3
"""
SLAM Toolbox Localization Launch File for DSS Platform
Localization using a previously saved map

Usage:
    ros2 launch dss_slam_toolbox slam_localization.launch.py \
        map_file:=/home/amap/ros2_ws/map/slam_toolbox_map/my_map

    Note: Do NOT include .posegraph extension - SLAM Toolbox adds it automatically

Requirements:
    - Pre-built SLAM Toolbox map files (.posegraph and .data)
    - DSS simulator running with /dss/sensor/lidar3d and /dss/sensor/imu topics
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    dss_slam_toolbox_share = get_package_share_directory('dss_slam_toolbox')
    # RViz config file path - use source directory for easy editing
    rviz_config_path = str(Path.home() / 'ros2_ws/src/SLAM/SLAM-Toolbox/dss_slam_toolbox/rviz/slam_toolbox_localization.rviz')
    # Default map path
    default_map_path = str(Path.home() / 'ros2_ws/map/slam_toolbox_map/slam_toolbox_map')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    map_file = LaunchConfiguration('map_file')

    # Convert 3D PointCloud2 to 2D LaserScan
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 1.0,
            'min_height': 0.3,  # Relative to base_link (exclude ground)
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00436,
            'scan_time': 0.1,
            'range_min': 0.5,
            'range_max': 200.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('cloud_in', '/dss/sensor/lidar3d'),
            ('scan', '/dss/sensor/lidar_scan')
        ]
    )

    # SLAM Toolbox node (Localization mode)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(dss_slam_toolbox_share, 'config', 'mapper_params_localization.yaml'),
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_file
            }
        ]
    )

    # Static transform: base_link -> lidar
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '1.5',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'lidar_link'
        ]
    )

    # Static transform: base_link -> imu_link
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link'
        ]
    )

    # ICP Odometry node (publishes odom -> base_link TF)
    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'wait_for_transform': 0.2,
            'scan_voxel_size': 0.1,
            'scan_normal_k': 10,
            'scan_range_min': 0.5,
            'scan_range_max': 50.0,
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '2.0',
            'Icp/MaxTranslation': '5.0',
            'Icp/Iterations': '30',
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'true',
        }],
        remappings=[
            ('scan_cloud', '/dss/sensor/lidar3d'),
            ('imu', '/dss/sensor/imu'),
        ]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time from /clock'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz visualization'),
        DeclareLaunchArgument('map_file', default_value=default_map_path,
                              description='Path to saved map file (without .posegraph extension)'),

        LogInfo(msg=['Loading map from: ', map_file]),

        static_tf_base_to_imu,
        static_tf_base_to_lidar,
        icp_odometry_node,
        pointcloud_to_laserscan_node,
        slam_toolbox_node,
        rviz_node,
    ])
