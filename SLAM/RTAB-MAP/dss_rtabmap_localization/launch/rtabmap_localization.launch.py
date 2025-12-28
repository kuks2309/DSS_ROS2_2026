#!/usr/bin/env python3
"""
RTAB-Map Localization Launch File for DSS Platform
Localization using a pre-built RTAB-Map database

Usage:
    ros2 launch dss_rtabmap_localization rtabmap_localization.launch.py \
        database_path:=/home/amap/ros2_ws/map/rtamp_map/slam_map_20251227_065358.db

Requirements:
    - Pre-built RTAB-Map .db file
    - DSS simulator running with /points and /dss/sensor/imu topics
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')

    # RViz config file path - use source directory for easy editing
    rviz_config_file = str(Path.home() / 'ros2_ws/src/SLAM/RTAB-MAP/dss_rtabmap_localization/rviz2/rtabmap_localization.rviz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'database_path',
            default_value=str(Path.home() / 'ros2_ws/map/rtamp_map/slam_map_20251227_065358.db'),
            description='Path to the RTAB-Map database file'
        ),

        # Static TF: base_link -> lidar_link (same as SLAM)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar_link',
            arguments=['0', '0', '1.5', '0', '0', '0', 'base_link', 'lidar_link'],
            output='screen'
        ),

        # Static TF: base_link -> imu_link (same as SLAM)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),

        # RTAB-Map Odometry Node (same parameters as SLAM)
        # Uses ICP odometry with IMU for gravity alignment
        Node(
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
                'approx_sync': False,

                # Scan preprocessing (same as SLAM)
                'scan_voxel_size': 0.1,
                'scan_normal_k': 10,
                'scan_range_min': 0.5,
                'scan_range_max': 50.0,
                'scan_cloud_max_points': 30000,

                # IMU parameters
                'wait_imu_to_init': True,
                'guess_frame_id': 'odom',

                # ICP Odometry parameters (same as SLAM)
                'Reg/Strategy': '1',
                'Icp/VoxelSize': '0.1',
                'Icp/MaxCorrespondenceDistance': '2.0',
                'Icp/PointToPlaneK': '10',
                'Icp/PointToPlaneRadius': '0',
                'Icp/MaxTranslation': '5.0',
                'Icp/MaxRotation': '0.78',
                'Icp/Epsilon': '0.001',
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '30',
                'Icp/CorrespondenceRatio': '0.2',
                'Odom/Strategy': '0',
                'Odom/GuessMotion': 'true',
                'Odom/Holonomic': 'false',
                'Odom/FillInfoData': 'true',
                'OdomF2M/ScanSubtractRadius': '0.1',
                'OdomF2M/ScanMaxSize': '30000',
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar3d'),
                ('scan', '/scan_not_used'),
                ('odom', '/rtabmap/odom'),
                ('imu', '/dss/sensor/imu'),
            ]
        ),

        # RTAB-Map Localization Node
        # Key differences from SLAM:
        # - Mem/IncrementalMemory: false (don't add new nodes)
        # - Mem/InitWMWithAllNodes: true (load all nodes at startup)
        # - More frequent updates for better localization
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan_cloud': True,
                'subscribe_odom_info': True,
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'wait_for_transform': 0.2,
                'approx_sync': False,
                'database_path': database_path,

                # ===== LOCALIZATION MODE SETTINGS =====
                'Mem/IncrementalMemory': 'false',    # Don't add new nodes (localization only)
                'Mem/InitWMWithAllNodes': 'true',    # Load all nodes at startup
                'Mem/STMSize': '30',

                # Registration (same as SLAM)
                'Reg/Strategy': '1',
                'Icp/VoxelSize': '0.1',
                'Icp/MaxCorrespondenceDistance': '1.0',
                'Icp/PointToPlaneK': '10',
                'Icp/MaxTranslation': '2.0',
                'Icp/Epsilon': '0.001',
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '10',
                'Icp/CorrespondenceRatio': '0.2',

                # Loop closure - more aggressive for localization
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityMaxGraphDepth': '0',
                'RGBD/ProximityPathMaxNeighbors': '10',
                'RGBD/AngularUpdate': '0.05',        # More frequent update for localization
                'RGBD/LinearUpdate': '0.1',          # More frequent update for localization

                # Graph optimization
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Optimizer/Epsilon': '0.00001',
                'Optimizer/Iterations': '100',
                'Optimizer/Slam2D': 'false',

                # Grid settings (for visualization)
                'RGBD/CreateOccupancyGrid': 'true',
                'Grid/RangeMax': '50.0',
                'Grid/RangeMin': '0.5',
                'Grid/FromDepth': 'false',
                'Grid/NormalsSegmentation': 'false',
                'Grid/MaxObstacleHeight': '5.0',
                'Grid/CellSize': '0.05',
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar3d'),
                ('odom', '/rtabmap/odom'),
            ]
            # NOTE: No '-d' argument - we want to LOAD the database, not delete it!
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config_file],
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),

    ])
