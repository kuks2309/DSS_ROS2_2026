#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')

    return LaunchDescription([

        # Set use_sim_time parameter
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Localization mode (true: localization only, false: mapping)
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Set to true for localization mode, false for mapping mode'
        ),

        # Database path for saving/loading maps
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/rtabmap.db',
            description='Path to RTAB-Map database file'
        ),

        # RTAB-Map Node (SLAM Mode)
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan_cloud': True,
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'approx_sync': False,

                # RTAB-Map parameters
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',

                # ICP parameters for LiDAR
                'Icp/VoxelSize': '0.1',
                'Icp/MaxCorrespondenceDistance': '1.0',
                'Icp/PointToPlaneK': '5',
                'Icp/PointToPlaneRadius': '0',
                'Icp/MaxTranslation': '2.0',
                'Icp/Epsilon': '0.001',
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '10',
                'Icp/CorrespondenceRatio': '0.2',

                # Odometry
                'Odom/Strategy': '0',  # 0=Frame-to-Map, 1=Frame-to-Frame
                'Odom/GuessMotion': 'true',
                'OdomF2M/ScanSubtractRadius': '0.1',
                'OdomF2M/ScanMaxSize': '15000',

                # Loop closure
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityMaxGraphDepth': '0',
                'RGBD/ProximityPathMaxNeighbors': '10',
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'Reg/Strategy': '1',  # 0=Visual, 1=ICP, 2=Visual+ICP

                # Graph optimization
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Optimizer/Epsilon': '0.00001',
                'Optimizer/Iterations': '100',
                'Optimizer/Slam2D': 'false',
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar'),
                ('odom', '/odom'),
            ],
            arguments=[
                '-d',  # Delete database if already exists (for new mapping)
            ]
        ),

        # RTAB-Map Node (Localization Mode)
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan_cloud': True,
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'approx_sync': False,

                # Localization mode
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar'),
                ('odom', '/odom'),
            ]
        ),

        # RTAB-Map Visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'subscribe_scan_cloud': True,
                'frame_id': 'base_link',
                'approx_sync': False,
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar'),
            ]
        ),

    ])
