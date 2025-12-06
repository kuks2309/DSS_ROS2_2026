#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # RTAB-Map Odometry Node (generates odometry from LiDAR)
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'wait_for_transform': 0.2,
                'approx_sync': False,

                # ICP Odometry parameters
                'Icp/VoxelSize': '0.1',
                'Icp/MaxCorrespondenceDistance': '1.0',
                'Icp/PointToPlaneK': '5',
                'Icp/PointToPlaneRadius': '0',
                'Icp/MaxTranslation': '2.0',
                'Icp/Epsilon': '0.001',
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '10',
                'Icp/CorrespondenceRatio': '0.2',
                'Odom/Strategy': '0',
                'Odom/GuessMotion': 'true',
                'OdomF2M/ScanSubtractRadius': '0.1',
                'OdomF2M/ScanMaxSize': '15000',
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar'),
            ]
        ),

        # RTAB-Map SLAM Node
        Node(
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
                'wait_for_transform': 0.2,
                'approx_sync': False,

                # Memory
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',

                # Registration
                'Reg/Strategy': '1',  # ICP
                'Icp/VoxelSize': '0.1',
                'Icp/MaxCorrespondenceDistance': '1.0',
                'Icp/PointToPlaneK': '5',
                'Icp/MaxTranslation': '2.0',
                'Icp/Epsilon': '0.001',
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '10',

                # Loop closure
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityMaxGraphDepth': '0',
                'RGBD/ProximityPathMaxNeighbors': '10',
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',

                # Graph optimization
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Optimizer/Epsilon': '0.00001',
                'Optimizer/Iterations': '100',
                'Optimizer/Slam2D': 'false',
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar'),
            ],
            arguments=[
                '-d',  # Delete database if exists
            ]
        ),

        # Visualization
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
