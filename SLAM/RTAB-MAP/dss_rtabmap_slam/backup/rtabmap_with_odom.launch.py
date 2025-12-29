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

                # Scan preprocessing - CRITICAL for proper mapping
                'scan_voxel_size': 0.1,
                'scan_normal_k': 10,
                'scan_range_min': 0.5,        # Minimum range
                'scan_range_max': 40.0,       # Maximum range (Livox MID-360)
                'scan_cloud_max_points': 30000,

                # ICP Odometry parameters
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
                'Icp/CorrespondenceRatio': '0.3',
                'Odom/Strategy': '0',
                'Odom/GuessMotion': 'true',
                'Odom/Holonomic': 'false',
                'Odom/FillInfoData': 'true',
                'OdomF2M/ScanSubtractRadius': '0.1',
                'OdomF2M/ScanMaxSize': '30000',
            }],
            remappings=[
                ('scan_cloud', '/points'),
                ('scan', '/scan_not_used'),  # Disable /scan subscription, use only scan_cloud
                ('odom', '/rtabmap/odom'),  # Output to /rtabmap/odom for RTAB-MAP
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
                'subscribe_odom_info': True,
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'wait_for_transform': 0.2,
                'approx_sync': False,

                # Memory
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                'Mem/STMSize': '30',

                # Registration
                'Reg/Strategy': '1',  # ICP
                'Icp/VoxelSize': '0.1',
                'Icp/MaxCorrespondenceDistance': '1.0',
                'Icp/PointToPlaneK': '10',
                'Icp/MaxTranslation': '2.0',
                'Icp/Epsilon': '0.001',
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '10',
                'Icp/CorrespondenceRatio': '0.3',

                # Loop closure
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityMaxGraphDepth': '0',
                'RGBD/ProximityPathMaxNeighbors': '10',
                'RGBD/AngularUpdate': '0.1',
                'RGBD/LinearUpdate': '0.3',

                # Graph optimization
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Optimizer/Epsilon': '0.00001',
                'Optimizer/Iterations': '100',
                'Optimizer/Slam2D': 'false',

                # Grid settings
                'RGBD/CreateOccupancyGrid': 'true',
                'Grid/RangeMax': '40.0',
                'Grid/RangeMin': '0.5',
                'Grid/FromDepth': 'false',
            }],
            remappings=[
                ('scan_cloud', '/points'),
                ('odom', '/rtabmap/odom'),
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
                ('scan_cloud', '/points'),
            ]
        ),

    ])
