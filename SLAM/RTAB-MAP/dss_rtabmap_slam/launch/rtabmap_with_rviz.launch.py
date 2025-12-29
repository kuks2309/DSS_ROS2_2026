#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # RViz config file path - use source directory for easy editing
    rviz_config_file = str(Path.home() / 'ros2_ws/src/SLAM/RTAB-MAP/dss_rtabmap_slam/rviz2/rtabmap_slam.rviz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Static TF: base_link -> lidar_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar_link',
            arguments=['0', '0', '1.5', '0', '0', '0', 'base_link', 'lidar_link'],
            output='screen'
        ),

        # Static TF: base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),

        # RTAB-Map Odometry Node (generates odometry from LiDAR + IMU)
        # DSS IMU already provides orientation, so use directly without madgwick filter
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
                'scan_range_min': 0.5,
                'scan_range_max': 50.0,
                'scan_cloud_max_points': 30000,

                # IMU parameters - use IMU orientation for gravity alignment
                'wait_imu_to_init': True,
                'guess_frame_id': 'odom',

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
                'Reg/Strategy': '1',
                'Icp/VoxelSize': '0.1',
                'Icp/MaxCorrespondenceDistance': '1.0',
                'Icp/PointToPlaneK': '10',
                'Icp/MaxTranslation': '2.0',
                'Icp/Epsilon': '0.001',
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '10',
                'Icp/CorrespondenceRatio': '0.2',

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
                'Grid/RangeMax': '50.0',
                'Grid/RangeMin': '0.5',
                'Grid/FromDepth': 'false',

                # Ground filtering disabled - use all points for grid
                'Grid/NormalsSegmentation': 'false',
                'Grid/MaxObstacleHeight': '5.0',
                'Grid/NormalK': '20',

                # Grid size settings
                'Grid/CellSize': '0.05',
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar3d'),
                ('odom', '/rtabmap/odom'),
            ],
            arguments=[
                '-d',
            ]
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
