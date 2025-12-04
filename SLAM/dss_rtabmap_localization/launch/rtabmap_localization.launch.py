#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')

    # Get package share directory
    pkg_share = FindPackageShare('dss_rtabmap_localization')

    # RViz config file path (shared with SLAM package)
    slam_pkg_share = FindPackageShare('dss_rtabmap_slam')
    rviz_config_file = PathJoinSubstitution([
        slam_pkg_share,
        'rviz2',
        'rtabmap_slam.rviz'
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'database_path',
            default_value='',
            description='Path to the RTAB-Map database file'
        ),

        # Static TF: base_link -> lidar (x=0, y=0, z=1.5m)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar',
            arguments=['0', '0', '1.5', '0', '0', '0', 'base_link', 'lidar'],
            output='screen'
        ),

        # ICP Odometry Node (same as SLAM)
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
                'scan_voxel_size': 0.1,
                'scan_normal_k': 5,
                'scan_cloud_max_points': 0,
                'Reg/Strategy': '1',  # ICP
                'Icp/VoxelSize': '0.1',
                'Icp/PointToPlaneK': '5',
                'Icp/MaxCorrespondenceDistance': '1.0',
                'Icp/Iterations': '10',
                'Icp/Epsilon': '0.001',
                'Icp/CorrespondenceRatio': '0.2',
                'Odom/Strategy': '0',  # Frame-to-Map
                'OdomF2M/ScanSubtractRadius': '0.1',
                'OdomF2M/ScanMaxSize': '15000',
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar'),
            ]
        ),

        # RTAB-Map Localization Node
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

                # Localization mode parameters
                'Mem/IncrementalMemory': 'false',  # Don't add new nodes
                'Mem/InitWMWithAllNodes': 'true',  # Load all nodes at startup

                # Registration / ICP (same as SLAM)
                'Reg/Strategy': '1',  # ICP
                'Icp/VoxelSize': '0.1',
                'Icp/MaxCorrespondenceDistance': '1.0',
                'Icp/PointToPlaneK': '5',
                'Icp/PointToPlaneRadius': '0',
                'Icp/MaxTranslation': '2.0',
                'Icp/Epsilon': '0.001',
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '10',
                'Icp/CorrespondenceRatio': '0.2',

                # Loop closure
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityMaxGraphDepth': '0',
                'RGBD/ProximityPathMaxNeighbors': '1',
                'RGBD/AngularUpdate': '0.01',  # Update more frequently in localization
                'RGBD/LinearUpdate': '0.01',

                # Graph optimization
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Optimizer/Epsilon': '0.00001',
                'Optimizer/Iterations': '100',
                'Optimizer/Slam2D': 'false',
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar'),
                ('odom', '/rtabmap/odom'),
            ]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),

    ])
