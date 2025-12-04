#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')

    # Get package share directory
    pkg_share = FindPackageShare('dss_rtabmap_slam')

    # RViz config file path
    rviz_config_file = PathJoinSubstitution([
        pkg_share,
        'rviz2',
        'rtabmap_slam.rviz'
    ])

    # Parameters file for rtabmap
    rtabmap_parameters = PathJoinSubstitution([
        pkg_share,
        'config',
        'rtabmap_params.yaml'
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/rtabmap.db',
            description='Path to save/load RTAB-Map database'
        ),

        DeclareLaunchArgument(
            'delete_db_on_start',
            default_value='true',
            description='Delete database on start (true=fresh start, false=continue mapping)'
        ),

        # Static TF: base_link -> lidar (x=0, y=0, z=1.5m)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar',
            arguments=['0', '0', '1.5', '0', '0', '0', 'base_link', 'lidar'],
            output='screen'
        ),

        # ICP Odometry Node
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
                'Icp/MaxCorrespondenceDistance': '3.0',  # Increase for vehicle
                'Icp/MaxTranslation': '3.0',  # Allow up to 3m translation per frame (vehicle speed)
                'Icp/MaxRotation': '3.14',  # Allow up to 180 degrees rotation per frame
                'Icp/Iterations': '20',  # Balance between accuracy and speed
                'Icp/Epsilon': '0.001',
                'Icp/CorrespondenceRatio': '0.2',
                'Odom/Strategy': '0',  # Frame-to-Map
                'OdomF2M/ScanSubtractRadius': '0.1',
                'OdomF2M/ScanMaxSize': '15000',
                'Odom/GuessMotion': 'true',  # Use previous motion as initial guess
                'Odom/Holonomic': 'false',  # Non-holonomic constraint
                'Odom/FillInfoData': 'true',  # Fill covariance data
                'Odom/ScanKeyFrameThr': '0.9',  # Create keyframe when scan similarity < 90%
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar'),
            ]
        ),

        # RTAB-Map SLAM Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'database_path': database_path,
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
                'Mem/STMSize': '30',  # Short-term memory size
                'Mem/NotLinkedNodesKept': 'true',  # Keep all nodes even if not linked

                # Registration / ICP
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
                'RGBD/ProximityPathMaxNeighbors': '10',  # Increase for better loop closure
                'RGBD/AngularUpdate': '0.1',  # 0.1 rad (~6 degrees) for vehicle
                'RGBD/LinearUpdate': '0.3',  # 30cm for vehicle (was too frequent at 5cm)

                # Graph optimization
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Optimizer/Epsilon': '0.00001',
                'Optimizer/Iterations': '100',
                'Optimizer/Slam2D': 'false',
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar'),
                ('odom', '/rtabmap/odom'),
            ],
            arguments=['-d']
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
