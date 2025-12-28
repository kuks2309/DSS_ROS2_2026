#!/usr/bin/env python3
"""
RTAB-MAP Launch File

Usage:
    SLAM Mode:
        ros2 launch dss_rtabmap_slam rtabmap.launch.py

    Localization Mode:
        ros2 launch dss_rtabmap_slam rtabmap.launch.py localization:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('database_path', default_value='/home/amap/.ros/rtabmap.db'),

        # Static TF: base_link -> lidar_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link'],
        ),

        # ICP Odometry
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'scan_voxel_size': 0.1,
                'scan_normal_k': 10,
                'scan_range_min': 0.5,
                'scan_range_max': 40.0,
                'Reg/Strategy': '1',
                'Icp/VoxelSize': '0.1',
                'Icp/MaxCorrespondenceDistance': '2.0',
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '30',
                'Odom/Strategy': '0',
                'Odom/GuessMotion': 'true',
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar3d'),
                ('scan', '/scan_not_used'),
            ]
        ),

        # RTAB-Map SLAM Mode
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'database_path': database_path,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan_cloud': True,
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                'Reg/Strategy': '1',
                'RGBD/CreateOccupancyGrid': 'true',
                'Grid/FromDepth': 'false',
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar3d'),
            ],
            arguments=['-d'],
        ),

        # RTAB-Map Localization Mode
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'database_path': database_path,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan_cloud': True,
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
                'Reg/Strategy': '1',
                'RGBD/CreateOccupancyGrid': 'true',
                'Grid/FromDepth': 'false',
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar3d'),
            ],
        ),

        # RTAB-Map Viz
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'subscribe_scan_cloud': True,
                'frame_id': 'base_link',
            }],
            remappings=[
                ('scan_cloud', '/dss/sensor/lidar3d'),
            ]
        ),

    ])
