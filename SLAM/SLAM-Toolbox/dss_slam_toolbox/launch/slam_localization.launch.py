"""
SLAM Toolbox Localization Launch File for DSS Platform
Localization using a previously saved map
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    dss_slam_toolbox_share = get_package_share_directory('dss_slam_toolbox')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz = LaunchConfiguration('rviz', default='true')
    map_file = LaunchConfiguration('map_file')

    # Convert 3D PointCloud2 to 2D LaserScan
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': -0.5,
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00436,
            'scan_time': 0.1,
            'range_min': 1.0,
            'range_max': 100.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('cloud_in', '/dss/sensor/lidar'),
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
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '1.5',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'lidar'
        ]
    )

    # RViz
    rviz_config_path = os.path.join(dss_slam_toolbox_share, 'rviz', 'slam_toolbox.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map_file', description='Path to saved map file (.posegraph)'),

        static_tf_base_to_lidar,
        pointcloud_to_laserscan_node,
        slam_toolbox_node,
        rviz_node,
    ])
