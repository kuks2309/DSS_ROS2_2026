"""
KISS-ICP Launch File for DSS Platform
LiDAR-only SLAM (no IMU required)
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
    dss_kiss_icp_share = get_package_share_directory('dss_kiss_icp')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz = LaunchConfiguration('rviz', default='true')

    # KISS-ICP odometry node
    # Note: v0.3.0 uses 'pointcloud_topic' as input topic
    kiss_icp_node = Node(
        package='kiss_icp',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[
            os.path.join(dss_kiss_icp_share, 'config', 'params.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('pointcloud_topic', '/dss/sensor/lidar'),
        ]
    )

    # Static transform: base_link -> lidar
    # DSS LiDAR is mounted at z=1.5m above base_link
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
    rviz_config_path = os.path.join(dss_kiss_icp_share, 'rviz', 'kiss_icp.rviz')
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

        static_tf_base_to_lidar,
        kiss_icp_node,
        rviz_node,
    ])
