#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    """
    DSS LIO-SAM ROS2 Launch File

    This launch file starts the LIO-SAM SLAM system for DSS Platform:
    - LiDAR-Inertial odometry fusion using DSS sensors
    - Loop closure detection
    - Factor graph optimization using GTSAM
    - Map building and localization
    """

    # Package directories
    dss_lio_sam_share = FindPackageShare('dss_lio_sam')

    # Configuration file path
    config_file = PathJoinSubstitution([
        dss_lio_sam_share,
        'config',
        'params.yaml'
    ])

    # Declare launch arguments
    declare_config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to DSS LIO-SAM configuration file'
    )

    declare_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz for visualization'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Static TF: base_link -> lidar (DSS Platform: z=1.5m)
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_lidar',
        arguments=['0', '0', '1.5', '0', '0', '0', 'base_link', 'lidar'],
        output='screen'
    )

    # Static TF: base_link -> imu_link (DSS Platform: IMU at base_link origin)
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen'
    )

    # IMU Preintegration Node
    imu_preintegration_node = Node(
        package='dss_lio_sam',
        executable='imu_preintegration_node',
        name='dss_imu_preintegration',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    # Image Projection Node (Point Cloud Processing)
    image_projection_node = Node(
        package='dss_lio_sam',
        executable='image_projection_node',
        name='dss_image_projection',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    # Map Optimization Node
    map_optimization_node = Node(
        package='dss_lio_sam',
        executable='map_optimization_node',
        name='dss_map_optimization',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    # RViz node
    rviz_config_file = PathJoinSubstitution([
        dss_lio_sam_share,
        'config',
        'dss_lio_sam.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'error'],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='log'  # Suppress terminal output, write to log file only
    )

    return LaunchDescription([
        declare_config_arg,
        declare_rviz_arg,
        declare_use_sim_time_arg,
        static_tf_base_to_lidar,
        static_tf_base_to_imu,
        imu_preintegration_node,
        image_projection_node,
        map_optimization_node,
        rviz_node,
    ])
