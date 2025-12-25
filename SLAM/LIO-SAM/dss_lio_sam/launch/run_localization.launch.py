import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    """
    Localization Mode Launch File for DSS LIO-SAM
    - Loads a pre-built map from PCD files
    - Performs localization only (no map updates)
    - Disables loop closure

    Usage:
        ros2 launch dss_lio_sam run_localization.launch.py
        ros2 launch dss_lio_sam run_localization.launch.py map_path:=/path/to/map/GlobalMap.pcd

    Prerequisites:
        - A pre-built map must exist at the path specified
        - Default path: /home/amap/ros2_ws/map/dss_map1/GlobalMap.pcd
    """

    share_dir = get_package_share_directory('dss_lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'dss_lio_sam.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params_localization.yaml'),
        description='Path to the ROS2 parameters file for Localization mode.')

    map_path_declare = DeclareLaunchArgument(
        'map_path',
        default_value='/home/amap/ros2_ws/map/dss_map1/GlobalMap.pcd',
        description='Path to the global map PCD file')

    print("======================================")
    print("  DSS LIO-SAM Localization Mode")
    print("  Loading pre-built map...")
    print("======================================")

    return LaunchDescription([
        params_declare,
        map_path_declare,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file],
            output='screen'
            ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path])
            }]
        ),
        Node(
            package='dss_lio_sam',
            executable='dss_lio_sam_imuPreintegration',
            name='dss_lio_sam_imuPreintegration',
            parameters=[parameter_file,
                       {'globalMapPath': LaunchConfiguration('map_path')}],
            output='screen'
        ),
        Node(
            package='dss_lio_sam',
            executable='dss_lio_sam_imageProjection',
            name='dss_lio_sam_imageProjection',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='dss_lio_sam',
            executable='dss_lio_sam_featureExtraction',
            name='dss_lio_sam_featureExtraction',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='dss_lio_sam',
            executable='dss_lio_sam_mapOptimization',
            name='dss_lio_sam_mapOptimization',
            parameters=[parameter_file,
                       {'globalMapPath': LaunchConfiguration('map_path')}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
