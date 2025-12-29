import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    """
    Localization Mode Launch File
    - Loads a pre-built map from PCD files
    - Performs localization only (no map updates)
    - Disables loop closure

    Usage:
        ros2 launch livox_lio_sam run_localization.launch.py
        ros2 launch livox_lio_sam run_localization.launch.py use_sim_time:=true  # for bag playback

    Prerequisites:
        - A pre-built map must exist at the path specified in params_localization.yaml
        - Default path: /home/amap/Downloads/LOAM/GlobalMap.pcd
    """

    share_dir = get_package_share_directory('livox_lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    # RViz config file path - use source directory for easy editing
    rviz_config_file = str(Path.home() / 'ros2_ws/src/SLAM/LIO-SAM/livox_lio_sam/config/rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params_localization.yaml'),
        description='Path to the ROS2 parameters file for Localization mode.')

    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (for bag playback)')

    print("======================================")
    print("  LIO-SAM Localization Mode")
    print("  Loading pre-built map...")
    print("======================================")

    return LaunchDescription([
        params_declare,
        use_sim_time_declare,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file,
                       {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_livox',
            arguments='1.07 0 0.15 0 0 0 base_link livox_frame'.split(' '),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
            ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path]),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        Node(
            package='livox_lio_sam',
            executable='livox_lio_sam_imuPreintegration',
            name='livox_lio_sam_imuPreintegration',
            parameters=[parameter_file,
                       {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        Node(
            package='livox_lio_sam',
            executable='livox_lio_sam_imageProjection',
            name='livox_lio_sam_imageProjection',
            parameters=[parameter_file,
                       {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        Node(
            package='livox_lio_sam',
            executable='livox_lio_sam_featureExtraction',
            name='livox_lio_sam_featureExtraction',
            parameters=[parameter_file,
                       {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        Node(
            package='livox_lio_sam',
            executable='livox_lio_sam_mapOptimization',
            name='livox_lio_sam_mapOptimization',
            parameters=[parameter_file,
                       {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        )
    ])
