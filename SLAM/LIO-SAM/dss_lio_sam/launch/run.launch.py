import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('dss_lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    # RViz config file path - use source directory for easy editing
    rviz_config_file = str(Path.home() / 'ros2_ws/src/SLAM/LIO-SAM/dss_lio_sam/config/rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([
        params_declare,
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
            parameters=[parameter_file],
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
            parameters=[parameter_file],
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
