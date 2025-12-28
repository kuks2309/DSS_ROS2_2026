#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    ICP Odometry 단독 테스트용 launch 파일
    SLAM 없이 ICP odometry만 실행하여 성능 확인

    테스트 방법:
    1. ros2 launch dss_rtabmap_slam icp_odom_test.launch.py
    2. 별도 터미널에서 확인:
       - ros2 topic echo /rtabmap/odom (odometry 출력 확인)
       - ros2 topic hz /rtabmap/odom (주파수 확인)
       - ros2 run tf2_ros tf2_echo odom base_link (TF 확인)
    """

    use_sim_time = LaunchConfiguration('use_sim_time')

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

        # ICP Odometry Node only (no SLAM)
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

                # Scan preprocessing
                'scan_voxel_size': 0.1,
                'scan_normal_k': 10,
                'scan_range_min': 0.5,
                'scan_range_max': 40.0,
                'scan_cloud_max_points': 30000,

                # ICP parameters
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
            ]
        ),

    ])
