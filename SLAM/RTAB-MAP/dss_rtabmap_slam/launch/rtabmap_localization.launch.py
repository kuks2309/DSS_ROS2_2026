#!/usr/bin/env python3
"""
RTAB-MAP Localization Launch File (tetra 설정 기반)

Usage:
    ros2 launch dss_rtabmap_slam rtabmap_localization.launch.py

    With custom map:
    ros2 launch dss_rtabmap_slam rtabmap_localization.launch.py map_path:=/path/to/rtabmap.db
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def launch_setup(context, *args, **kwargs):
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = (use_sim_time_str.lower() == 'true')
    map_path = LaunchConfiguration('map_path').perform(context)
    lidar_topic = LaunchConfiguration('lidar_topic').perform(context)
    initial_pose = LaunchConfiguration('initial_pose').perform(context)

    # 맵 파일 확인
    if not os.path.isfile(map_path):
        raise RuntimeError(f"Map file not found at {map_path}")

    # RTAB-Map arguments (tetra 설정 기반)
    # Note: database_path는 파라미터로 설정함 (--udb 사용하지 않음)
    rtabmap_arguments = [
        'Mem/NotLinkedNodesKept', 'false',
        'Mem/STMSize', '30',
        'Mem/LaserScanNormalK', '20',
        'Reg/Strategy', '1',
        'Icp/VoxelSize', '0.5',
        'Icp/PointToPlaneK', '20',
        'Icp/PointToPlaneRadius', '0',
        'Icp/PointToPlane', 'true',
        'Icp/Iterations', '5',
        'Icp/Epsilon', '0.001',
        'Icp/MaxTranslation', '3',
        'Icp/MaxCorrespondenceDistance', '1',
        'Icp/Strategy', '1',
        'Icp/OutlierRatio', '0.7',
        'Icp/CorrespondenceRatio', '0.2',
        'Rtabmap/DetectionRate', '10.0',
        # Localization 속도 개선
        'RGBD/ProximityMaxGraphDepth', '5',  # 비교할 노드 깊이 제한
        'RGBD/LocalRadius', '5',  # 로컬 영역 반경 제한 (5m)
    ]

    # Static TF: base_link -> lidar_link (센서 장착 위치)
    # TF 체인: map -> odom -> base_link -> lidar_link
    #   - map -> odom: rtabmap이 발행 (localization)
    #   - odom -> base_link: icp_odometry가 발행
    #   - base_link -> lidar_link: static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Point Cloud Assembler
    rtabmap_util = Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        output='screen',
        parameters=[{
            'max_clouds': 20,
            'assembling_time': 0.1,
            'fixed_frame_id': 'map',
            'use_sim_time': use_sim_time,
        }],
    )

    # ICP Odometry Node
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'wait_for_transform': 1.0,
            'approx_sync': True,
            # Scan preprocessing
            'scan_voxel_size': 0.5,
            'scan_normal_k': 20,
            'scan_range_min': 0.5,
            'scan_range_max': 40.0,
            # ICP parameters (tetra 설정)
            'Reg/Strategy': '1',
            'Icp/VoxelSize': '0.5',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '5',
            'Icp/Epsilon': '0.001',
            'Icp/MaxTranslation': '3',
            'Icp/MaxCorrespondenceDistance': '1',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Icp/CorrespondenceRatio': '0.1',
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'true',
            'Odom/Holonomic': 'false',
            'Odom/ResetCountdown': '1',
        }],
        remappings=[
            ('scan_cloud', lidar_topic),
            ('scan', '/scan_not_used'),
            ('odom', '/odom'),
        ]
    )

    # RTAB-Map Node (Localization Mode)
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        additional_env={'OMP_NUM_THREADS': '12'},
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'wait_for_transform': 1.0,
            'use_sim_time': use_sim_time,
            # Database path - use parameter instead of --udb argument
            'database_path': map_path,
            # Scan cloud settings (24546 -> 5000 포인트로 제한)
            'scan_cloud_max_points': 5000,
            'scan_cloud_is_2d': False,
            # Localization mode
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'false',  # 초기 위치 기반으로 로컬 노드만 로드
            # 초기 위치 설정 (x y z roll pitch yaw)
            'initial_pose': initial_pose,
            # Grid settings
            'Grid/FromScan': 'true',
            'Grid/RayTracing': 'true',
            'Grid/3D': 'true',
            'Grid/RangeMax': '15.0',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MaxGroundHeight': '0.05',
            'Grid/NormalsSegmentation': 'false',
            'Grid/CellSize': '0.05',
        }],
        arguments=rtabmap_arguments,
        remappings=[
            ('scan_cloud', lidar_topic),
            ('odom', '/odom'),
        ],
    )

    # RViz2 config path
    rviz_config = '/home/amap/ros2_ws/src/SLAM/RTAB-MAP/dss_rtabmap_localization/rviz2/rtabmap_localization.rviz'

    # RViz2
    rtabmap_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # publish_map 서비스 호출 (5초 후)
    publish_map_cmd = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/rtabmap/publish_map',
                     'rtabmap_msgs/srv/PublishMap',
                     '{global_map: true, optimized: true, graph_only: false}'],
                output='screen'
            )
        ]
    )

    # 지연 시작 노드들 (rtabmap 맵 로딩 완료 후)
    delayed_nodes = TimerAction(
        period=10.0,  # rtabmap 맵 로딩 시간 대기
        actions=[
            static_tf,
            icp_odometry,
            rtabmap_rviz2,
        ]
    )

    return [
        # 1. rtabmap 먼저 시작 (맵 로딩)
        rtabmap_slam,
        # 2. 맵 로딩 완료 후 다른 노드들 시작
        delayed_nodes,
        # 3. 맵 발행 서비스 호출 (15초 후)
        TimerAction(
            period=15.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/rtabmap/publish_map',
                         'rtabmap_msgs/srv/PublishMap',
                         '{global_map: true, optimized: true, graph_only: false}'],
                    output='screen'
                )
            ]
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        # OpenMP 멀티스레드 설정 (CPU 코어 수에 맞게)
        SetEnvironmentVariable('OMP_NUM_THREADS', '12'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'map_path',
            default_value='/home/amap/ros2_ws/map/rtamp_map/slam_map_20251227_065358.db',
            description='Path to RTAB-Map database file'
        ),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/dss/sensor/lidar3d',
            description='LiDAR point cloud topic'
        ),
        DeclareLaunchArgument(
            'initial_pose',
            default_value='0 0 0 0 0 0',
            description='Initial pose (x y z roll pitch yaw)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
