from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 🔥 모든 노드가 사용할 공통 파라미터
    common_params = {
        "nats_server": "nats://172.26.160.1:4222",
        "dss_server": "172.26.160.1",
        "dss_port": 8886,
        "nats_port": 4222,
    }

    return LaunchDescription([

        # Camera
        Node(
            package='dss_ros2_bridge',
            executable='DSSToROSImageNode',
            name='Image',
            output='screen',
            parameters=[common_params],
        ),

        # IMU
        Node(
            package='dss_ros2_bridge',
            executable='DSSToROSIMUNode',
            name='IMU',
            output='screen',
            parameters=[common_params],
        ),

        # LiDAR
        Node(
            package='dss_ros2_bridge',
            executable='DSSToROSPointCloudNode',
            name='PointCloud',
            output='screen',
            parameters=[common_params],
        ),

        # GPS (필요하면 활성화)
        # Node(
        #     package='dss_ros2_bridge',
        #     executable='DSSToGPSNode',
        #     name='GPS',
        #     output='screen',
        #     parameters=[common_params],
        # ),

        # DSS Demo
        # Node(
        #     package='dss_ros2_bridge',
        #     executable='DSSDemoNode',
        #     name='Demo',
        #     output='screen',
        #     parameters=[common_params],
        # ),
    ])
