from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='dss_controller',
            executable='dss_controller',
            name='dss_controller_node',
            output='screen',
            parameters=[
                {"nats_server": "nats://127.0.0.1:4222"},
                {"publish_topic": "dss/controller_output"}
            ]
        )
    ])
