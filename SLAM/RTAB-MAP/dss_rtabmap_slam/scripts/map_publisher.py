#!/usr/bin/env python3
"""
Map Publisher Node for RTAB-MAP Localization Mode
Calls /rtabmap/publish_map service periodically to publish the saved map
"""

import rclpy
from rclpy.node import Node
from rtabmap_msgs.srv import PublishMap


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        # Parameters
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('wait_for_rtabmap', 10.0)  # seconds

        publish_rate = self.get_parameter('publish_rate').value
        wait_time = self.get_parameter('wait_for_rtabmap').value

        self.get_logger().info(f'Map Publisher Node started (rate: {publish_rate} Hz)')

        # Service client
        self.client = self.create_client(PublishMap, '/rtabmap/publish_map')

        # Wait for service
        self.get_logger().info('Waiting for /rtabmap/publish_map service...')
        if not self.client.wait_for_service(timeout_sec=wait_time):
            self.get_logger().error('Service not available after waiting')
            return

        self.get_logger().info('Service available, starting map publishing')

        # Initial publish
        self.publish_map()

        # Timer for periodic publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_map)

    def publish_map(self):
        request = PublishMap.Request()
        request.global_map = True
        request.optimized = True
        request.graph_only = False

        future = self.client.call_async(request)
        future.add_done_callback(self.publish_callback)

    def publish_callback(self, future):
        try:
            future.result()
            self.get_logger().debug('Map published successfully')
        except Exception as e:
            self.get_logger().warn(f'Failed to publish map: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
