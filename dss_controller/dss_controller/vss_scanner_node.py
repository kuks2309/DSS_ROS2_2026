#!/usr/bin/env python3
"""
VSS Scanner Node - Scan and display all VSS data from DSS
DSS에서 발행되는 VSS 데이터를 스캔하여 표시합니다.

사용법:
  ros2 run dss_controller vss_scanner_node
"""

import rclpy
from rclpy.node import Node
import time
from typing import Dict
from dss_controller.dss_vss_client import DSSVssClient


class VssScannerNode(Node):
    def __init__(self):
        super().__init__('vss_scanner_node')

        # Parameters
        self.declare_parameter('dss_ip', '172.26.160.1')
        self.declare_parameter('vss_port', 4222)

        dss_ip = self.get_parameter('dss_ip').get_parameter_value().string_value
        vss_port = self.get_parameter('vss_port').get_parameter_value().integer_value

        # VSS Client
        self.vss_client = DSSVssClient.singleton()
        self.vss_client.start(ip=dss_ip, drive_port=8886, vss_port=vss_port)

        # Wait for NATS connection
        time.sleep(1.0)

        # Store received VSS data
        self.vss_data: Dict[str, str] = {}
        self.vss_count: Dict[str, int] = {}

        # Subscribe to all VSS topics with wildcard
        self._setup_vss_subscriptions()

        # Display timer (1 Hz)
        self.display_timer = self.create_timer(1.0, self._on_display)

        self.get_logger().info("=" * 70)
        self.get_logger().info("VSS Scanner Node - Scanning DSS VSS Data")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"DSS Server: {dss_ip}:{vss_port}")
        self.get_logger().info("Subscribing to: vss.> (all VSS topics)")
        self.get_logger().info("=" * 70)

    def _setup_vss_subscriptions(self):
        """Subscribe to all VSS topics using wildcard"""
        # Try multiple wildcard patterns (NATS uses > for multi-level, * for single-level)
        wildcards = [
            ">",           # All topics
            "vss.>",       # All vss topics
            "vss.*",       # Single level under vss
            "vss.*.*",     # Two levels under vss
            "vss.*.*.*",   # Three levels under vss
            "dss.>",       # All dss topics
            "dss.*",       # Single level under dss
            "Vehicle.>",   # All Vehicle topics
        ]

        for pattern in wildcards:
            self.vss_client.subscribe(pattern, self._on_vss_any)

        # Also try specific topics we expect
        expected_topics = [
            "vss.Vehicle.AngularVelocity.Yaw",
            "vss.Vehicle.AngularVelocity.Pitch",
            "vss.Vehicle.AngularVelocity.Roll",
            "vss.Vehicle.Acceleration.Longitudinal",
            "vss.Vehicle.Acceleration.Lateral",
            "vss.Vehicle.Acceleration.Vertical",
            "vss.Vehicle.Speed",
            "vss.sensor.speed",
            "Vehicle.AngularVelocity.Yaw",
            "Vehicle.Speed",
            "dss.vehicle.snapshot",
            "dss.ego.vehicle",
        ]

        for topic in expected_topics:
            self.vss_client.subscribe(topic, self._on_vss_any)

        self.get_logger().info(f"Subscribed to {len(wildcards)} wildcards and {len(expected_topics)} specific topics")

    def _on_vss_any(self, subject: str, data: str):
        """Callback for any VSS data"""
        self.vss_data[subject] = data
        self.vss_count[subject] = self.vss_count.get(subject, 0) + 1

    def _on_display(self):
        """Display received VSS data"""
        self.get_logger().info("-" * 70)

        if not self.vss_data:
            self.get_logger().warn("No VSS data received yet...")
            self.get_logger().info("Make sure:")
            self.get_logger().info("  1. DSS simulator is running")
            self.get_logger().info("  2. Vehicle is moving (throttle applied)")
            self.get_logger().info("  3. NATS connection is established")
            return

        self.get_logger().info(f"Received {len(self.vss_data)} VSS topics:")

        # Sort by topic name
        for topic in sorted(self.vss_data.keys()):
            value = self.vss_data[topic]
            count = self.vss_count.get(topic, 0)

            # Truncate long values
            if len(value) > 50:
                value = value[:50] + "..."

            self.get_logger().info(f"  [{count:4d}] {topic}: {value}")


def main(args=None):
    rclpy.init(args=args)
    node = VssScannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Scanner stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
