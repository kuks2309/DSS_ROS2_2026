#!/usr/bin/env python3
"""
IMU Validation Node - Compare IMU with KISS-ICP Odometry
IMU angular_velocity를 KISS-ICP odometry의 회전 변화율과 비교하여 IMU 정확도를 검증합니다.

사용법:
  ros2 run dss_controller imu_validation_node

필요 조건:
  - KISS-ICP가 실행 중이어야 함 (ros2 launch dss_kiss_icp run.launch.py)
  - DSS 시뮬레이터가 IMU 데이터를 발행 중이어야 함
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import time
from collections import deque
from typing import Optional, Tuple


def quaternion_to_euler(q: Quaternion) -> Tuple[float, float, float]:
    """Quaternion을 Euler angles (roll, pitch, yaw)로 변환"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ImuValidationNode(Node):
    def __init__(self):
        super().__init__('imu_validation_node')

        # IMU subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            '/dss/sensor/imu',
            self._on_imu,
            10
        )

        # KISS-ICP odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/kiss/odometry',
            self._on_odom,
            10
        )

        # Data storage
        self.last_imu: Optional[Imu] = None
        self.last_odom: Optional[Odometry] = None
        self.prev_odom: Optional[Odometry] = None
        self.prev_odom_time: Optional[float] = None

        # IMU integration for yaw
        self.imu_integrated_yaw = 0.0
        self.imu_prev_time: Optional[float] = None

        # Statistics
        self.imu_gyro_z_history = deque(maxlen=100)
        self.odom_yaw_rate_history = deque(maxlen=100)
        self.error_history = deque(maxlen=100)

        # Counters
        self.imu_count = 0
        self.odom_count = 0

        # Display timer (1 Hz)
        self.display_timer = self.create_timer(1.0, self._on_display)

        # Validation timer (10 Hz)
        self.validation_timer = self.create_timer(0.1, self._on_validate)

        self.get_logger().info("=" * 70)
        self.get_logger().info("IMU Validation Node - Comparing IMU with KISS-ICP")
        self.get_logger().info("=" * 70)
        self.get_logger().info("Subscribed topics:")
        self.get_logger().info("  - IMU: /dss/sensor/imu")
        self.get_logger().info("  - Odometry: /kiss/odometry")
        self.get_logger().info("")
        self.get_logger().info("Validation method:")
        self.get_logger().info("  - Compare IMU angular_velocity.z with odometry yaw rate")
        self.get_logger().info("  - Integrate IMU gyro_z and compare with odometry yaw")
        self.get_logger().info("=" * 70)

    def _on_imu(self, msg: Imu):
        self.last_imu = msg
        self.imu_count += 1

        # Integrate gyro_z for yaw
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.imu_prev_time is not None:
            dt = current_time - self.imu_prev_time
            if 0 < dt < 0.1:  # Sanity check
                self.imu_integrated_yaw += msg.angular_velocity.z * dt
        self.imu_prev_time = current_time

        # Store gyro_z
        self.imu_gyro_z_history.append(msg.angular_velocity.z)

    def _on_odom(self, msg: Odometry):
        self.prev_odom = self.last_odom
        self.last_odom = msg
        self.odom_count += 1

        # Calculate yaw rate from odometry
        if self.prev_odom is not None:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if self.prev_odom_time is not None:
                dt = current_time - self.prev_odom_time
                if 0 < dt < 0.5:  # Sanity check
                    # Get yaw from quaternion
                    _, _, current_yaw = quaternion_to_euler(msg.pose.pose.orientation)
                    _, _, prev_yaw = quaternion_to_euler(self.prev_odom.pose.pose.orientation)

                    # Calculate yaw difference (handle wrap-around)
                    yaw_diff = current_yaw - prev_yaw
                    if yaw_diff > math.pi:
                        yaw_diff -= 2 * math.pi
                    elif yaw_diff < -math.pi:
                        yaw_diff += 2 * math.pi

                    yaw_rate = yaw_diff / dt
                    self.odom_yaw_rate_history.append(yaw_rate)

            self.prev_odom_time = current_time

    def _on_validate(self):
        """Compare IMU and odometry data"""
        if self.last_imu is None or self.last_odom is None:
            return

        if len(self.imu_gyro_z_history) == 0 or len(self.odom_yaw_rate_history) == 0:
            return

        # Current values
        imu_gyro_z = self.last_imu.angular_velocity.z

        # Get latest odom yaw rate
        if len(self.odom_yaw_rate_history) > 0:
            odom_yaw_rate = self.odom_yaw_rate_history[-1]
            error = abs(imu_gyro_z - odom_yaw_rate)
            self.error_history.append(error)

    def _on_display(self):
        """Display validation results"""
        self.get_logger().info("-" * 70)

        # Check data availability
        if self.imu_count == 0:
            self.get_logger().warn("No IMU data received! Check /dss/sensor/imu topic")
            return
        if self.odom_count == 0:
            self.get_logger().warn("No KISS-ICP data received! Check /kiss/odometry topic")
            return

        self.get_logger().info(f"Data received - IMU: {self.imu_count}, Odometry: {self.odom_count}")

        # Current IMU values
        if self.last_imu is not None:
            imu = self.last_imu
            self.get_logger().info(
                f"IMU: gyro_z={imu.angular_velocity.z:+.4f} rad/s, "
                f"acc_z={imu.linear_acceleration.z:+.4f} g"
            )

        # Current odometry yaw rate
        if len(self.odom_yaw_rate_history) > 0:
            odom_yaw_rate = self.odom_yaw_rate_history[-1]
            self.get_logger().info(f"KISS-ICP: yaw_rate={odom_yaw_rate:+.4f} rad/s")

        # Statistics
        if len(self.imu_gyro_z_history) > 10 and len(self.odom_yaw_rate_history) > 10:
            imu_avg = sum(self.imu_gyro_z_history) / len(self.imu_gyro_z_history)
            odom_avg = sum(self.odom_yaw_rate_history) / len(self.odom_yaw_rate_history)

            self.get_logger().info(f"Average (last {len(self.imu_gyro_z_history)} samples):")
            self.get_logger().info(f"  IMU gyro_z avg:    {imu_avg:+.4f} rad/s")
            self.get_logger().info(f"  Odom yaw_rate avg: {odom_avg:+.4f} rad/s")
            self.get_logger().info(f"  Difference:        {abs(imu_avg - odom_avg):.4f} rad/s")

        # Error statistics
        if len(self.error_history) > 10:
            avg_error = sum(self.error_history) / len(self.error_history)
            max_error = max(self.error_history)

            self.get_logger().info(f"Error stats:")
            self.get_logger().info(f"  Average error: {avg_error:.4f} rad/s")
            self.get_logger().info(f"  Max error:     {max_error:.4f} rad/s")

            # Validation result
            if avg_error < 0.1:  # 0.1 rad/s threshold
                self.get_logger().info("  [RESULT] IMU angular_velocity is ACCURATE")
            elif avg_error < 0.3:
                self.get_logger().warn("  [RESULT] IMU angular_velocity has MODERATE error")
            else:
                self.get_logger().error("  [RESULT] IMU angular_velocity has LARGE error")

        # Integrated yaw comparison
        if self.last_odom is not None:
            _, _, odom_yaw = quaternion_to_euler(self.last_odom.pose.pose.orientation)
            self.get_logger().info(f"Integrated yaw comparison:")
            self.get_logger().info(f"  IMU integrated:  {self.imu_integrated_yaw:+.4f} rad ({math.degrees(self.imu_integrated_yaw):+.1f} deg)")
            self.get_logger().info(f"  Odom yaw:        {odom_yaw:+.4f} rad ({math.degrees(odom_yaw):+.1f} deg)")

        # Gravity check (DSS uses normalized g units, so ~1.0 for gravity)
        if self.last_imu is not None:
            acc_z = self.last_imu.linear_acceleration.z
            # DSS IMU outputs in g units (1.0 = gravity)
            gravity_ok = 0.9 < abs(acc_z) < 1.1
            if gravity_ok:
                self.get_logger().info(f"  [CHECK] Gravity (acc_z): OK ({acc_z:.3f}g, expected ~1.0g)")
            else:
                self.get_logger().warn(f"  [CHECK] Gravity (acc_z): UNEXPECTED ({acc_z:.3f}g, expected ~1.0g)")


def main(args=None):
    rclpy.init(args=args)
    node = ImuValidationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Validation stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
