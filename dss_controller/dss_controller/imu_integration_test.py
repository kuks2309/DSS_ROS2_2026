#!/usr/bin/env python3
"""
IMU Integration Test Node
IMU 가속도를 적분하여 속도/거리를 계산하고 정확성을 검증합니다.

사용법:
  ros2 run dss_controller imu_integration_test

테스트 방법:
  1. 차량을 일정 속도로 직선 주행
  2. IMU 적분 거리 vs 실제 주행 거리 비교
  3. IMU drift 분석

출력:
  - 실시간 속도 (m/s)
  - 누적 이동 거리 (m)
  - 누적 회전 각도 (deg)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time
import math
import numpy as np


class ImuIntegrationTest(Node):
    def __init__(self):
        super().__init__('imu_integration_test')

        # Parameters
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('gravity_axis', 'z')  # z축에 중력
        self.declare_parameter('gravity_sign', -1)   # 아래 방향이 마이너스
        self.declare_parameter('use_orientation', False)  # IMU orientation 사용 여부

        self.gravity = self.get_parameter('gravity').get_parameter_value().double_value
        self.gravity_axis = self.get_parameter('gravity_axis').get_parameter_value().string_value
        self.gravity_sign = self.get_parameter('gravity_sign').get_parameter_value().integer_value
        self.use_orientation = self.get_parameter('use_orientation').get_parameter_value().bool_value

        # State variables (NED frame: x=North, y=East, z=Down)
        self.velocity = np.array([0.0, 0.0, 0.0])  # m/s
        self.position = np.array([0.0, 0.0, 0.0])  # m
        self.orientation = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw (rad)

        # For drift analysis
        self.stationary_acc_bias = np.array([0.0, 0.0, 0.0])
        self.stationary_gyro_bias = np.array([0.0, 0.0, 0.0])
        self.calibration_samples = []
        self.is_calibrated = False
        self.calibration_count = 0
        self.calibration_target = 100  # 100 samples for calibration

        # Time tracking
        self.last_imu_time = None
        self.start_time = None
        self.imu_count = 0

        # Statistics
        self.max_velocity = 0.0
        self.total_distance = 0.0
        self.total_rotation = 0.0

        # Raw data logging
        self.raw_acc_log = []
        self.raw_gyro_log = []

        # IMU subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            '/dss/sensor/imu',
            self._on_imu,
            10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/imu_test/odometry', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Display timer (2 Hz)
        self.display_timer = self.create_timer(0.5, self._on_display)

        # Summary timer (10초마다)
        self.summary_timer = self.create_timer(10.0, self._on_summary)

        self.get_logger().info("=" * 70)
        self.get_logger().info("IMU Integration Test Node Started")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Gravity: {self.gravity} m/s², axis={self.gravity_axis}, sign={self.gravity_sign}")
        self.get_logger().info("Calibrating bias... Please keep the vehicle STATIONARY for 2 seconds")
        self.get_logger().info("=" * 70)

    def _on_imu(self, msg: Imu):
        current_time = self.get_clock().now()
        current_time_sec = current_time.nanoseconds / 1e9

        self.imu_count += 1

        # Extract raw IMU data
        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Log raw data (최근 100개만)
        self.raw_acc_log.append(acc.copy())
        self.raw_gyro_log.append(gyro.copy())
        if len(self.raw_acc_log) > 100:
            self.raw_acc_log.pop(0)
            self.raw_gyro_log.pop(0)

        # Calibration phase
        if not self.is_calibrated:
            self.calibration_samples.append({'acc': acc.copy(), 'gyro': gyro.copy()})
            self.calibration_count += 1

            if self.calibration_count >= self.calibration_target:
                self._perform_calibration()
            return

        # Calculate dt
        if self.last_imu_time is None:
            self.last_imu_time = current_time_sec
            self.start_time = current_time_sec
            return

        dt = current_time_sec - self.last_imu_time
        self.last_imu_time = current_time_sec

        # Skip invalid dt
        if dt <= 0 or dt > 0.5:
            self.get_logger().warn(f"Invalid dt: {dt:.4f}s, skipping")
            return

        # Remove bias
        acc_corrected = acc - self.stationary_acc_bias
        gyro_corrected = gyro - self.stationary_gyro_bias

        # Remove gravity (assuming z-axis points down)
        acc_no_gravity = acc_corrected.copy()
        if self.gravity_axis == 'z':
            acc_no_gravity[2] -= self.gravity_sign * self.gravity

        # Integrate angular velocity -> orientation (simple Euler integration)
        self.orientation += gyro_corrected * dt
        self.total_rotation += np.linalg.norm(gyro_corrected) * dt

        # Transform acceleration to world frame using current orientation
        # Simple 2D rotation (yaw only) for horizontal motion
        yaw = self.orientation[2]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Rotate acceleration to world frame (2D)
        acc_world = np.array([
            acc_no_gravity[0] * cos_yaw - acc_no_gravity[1] * sin_yaw,
            acc_no_gravity[0] * sin_yaw + acc_no_gravity[1] * cos_yaw,
            acc_no_gravity[2]
        ])

        # Integrate acceleration -> velocity
        self.velocity += acc_world * dt

        # Simple velocity decay to reduce drift (high-pass filter effect)
        # 이것은 테스트용이므로 실제로는 사용하지 않아야 함
        # self.velocity *= 0.999

        # Integrate velocity -> position
        delta_pos = self.velocity * dt
        self.position += delta_pos
        self.total_distance += np.linalg.norm(delta_pos[:2])  # XY plane distance

        # Track max velocity
        speed = np.linalg.norm(self.velocity[:2])
        if speed > self.max_velocity:
            self.max_velocity = speed

        # Publish odometry
        self._publish_odometry(msg.header.stamp)

    def _perform_calibration(self):
        """Calculate bias from stationary samples"""
        acc_sum = np.array([0.0, 0.0, 0.0])
        gyro_sum = np.array([0.0, 0.0, 0.0])

        for sample in self.calibration_samples:
            acc_sum += sample['acc']
            gyro_sum += sample['gyro']

        n = len(self.calibration_samples)
        avg_acc = acc_sum / n
        avg_gyro = gyro_sum / n

        # Gyro bias is just the average (should be near zero when stationary)
        self.stationary_gyro_bias = avg_gyro.copy()

        # Acc bias: remove gravity from measurement
        # If z points down and gravity is -9.81, stationary reading should be [0, 0, -9.81]
        # Bias = measurement - expected
        expected_acc = np.array([0.0, 0.0, self.gravity_sign * self.gravity])
        self.stationary_acc_bias = avg_acc - expected_acc

        self.is_calibrated = True

        self.get_logger().info("=" * 70)
        self.get_logger().info("Calibration Complete!")
        self.get_logger().info(f"  Samples: {n}")
        self.get_logger().info(f"  Raw acc avg:  [{avg_acc[0]:+.4f}, {avg_acc[1]:+.4f}, {avg_acc[2]:+.4f}] m/s²")
        self.get_logger().info(f"  Expected acc: [{expected_acc[0]:+.4f}, {expected_acc[1]:+.4f}, {expected_acc[2]:+.4f}] m/s²")
        self.get_logger().info(f"  Acc bias:     [{self.stationary_acc_bias[0]:+.4f}, {self.stationary_acc_bias[1]:+.4f}, {self.stationary_acc_bias[2]:+.4f}] m/s²")
        self.get_logger().info(f"  Gyro bias:    [{self.stationary_gyro_bias[0]:+.6f}, {self.stationary_gyro_bias[1]:+.6f}, {self.stationary_gyro_bias[2]:+.6f}] rad/s")
        self.get_logger().info("=" * 70)
        self.get_logger().info("Integration started! Move the vehicle to test.")
        self.get_logger().info("Press Ctrl+C to stop and see final summary.")
        self.get_logger().info("=" * 70)

    def _publish_odometry(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "imu_integrated"

        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]

        # Convert euler to quaternion (yaw only for simplicity)
        yaw = self.orientation[2]
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)

        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = self.velocity[2]

        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "imu_integrated"
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _on_display(self):
        if not self.is_calibrated:
            progress = self.calibration_count * 100 // self.calibration_target
            self.get_logger().info(f"Calibrating... {progress}% ({self.calibration_count}/{self.calibration_target})")
            return

        speed = np.linalg.norm(self.velocity[:2])
        speed_kmh = speed * 3.6

        elapsed = 0.0
        if self.start_time is not None and self.last_imu_time is not None:
            elapsed = self.last_imu_time - self.start_time

        yaw_deg = math.degrees(self.orientation[2])

        self.get_logger().info(
            f"[{elapsed:6.1f}s] "
            f"Vel: {speed:5.2f} m/s ({speed_kmh:5.1f} km/h) | "
            f"Dist: {self.total_distance:7.2f} m | "
            f"Pos: ({self.position[0]:+7.2f}, {self.position[1]:+7.2f}) m | "
            f"Yaw: {yaw_deg:+6.1f}°"
        )

    def _on_summary(self):
        if not self.is_calibrated:
            return

        elapsed = 0.0
        if self.start_time is not None and self.last_imu_time is not None:
            elapsed = self.last_imu_time - self.start_time

        self.get_logger().info("-" * 70)
        self.get_logger().info(f"[{elapsed:.0f}s Summary]")
        self.get_logger().info(f"  Total distance: {self.total_distance:.2f} m")
        self.get_logger().info(f"  Max velocity: {self.max_velocity:.2f} m/s ({self.max_velocity*3.6:.1f} km/h)")
        self.get_logger().info(f"  Total rotation: {math.degrees(self.total_rotation):.1f}°")
        self.get_logger().info(f"  Current position: ({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}) m")
        self.get_logger().info(f"  IMU messages: {self.imu_count}")

        # Check for drift
        if elapsed > 0:
            drift_rate = self.total_distance / elapsed if self.total_distance > 0 else 0
            if self.total_distance > 1.0 and drift_rate < 0.5:
                self.get_logger().warn(f"  Possible drift detected! Distance/time ratio: {drift_rate:.3f} m/s")
        self.get_logger().info("-" * 70)

    def _print_final_summary(self):
        self.get_logger().info("")
        self.get_logger().info("=" * 70)
        self.get_logger().info("FINAL SUMMARY")
        self.get_logger().info("=" * 70)

        elapsed = 0.0
        if self.start_time is not None and self.last_imu_time is not None:
            elapsed = self.last_imu_time - self.start_time

        self.get_logger().info(f"Test duration: {elapsed:.1f} seconds")
        self.get_logger().info(f"IMU samples: {self.imu_count}")
        self.get_logger().info(f"IMU rate: {self.imu_count/elapsed:.1f} Hz" if elapsed > 0 else "N/A")
        self.get_logger().info("")
        self.get_logger().info("Integration Results:")
        self.get_logger().info(f"  Total distance traveled: {self.total_distance:.2f} m")
        self.get_logger().info(f"  Final position: ({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}) m")
        self.get_logger().info(f"  Distance from origin: {np.linalg.norm(self.position[:2]):.2f} m")
        self.get_logger().info(f"  Max velocity: {self.max_velocity:.2f} m/s ({self.max_velocity*3.6:.1f} km/h)")
        self.get_logger().info(f"  Total rotation: {math.degrees(self.total_rotation):.1f}°")
        self.get_logger().info(f"  Final yaw: {math.degrees(self.orientation[2]):.1f}°")
        self.get_logger().info("")
        self.get_logger().info("Bias (measured during calibration):")
        self.get_logger().info(f"  Acc bias:  [{self.stationary_acc_bias[0]:+.4f}, {self.stationary_acc_bias[1]:+.4f}, {self.stationary_acc_bias[2]:+.4f}] m/s²")
        self.get_logger().info(f"  Gyro bias: [{self.stationary_gyro_bias[0]:+.6f}, {self.stationary_gyro_bias[1]:+.6f}, {self.stationary_gyro_bias[2]:+.6f}] rad/s")

        # Drift analysis
        if elapsed > 0 and self.total_distance < 0.1:
            # Vehicle was stationary - check position drift
            drift = np.linalg.norm(self.position[:2])
            drift_rate = drift / elapsed
            self.get_logger().info("")
            self.get_logger().info("Drift Analysis (vehicle was mostly stationary):")
            self.get_logger().info(f"  Position drift: {drift:.3f} m")
            self.get_logger().info(f"  Drift rate: {drift_rate:.4f} m/s = {drift_rate*60:.2f} m/min")

        self.get_logger().info("=" * 70)

    def destroy_node(self):
        self._print_final_summary()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuIntegrationTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
