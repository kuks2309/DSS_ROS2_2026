#!/usr/bin/env python3
"""
IMU Verification Test Node
차량을 가속/감속/회전시키면서 IMU 데이터가 정상적으로 응답하는지 확인합니다.

사용법:
  ros2 run dss_controller imu_test_node

테스트 시나리오:
  1. 정지 상태 (5초) - 중력값 확인 (z축 약 9.8 m/s²)
  2. 직선 가속 (5초) - x축 가속도 증가 확인
  3. 정지 (5초) - 가속도 0으로 복귀 확인
  4. 좌회전 (5초) - z축 각속도 확인
  5. 우회전 (5초) - z축 각속도 반대 방향 확인
  6. 정지 (5초) - 모든 값 안정화 확인
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import socket
import time
import math
import sys

# dss_pb2를 임포트하기 위해 경로 추가
sys.path.insert(0, '/home/amap/ros2_ws/src/dss_controller/dss_controller')
try:
    import dss_pb2
except ImportError:
    from dss_controller import dss_pb2


class ImuTestNode(Node):
    def __init__(self):
        super().__init__('imu_test_node')

        # Parameters
        self.declare_parameter('dss_ip', '172.26.160.1')
        self.declare_parameter('drive_port', 8886)

        dss_ip = self.get_parameter('dss_ip').get_parameter_value().string_value
        drive_port = self.get_parameter('drive_port').get_parameter_value().integer_value

        self.get_logger().info(f"IMU Test Node Started")
        self.get_logger().info(f"DSS Server: {dss_ip}:{drive_port}")

        # UDP socket for vehicle control
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.drive_addr = (dss_ip, drive_port)

        # IMU subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            '/dss/sensor/imu',
            self._on_imu,
            10
        )

        # IMU data storage
        self.last_imu = None
        self.imu_count = 0

        # Test state
        self.test_phase = 0
        self.phase_start_time = time.time()
        self.test_phases = [
            {'name': 'IDLE (gravity check)', 'duration': 5.0, 'throttle': 0.0, 'steer': 0.0, 'brake': 1.0},
            {'name': 'ACCELERATE (forward)', 'duration': 5.0, 'throttle': 0.5, 'steer': 0.0, 'brake': 0.0},
            {'name': 'COAST', 'duration': 3.0, 'throttle': 0.0, 'steer': 0.0, 'brake': 0.0},
            {'name': 'BRAKE', 'duration': 3.0, 'throttle': 0.0, 'steer': 0.0, 'brake': 1.0},
            {'name': 'TURN LEFT', 'duration': 5.0, 'throttle': 0.3, 'steer': -0.5, 'brake': 0.0},
            {'name': 'TURN RIGHT', 'duration': 5.0, 'throttle': 0.3, 'steer': 0.5, 'brake': 0.0},
            {'name': 'FINAL STOP', 'duration': 5.0, 'throttle': 0.0, 'steer': 0.0, 'brake': 1.0},
        ]

        # Stats for each phase
        self.phase_stats = {
            'acc_x_sum': 0.0, 'acc_y_sum': 0.0, 'acc_z_sum': 0.0,
            'gyro_x_sum': 0.0, 'gyro_y_sum': 0.0, 'gyro_z_sum': 0.0,
            'count': 0
        }

        # Control timer (50 Hz)
        self.control_timer = self.create_timer(1.0 / 50.0, self._on_control)

        # Display timer (1 Hz)
        self.display_timer = self.create_timer(1.0, self._on_display)

        self.get_logger().info("=" * 60)
        self.get_logger().info("IMU Verification Test")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Expected IMU behavior:")
        self.get_logger().info("  - Stationary: linear_accel.z ~ -9.8 m/s² (gravity)")
        self.get_logger().info("  - Forward accel: linear_accel.x increases")
        self.get_logger().info("  - Left turn: angular_vel.z > 0")
        self.get_logger().info("  - Right turn: angular_vel.z < 0")
        self.get_logger().info("=" * 60)

    def _on_imu(self, msg: Imu):
        self.last_imu = msg
        self.imu_count += 1

        # Accumulate stats
        self.phase_stats['acc_x_sum'] += msg.linear_acceleration.x
        self.phase_stats['acc_y_sum'] += msg.linear_acceleration.y
        self.phase_stats['acc_z_sum'] += msg.linear_acceleration.z
        self.phase_stats['gyro_x_sum'] += msg.angular_velocity.x
        self.phase_stats['gyro_y_sum'] += msg.angular_velocity.y
        self.phase_stats['gyro_z_sum'] += msg.angular_velocity.z
        self.phase_stats['count'] += 1

    def _send_control(self, throttle: float, steer: float, brake: float):
        ctrl = dss_pb2.DssSetControl()
        ctrl.identifier = "DSS.IMU.TestNode"
        ctrl.timestamp = int(time.time() * 1000)
        ctrl.throttle = throttle
        ctrl.brake = brake
        ctrl.steer = steer

        try:
            self.udp_socket.sendto(ctrl.SerializeToString(), self.drive_addr)
        except Exception as e:
            self.get_logger().error(f"UDP send error: {e}")

    def _on_control(self):
        if self.test_phase >= len(self.test_phases):
            # Test complete - stop vehicle
            self._send_control(0.0, 0.0, 1.0)
            return

        phase = self.test_phases[self.test_phase]
        elapsed = time.time() - self.phase_start_time

        if elapsed >= phase['duration']:
            # Move to next phase
            self._print_phase_summary()
            self.test_phase += 1
            self.phase_start_time = time.time()

            # Reset stats
            self.phase_stats = {
                'acc_x_sum': 0.0, 'acc_y_sum': 0.0, 'acc_z_sum': 0.0,
                'gyro_x_sum': 0.0, 'gyro_y_sum': 0.0, 'gyro_z_sum': 0.0,
                'count': 0
            }

            if self.test_phase >= len(self.test_phases):
                self.get_logger().info("=" * 60)
                self.get_logger().info("TEST COMPLETE!")
                self.get_logger().info("=" * 60)
                return

            new_phase = self.test_phases[self.test_phase]
            self.get_logger().info("-" * 60)
            self.get_logger().info(f"Phase {self.test_phase + 1}/{len(self.test_phases)}: {new_phase['name']}")
            self.get_logger().info(f"  throttle={new_phase['throttle']}, steer={new_phase['steer']}, brake={new_phase['brake']}")

        # Send control command
        self._send_control(phase['throttle'], phase['steer'], phase['brake'])

    def _print_phase_summary(self):
        if self.phase_stats['count'] == 0:
            return

        n = self.phase_stats['count']
        phase = self.test_phases[self.test_phase]

        avg_acc_x = self.phase_stats['acc_x_sum'] / n
        avg_acc_y = self.phase_stats['acc_y_sum'] / n
        avg_acc_z = self.phase_stats['acc_z_sum'] / n
        avg_gyro_x = self.phase_stats['gyro_x_sum'] / n
        avg_gyro_y = self.phase_stats['gyro_y_sum'] / n
        avg_gyro_z = self.phase_stats['gyro_z_sum'] / n

        self.get_logger().info(f"\n>>> Phase '{phase['name']}' Summary ({n} samples):")
        self.get_logger().info(f"    linear_accel:   x={avg_acc_x:+8.4f}, y={avg_acc_y:+8.4f}, z={avg_acc_z:+8.4f} m/s²")
        self.get_logger().info(f"    angular_vel:    x={avg_gyro_x:+8.4f}, y={avg_gyro_y:+8.4f}, z={avg_gyro_z:+8.4f} rad/s")

        # Validation checks
        if 'gravity' in phase['name'].lower() or 'idle' in phase['name'].lower() or 'stop' in phase['name'].lower():
            # Should see gravity on z-axis
            gravity_ok = abs(abs(avg_acc_z) - 9.81) < 1.0
            self.get_logger().info(f"    [CHECK] Gravity on Z: {'OK' if gravity_ok else 'FAIL'} (expected ~±9.81)")

        if 'accelerate' in phase['name'].lower():
            # Should see positive acceleration on x-axis
            accel_ok = avg_acc_x > 0.1
            self.get_logger().info(f"    [CHECK] Forward accel: {'OK' if accel_ok else 'FAIL'} (expected acc_x > 0)")

        if 'left' in phase['name'].lower():
            # Left turn = positive angular velocity around z
            turn_ok = avg_gyro_z > 0.01
            self.get_logger().info(f"    [CHECK] Left turn gyro: {'OK' if turn_ok else 'FAIL'} (expected gyro_z > 0)")

        if 'right' in phase['name'].lower():
            # Right turn = negative angular velocity around z
            turn_ok = avg_gyro_z < -0.01
            self.get_logger().info(f"    [CHECK] Right turn gyro: {'OK' if turn_ok else 'FAIL'} (expected gyro_z < 0)")

    def _on_display(self):
        if self.test_phase >= len(self.test_phases):
            return

        phase = self.test_phases[self.test_phase]
        elapsed = time.time() - self.phase_start_time
        remaining = phase['duration'] - elapsed

        if self.last_imu is None:
            self.get_logger().warn("No IMU data received yet!")
            return

        imu = self.last_imu
        self.get_logger().info(
            f"[{phase['name'][:20]:20s}] "
            f"acc: ({imu.linear_acceleration.x:+7.3f}, {imu.linear_acceleration.y:+7.3f}, {imu.linear_acceleration.z:+7.3f}) "
            f"gyro: ({imu.angular_velocity.x:+6.4f}, {imu.angular_velocity.y:+6.4f}, {imu.angular_velocity.z:+6.4f}) "
            f"[{remaining:.1f}s left]"
        )

    def destroy_node(self):
        # Stop vehicle before exit
        self._send_control(0.0, 0.0, 1.0)
        self.udp_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
