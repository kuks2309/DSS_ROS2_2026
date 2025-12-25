#!/usr/bin/env python3
"""
IMU vs Ground Truth Validation Node
IMU 데이터를 DSS Ground Truth와 비교하여 검증합니다.

사용법:
  ros2 run dss_controller imu_vss_validation_node

비교 항목:
  - IMU angular_velocity vs Ground Truth (orientation 미분으로 계산)
  - IMU linear_acceleration vs Ground Truth (velocity 미분으로 계산)

DSS IMU 구현 방식:
  - 선형 가속도: 물리 엔진 선형 속도의 시간 미분 → Body-frame 변환
  - 각속도: 회전 변화량 / Δt → Body-frame 변환
  - Orientation: 현재 회전을 쿼터니언으로 변환
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import json
import time
import numpy as np
from collections import deque
from typing import Optional, Tuple
from dss_controller.dss_vss_client import DSSVssClient


def quaternion_to_euler(q: dict) -> Tuple[float, float, float]:
    """Quaternion을 Euler angles (roll, pitch, yaw)로 변환"""
    x, y, z, w = q.get('x', 0), q.get('y', 0), q.get('z', 0), q.get('w', 1)

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_to_rotation_matrix(q: dict) -> np.ndarray:
    """Quaternion을 3x3 회전 행렬로 변환 (World to Body)"""
    x, y, z, w = q.get('x', 0), q.get('y', 0), q.get('z', 0), q.get('w', 1)

    # Normalize
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm > 0:
        x, y, z, w = x/norm, y/norm, z/norm, w/norm

    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])
    return R


class ImuVssValidationNode(Node):
    def __init__(self):
        super().__init__('imu_vss_validation_node')

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

        # IMU subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            '/dss/sensor/imu',
            self._on_imu,
            10
        )

        # Ground Truth data storage (calculated from derivatives)
        self.gt_angular_velocity: Optional[dict] = None  # {x, y, z} rad/s
        self.gt_linear_acceleration: Optional[dict] = None  # {x, y, z} m/s² (body frame)
        self.gt_speed: Optional[float] = None  # m/s

        # Previous GT data for derivative calculation
        self.prev_gt_timestamp: Optional[float] = None
        self.prev_gt_orientation: Optional[dict] = None  # quaternion {x, y, z, w}
        self.prev_gt_velocity: Optional[dict] = None  # {x, y, z} m/s (world frame)
        self.prev_euler: Optional[Tuple[float, float, float]] = None  # (roll, pitch, yaw)

        # IMU data storage
        self.last_imu: Optional[Imu] = None
        self.imu_count = 0
        self.gt_count = 0

        # Statistics
        self.gyro_x_error_history = deque(maxlen=100)
        self.gyro_y_error_history = deque(maxlen=100)
        self.gyro_z_error_history = deque(maxlen=100)
        self.acc_x_error_history = deque(maxlen=100)
        self.acc_y_error_history = deque(maxlen=100)
        self.acc_z_error_history = deque(maxlen=100)

        # Subscribe to Ground Truth topic
        self._setup_subscriptions()

        # Display timer (1 Hz)
        self.display_timer = self.create_timer(1.0, self._on_display)

        # Validation timer (10 Hz)
        self.validation_timer = self.create_timer(0.1, self._on_validate)

        # Auto-drive control (50 Hz)
        self.control_timer = self.create_timer(0.02, self._on_control)
        self.drive_phase = 0  # 0: accelerate, 1: brake, 2: steer left, 3: steer right
        self.phase_start_time = time.time()
        self.phase_duration = 3.0  # seconds per phase
        self.auto_drive_enabled = True

        self.get_logger().info("=" * 70)
        self.get_logger().info("IMU vs Ground Truth Validation Node")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"DSS Server: {dss_ip}:{vss_port}")
        self.get_logger().info("")
        self.get_logger().info("Subscribed topics:")
        self.get_logger().info("  - ROS2 IMU: /dss/sensor/imu")
        self.get_logger().info("  - NATS: dss.simulation.groundTruth.json")
        self.get_logger().info("")
        self.get_logger().info("Validation method (DSS IMU implementation):")
        self.get_logger().info("  - Angular velocity: d(orientation)/dt → Body-frame")
        self.get_logger().info("  - Linear acceleration: d(velocity)/dt → Body-frame")
        self.get_logger().info("")
        self.get_logger().info("Auto-drive mode: ENABLED")
        self.get_logger().info("  Phase 0: Accelerate (3s)")
        self.get_logger().info("  Phase 1: Brake (3s)")
        self.get_logger().info("  Phase 2: Steer Left + Throttle (3s)")
        self.get_logger().info("  Phase 3: Steer Right + Throttle (3s)")
        self.get_logger().info("=" * 70)

    def _setup_subscriptions(self):
        """Subscribe to Ground Truth topic"""
        self.vss_client.subscribe(
            "dss.simulation.groundTruth.json",
            self._on_ground_truth
        )
        self.get_logger().info("Subscribed to dss.simulation.groundTruth.json")

    def _on_ground_truth(self, subject: str, data: str):
        """Ground Truth JSON callback - Extract IMU values directly from GT"""
        try:
            gt = json.loads(data)
            self.gt_count += 1

            # Debug: Print structure on first message
            if self.gt_count == 1:
                self.get_logger().info(f"Ground Truth JSON keys: {list(gt.keys())}")
                if 'moving_object' in gt and len(gt['moving_object']) > 0:
                    ego = gt['moving_object'][0]
                    self.get_logger().info(f"Ego vehicle keys: {list(ego.keys())}")
                    if 'base' in ego:
                        base_keys = list(ego['base'].keys())
                        self.get_logger().info(f"Ego base keys: {base_keys}")
                        # Print actual values for debugging
                        base = ego['base']
                        if 'orientation_rate' in base:
                            self.get_logger().info(f"  orientation_rate: {base['orientation_rate']}")
                        if 'acceleration' in base:
                            self.get_logger().info(f"  acceleration: {base['acceleration']}")
                        if 'velocity' in base:
                            self.get_logger().info(f"  velocity: {base['velocity']}")

            # Extract ego vehicle data from moving_object[0]
            if 'moving_object' not in gt or len(gt['moving_object']) == 0:
                return

            ego = gt['moving_object'][0]
            if 'base' not in ego:
                return

            base = ego['base']

            # ============================================
            # Get orientation (quaternion) for body-frame transform
            # ============================================
            orientation = base.get('orientation', {})
            curr_orientation = {
                'x': float(orientation.get('x', 0)),
                'y': float(orientation.get('y', 0)),
                'z': float(orientation.get('z', 0)),
                'w': float(orientation.get('w', 1))
            }

            # ============================================
            # Get velocity (world frame) and calculate speed
            # velocity can be dict or nested structure
            # ============================================
            velocity = base.get('velocity', {})
            if isinstance(velocity, dict):
                vel_x = float(velocity.get('x', 0))
                vel_y = float(velocity.get('y', 0))
                vel_z = float(velocity.get('z', 0))
            else:
                vel_x = vel_y = vel_z = 0.0

            self.gt_speed = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)

            # ============================================
            # Angular Velocity - Use orientation_rate directly!
            # orientation_rate is in rad/s (world frame)
            # ============================================
            orientation_rate = base.get('orientation_rate', {})
            if isinstance(orientation_rate, dict):
                # Extract roll/pitch/yaw rates
                or_roll = float(orientation_rate.get('roll', orientation_rate.get('x', 0)))
                or_pitch = float(orientation_rate.get('pitch', orientation_rate.get('y', 0)))
                or_yaw = float(orientation_rate.get('yaw', orientation_rate.get('z', 0)))

                # Transform to body frame using rotation matrix
                R = quaternion_to_rotation_matrix(curr_orientation)
                world_omega = np.array([or_roll, or_pitch, or_yaw])
                body_omega = R.T @ world_omega  # World to body

                self.gt_angular_velocity = {
                    'x': float(body_omega[0]),
                    'y': float(body_omega[1]),
                    'z': float(body_omega[2])
                }

            # ============================================
            # Linear Acceleration - Use acceleration directly!
            # GT acceleration is coordinate acceleration (no gravity)
            # We need to add gravity in body frame to compare with IMU
            # ============================================
            acceleration = base.get('acceleration', {})
            if isinstance(acceleration, dict):
                # GT acceleration is in world frame, no gravity
                acc_x = float(acceleration.get('x', 0))
                acc_y = float(acceleration.get('y', 0))
                acc_z = float(acceleration.get('z', 0))

                GRAVITY = 9.81
                R = quaternion_to_rotation_matrix(curr_orientation)

                # Transform acceleration from world to body frame
                world_acc = np.array([acc_x, acc_y, acc_z])
                body_acc = R.T @ world_acc

                # Add gravity in body frame (gravity is -Z in world, transformed to body)
                # World gravity vector: [0, 0, -g], but IMU measures reaction: [0, 0, +g]
                world_gravity_reaction = np.array([0, 0, GRAVITY])
                body_gravity = R.T @ world_gravity_reaction

                self.gt_linear_acceleration = {
                    'x': float(body_acc[0] + body_gravity[0]),
                    'y': float(body_acc[1] + body_gravity[1]),
                    'z': float(body_acc[2] + body_gravity[2])
                }

        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as e:
            if self.gt_count < 5:
                self.get_logger().warn(f"GT parse error: {e}")

    def _angle_diff(self, a: float, b: float) -> float:
        """Calculate shortest angle difference (handles wrap-around)"""
        diff = a - b
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def _on_control(self):
        """Auto-drive control for testing IMU during motion"""
        if not self.auto_drive_enabled:
            return

        elapsed = time.time() - self.phase_start_time

        # Transition to next phase
        if elapsed > self.phase_duration:
            self.drive_phase = (self.drive_phase + 1) % 4
            self.phase_start_time = time.time()
            phase_names = ["Accelerate", "Brake", "Steer Left", "Steer Right"]
            self.get_logger().info(f"[AUTO-DRIVE] Switching to Phase {self.drive_phase}: {phase_names[self.drive_phase]}")

        # Apply control based on phase
        throttle = 0.0
        steer = 0.0
        brake = 0.0

        if self.drive_phase == 0:  # Accelerate
            throttle = 0.5
        elif self.drive_phase == 1:  # Brake
            brake = 0.5
        elif self.drive_phase == 2:  # Steer Left + Throttle
            throttle = 0.3
            steer = -0.3
        elif self.drive_phase == 3:  # Steer Right + Throttle
            throttle = 0.3
            steer = 0.3

        self.vss_client.set_drive_control(throttle, steer, brake)

    def _on_imu(self, msg: Imu):
        self.last_imu = msg
        self.imu_count += 1

    def _on_validate(self):
        """Compare IMU and Ground Truth data"""
        if self.last_imu is None:
            return

        imu = self.last_imu

        # Compare Angular Velocity (both in rad/s)
        if self.gt_angular_velocity is not None:
            self.gyro_x_error_history.append(
                abs(imu.angular_velocity.x - self.gt_angular_velocity['x'])
            )
            self.gyro_y_error_history.append(
                abs(imu.angular_velocity.y - self.gt_angular_velocity['y'])
            )
            self.gyro_z_error_history.append(
                abs(imu.angular_velocity.z - self.gt_angular_velocity['z'])
            )

        # Compare Linear Acceleration
        # Check if IMU is in g units or m/s² by looking at z-axis (should be ~1g or ~9.81)
        GRAVITY = 9.81

        if self.gt_linear_acceleration is not None:
            # Detect IMU unit: if z is close to 1, it's in g units
            # if z is close to 9.81, it's in m/s²
            imu_z = imu.linear_acceleration.z
            if abs(imu_z - 1.0) < abs(imu_z - GRAVITY):
                # IMU is in g units, convert to m/s²
                imu_acc_x = imu.linear_acceleration.x * GRAVITY
                imu_acc_y = imu.linear_acceleration.y * GRAVITY
                imu_acc_z = imu.linear_acceleration.z * GRAVITY
            else:
                # IMU is already in m/s²
                imu_acc_x = imu.linear_acceleration.x
                imu_acc_y = imu.linear_acceleration.y
                imu_acc_z = imu.linear_acceleration.z

            self.acc_x_error_history.append(
                abs(imu_acc_x - self.gt_linear_acceleration['x'])
            )
            self.acc_y_error_history.append(
                abs(imu_acc_y - self.gt_linear_acceleration['y'])
            )
            self.acc_z_error_history.append(
                abs(imu_acc_z - self.gt_linear_acceleration['z'])
            )

    def _on_display(self):
        """Display validation results"""
        self.get_logger().info("-" * 70)

        # Check data availability
        if self.imu_count == 0:
            self.get_logger().warn("No IMU data received!")
            return

        self.get_logger().info(f"Data count - IMU: {self.imu_count}, Ground Truth: {self.gt_count}")

        if self.gt_count == 0:
            self.get_logger().warn("No Ground Truth data received!")
            return

        if self.gt_count < 3:
            self.get_logger().info("Waiting for derivative calculation (need 2+ GT samples)...")
            return

        GRAVITY = 9.81

        # Current values comparison
        if self.last_imu is not None:
            imu = self.last_imu

            # Detect IMU unit
            imu_z = imu.linear_acceleration.z
            is_g_unit = abs(imu_z - 1.0) < abs(imu_z - GRAVITY)
            unit_str = "g" if is_g_unit else "m/s²"

            self.get_logger().info("--- Angular Velocity (rad/s) ---")
            self.get_logger().info(
                f"  IMU:  x={imu.angular_velocity.x:+.4f}, "
                f"y={imu.angular_velocity.y:+.4f}, "
                f"z={imu.angular_velocity.z:+.4f}"
            )
            if self.gt_angular_velocity is not None:
                gt = self.gt_angular_velocity
                self.get_logger().info(
                    f"  GT:   x={gt['x']:+.4f}, y={gt['y']:+.4f}, z={gt['z']:+.4f}"
                )
                self.get_logger().info(
                    f"  Diff: x={abs(imu.angular_velocity.x - gt['x']):.4f}, "
                    f"y={abs(imu.angular_velocity.y - gt['y']):.4f}, "
                    f"z={abs(imu.angular_velocity.z - gt['z']):.4f}"
                )
            else:
                self.get_logger().warn("  GT angular velocity not yet calculated")

            self.get_logger().info(f"--- Linear Acceleration (IMU unit: {unit_str}) ---")
            if is_g_unit:
                imu_acc_x = imu.linear_acceleration.x * GRAVITY
                imu_acc_y = imu.linear_acceleration.y * GRAVITY
                imu_acc_z = imu.linear_acceleration.z * GRAVITY
            else:
                imu_acc_x = imu.linear_acceleration.x
                imu_acc_y = imu.linear_acceleration.y
                imu_acc_z = imu.linear_acceleration.z

            self.get_logger().info(
                f"  IMU:  x={imu_acc_x:+.4f}, y={imu_acc_y:+.4f}, z={imu_acc_z:+.4f} m/s²"
            )
            if self.gt_linear_acceleration is not None:
                gt = self.gt_linear_acceleration
                self.get_logger().info(
                    f"  GT:   x={gt['x']:+.4f}, y={gt['y']:+.4f}, z={gt['z']:+.4f} m/s²"
                )
                # Show signed difference for debugging direction mismatch
                diff_x = imu_acc_x - gt['x']
                diff_y = imu_acc_y - gt['y']
                diff_z = imu_acc_z - gt['z']
                self.get_logger().info(
                    f"  Diff: x={diff_x:+.4f}, y={diff_y:+.4f}, z={diff_z:+.4f} (signed)"
                )
            else:
                self.get_logger().warn("  GT linear acceleration not yet calculated")

        # Speed reference and auto-drive phase
        if self.gt_speed is not None:
            phase_names = ["Accelerate", "Brake", "Steer Left", "Steer Right"]
            phase_name = phase_names[self.drive_phase] if self.auto_drive_enabled else "Manual"
            self.get_logger().info(
                f"--- Speed: {self.gt_speed:.2f} m/s ({self.gt_speed * 3.6:.1f} km/h) | Phase: {phase_name} ---"
            )

        # Error statistics - Angular Velocity
        if len(self.gyro_z_error_history) > 10:
            avg_gyro_x_error = sum(self.gyro_x_error_history) / len(self.gyro_x_error_history)
            avg_gyro_y_error = sum(self.gyro_y_error_history) / len(self.gyro_y_error_history)
            avg_gyro_z_error = sum(self.gyro_z_error_history) / len(self.gyro_z_error_history)
            max_gyro_z_error = max(self.gyro_z_error_history)

            self.get_logger().info("--- Angular Velocity Error Stats ---")
            self.get_logger().info(
                f"  Avg error: x={avg_gyro_x_error:.4f}, y={avg_gyro_y_error:.4f}, z={avg_gyro_z_error:.4f} rad/s"
            )
            self.get_logger().info(f"  Avg gyro_z: {avg_gyro_z_error:.4f} rad/s ({math.degrees(avg_gyro_z_error):.2f} deg/s)")
            self.get_logger().info(f"  Max gyro_z: {max_gyro_z_error:.4f} rad/s")

            # Validation result
            if avg_gyro_z_error < 0.02:  # 0.02 rad/s ≈ 1.1 deg/s
                self.get_logger().info("  [RESULT] IMU angular_velocity is ACCURATE ✓")
            elif avg_gyro_z_error < 0.1:  # 0.1 rad/s ≈ 5.7 deg/s
                self.get_logger().warn("  [RESULT] IMU angular_velocity has MODERATE error")
            else:
                self.get_logger().error("  [RESULT] IMU angular_velocity has LARGE error")

        # Error statistics - Linear Acceleration
        if len(self.acc_z_error_history) > 10:
            avg_acc_x_error = sum(self.acc_x_error_history) / len(self.acc_x_error_history)
            avg_acc_y_error = sum(self.acc_y_error_history) / len(self.acc_y_error_history)
            avg_acc_z_error = sum(self.acc_z_error_history) / len(self.acc_z_error_history)

            self.get_logger().info("--- Linear Acceleration Error Stats ---")
            self.get_logger().info(
                f"  Avg error: x={avg_acc_x_error:.4f}, y={avg_acc_y_error:.4f}, z={avg_acc_z_error:.4f} m/s²"
            )

            # Validation result
            if avg_acc_z_error < 0.5:  # 0.5 m/s²
                self.get_logger().info("  [RESULT] IMU linear_acceleration is ACCURATE ✓")
            elif avg_acc_z_error < 2.0:
                self.get_logger().warn("  [RESULT] IMU linear_acceleration has MODERATE error")
            else:
                self.get_logger().error("  [RESULT] IMU linear_acceleration has LARGE error")


def main(args=None):
    rclpy.init(args=args)
    node = ImuVssValidationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Validation stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
