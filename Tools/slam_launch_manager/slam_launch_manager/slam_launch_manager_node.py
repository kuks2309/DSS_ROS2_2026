#!/usr/bin/env python3

import sys
import os
import subprocess
import signal
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2, Imu, Image, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import time
import threading

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer, QDateTime
from PyQt5.QtWidgets import QFileDialog, QMessageBox


class SlamLaunchManagerNode(Node):
    def __init__(self, ui_window):
        super().__init__('slam_launch_manager_node')
        self.ui = ui_window

        # Dictionary to store running processes
        self.processes = {
            'dss': None,
            'livox': None,
            'rtabmap': None,
            'rtabmap_loc': None,
            'liosam': None,
            'liosam_loc': None,
            'dss_lio_sam': None,
            'dss_lio_sam_loc': None,
            'kissicp': None,
            'slamtoolbox': None,
            'slamtoolbox_loc': None,
            'localization': None,
            'custom': None
        }

        # Store launch file paths
        self.launch_files = {
            'dss': None,  # DSS ROS2 Bridge
            'livox': None,  # Livox MID-360 driver
            'rtabmap': None,  # RTAB-MAP SLAM mode
            'rtabmap_loc': None,  # RTAB-MAP Localization mode
            'liosam': None,  # LIO-SAM SLAM mode
            'liosam_loc': None,  # LIO-SAM Localization mode
            'dss_lio_sam': None,  # DSS LIO-SAM for simulation
            'dss_lio_sam_loc': None,  # DSS LIO-SAM Localization mode
            'kissicp': None,  # Will be set from config or UI
            'slamtoolbox': None,  # Will be set from config or UI
            'slamtoolbox_loc': None,  # Will be set from config or UI
            'localization': None,  # Will be set from config or UI
            'custom': None
        }

        # Store map database path
        self.map_database_path = None
        self.slamtoolbox_map_path = None

        # Sensor status tracking
        self.sensor_last_time = {
            'lidar': 0.0,
            'imu': 0.0,
            'camera': 0.0,
            'gps': 0.0
        }
        self.sensor_timeout = 2.0  # seconds

        # QoS profile for sensor topics (best effort to match typical sensor publishers)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscriptions for sensor topics with appropriate QoS
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/dss/sensor/lidar3d', self.lidar_callback, sensor_qos)
        self.imu_sub = self.create_subscription(
            Imu, '/dss/sensor/imu', self.imu_callback, sensor_qos)
        self.camera_sub = self.create_subscription(
            Image, '/dss/sensor/camera/rgb', self.camera_callback, sensor_qos)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/dss/sensor/gps/fix', self.gps_callback, sensor_qos)

        # Subscribe to /initialpose for automatic localization reset
        initialpose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, initialpose_qos)

        # Service clients for localization reset
        self.clear_localization_buffer_client = None
        self.reset_odom_client = None

        self.get_logger().info('Launch Manager Node initialized')

    def start_launch_file(self, launch_key, launch_file_path, extra_args=None):
        """Start a ROS2 launch file"""
        if self.processes[launch_key] is not None:
            self.ui.log(f"Launch '{launch_key}' is already running!")
            return False

        if not launch_file_path or not os.path.exists(launch_file_path):
            self.ui.log(f"Launch file not found: {launch_file_path}")
            return False

        try:
            # For RTAB-Map, ensure clean DDS state BEFORE starting
            if launch_key == 'rtabmap':
                self.ui.log("Cleaning DDS state before RTAB-Map start...")
                import time
                try:
                    subprocess.run(['ros2', 'daemon', 'stop'], timeout=5, capture_output=True)
                    time.sleep(0.5)
                    subprocess.run(['ros2', 'daemon', 'start'], timeout=5, capture_output=True)
                    time.sleep(1)  # Give daemon time to fully restart
                    self.ui.log("DDS state cleaned")
                except Exception as e:
                    self.ui.log(f"Warning: Could not clean DDS state: {e}")

            # Start the launch file using ros2 launch command
            cmd = ['ros2', 'launch', launch_file_path]

            # Add extra arguments (e.g., database_path for localization)
            if extra_args:
                cmd.extend(extra_args)

            # Inherit environment variables including DISPLAY for GUI applications
            env = os.environ.copy()

            # Ensure ROS_DOMAIN_ID is set (use default 0 if not set)
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'

            # Clear any RMW implementation cache
            env.pop('RMW_IMPLEMENTATION', None)

            # Write PID to a temporary file so we can track the detached process
            import tempfile
            pid_file = tempfile.NamedTemporaryFile(mode='w', suffix='.pid', delete=False)
            pid_file_path = pid_file.name
            pid_file.close()

            # Create launch script with proper environment setup
            script_content = f"""#!/bin/bash
set -e

# Source ROS2 setup
source {os.path.expanduser('~/ros2_ws/install/setup.bash')}

# Write PID for tracking
echo $$ > {pid_file_path}

# Change to home directory (like terminal does)
cd {os.path.expanduser('~')}

# Execute launch command
exec {' '.join(cmd)}
"""

            # Create temporary script file
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sh', delete=False) as f:
                script_path = f.name
                f.write(script_content)

            os.chmod(script_path, 0o755)

            # Start the process completely detached using setsid
            # DO NOT redirect stdout/stderr to allow ROS2 nodes to communicate properly
            process = subprocess.Popen(
                ['setsid', 'bash', script_path],
                env=env,
                stdin=subprocess.DEVNULL,
                cwd=os.path.expanduser('~'),
                preexec_fn=os.setpgrp  # Create new process group
            )

            # Wait a moment for PID file to be written
            import time
            time.sleep(0.5)

            # Read the actual PID from the file
            actual_pid = None
            try:
                with open(pid_file_path, 'r') as f:
                    actual_pid = int(f.read().strip())
            except:
                actual_pid = process.pid

            # Clean up temp files after a delay
            import threading
            def cleanup_files():
                import time
                time.sleep(5)
                try:
                    os.unlink(script_path)
                    os.unlink(pid_file_path)
                except:
                    pass
            threading.Thread(target=cleanup_files, daemon=True).start()

            # Store a pseudo-process object with the actual PID
            class ProcessTracker:
                def __init__(self, pid):
                    self.pid = pid

                def poll(self):
                    # Check if process is still running
                    try:
                        os.kill(self.pid, 0)  # Signal 0 just checks existence
                        return None  # Still running
                    except OSError:
                        return 0  # Process ended

            self.processes[launch_key] = ProcessTracker(actual_pid)
            self.ui.log(f"Started launch file: {launch_file_path}")
            if extra_args:
                self.ui.log(f"  with args: {' '.join(extra_args)}")
            self.get_logger().info(f"Started {launch_key}: PID={actual_pid}")
            return True

        except Exception as e:
            self.ui.log(f"Failed to start launch file: {str(e)}")
            self.get_logger().error(f"Failed to start {launch_key}: {str(e)}")
            return False

    def stop_launch_file(self, launch_key):
        """Stop a running launch file"""
        if self.processes[launch_key] is None:
            self.ui.log(f"Launch '{launch_key}' is not running!")
            return False

        try:
            process = self.processes[launch_key]
            import time

            # Get all child processes recursively
            def get_process_tree(pid):
                """Get all child processes of a given PID"""
                try:
                    result = subprocess.run(
                        ['pgrep', '-P', str(pid)],
                        capture_output=True,
                        text=True,
                        timeout=2
                    )
                    child_pids = [int(p) for p in result.stdout.strip().split('\n') if p]
                    all_pids = child_pids.copy()
                    for child_pid in child_pids:
                        all_pids.extend(get_process_tree(child_pid))
                    return all_pids
                except:
                    return []

            # Get all processes in the tree
            all_pids = [process.pid] + get_process_tree(process.pid)

            # For dss launch, also find processes by name pattern
            if launch_key == 'dss':
                try:
                    # Find all dss_ros2_bridge related processes
                    result = subprocess.run(
                        ['pgrep', '-f', 'dss_ros2_bridge'],
                        capture_output=True,
                        text=True,
                        timeout=2
                    )
                    dss_pids = [int(p) for p in result.stdout.strip().split('\n') if p]
                    for pid in dss_pids:
                        if pid not in all_pids:
                            all_pids.append(pid)
                except:
                    pass

            self.ui.log(f"Stopping process tree: {all_pids}")

            # Send SIGINT to all processes
            for pid in reversed(all_pids):  # Kill children first
                try:
                    os.kill(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                except Exception as e:
                    self.ui.log(f"Warning: Could not send SIGINT to {pid}: {e}")

            # Wait for processes to terminate
            time.sleep(2)

            # Check if any processes are still alive and force kill them
            surviving_pids = []
            for pid in all_pids:
                try:
                    os.kill(pid, 0)  # Check if process exists
                    surviving_pids.append(pid)
                except ProcessLookupError:
                    pass

            if surviving_pids:
                self.ui.log(f"Force killing surviving processes: {surviving_pids}")
                for pid in reversed(surviving_pids):
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    except Exception as e:
                        self.ui.log(f"Warning: Could not force kill {pid}: {e}")

                time.sleep(1)

            self.processes[launch_key] = None
            self.ui.log(f"Stopped launch: {launch_key}")
            self.get_logger().info(f"Stopped {launch_key}")

            # For RTAB-Map, restart ROS2 daemon to ensure clean DDS state
            if launch_key == 'rtabmap':
                self.ui.log("Restarting ROS2 daemon for clean DDS state...")
                try:
                    subprocess.run(['ros2', 'daemon', 'stop'], timeout=5, capture_output=True)
                    time.sleep(0.5)
                    subprocess.run(['ros2', 'daemon', 'start'], timeout=5, capture_output=True)
                    self.ui.log("ROS2 daemon restarted")
                except Exception as e:
                    self.ui.log(f"Warning: Could not restart daemon: {e}")

            # Give sufficient time for all nodes, DDS participants, and topics to fully clean up
            self.ui.log("Waiting for cleanup to complete...")
            time.sleep(2)
            self.ui.log("Cleanup complete")

            return True

        except Exception as e:
            self.ui.log(f"Failed to stop launch: {str(e)}")
            self.get_logger().error(f"Failed to stop {launch_key}: {str(e)}")
            return False

    def stop_all_launches(self):
        """Stop all running launch files"""
        for key in self.processes.keys():
            if self.processes[key] is not None:
                self.stop_launch_file(key)
        self.ui.log("All launches stopped")

    def lidar_callback(self, msg):
        self.sensor_last_time['lidar'] = time.time()

    def imu_callback(self, msg):
        self.sensor_last_time['imu'] = time.time()

    def camera_callback(self, msg):
        self.sensor_last_time['camera'] = time.time()

    def gps_callback(self, msg):
        self.sensor_last_time['gps'] = time.time()

    def initialpose_callback(self, msg):
        """Handle /initialpose messages for automatic localization reset"""
        # Check if SLAM-Toolbox localization is running
        if not self.is_running('slamtoolbox_loc'):
            return

        self.get_logger().info(f'Received /initialpose: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')
        self.ui.log(f'Received /initialpose: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')

        # Call services in a separate thread to avoid blocking
        def reset_localization():
            try:
                # 1. Clear SLAM-Toolbox localization buffer
                if self.clear_localization_buffer_client is None:
                    self.clear_localization_buffer_client = self.create_client(
                        Empty, '/slam_toolbox/clear_localization_buffer')

                if self.clear_localization_buffer_client.wait_for_service(timeout_sec=2.0):
                    future = self.clear_localization_buffer_client.call_async(Empty.Request())
                    self.get_logger().info('Called /slam_toolbox/clear_localization_buffer')
                    self.ui.log('Cleared SLAM-Toolbox localization buffer')
                else:
                    self.get_logger().warn('clear_localization_buffer service not available')
                    self.ui.log('Warning: clear_localization_buffer service not available')

                # 2. Reset ICP odometry
                if self.reset_odom_client is None:
                    self.reset_odom_client = self.create_client(Empty, '/reset_odom')

                if self.reset_odom_client.wait_for_service(timeout_sec=2.0):
                    future = self.reset_odom_client.call_async(Empty.Request())
                    self.get_logger().info('Called /reset_odom')
                    self.ui.log('Reset ICP odometry')
                else:
                    self.get_logger().warn('reset_odom service not available')
                    self.ui.log('Warning: reset_odom service not available')

                self.ui.log('Localization reset complete - scan matching will start from new position')

            except Exception as e:
                self.get_logger().error(f'Error resetting localization: {e}')
                self.ui.log(f'Error resetting localization: {e}')

        # Run in separate thread
        reset_thread = threading.Thread(target=reset_localization, daemon=True)
        reset_thread.start()

    def get_sensor_status(self, sensor_name):
        """Check if sensor is active (received data within timeout)"""
        last_time = self.sensor_last_time.get(sensor_name, 0.0)
        if last_time == 0.0:
            return False
        return (time.time() - last_time) < self.sensor_timeout

    def is_running(self, launch_key):
        """Check if a launch file is currently running"""
        if self.processes[launch_key] is None:
            return False

        # Check if process is still alive
        poll = self.processes[launch_key].poll()
        if poll is not None:
            # Process has terminated
            self.processes[launch_key] = None
            return False

        return True


class SlamLaunchManagerUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # Load UI file
        ui_file = Path(__file__).parent / 'ui' / 'slam_launch_manager.ui'
        uic.loadUi(ui_file, self)

        # ROS2 node (will be initialized later)
        self.node = None

        # Connect buttons - DSS Bridge
        self.btnStartDSS.clicked.connect(self.on_start_dss)
        self.btnStopDSS.clicked.connect(self.on_stop_dss)

        # Connect buttons - Livox
        self.btnStartLivox.clicked.connect(self.on_start_livox)
        self.btnStopLivox.clicked.connect(self.on_stop_livox)

        self.btnStartLioSam.clicked.connect(self.on_start_liosam)
        self.btnStopLioSam.clicked.connect(self.on_stop_liosam)
        self.btnSaveLioSamMap.clicked.connect(self.on_save_liosam_map)
        self.btnStartLioSamLoc.clicked.connect(self.on_start_liosam_loc)
        self.btnStopLioSamLoc.clicked.connect(self.on_stop_liosam_loc)

        self.btnStartDssLioSam.clicked.connect(self.on_start_dss_lio_sam)
        self.btnStopDssLioSam.clicked.connect(self.on_stop_dss_lio_sam)
        self.btnSaveDssLioSamMap.clicked.connect(self.on_save_dss_lio_sam_map)

        self.btnBrowseDssLioSamMap.clicked.connect(self.on_browse_dss_lio_sam_map)
        self.btnStartDssLioSamLoc.clicked.connect(self.on_start_dss_lio_sam_loc)
        self.btnStopDssLioSamLoc.clicked.connect(self.on_stop_dss_lio_sam_loc)

        # Connect buttons - RTAB-MAP SLAM
        self.btnBrowseRtabmapDb.clicked.connect(self.on_browse_rtabmap_db)
        self.btnStartRtabmap.clicked.connect(self.on_start_rtabmap)
        self.btnStopRtabmap.clicked.connect(self.on_stop_rtabmap)
        self.btnSaveRtabmapMap.clicked.connect(self.on_save_rtabmap_map)

        # Connect buttons - RTAB-MAP Localization
        self.btnBrowseRtabmapLocDb.clicked.connect(self.on_browse_rtabmap_loc_db)
        self.btnStartRtabmapLoc.clicked.connect(self.on_start_rtabmap_loc)
        self.btnStopRtabmapLoc.clicked.connect(self.on_stop_rtabmap_loc)

        # Connect buttons - KISS-ICP
        self.btnStartKissIcp.clicked.connect(self.on_start_kissicp)
        self.btnStopKissIcp.clicked.connect(self.on_stop_kissicp)
        self.btnSaveKissIcpMap.clicked.connect(self.on_save_kissicp_map)

        # Connect buttons - SLAM-Toolbox
        self.btnStartSlamToolbox.clicked.connect(self.on_start_slamtoolbox)
        self.btnStopSlamToolbox.clicked.connect(self.on_stop_slamtoolbox)
        self.btnSaveSlamToolboxMap.clicked.connect(self.on_save_slamtoolbox_map)
        self.btnBrowseSlamToolboxMap.clicked.connect(self.on_browse_slamtoolbox_map)
        self.btnStartSlamToolboxLoc.clicked.connect(self.on_start_slamtoolbox_loc)
        self.btnStopSlamToolboxLoc.clicked.connect(self.on_stop_slamtoolbox_loc)

        self.btnStartCustom.clicked.connect(self.on_start_custom)
        self.btnStopCustom.clicked.connect(self.on_stop_custom)
        self.btnBrowse.clicked.connect(self.on_browse)

        self.btnStopAll.clicked.connect(self.on_stop_all)

        # Timer to check process status
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_button_states)
        self.status_timer.start(500)  # Check every 500ms

        # Timer to update sensor status
        self.sensor_timer = QTimer()
        self.sensor_timer.timeout.connect(self.update_sensor_status)
        self.sensor_timer.start(500)  # Check every 500ms

        self.log("Launch Manager UI Ready")

    def set_node(self, node):
        """Set the ROS2 node"""
        self.node = node

        # Try to auto-detect launch files
        self.auto_detect_launch_files()

    def auto_detect_launch_files(self):
        """Auto-detect launch files in the workspace"""
        home = Path.home()
        workspace = home / 'parking_robot_ros2_ws' / 'src'

        # Look for Livox MID-360 launch file
        livox_launch = home / 'ws_livox' / 'install' / 'livox_ros_driver2' / 'share' / 'livox_ros_driver2' / 'launch_ROS2' / 'msg_MID360_launch.py'
        if livox_launch.exists():
            self.node.launch_files['livox'] = str(livox_launch)
            self.log(f"Found Livox MID-360 launch: {livox_launch}")

        # Look for LIO-SAM SLAM launch file
        liosam_launch = workspace / 'SLAM' / 'LIO-SAM' / 'livox_lio_sam' / 'launch' / 'run_slam.launch.py'
        if liosam_launch.exists():
            self.node.launch_files['liosam'] = str(liosam_launch)
            self.log(f"Found LIO-SAM SLAM launch: {liosam_launch}")

        # Look for LIO-SAM Localization launch file
        liosam_loc_launch = workspace / 'SLAM' / 'LIO-SAM' / 'livox_lio_sam' / 'launch' / 'run_localization.launch.py'
        if liosam_loc_launch.exists():
            self.node.launch_files['liosam_loc'] = str(liosam_loc_launch)
            self.log(f"Found LIO-SAM Localization launch: {liosam_loc_launch}")

        # Look for RTAB-Map SLAM launch file
        rtabmap_launch = workspace / 'SLAM' / 'rtab_map' / 'livox_rtabmap_slam' / 'launch' / 'rtabmap_with_rviz.launch.py'
        if rtabmap_launch.exists():
            self.node.launch_files['rtabmap'] = str(rtabmap_launch)
            self.log(f"Found RTAB-Map launch: {rtabmap_launch}")

        # Look for DSS ROS2 Bridge launch file
        dss_workspace = home / 'ros2_ws' / 'src'
        dss_launch = dss_workspace / 'dss_ros2_bridge' / 'launch' / 'launch.py'
        if dss_launch.exists():
            self.node.launch_files['dss'] = str(dss_launch)
            self.log(f"Found DSS Bridge launch: {dss_launch}")

        # Look for DSS LIO-SAM launch file (for simulation)
        dss_lio_sam_launch = dss_workspace / 'SLAM' / 'LIO-SAM' / 'dss_lio_sam' / 'launch' / 'run.launch.py'
        if dss_lio_sam_launch.exists():
            self.node.launch_files['dss_lio_sam'] = str(dss_lio_sam_launch)
            self.log(f"Found DSS LIO-SAM launch: {dss_lio_sam_launch}")

        # Look for DSS LIO-SAM Localization launch file
        dss_lio_sam_loc_launch = dss_workspace / 'SLAM' / 'LIO-SAM' / 'dss_lio_sam' / 'launch' / 'run_localization.launch.py'
        if dss_lio_sam_loc_launch.exists():
            self.node.launch_files['dss_lio_sam_loc'] = str(dss_lio_sam_loc_launch)
            self.log(f"Found DSS LIO-SAM Localization launch: {dss_lio_sam_loc_launch}")

        # Look for DSS RTAB-MAP SLAM launch file (for simulation)
        dss_rtabmap_launch = dss_workspace / 'SLAM' / 'RTAB-MAP' / 'dss_rtabmap_slam' / 'launch' / 'rtabmap_with_rviz.launch.py'
        if dss_rtabmap_launch.exists():
            self.node.launch_files['rtabmap'] = str(dss_rtabmap_launch)
            self.log(f"Found DSS RTAB-MAP launch: {dss_rtabmap_launch}")

        # Look for DSS RTAB-MAP Localization launch file (dedicated localization package)
        dss_rtabmap_loc_launch = dss_workspace / 'SLAM' / 'RTAB-MAP' / 'dss_rtabmap_localization' / 'launch' / 'rtabmap_localization.launch.py'
        if dss_rtabmap_loc_launch.exists():
            self.node.launch_files['rtabmap_loc'] = str(dss_rtabmap_loc_launch)
            self.log(f"Found DSS RTAB-MAP Localization launch: {dss_rtabmap_loc_launch}")

        # Look for DSS KISS-ICP launch file
        dss_kissicp_launch = dss_workspace / 'SLAM' / 'KISS-ICP' / 'dss_kiss_icp' / 'launch' / 'run.launch.py'
        if dss_kissicp_launch.exists():
            self.node.launch_files['kissicp'] = str(dss_kissicp_launch)
            self.log(f"Found DSS KISS-ICP launch: {dss_kissicp_launch}")

        # Look for DSS SLAM-Toolbox launch files
        dss_slamtoolbox_launch = dss_workspace / 'SLAM' / 'SLAM-Toolbox' / 'dss_slam_toolbox' / 'launch' / 'slam_mapping.launch.py'
        if dss_slamtoolbox_launch.exists():
            self.node.launch_files['slamtoolbox'] = str(dss_slamtoolbox_launch)
            self.log(f"Found DSS SLAM-Toolbox launch: {dss_slamtoolbox_launch}")

        dss_slamtoolbox_loc_launch = dss_workspace / 'SLAM' / 'SLAM-Toolbox' / 'dss_slam_toolbox' / 'launch' / 'slam_localization.launch.py'
        if dss_slamtoolbox_loc_launch.exists():
            self.node.launch_files['slamtoolbox_loc'] = str(dss_slamtoolbox_loc_launch)
            self.log(f"Found DSS SLAM-Toolbox Localization launch: {dss_slamtoolbox_loc_launch}")

    def log(self, message):
        """Add message to log"""
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.txtLog.append(f"[{timestamp}] {message}")

    def on_start_dss(self):
        if self.node.launch_files['dss']:
            extra_args = ['use_sim_time:=true']
            if self.node.start_launch_file('dss', self.node.launch_files['dss'], extra_args):
                self.update_button_states()
        else:
            self.log("DSS launch file not configured!")
            QMessageBox.warning(self, "Error", "DSS launch file not found!")

    def on_stop_dss(self):
        if self.node.stop_launch_file('dss'):
            self.update_button_states()

    def on_start_livox(self):
        if self.node.launch_files['livox']:
            if self.node.start_launch_file('livox', self.node.launch_files['livox']):
                self.update_button_states()
        else:
            self.log("Livox MID-360 launch file not configured!")
            QMessageBox.warning(self, "Error", "Livox MID-360 launch file not found!\nPlease install livox_ros_driver2.")

    def on_stop_livox(self):
        if self.node.stop_launch_file('livox'):
            self.update_button_states()

    def on_start_liosam(self):
        if self.node.launch_files['liosam']:
            if self.node.start_launch_file('liosam', self.node.launch_files['liosam']):
                self.update_button_states()
        else:
            self.log("LIO-SAM SLAM launch file not configured!")
            QMessageBox.warning(self, "Error", "LIO-SAM SLAM launch file not found!")

    def on_stop_liosam(self):
        if self.node.stop_launch_file('liosam'):
            self.update_button_states()

    def on_save_liosam_map(self):
        """Save LIO-SAM map using service call with folder selection"""
        try:
            # Open folder selection dialog
            default_path = str(Path.home() / "parking_robot_ros2_ws/src/SLAM/LIO-SAM/livox_lio_sam/map")
            save_dir = QFileDialog.getExistingDirectory(
                self,
                "Select Directory to Save Map",
                default_path,
                QFileDialog.ShowDirsOnly
            )

            if not save_dir:
                self.log("Map save cancelled by user")
                return

            # Ask for map name
            from PyQt5.QtWidgets import QInputDialog
            map_name, ok = QInputDialog.getText(
                self,
                "Map Name",
                "Enter map name (without extension):",
                text="livox_map"
            )

            if not ok or not map_name:
                self.log("Map save cancelled by user")
                return

            # Full save path
            save_path = os.path.join(save_dir, map_name)

            self.log(f"Saving map to: {save_path}")

            import subprocess
            result = subprocess.run(
                ['ros2', 'service', 'call', '/lio_sam/save_map',
                 'livox_lio_sam/srv/SaveMap',
                 f'{{"resolution": 0.2, "destination": "{save_path}"}}'],
                capture_output=True,
                text=True,
                timeout=60
            )

            if result.returncode == 0:
                self.log(f"LIO-SAM map saved successfully to: {save_path}")
                QMessageBox.information(self, "Success", f"Map saved successfully!\n\nLocation: {save_path}")
            else:
                self.log(f"Failed to save map: {result.stderr}")
                QMessageBox.warning(self, "Error", f"Failed to save map:\n{result.stderr}")

        except Exception as e:
            self.log(f"Failed to save map: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to save map:\n{str(e)}")

    def on_start_liosam_loc(self):
        if self.node.launch_files['liosam_loc']:
            if self.node.start_launch_file('liosam_loc', self.node.launch_files['liosam_loc']):
                self.update_button_states()
        else:
            self.log("LIO-SAM Localization launch file not configured!")
            QMessageBox.warning(self, "Error", "LIO-SAM Localization launch file not found!")

    def on_stop_liosam_loc(self):
        if self.node.stop_launch_file('liosam_loc'):
            self.update_button_states()

    def on_start_dss_lio_sam(self):
        if self.node.launch_files['dss_lio_sam']:
            if self.node.start_launch_file('dss_lio_sam', self.node.launch_files['dss_lio_sam']):
                self.update_button_states()
        else:
            self.log("DSS LIO-SAM launch file not configured!")
            QMessageBox.warning(self, "Error", "DSS LIO-SAM launch file not found!")

    def on_stop_dss_lio_sam(self):
        if self.node.stop_launch_file('dss_lio_sam'):
            self.update_button_states()

    def on_save_dss_lio_sam_map(self):
        """Save DSS LIO-SAM map using service call with folder selection"""
        try:
            # Open folder selection dialog
            default_path = str(Path.home() / "ros2_ws/src/SLAM/LIO-SAM/dss_lio_sam/map")
            save_dir = QFileDialog.getExistingDirectory(
                self,
                "Select Directory to Save Map",
                default_path,
                QFileDialog.ShowDirsOnly
            )

            if not save_dir:
                self.log("Map save cancelled by user")
                return

            # Ask for map name
            from PyQt5.QtWidgets import QInputDialog
            map_name, ok = QInputDialog.getText(
                self,
                "Map Name",
                "Enter map name (without extension):",
                text="dss_map"
            )

            if not ok or not map_name:
                self.log("Map save cancelled by user")
                return

            # Full save path
            save_path = os.path.join(save_dir, map_name)

            self.log(f"Saving map to: {save_path}")

            import subprocess

            # First check if service exists
            check_result = subprocess.run(
                ['ros2', 'service', 'list'],
                capture_output=True,
                text=True,
                timeout=10
            )

            if '/lio_sam/save_map' not in check_result.stdout:
                self.log("Error: /lio_sam/save_map service not found!")
                self.log("Make sure DSS LIO-SAM is running.")
                QMessageBox.warning(self, "Error", "save_map service not found!\n\nMake sure DSS LIO-SAM is running.")
                return

            self.log("Calling save_map service...")
            result = subprocess.run(
                ['ros2', 'service', 'call', '/lio_sam/save_map',
                 'dss_lio_sam/srv/SaveMap',
                 f'{{"resolution": 0.2, "destination": "{save_path}"}}'],
                capture_output=True,
                text=True,
                timeout=120
            )

            self.log(f"Service call stdout: {result.stdout}")
            if result.stderr:
                self.log(f"Service call stderr: {result.stderr}")

            if result.returncode == 0 and 'success=True' in result.stdout:
                self.log(f"DSS LIO-SAM map saved successfully to: {save_path}")
                QMessageBox.information(self, "Success", f"Map saved successfully!\n\nLocation: {save_path}")
            elif result.returncode == 0:
                self.log(f"Map save completed: {result.stdout}")
                QMessageBox.information(self, "Complete", f"Map save completed.\n\nCheck: {save_path}")
            else:
                self.log(f"Failed to save map: {result.stderr}")
                QMessageBox.warning(self, "Error", f"Failed to save map:\n{result.stderr}")

        except Exception as e:
            self.log(f"Failed to save map: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to save map:\n{str(e)}")

    def on_browse_dss_lio_sam_map(self):
        """Browse for DSS LIO-SAM map folder"""
        default_path = str(Path.home() / "ros2_ws/map")
        map_dir = QFileDialog.getExistingDirectory(
            self,
            "Select Map Folder (containing GlobalMap.pcd)",
            default_path,
            QFileDialog.ShowDirsOnly
        )
        if map_dir:
            # Check if GlobalMap.pcd exists
            global_map_path = os.path.join(map_dir, "GlobalMap.pcd")
            if os.path.exists(global_map_path):
                self.txtDssLioSamMapPath.setText(global_map_path)
                self.log(f"Selected map: {global_map_path}")
            else:
                self.log(f"Warning: GlobalMap.pcd not found in {map_dir}")
                QMessageBox.warning(self, "Warning", f"GlobalMap.pcd not found in:\n{map_dir}\n\nPlease select a valid map folder.")

    def on_start_dss_lio_sam_loc(self):
        """Start DSS LIO-SAM Localization mode"""
        map_path = self.txtDssLioSamMapPath.text()
        if not map_path:
            self.log("Please select a map file first!")
            QMessageBox.warning(self, "Error", "Please select a map file first!")
            return

        if not os.path.exists(map_path):
            self.log(f"Map file not found: {map_path}")
            QMessageBox.warning(self, "Error", f"Map file not found:\n{map_path}")
            return

        if self.node.launch_files.get('dss_lio_sam_loc'):
            extra_args = [f'map_path:={map_path}']
            if self.node.start_launch_file('dss_lio_sam_loc', self.node.launch_files['dss_lio_sam_loc'], extra_args):
                self.log(f"Started DSS LIO-SAM Localization with map: {map_path}")
                self.update_button_states()
        else:
            self.log("DSS LIO-SAM Localization launch file not found!")
            QMessageBox.warning(self, "Error", "DSS LIO-SAM Localization launch file not found!")

    def on_stop_dss_lio_sam_loc(self):
        """Stop DSS LIO-SAM Localization mode"""
        if self.node.stop_launch_file('dss_lio_sam_loc'):
            self.update_button_states()

    def on_browse_rtabmap_db(self):
        """Browse for RTAB-MAP database path (SLAM mode)"""
        default_path = str(Path.home() / "ros2_ws/map")
        db_path, _ = QFileDialog.getSaveFileName(
            self,
            "Select RTAB-MAP Database Path",
            os.path.join(default_path, "rtabmap.db"),
            "Database Files (*.db);;All Files (*)"
        )
        if db_path:
            self.txtRtabmapDbPath.setText(db_path)
            self.log(f"Selected RTAB-MAP database: {db_path}")

    def on_start_rtabmap(self):
        """Start RTAB-MAP SLAM mode"""
        db_path = self.txtRtabmapDbPath.text()

        if self.node.launch_files.get('rtabmap'):
            extra_args = ['use_sim_time:=true']  # Always use simulation time
            if db_path:
                extra_args.append(f'database_path:={db_path}')
                extra_args.append('delete_db_on_start:=true')
            if self.node.start_launch_file('rtabmap', self.node.launch_files['rtabmap'], extra_args):
                self.log(f"Started RTAB-MAP SLAM mode")
                if db_path:
                    self.log(f"  Database: {db_path}")
                self.update_button_states()
        else:
            self.log("RTAB-MAP launch file not found!")
            QMessageBox.warning(self, "Error", "RTAB-MAP launch file not found!")

    def on_stop_rtabmap(self):
        """Stop RTAB-MAP SLAM mode"""
        if self.node.stop_launch_file('rtabmap'):
            self.update_button_states()

    def on_save_rtabmap_map(self):
        """Save RTAB-MAP map by copying database file"""
        try:
            # Open folder selection dialog
            default_path = str(Path.home() / "ros2_ws/map")
            save_dir = QFileDialog.getExistingDirectory(
                self,
                "Select Directory to Save Map",
                default_path,
                QFileDialog.ShowDirsOnly
            )

            if not save_dir:
                self.log("Map save cancelled by user")
                return

            # Ask for map name
            from PyQt5.QtWidgets import QInputDialog
            map_name, ok = QInputDialog.getText(
                self,
                "Map Name",
                "Enter map name (without extension):",
                text="rtabmap_map"
            )

            if not ok or not map_name:
                self.log("Map save cancelled by user")
                return

            # Full save path
            save_path = os.path.join(save_dir, f"{map_name}.db")

            self.log(f"Saving RTAB-MAP map to: {save_path}")

            # Get current database path from UI or use default
            current_db_path = self.txtRtabmapDbPath.text()
            if not current_db_path:
                current_db_path = os.path.expanduser("~/.ros/rtabmap.db")

            # Expand path
            current_db_path = os.path.expanduser(current_db_path)

            if not os.path.exists(current_db_path):
                self.log(f"Database file not found: {current_db_path}")
                QMessageBox.warning(self, "Error", f"Database file not found:\n{current_db_path}")
                return

            # Copy the database file
            import shutil
            shutil.copy2(current_db_path, save_path)

            self.log(f"RTAB-MAP map saved successfully to: {save_path}")
            QMessageBox.information(self, "Success", f"Map saved successfully!\n\nLocation: {save_path}")

        except Exception as e:
            self.log(f"Failed to save map: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to save map:\n{str(e)}")

    def on_browse_rtabmap_loc_db(self):
        """Browse for existing RTAB-MAP database (Localization mode)"""
        default_path = str(Path.home() / "ros2_ws/map")
        db_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select RTAB-MAP Database for Localization",
            default_path,
            "Database Files (*.db);;All Files (*)"
        )
        if db_path:
            if os.path.exists(db_path):
                self.txtRtabmapLocDbPath.setText(db_path)
                self.log(f"Selected RTAB-MAP map database: {db_path}")
            else:
                self.log(f"Database file not found: {db_path}")
                QMessageBox.warning(self, "Error", f"Database file not found:\n{db_path}")

    def on_start_rtabmap_loc(self):
        """Start RTAB-MAP Localization mode"""
        db_path = self.txtRtabmapLocDbPath.text()
        if not db_path:
            self.log("Please select a database file first!")
            QMessageBox.warning(self, "Error", "Please select a database file first!")
            return

        if not os.path.exists(db_path):
            self.log(f"Database file not found: {db_path}")
            QMessageBox.warning(self, "Error", f"Database file not found:\n{db_path}")
            return

        if self.node.launch_files.get('rtabmap_loc'):
            # New dedicated localization launch file only needs database_path
            extra_args = [f'database_path:={db_path}']
            if self.node.start_launch_file('rtabmap_loc', self.node.launch_files['rtabmap_loc'], extra_args):
                self.log(f"Started RTAB-MAP Localization with database: {db_path}")
                self.update_button_states()
        else:
            self.log("RTAB-MAP Localization launch file not found!")
            QMessageBox.warning(self, "Error", "RTAB-MAP Localization launch file not found!")

    def on_stop_rtabmap_loc(self):
        """Stop RTAB-MAP Localization mode"""
        if self.node.stop_launch_file('rtabmap_loc'):
            self.update_button_states()

    def on_start_kissicp(self):
        """Start KISS-ICP odometry"""
        if self.node.launch_files.get('kissicp'):
            extra_args = ['use_sim_time:=true']
            if self.node.start_launch_file('kissicp', self.node.launch_files['kissicp'], extra_args):
                self.log("Started KISS-ICP odometry")
                self.update_button_states()
        else:
            self.log("KISS-ICP launch file not found!")
            QMessageBox.warning(self, "Error", "KISS-ICP launch file not found!")

    def on_stop_kissicp(self):
        """Stop KISS-ICP odometry"""
        if self.node.stop_launch_file('kissicp'):
            self.update_button_states()

    def on_save_kissicp_map(self):
        """Save KISS-ICP map by calling save_map service"""
        try:
            # Open folder selection dialog
            default_path = str(Path.home() / "ros2_ws/map/kiss_icp_map")
            save_dir = QFileDialog.getExistingDirectory(
                self,
                "Select Directory to Save Map",
                default_path,
                QFileDialog.ShowDirsOnly
            )

            if not save_dir:
                self.log("Map save cancelled by user")
                return

            # Ask for map name
            from PyQt5.QtWidgets import QInputDialog
            map_name, ok = QInputDialog.getText(
                self,
                "Map Name",
                "Enter map name (without extension):",
                text="kiss_icp_map"
            )

            if not ok or not map_name:
                self.log("Map save cancelled by user")
                return

            # Full save path
            save_path = os.path.join(save_dir, f"{map_name}.pcd")

            self.log(f"Saving KISS-ICP map to: {save_path}")

            # Call KISS-ICP save service
            result = subprocess.run(
                ['ros2', 'service', 'call', '/kiss_icp/save_map',
                 'std_srvs/srv/Empty', '{}'],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                self.log(f"KISS-ICP map save triggered")
                # Note: KISS-ICP typically saves to a default location
                # You may need to copy from default location to save_path
                QMessageBox.information(self, "Info",
                    f"Map save triggered.\n\nCheck KISS-ICP output for the saved map location.")
            else:
                self.log(f"Failed to save map: {result.stderr}")
                QMessageBox.warning(self, "Error", f"Failed to save map:\n{result.stderr}")

        except Exception as e:
            self.log(f"Failed to save map: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to save map:\n{str(e)}")

    def on_start_slamtoolbox(self):
        """Start SLAM-Toolbox mapping mode"""
        if self.node.launch_files.get('slamtoolbox'):
            extra_args = ['use_sim_time:=true']
            if self.node.start_launch_file('slamtoolbox', self.node.launch_files['slamtoolbox'], extra_args):
                self.log("Started SLAM-Toolbox mapping")
                self.update_button_states()
        else:
            self.log("SLAM-Toolbox launch file not found!")
            QMessageBox.warning(self, "Error", "SLAM-Toolbox launch file not found!")

    def on_stop_slamtoolbox(self):
        """Stop SLAM-Toolbox mapping mode"""
        if self.node.stop_launch_file('slamtoolbox'):
            self.update_button_states()

    def on_save_slamtoolbox_map(self):
        """Save SLAM-Toolbox map using service call"""
        try:
            # First check if the service exists
            check_result = subprocess.run(
                ['ros2', 'service', 'list'],
                capture_output=True,
                text=True,
                timeout=10
            )

            if '/slam_toolbox/serialize_map' not in check_result.stdout:
                self.log("Error: /slam_toolbox/serialize_map service not found!")
                self.log("Make sure SLAM-Toolbox is running.")
                QMessageBox.warning(self, "Error", "serialize_map service not found!\n\nMake sure SLAM-Toolbox is running.")
                return

            # Create default map directory if it doesn't exist
            default_map_dir = Path.home() / "ros2_ws/map/slam_toolbox_map"
            default_map_dir.mkdir(parents=True, exist_ok=True)

            # Open folder selection dialog
            save_dir = QFileDialog.getExistingDirectory(
                self,
                "Select Directory to Save Map",
                str(default_map_dir),
                QFileDialog.ShowDirsOnly
            )

            if not save_dir:
                self.log("Map save cancelled by user")
                return

            # Ask for map name
            from PyQt5.QtWidgets import QInputDialog
            map_name, ok = QInputDialog.getText(
                self,
                "Map Name",
                "Enter map name (without extension):",
                text="slam_toolbox_map"
            )

            if not ok or not map_name:
                self.log("Map save cancelled by user")
                return

            # Full save path (without extension - SLAM Toolbox adds .posegraph and .data)
            save_path = os.path.join(save_dir, map_name)

            self.log(f"Saving SLAM-Toolbox map to: {save_path}")
            self.log("Calling serialize_map service...")

            # Call SLAM-Toolbox serialize_map service
            result = subprocess.run(
                ['ros2', 'service', 'call', '/slam_toolbox/serialize_map',
                 'slam_toolbox/srv/SerializePoseGraph',
                 f'{{"filename": "{save_path}"}}'],
                capture_output=True,
                text=True,
                timeout=60
            )

            self.log(f"Service call stdout: {result.stdout}")
            if result.stderr:
                self.log(f"Service call stderr: {result.stderr}")

            # Check result - RESULT_SUCCESS=0, RESULT_FAILED_TO_WRITE_FILE=255
            if result.returncode == 0 and 'result=0' in result.stdout:
                # Verify files were created
                posegraph_file = f"{save_path}.posegraph"
                data_file = f"{save_path}.data"

                if os.path.exists(posegraph_file) and os.path.exists(data_file):
                    posegraph_size = os.path.getsize(posegraph_file)
                    data_size = os.path.getsize(data_file)
                    self.log(f"SLAM-Toolbox map saved successfully!")
                    self.log(f"  - {posegraph_file} ({posegraph_size} bytes)")
                    self.log(f"  - {data_file} ({data_size} bytes)")
                    QMessageBox.information(self, "Success",
                        f"Map saved successfully!\n\nFiles:\n- {posegraph_file}\n- {data_file}")
                else:
                    self.log(f"Warning: Service returned success but files not found")
                    QMessageBox.warning(self, "Warning",
                        f"Service returned success but map files were not found.\n\nExpected:\n- {posegraph_file}\n- {data_file}")
            elif result.returncode == 0 and 'result=255' in result.stdout:
                self.log(f"Failed to save map: Could not write to file")
                QMessageBox.warning(self, "Error",
                    f"Failed to write map file.\n\nCheck that the directory exists and is writable:\n{save_dir}")
            else:
                self.log(f"Failed to save map: {result.stderr if result.stderr else result.stdout}")
                QMessageBox.warning(self, "Error", f"Failed to save map:\n{result.stderr if result.stderr else 'Unknown error'}")

        except subprocess.TimeoutExpired:
            self.log("Map save timed out - service call took too long")
            QMessageBox.warning(self, "Timeout", "Map save operation timed out.\n\nThe map might be too large or the service is not responding.")
        except Exception as e:
            self.log(f"Failed to save map: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to save map:\n{str(e)}")

    def on_browse_slamtoolbox_map(self):
        """Browse for SLAM-Toolbox map file"""
        default_path = str(Path.home() / "ros2_ws/map")
        map_file, _ = QFileDialog.getOpenFileName(
            self,
            "Select SLAM-Toolbox Map File",
            default_path,
            "PoseGraph Files (*.posegraph);;All Files (*)"
        )
        if map_file:
            # Remove .posegraph extension for SLAM-Toolbox
            if map_file.endswith('.posegraph'):
                map_file = map_file[:-10]  # Remove .posegraph
            self.txtSlamToolboxMapPath.setText(map_file)
            self.node.slamtoolbox_map_path = map_file
            self.log(f"Selected SLAM-Toolbox map: {map_file}")

    def on_start_slamtoolbox_loc(self):
        """Start SLAM-Toolbox localization mode"""
        map_file = self.txtSlamToolboxMapPath.text()
        if not map_file:
            self.log("Please select a map file first!")
            QMessageBox.warning(self, "Error", "Please select a map file first!")
            return

        # Check if map files exist
        if not os.path.exists(f"{map_file}.posegraph"):
            self.log(f"Map file not found: {map_file}.posegraph")
            QMessageBox.warning(self, "Error", f"Map file not found:\n{map_file}.posegraph")
            return

        if self.node.launch_files.get('slamtoolbox_loc'):
            extra_args = ['use_sim_time:=true', f'map_file:={map_file}']
            if self.node.start_launch_file('slamtoolbox_loc', self.node.launch_files['slamtoolbox_loc'], extra_args):
                self.log(f"Started SLAM-Toolbox Localization with map: {map_file}")
                self.update_button_states()
        else:
            self.log("SLAM-Toolbox Localization launch file not found!")
            QMessageBox.warning(self, "Error", "SLAM-Toolbox Localization launch file not found!")

    def on_stop_slamtoolbox_loc(self):
        """Stop SLAM-Toolbox localization mode"""
        if self.node.stop_launch_file('slamtoolbox_loc'):
            self.update_button_states()

    def on_start_custom(self):
        custom_path = self.txtLaunchFile.text()
        if custom_path:
            self.node.launch_files['custom'] = custom_path
            if self.node.start_launch_file('custom', custom_path):
                self.update_button_states()
        else:
            self.log("Please specify a launch file!")
            QMessageBox.warning(self, "Error", "Please specify a launch file path!")

    def on_stop_custom(self):
        if self.node.stop_launch_file('custom'):
            self.update_button_states()

    def on_browse(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Launch File",
            str(Path.home() / "ros2_ws/src"),
            "Launch Files (*.py *.launch.py);;All Files (*)"
        )
        if file_path:
            self.txtLaunchFile.setText(file_path)

    def on_stop_all(self):
        reply = QMessageBox.question(
            self,
            "Confirm",
            "Stop all running launch files?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.node.stop_all_launches()
            self.update_button_states()

    def update_sensor_status(self):
        """Update sensor status labels"""
        if self.node is None:
            return

        # LiDAR status
        lidar_active = self.node.get_sensor_status('lidar')
        if lidar_active:
            self.lblLidarStatus.setText("LiDAR: OK")
            self.lblLidarStatus.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            self.lblLidarStatus.setText("LiDAR: --")
            self.lblLidarStatus.setStyleSheet("color: #666666;")

        # IMU status
        imu_active = self.node.get_sensor_status('imu')
        if imu_active:
            self.lblImuStatus.setText("IMU: OK")
            self.lblImuStatus.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            self.lblImuStatus.setText("IMU: --")
            self.lblImuStatus.setStyleSheet("color: #666666;")

        # Camera status
        camera_active = self.node.get_sensor_status('camera')
        if camera_active:
            self.lblCameraStatus.setText("Camera: OK")
            self.lblCameraStatus.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            self.lblCameraStatus.setText("Camera: --")
            self.lblCameraStatus.setStyleSheet("color: #666666;")

        # GPS status
        gps_active = self.node.get_sensor_status('gps')
        if gps_active:
            self.lblGpsStatus.setText("GPS: OK")
            self.lblGpsStatus.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            self.lblGpsStatus.setText("GPS: --")
            self.lblGpsStatus.setStyleSheet("color: #666666;")

    def update_button_states(self):
        """Update button enabled/disabled states based on running processes"""
        if self.node is None:
            return

        # DSS Bridge
        dss_running = self.node.is_running('dss')
        self.btnStartDSS.setEnabled(not dss_running)
        self.btnStopDSS.setEnabled(dss_running)
        if dss_running:
            self.btnStartDSS.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }")
        else:
            self.btnStartDSS.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 10px; } QPushButton:disabled { background-color: #cccccc; color: #666666; }")

        # Livox MID-360
        livox_running = self.node.is_running('livox')
        self.btnStartLivox.setEnabled(not livox_running)
        self.btnStopLivox.setEnabled(livox_running)
        if livox_running:
            self.btnStartLivox.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }")
        else:
            self.btnStartLivox.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 10px; } QPushButton:disabled { background-color: #cccccc; color: #666666; }")

        # LIO-SAM SLAM
        liosam_running = self.node.is_running('liosam')
        liosam_loc_running = self.node.is_running('liosam_loc')
        self.btnStartLioSam.setEnabled(livox_running and not liosam_running and not liosam_loc_running)
        self.btnStopLioSam.setEnabled(liosam_running)
        self.btnSaveLioSamMap.setEnabled(liosam_running)
        if liosam_running:
            self.btnStartLioSam.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }")
        else:
            self.btnStartLioSam.setStyleSheet("QPushButton { background-color: #9C27B0; color: white; font-weight: bold; padding: 10px; } QPushButton:disabled { background-color: #cccccc; color: #666666; }")

        # LIO-SAM Localization
        self.btnStartLioSamLoc.setEnabled(livox_running and not liosam_loc_running and not liosam_running)
        self.btnStopLioSamLoc.setEnabled(liosam_loc_running)
        if liosam_loc_running:
            self.btnStartLioSamLoc.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }")
        else:
            self.btnStartLioSamLoc.setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; padding: 10px; } QPushButton:disabled { background-color: #cccccc; color: #666666; }")

        # DSS LIO-SAM (for simulation - requires DSS Bridge)
        dss_lio_sam_running = self.node.is_running('dss_lio_sam')
        dss_lio_sam_loc_running = self.node.is_running('dss_lio_sam_loc')
        self.btnStartDssLioSam.setEnabled(dss_running and not dss_lio_sam_running and not dss_lio_sam_loc_running)
        self.btnStopDssLioSam.setEnabled(dss_lio_sam_running)
        self.btnSaveDssLioSamMap.setEnabled(dss_lio_sam_running)
        if dss_lio_sam_running:
            self.btnStartDssLioSam.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }")
        else:
            self.btnStartDssLioSam.setStyleSheet("QPushButton { background-color: #FF5722; color: white; font-weight: bold; padding: 10px; } QPushButton:disabled { background-color: #cccccc; color: #666666; }")

        # DSS LIO-SAM Localization
        self.btnStartDssLioSamLoc.setEnabled(dss_running and not dss_lio_sam_loc_running and not dss_lio_sam_running)
        self.btnStopDssLioSamLoc.setEnabled(dss_lio_sam_loc_running)
        if dss_lio_sam_loc_running:
            self.btnStartDssLioSamLoc.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }")
        else:
            self.btnStartDssLioSamLoc.setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; padding: 10px; } QPushButton:disabled { background-color: #cccccc; color: #666666; }")

        # RTAB-MAP SLAM (for simulation - requires DSS Bridge)
        rtabmap_running = self.node.is_running('rtabmap')
        rtabmap_loc_running = self.node.is_running('rtabmap_loc')
        self.btnStartRtabmap.setEnabled(dss_running and not rtabmap_running and not rtabmap_loc_running)
        self.btnStopRtabmap.setEnabled(rtabmap_running)
        self.btnSaveRtabmapMap.setEnabled(rtabmap_running)
        if rtabmap_running:
            self.btnStartRtabmap.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }")
        else:
            self.btnStartRtabmap.setStyleSheet("QPushButton { background-color: #00BCD4; color: white; font-weight: bold; padding: 10px; } QPushButton:disabled { background-color: #cccccc; color: #666666; }")

        # RTAB-MAP Localization
        self.btnStartRtabmapLoc.setEnabled(dss_running and not rtabmap_loc_running and not rtabmap_running)
        self.btnStopRtabmapLoc.setEnabled(rtabmap_loc_running)
        if rtabmap_loc_running:
            self.btnStartRtabmapLoc.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }")
        else:
            self.btnStartRtabmapLoc.setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; padding: 10px; } QPushButton:disabled { background-color: #cccccc; color: #666666; }")

        # KISS-ICP (requires DSS Bridge for simulation)
        kissicp_running = self.node.is_running('kissicp')
        self.btnStartKissIcp.setEnabled(dss_running and not kissicp_running)
        self.btnStopKissIcp.setEnabled(kissicp_running)
        self.btnSaveKissIcpMap.setEnabled(kissicp_running)
        if kissicp_running:
            self.btnStartKissIcp.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }")
        else:
            self.btnStartKissIcp.setStyleSheet("QPushButton { background-color: #E91E63; color: white; font-weight: bold; padding: 10px; } QPushButton:disabled { background-color: #cccccc; color: #666666; }")

        # SLAM-Toolbox (requires DSS Bridge for simulation)
        slamtoolbox_running = self.node.is_running('slamtoolbox')
        slamtoolbox_loc_running = self.node.is_running('slamtoolbox_loc')
        self.btnStartSlamToolbox.setEnabled(dss_running and not slamtoolbox_running and not slamtoolbox_loc_running)
        self.btnStopSlamToolbox.setEnabled(slamtoolbox_running)
        self.btnSaveSlamToolboxMap.setEnabled(slamtoolbox_running)
        if slamtoolbox_running:
            self.btnStartSlamToolbox.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }")
        else:
            self.btnStartSlamToolbox.setStyleSheet("QPushButton { background-color: #673AB7; color: white; font-weight: bold; padding: 10px; } QPushButton:disabled { background-color: #cccccc; color: #666666; }")

        # SLAM-Toolbox Localization
        self.btnStartSlamToolboxLoc.setEnabled(dss_running and not slamtoolbox_loc_running and not slamtoolbox_running)
        self.btnStopSlamToolboxLoc.setEnabled(slamtoolbox_loc_running)
        if slamtoolbox_loc_running:
            self.btnStartSlamToolboxLoc.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }")
        else:
            self.btnStartSlamToolboxLoc.setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; padding: 10px; } QPushButton:disabled { background-color: #cccccc; color: #666666; }")

        # Custom
        custom_running = self.node.is_running('custom')
        self.btnStartCustom.setEnabled(not custom_running)
        self.btnStopCustom.setEnabled(custom_running)

    def closeEvent(self, event):
        """Handle window close event"""
        reply = QMessageBox.question(
            self,
            "Confirm Exit",
            "Stop all launches and exit?",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            if self.node:
                self.node.stop_all_launches()
            event.accept()
        else:
            event.ignore()


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create Qt Application
    app = QtWidgets.QApplication(sys.argv)

    # Create UI
    ui = SlamLaunchManagerUI()

    # Create ROS2 Node
    node = SlamLaunchManagerNode(ui)
    ui.set_node(node)

    # Show UI
    ui.show()

    # Timer for ROS2 spinning
    ros_timer = QTimer()

    def spin_ros():
        if rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0)
            except Exception:
                pass

    ros_timer.timeout.connect(spin_ros)
    ros_timer.start(10)  # Spin every 10ms

    # Run Qt event loop
    exit_code = app.exec_()

    # Cleanup - stop timer first before shutting down rclpy
    ros_timer.stop()
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
