#!/usr/bin/env python3

import sys
import os
import subprocess
import signal
from pathlib import Path

import rclpy
from rclpy.node import Node

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer, QDateTime
from PyQt5.QtWidgets import QFileDialog, QMessageBox


class LaunchManagerNode(Node):
    def __init__(self, ui_window):
        super().__init__('launch_manager_node')
        self.ui = ui_window

        # Dictionary to store running processes
        self.processes = {
            'dss': None,
            'rtabmap': None,
            'liosam': None,
            'kissicp': None,
            'slamtoolbox': None,
            'slamtoolbox_loc': None,
            'localization': None,
            'custom': None
        }

        # Store launch file paths
        self.launch_files = {
            'dss': None,  # Will be set from config or UI
            'rtabmap': None,  # Will be set from config or UI
            'liosam': None,  # Will be set from config or UI
            'kissicp': None,  # Will be set from config or UI
            'slamtoolbox': None,  # Will be set from config or UI
            'slamtoolbox_loc': None,  # Will be set from config or UI
            'localization': None,  # Will be set from config or UI
            'custom': None
        }

        # Store map database path
        self.map_database_path = None
        self.slamtoolbox_map_path = None

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


class LaunchManagerUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # Load UI file
        ui_file = Path(__file__).parent / 'ui' / 'launch_manager.ui'
        uic.loadUi(ui_file, self)

        # ROS2 node (will be initialized later)
        self.node = None

        # Connect buttons
        self.btnStartDSS.clicked.connect(self.on_start_dss)
        self.btnStopDSS.clicked.connect(self.on_stop_dss)

        self.btnStartRtabmap.clicked.connect(self.on_start_rtabmap)
        self.btnStopRtabmap.clicked.connect(self.on_stop_rtabmap)
        self.btnSaveMap.clicked.connect(self.on_save_map)

        self.btnStartLioSam.clicked.connect(self.on_start_liosam)
        self.btnStopLioSam.clicked.connect(self.on_stop_liosam)

        self.btnStartKissIcp.clicked.connect(self.on_start_kissicp)
        self.btnStopKissIcp.clicked.connect(self.on_stop_kissicp)

        self.btnStartSlamToolbox.clicked.connect(self.on_start_slamtoolbox)
        self.btnStopSlamToolbox.clicked.connect(self.on_stop_slamtoolbox)
        self.btnSaveSlamToolboxMap.clicked.connect(self.on_save_slamtoolbox_map)
        self.btnLoadSlamToolboxMap.clicked.connect(self.on_load_slamtoolbox_map)
        self.btnStartSlamToolboxLoc.clicked.connect(self.on_start_slamtoolbox_loc)
        self.btnStopSlamToolboxLoc.clicked.connect(self.on_stop_slamtoolbox_loc)

        self.btnLoadMap.clicked.connect(self.on_load_map)
        self.btnStartLocalization.clicked.connect(self.on_start_localization)
        self.btnStopLocalization.clicked.connect(self.on_stop_localization)

        self.btnStartCustom.clicked.connect(self.on_start_custom)
        self.btnStopCustom.clicked.connect(self.on_stop_custom)
        self.btnBrowse.clicked.connect(self.on_browse)

        self.btnStopAll.clicked.connect(self.on_stop_all)

        # Timer to check process status
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_button_states)
        self.status_timer.start(500)  # Check every 500ms

        self.log("Launch Manager UI Ready")

    def set_node(self, node):
        """Set the ROS2 node"""
        self.node = node

        # Try to auto-detect launch files
        self.auto_detect_launch_files()

    def auto_detect_launch_files(self):
        """Auto-detect launch files in the workspace"""
        workspace = Path('/home/amap/ros2_ws/src')

        # Look for DSS ROS2 Bridge launch file
        dss_launch = workspace / 'dss_ros2_bridge' / 'launch' / 'launch.py'
        if dss_launch.exists():
            self.node.launch_files['dss'] = str(dss_launch)
            self.log(f"Found DSS Bridge launch: {dss_launch}")

        # Look for RTAB-Map SLAM launch file
        rtabmap_launch = workspace / 'SLAM' / 'dss_rtabmap_slam' / 'launch' / 'rtabmap_with_rviz.launch.py'
        if rtabmap_launch.exists():
            self.node.launch_files['rtabmap'] = str(rtabmap_launch)
            self.log(f"Found RTAB-Map launch: {rtabmap_launch}")

        # Look for LIO-SAM launch file
        liosam_launch = workspace / 'SLAM' / 'LIO-SAM' / 'dss_lio_sam' / 'launch' / 'run.launch.py'
        if liosam_launch.exists():
            self.node.launch_files['liosam'] = str(liosam_launch)
            self.log(f"Found LIO-SAM launch: {liosam_launch}")

        # Look for KISS-ICP launch file
        kissicp_launch = workspace / 'SLAM' / 'KISS-ICP' / 'dss_kiss_icp' / 'launch' / 'run.launch.py'
        if kissicp_launch.exists():
            self.node.launch_files['kissicp'] = str(kissicp_launch)
            self.log(f"Found KISS-ICP launch: {kissicp_launch}")

        # Look for SLAM Toolbox launch files
        slamtoolbox_launch = workspace / 'SLAM' / 'SLAM-Toolbox' / 'dss_slam_toolbox' / 'launch' / 'slam_mapping.launch.py'
        if slamtoolbox_launch.exists():
            self.node.launch_files['slamtoolbox'] = str(slamtoolbox_launch)
            self.log(f"Found SLAM Toolbox launch: {slamtoolbox_launch}")

        slamtoolbox_loc_launch = workspace / 'SLAM' / 'SLAM-Toolbox' / 'dss_slam_toolbox' / 'launch' / 'slam_localization.launch.py'
        if slamtoolbox_loc_launch.exists():
            self.node.launch_files['slamtoolbox_loc'] = str(slamtoolbox_loc_launch)
            self.log(f"Found SLAM Toolbox Localization launch: {slamtoolbox_loc_launch}")

        # Look for RTAB-Map Localization launch file
        localization_launch = workspace / 'SLAM' / 'dss_rtabmap_localization' / 'launch' / 'rtabmap_localization.launch.py'
        if localization_launch.exists():
            self.node.launch_files['localization'] = str(localization_launch)
            self.log(f"Found Localization launch: {localization_launch}")

    def log(self, message):
        """Add message to log"""
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.txtLog.append(f"[{timestamp}] {message}")

    def on_start_dss(self):
        if self.node.launch_files['dss']:
            if self.node.start_launch_file('dss', self.node.launch_files['dss']):
                self.update_button_states()
        else:
            self.log("DSS launch file not configured!")
            QMessageBox.warning(self, "Error", "DSS launch file not found!")

    def on_stop_dss(self):
        if self.node.stop_launch_file('dss'):
            self.update_button_states()

    def on_start_rtabmap(self):
        if self.node.launch_files['rtabmap']:
            if self.node.start_launch_file('rtabmap', self.node.launch_files['rtabmap']):
                self.update_button_states()
        else:
            self.log("RTAB-Map launch file not configured!")
            QMessageBox.warning(self, "Error", "RTAB-Map launch file not found! Please create one first.")

    def on_stop_rtabmap(self):
        if self.node.stop_launch_file('rtabmap'):
            self.update_button_states()

    def on_start_liosam(self):
        if self.node.launch_files['liosam']:
            if self.node.start_launch_file('liosam', self.node.launch_files['liosam']):
                self.update_button_states()
        else:
            self.log("LIO-SAM launch file not configured!")
            QMessageBox.warning(self, "Error", "LIO-SAM launch file not found!")

    def on_stop_liosam(self):
        if self.node.stop_launch_file('liosam'):
            self.update_button_states()

    def on_start_kissicp(self):
        if self.node.launch_files['kissicp']:
            # Get parameters from UI
            rviz_enabled = 'true' if self.chkRviz.isChecked() else 'false'
            extra_args = [f'rviz:={rviz_enabled}']
            if self.node.start_launch_file('kissicp', self.node.launch_files['kissicp'], extra_args):
                self.update_button_states()
        else:
            self.log("KISS-ICP launch file not configured!")
            QMessageBox.warning(self, "Error", "KISS-ICP launch file not found!")

    def on_stop_kissicp(self):
        if self.node.stop_launch_file('kissicp'):
            self.update_button_states()

    def on_start_slamtoolbox(self):
        if self.node.launch_files['slamtoolbox']:
            if self.node.start_launch_file('slamtoolbox', self.node.launch_files['slamtoolbox']):
                self.update_button_states()
        else:
            self.log("SLAM Toolbox launch file not configured!")
            QMessageBox.warning(self, "Error", "SLAM Toolbox launch file not found!")

    def on_stop_slamtoolbox(self):
        if self.node.stop_launch_file('slamtoolbox'):
            self.update_button_states()

    def on_save_slamtoolbox_map(self):
        """Save SLAM Toolbox map using service call"""
        maps_dir = Path('/home/amap/ros2_ws/src/SLAM/SLAM-Toolbox/dss_slam_toolbox/maps')
        maps_dir.mkdir(parents=True, exist_ok=True)
        default_path = str(maps_dir / 'slam_toolbox_map')

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save SLAM Toolbox Map",
            default_path,
            "All Files (*)"
        )

        if not file_path:
            self.log("Map save cancelled")
            return

        try:
            # Call SLAM Toolbox serialize_map service
            import subprocess
            result = subprocess.run(
                ['ros2', 'service', 'call', '/slam_toolbox/serialize_map',
                 'slam_toolbox/srv/SerializePoseGraph',
                 f'{{"filename": "{file_path}"}}'],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                self.log(f"Map saved to: {file_path}.posegraph and {file_path}.data")
                QMessageBox.information(self, "Success", f"Map saved successfully!\n\nFiles:\n{file_path}.posegraph\n{file_path}.data")
            else:
                self.log(f"Failed to save map: {result.stderr}")
                QMessageBox.warning(self, "Error", f"Failed to save map:\n{result.stderr}")

        except Exception as e:
            self.log(f"Failed to save map: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to save map:\n{str(e)}")

    def on_load_slamtoolbox_map(self):
        """Load a saved SLAM Toolbox map for localization"""
        maps_dir = Path('/home/amap/ros2_ws/src/SLAM/SLAM-Toolbox/dss_slam_toolbox/maps')
        maps_dir.mkdir(parents=True, exist_ok=True)

        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select SLAM Toolbox Map",
            str(maps_dir),
            "Posegraph Files (*.posegraph);;All Files (*)"
        )

        if not file_path:
            self.log("Map load cancelled")
            return

        # Remove .posegraph extension for SLAM Toolbox
        if file_path.endswith('.posegraph'):
            file_path = file_path[:-10]

        if not Path(file_path + '.posegraph').exists():
            QMessageBox.warning(self, "Error", f"Map file not found:\n{file_path}.posegraph")
            return

        self.node.slamtoolbox_map_path = file_path
        self.log(f"SLAM Toolbox map loaded: {file_path}")
        self.btnStartSlamToolboxLoc.setEnabled(True)

        QMessageBox.information(self, "Map Loaded", f"Map loaded!\n\n{file_path}\n\nYou can now start localization.")

    def on_start_slamtoolbox_loc(self):
        if not self.node.slamtoolbox_map_path:
            QMessageBox.warning(self, "No Map", "Please load a map first!")
            return

        if self.node.launch_files['slamtoolbox_loc']:
            extra_args = [f'map_file:={self.node.slamtoolbox_map_path}']
            if self.node.start_launch_file('slamtoolbox_loc', self.node.launch_files['slamtoolbox_loc'], extra_args):
                self.update_button_states()
        else:
            self.log("SLAM Toolbox Localization launch file not configured!")
            QMessageBox.warning(self, "Error", "SLAM Toolbox Localization launch file not found!")

    def on_stop_slamtoolbox_loc(self):
        if self.node.stop_launch_file('slamtoolbox_loc'):
            self.update_button_states()

    def on_save_map(self):
        """Save RTAB-Map database"""
        # Ask user for save location
        maps_dir = Path('/home/amap/ros2_ws/src/SLAM/dss_rtabmap_localization/maps')
        maps_dir.mkdir(parents=True, exist_ok=True)
        default_path = str(maps_dir / 'rtabmap_map.db')
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save RTAB-Map Database",
            default_path,
            "Database Files (*.db);;All Files (*)"
        )

        if not file_path:
            self.log("Map save cancelled")
            return

        # Ensure .db extension
        if not file_path.endswith('.db'):
            file_path += '.db'

        # Create directory if it doesn't exist
        save_dir = Path(file_path).parent
        save_dir.mkdir(parents=True, exist_ok=True)

        try:
            # Copy the current rtabmap database
            source_db = Path.home() / '.ros' / 'rtabmap.db'

            if not source_db.exists():
                QMessageBox.warning(
                    self,
                    "Error",
                    "No RTAB-Map database found!\nPlease run SLAM first to create a map."
                )
                self.log("Error: No rtabmap.db found")
                return

            # Copy database
            import shutil
            shutil.copy2(source_db, file_path)

            self.log(f"Map saved successfully to: {file_path}")
            self.log(f"Database size: {Path(file_path).stat().st_size / (1024*1024):.2f} MB")

            QMessageBox.information(
                self,
                "Success",
                f"Map saved successfully!\n\nLocation: {file_path}\n\nYou can load this map for localization using 'Load Map' button."
            )

        except Exception as e:
            self.log(f"Failed to save map: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to save map:\n{str(e)}")

    def on_load_map(self):
        """Load a saved map for localization"""
        # Ask user to select a map file
        maps_dir = Path('/home/amap/ros2_ws/src/SLAM/dss_rtabmap_localization/maps')
        maps_dir.mkdir(parents=True, exist_ok=True)  # Ensure directory exists
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select RTAB-Map Database",
            str(maps_dir),
            "Database Files (*.db);;All Files (*)"
        )

        if not file_path:
            self.log("Map load cancelled")
            return

        if not Path(file_path).exists():
            QMessageBox.warning(self, "Error", f"File not found:\n{file_path}")
            self.log(f"Error: Map file not found: {file_path}")
            return

        # Store the path for localization
        self.node.map_database_path = file_path

        self.log(f"Map loaded: {file_path}")
        self.log(f"Database size: {Path(file_path).stat().st_size / (1024*1024):.2f} MB")

        # Enable localization button
        self.btnStartLocalization.setEnabled(True)

        QMessageBox.information(
            self,
            "Map Loaded",
            f"Map loaded successfully!\n\nLocation: {file_path}\n\nYou can now start localization."
        )

    def on_start_localization(self):
        if not self.node.map_database_path:
            QMessageBox.warning(
                self,
                "No Map Loaded",
                "Please load a map first using 'Load Map' button."
            )
            self.log("Error: No map loaded. Please load a map first.")
            return

        if self.node.launch_files['localization']:
            # Pass database_path as launch argument
            extra_args = [f'database_path:={self.node.map_database_path}']
            if self.node.start_launch_file('localization', self.node.launch_files['localization'], extra_args):
                self.update_button_states()
                self.log(f"Starting localization with map: {self.node.map_database_path}")
        else:
            self.log("Localization launch file not configured!")
            QMessageBox.warning(self, "Error", "Localization launch file not found!")

    def on_stop_localization(self):
        if self.node.stop_launch_file('localization'):
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
            "/home/amap/ros2_ws/src",
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

    def update_button_states(self):
        """Update button enabled/disabled states based on running processes"""
        if self.node is None:
            return

        # DSS Controller
        dss_running = self.node.is_running('dss')
        self.btnStartDSS.setEnabled(not dss_running)
        self.btnStopDSS.setEnabled(dss_running)

        # RTAB-Map
        rtabmap_running = self.node.is_running('rtabmap')
        self.btnStartRtabmap.setEnabled(not rtabmap_running)
        self.btnStopRtabmap.setEnabled(rtabmap_running)
        self.btnSaveMap.setEnabled(rtabmap_running)  # Enable Save Map only when RTAB-Map is running

        # LIO-SAM (disabled due to IMU bug)
        liosam_running = self.node.is_running('liosam')
        self.btnStartLioSam.setEnabled(False)  # Disabled due to DSS IMU bug
        self.btnStopLioSam.setEnabled(liosam_running)

        # KISS-ICP
        kissicp_running = self.node.is_running('kissicp')
        self.btnStartKissIcp.setEnabled(not kissicp_running)
        self.btnStopKissIcp.setEnabled(kissicp_running)

        # SLAM Toolbox
        slamtoolbox_running = self.node.is_running('slamtoolbox')
        slamtoolbox_loc_running = self.node.is_running('slamtoolbox_loc')
        self.btnStartSlamToolbox.setEnabled(not slamtoolbox_running and not slamtoolbox_loc_running)
        self.btnStopSlamToolbox.setEnabled(slamtoolbox_running)
        self.btnSaveSlamToolboxMap.setEnabled(slamtoolbox_running)
        self.btnLoadSlamToolboxMap.setEnabled(not slamtoolbox_running and not slamtoolbox_loc_running)
        has_slamtoolbox_map = self.node.slamtoolbox_map_path is not None
        self.btnStartSlamToolboxLoc.setEnabled(has_slamtoolbox_map and not slamtoolbox_loc_running and not slamtoolbox_running)
        self.btnStopSlamToolboxLoc.setEnabled(slamtoolbox_loc_running)

        # Localization
        localization_running = self.node.is_running('localization')
        # Load Map button is always enabled (to allow loading map anytime)
        self.btnLoadMap.setEnabled(True)
        # Enable Start Localization only if we have a saved map, it's not running, and RTAB-Map SLAM is not running
        has_map = self.node.map_database_path is not None
        self.btnStartLocalization.setEnabled(has_map and not localization_running and not rtabmap_running)
        self.btnStopLocalization.setEnabled(localization_running)

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
    ui = LaunchManagerUI()

    # Create ROS2 Node
    node = LaunchManagerNode(ui)
    ui.set_node(node)

    # Show UI
    ui.show()

    # Timer for ROS2 spinning
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_timer.start(10)  # Spin every 10ms

    # Run Qt event loop
    exit_code = app.exec_()

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
