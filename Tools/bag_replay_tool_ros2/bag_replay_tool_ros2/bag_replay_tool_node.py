#!/usr/bin/env python3
"""
Bag Replay Tool - GUI for playing ROS2 bag files
Converted from ROS1 to ROS2
"""

import sys
import os
import subprocess
import signal
import yaml
import json
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from ament_index_python.packages import get_package_share_directory
import sqlite3

from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtWidgets import QFileDialog, QMessageBox, QListWidgetItem, QAction


class BagReplayTool(QtWidgets.QMainWindow):
    """GUI tool for replaying ROS2 bag files with playback control"""

    def __init__(self, node):
        super(BagReplayTool, self).__init__()

        self.node = node
        self.node.get_logger().info("🎬 Bag Replay Tool starting...")

        # Load UI file - Try package share directory first
        try:
            pkg_share = get_package_share_directory('bag_replay_tool_ros2')
            ui_file_path = os.path.join(pkg_share, 'ui', 'bag_replay_tool.ui')
        except:
            # Fallback to relative path for development
            pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            ui_file_path = os.path.join(pkg_dir, 'ui', 'bag_replay_tool.ui')

        if not os.path.exists(ui_file_path):
            self.node.get_logger().error(f"❌ UI file not found: {ui_file_path}")
            sys.exit(1)

        uic.loadUi(ui_file_path, self)
        self.node.get_logger().info(f"✅ UI loaded from: {ui_file_path}")

        # Rosbag playback thread
        self.playback_thread = None
        self.stop_playback_flag = threading.Event()
        self.pause_playback_flag = threading.Event()
        self.current_bag_file = None
        self.bag_topics = {}  # topic_name -> topic_type
        self.publishers = {}  # topic_name -> publisher
        self.is_paused = False  # Track pause state

        # Recent files management
        self.max_recent_files = 10
        self.recent_files = []
        self.recent_files_config_path = os.path.expanduser('~/.ros2/bag_replay_tool_recent.json')
        self.load_recent_files()

        # Connect UI signals
        self.connect_signals()

        # Setup menu actions after UI is loaded
        self.setup_menu_actions()

        # Set default values
        self.set_default_values()

        # Set window properties
        self.setWindowTitle("ROS2 Bag Replay Tool")
        self.show()

        self.node.get_logger().info("✅ Bag Replay Tool initialized")

    def set_default_values(self):
        """Set default values for UI controls"""
        # Set loop checkbox to checked by default
        if hasattr(self, 'checkbox_loop'):
            self.checkbox_loop.setChecked(True)
            self.node.get_logger().info("✅ Loop checkbox set to checked by default")

        # Set default playback rate to 1.0
        if hasattr(self, 'spinbox_rate'):
            self.spinbox_rate.setValue(1.0)
            self.node.get_logger().info("✅ Playback rate set to 1.0")

        # Set clock checkbox to unchecked by default
        if hasattr(self, 'checkbox_clock'):
            self.checkbox_clock.setChecked(False)
            self.node.get_logger().info("✅ Clock checkbox set to unchecked")

    def setup_menu_actions(self):
        """Setup menu bar actions"""
        # Connect File menu actions
        if hasattr(self, 'action_open'):
            self.action_open.triggered.connect(self.on_browse_clicked)
            self.node.get_logger().info("✅ Open action connected")

        if hasattr(self, 'action_clear_recent'):
            self.action_clear_recent.triggered.connect(self.clear_recent_files)
            self.node.get_logger().info("✅ Clear recent action connected")

        # Update recent files menu
        self.update_recent_files_menu()

    def load_recent_files(self):
        """Load recent files list from config file"""
        try:
            if os.path.exists(self.recent_files_config_path):
                with open(self.recent_files_config_path, 'r') as f:
                    self.recent_files = json.load(f)
                    # Filter out files that no longer exist
                    self.recent_files = [f for f in self.recent_files if os.path.exists(f)]
                    self.node.get_logger().info(f"✅ Loaded {len(self.recent_files)} recent files")
            else:
                self.recent_files = []
                self.node.get_logger().info("📋 No recent files config found, starting fresh")
        except Exception as e:
            self.node.get_logger().error(f"❌ Error loading recent files: {e}")
            self.recent_files = []

    def save_recent_files(self):
        """Save recent files list to config file"""
        try:
            # Ensure .ros2 directory exists
            ros_dir = os.path.dirname(self.recent_files_config_path)
            if not os.path.exists(ros_dir):
                os.makedirs(ros_dir)

            with open(self.recent_files_config_path, 'w') as f:
                json.dump(self.recent_files, f, indent=2)
                self.node.get_logger().info(f"💾 Saved {len(self.recent_files)} recent files")
        except Exception as e:
            self.node.get_logger().error(f"❌ Error saving recent files: {e}")

    def add_recent_file(self, file_path):
        """Add a file to the recent files list"""
        # Remove if already exists (to move to top)
        if file_path in self.recent_files:
            self.recent_files.remove(file_path)

        # Add to beginning
        self.recent_files.insert(0, file_path)

        # Keep only max_recent_files
        self.recent_files = self.recent_files[:self.max_recent_files]

        # Save to disk
        self.save_recent_files()

        # Update menu
        self.update_recent_files_menu()

    def update_recent_files_menu(self):
        """Update the recent files submenu"""
        try:
            if not hasattr(self, 'menu_file'):
                return

            # Find or create "Recent Files" submenu
            # First, remove any existing recent files actions
            for action in self.menu_file.actions():
                if hasattr(action, 'recent_file_path'):
                    self.menu_file.removeAction(action)

            # Remove existing recent menu if it exists
            if hasattr(self, 'recent_files_menu'):
                self.menu_file.removeAction(self.recent_files_menu.menuAction())

            # Create new recent files submenu
            if self.recent_files:
                self.recent_files_menu = QtWidgets.QMenu("Recent Files", self)

                for file_path in self.recent_files:
                    # Create action with just filename, but store full path
                    filename = os.path.basename(file_path)
                    action = QAction(filename, self)
                    action.setToolTip(file_path)  # Show full path in tooltip
                    action.recent_file_path = file_path
                    action.triggered.connect(lambda checked, path=file_path: self.open_recent_file(path))
                    self.recent_files_menu.addAction(action)

                # Insert submenu after "Open" action
                actions = self.menu_file.actions()
                if len(actions) > 1:
                    self.menu_file.insertMenu(actions[1], self.recent_files_menu)
                else:
                    self.menu_file.addMenu(self.recent_files_menu)

                self.node.get_logger().info(f"📋 Updated recent files menu with {len(self.recent_files)} items")

        except Exception as e:
            self.node.get_logger().error(f"❌ Error updating recent files menu: {e}")
            import traceback
            traceback.print_exc()

    def open_recent_file(self, file_path):
        """Open a file from the recent files list"""
        self.node.get_logger().info(f"📂 Opening recent file: {file_path}")

        if not os.path.exists(file_path):
            QMessageBox.warning(self, "File Not Found",
                              f"The file no longer exists:\n{file_path}")
            # Remove from recent files
            self.recent_files.remove(file_path)
            self.save_recent_files()
            self.update_recent_files_menu()
            return

        self.current_bag_file = file_path

        # Update UI
        if hasattr(self, 'line_edit_file_path'):
            self.line_edit_file_path.setText(file_path)

        # Get bag info and topics
        self.display_bag_info(file_path)
        self.load_bag_topics(file_path)

        # Move to top of recent files
        self.add_recent_file(file_path)

    def clear_recent_files(self):
        """Clear the recent files list"""
        reply = QMessageBox.question(
            self,
            'Clear Recent Files',
            'Are you sure you want to clear the recent files list?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.recent_files = []
            self.save_recent_files()
            self.update_recent_files_menu()
            self.node.get_logger().info("🗑️ Cleared recent files list")

    def connect_signals(self):
        """Connect UI signals to slots"""
        # File selection
        if hasattr(self, 'btn_browse'):
            self.btn_browse.clicked.connect(self.on_browse_clicked)
            self.node.get_logger().info("✅ Browse button connected")

        # Playback controls
        if hasattr(self, 'btn_play'):
            self.btn_play.clicked.connect(self.on_play_clicked)
            self.node.get_logger().info("✅ Play button connected")

        if hasattr(self, 'btn_pause'):
            self.btn_pause.clicked.connect(self.on_pause_clicked)
            self.node.get_logger().info("✅ Pause button connected")

        if hasattr(self, 'btn_stop'):
            self.btn_stop.clicked.connect(self.on_stop_clicked)
            self.node.get_logger().info("✅ Stop button connected")

        # Playback options
        if hasattr(self, 'checkbox_loop'):
            self.node.get_logger().info("✅ Loop checkbox available")

        if hasattr(self, 'spinbox_rate'):
            self.node.get_logger().info("✅ Rate spinbox available")

        if hasattr(self, 'checkbox_clock'):
            self.node.get_logger().info("✅ Clock checkbox available")

        # Topic list
        if hasattr(self, 'list_topics'):
            self.node.get_logger().info("✅ Topic list available")

    def on_browse_clicked(self):
        """Handle browse button click - open file dialog"""
        try:
            # Open file dialog for ROS2 bag (directory-based)
            file_path = QFileDialog.getExistingDirectory(
                self,
                "Select ROS2 Bag Directory",
                os.path.expanduser("~"),
            )

            if file_path:
                # Check if it's a valid ROS2 bag (should contain metadata.yaml)
                metadata_path = os.path.join(file_path, 'metadata.yaml')
                if not os.path.exists(metadata_path):
                    QMessageBox.warning(self, "Invalid Bag",
                                      "Selected directory is not a valid ROS2 bag.\nMissing metadata.yaml")
                    return

                self.current_bag_file = file_path
                self.node.get_logger().info(f"📁 Selected bag file: {file_path}")

                # Update UI
                if hasattr(self, 'line_edit_file_path'):
                    self.line_edit_file_path.setText(file_path)

                # Get bag info and topics
                self.display_bag_info(file_path)
                self.load_bag_topics(file_path)

                # Add to recent files
                self.add_recent_file(file_path)

        except Exception as e:
            self.node.get_logger().error(f"❌ Error browsing file: {e}")
            QMessageBox.critical(self, "Error", f"Failed to select file:\n{e}")

    def display_bag_info(self, bag_file):
        """Display bag file information using ros2 bag info"""
        try:
            # Run ros2 bag info command
            result = subprocess.run(
                ['ros2', 'bag', 'info', bag_file],
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode == 0:
                info_text = result.stdout
                if hasattr(self, 'text_bag_info'):
                    self.text_bag_info.setPlainText(info_text)
                self.node.get_logger().info("✅ Bag info displayed")
            else:
                self.node.get_logger().error(f"❌ Failed to get bag info: {result.stderr}")
                if hasattr(self, 'text_bag_info'):
                    self.text_bag_info.setPlainText(f"Error: {result.stderr}")

        except subprocess.TimeoutExpired:
            self.node.get_logger().error("❌ Timeout getting bag info")
            if hasattr(self, 'text_bag_info'):
                self.text_bag_info.setPlainText("Error: Timeout getting bag info")
        except Exception as e:
            self.node.get_logger().error(f"❌ Error getting bag info: {e}")

    def load_bag_topics(self, bag_file):
        """Load topics from ROS2 bag file"""
        try:
            # Read metadata.yaml
            metadata_path = os.path.join(bag_file, 'metadata.yaml')
            with open(metadata_path, 'r') as f:
                metadata = yaml.safe_load(f)

            # Clear existing topics
            self.bag_topics.clear()
            if hasattr(self, 'list_topics'):
                self.list_topics.clear()

            # Extract topics from metadata
            if 'rosbag2_bagfile_information' in metadata:
                bag_info = metadata['rosbag2_bagfile_information']
                if 'topics_with_message_count' in bag_info:
                    for topic_info in bag_info['topics_with_message_count']:
                        topic_name = topic_info['topic_metadata']['name']
                        topic_type = topic_info['topic_metadata']['type']
                        message_count = topic_info['message_count']

                        self.bag_topics[topic_name] = topic_type

                        # Add to list widget with checkbox
                        if hasattr(self, 'list_topics'):
                            item = QListWidgetItem(f"{topic_name} ({topic_type}) - {message_count} msgs")
                            item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
                            item.setCheckState(QtCore.Qt.Checked)  # Default: all selected
                            item.setData(QtCore.Qt.UserRole, topic_name)  # Store topic name
                            self.list_topics.addItem(item)

                    self.node.get_logger().info(f"✅ Loaded {len(self.bag_topics)} topics from bag file")
                else:
                    self.node.get_logger().warn("⚠️ No topics found in bag file")
            else:
                self.node.get_logger().warn("⚠️ Invalid metadata format")

        except Exception as e:
            self.node.get_logger().error(f"❌ Error loading bag topics: {e}")
            import traceback
            traceback.print_exc()

    def get_selected_topics(self):
        """Get list of selected topics from topic list"""
        selected_topics = []

        if hasattr(self, 'list_topics'):
            for i in range(self.list_topics.count()):
                item = self.list_topics.item(i)
                if item.checkState() == QtCore.Qt.Checked:
                    topic_name = item.data(QtCore.Qt.UserRole)
                    selected_topics.append(topic_name)

        return selected_topics

    def _playback_thread_worker(self):
        """Thread worker for ROS2 bag playback using ros2 bag play subprocess"""
        process = None
        try:
            # Get playback parameters
            loop = self.checkbox_loop.isChecked() if hasattr(self, 'checkbox_loop') else False
            rate = self.spinbox_rate.value() if hasattr(self, 'spinbox_rate') else 1.0
            use_sim_time = self.checkbox_clock.isChecked() if hasattr(self, 'checkbox_clock') else False
            selected_topics = self.get_selected_topics()

            # Build ros2 bag play command
            cmd = ['ros2', 'bag', 'play', self.current_bag_file]

            if loop:
                cmd.append('--loop')

            cmd.extend(['--rate', str(rate)])

            if use_sim_time:
                cmd.append('--clock')

            # Add topic filter if specific topics are selected
            if selected_topics and len(selected_topics) < len(self.bag_topics):
                cmd.append('--topics')
                cmd.extend(selected_topics)
                self.node.get_logger().info(f"📋 Playing {len(selected_topics)} selected topics")
            else:
                self.node.get_logger().info(f"📋 Playing all topics")

            self.node.get_logger().info(f"🎬 Starting playback: {' '.join(cmd)}")

            # Start subprocess
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            # Monitor playback
            while not self.stop_playback_flag.is_set():
                # Check if process is still running
                if process.poll() is not None:
                    # Process finished
                    break

                # Handle pause (using signals)
                if self.pause_playback_flag.is_set():
                    if not self.is_paused:
                        process.send_signal(signal.SIGSTOP)
                        self.is_paused = True
                else:
                    if self.is_paused:
                        process.send_signal(signal.SIGCONT)
                        self.is_paused = False

                time.sleep(0.1)

            # Stop process if still running
            if process.poll() is None:
                self.node.get_logger().info("🛑 Terminating playback process")
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.node.get_logger().warn("⚠️ Force killing playback process")
                    process.kill()

            # Get output
            stdout, stderr = process.communicate()
            if stderr:
                self.node.get_logger().warn(f"Playback stderr: {stderr}")

            self.node.get_logger().info("✅ Playback completed")

        except Exception as e:
            self.node.get_logger().error(f"❌ Playback error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # Clean up process
            if process and process.poll() is None:
                try:
                    process.terminate()
                    process.wait(timeout=2)
                except:
                    try:
                        process.kill()
                    except:
                        pass

            # Update UI
            if hasattr(self, 'label_status'):
                self.label_status.setText("Status: Stopped")
                self.label_status.setStyleSheet("color: gray; font-weight: bold;")

            self.playback_thread = None
            self.is_paused = False
            self.node.get_logger().info("🧹 Playback thread cleaned up")

    def on_play_clicked(self):
        """Handle play button click"""
        try:
            if not self.current_bag_file:
                QMessageBox.warning(self, "Warning", "Please select a bag file first!")
                return

            if not os.path.exists(self.current_bag_file):
                QMessageBox.critical(self, "Error", "Selected bag file does not exist!")
                return

            # Check if we should resume from pause
            if self.is_paused and self.playback_thread and self.playback_thread.is_alive():
                # Resume paused playback
                self.node.get_logger().info("▶️ Resuming playback from pause")
                self.pause_playback_flag.clear()

                # Update UI
                if hasattr(self, 'label_status'):
                    self.label_status.setText("Status: Playing")
                    self.label_status.setStyleSheet("color: green; font-weight: bold;")

                self.node.get_logger().info("✅ Playback resumed")
                return

            # Stop existing playback if running
            if self.playback_thread and self.playback_thread.is_alive():
                self.node.get_logger().warn("⚠️ Stopping existing playback")
                self.stop_playback()
                time.sleep(0.5)

            # Reset flags
            self.stop_playback_flag.clear()
            self.pause_playback_flag.clear()
            self.is_paused = False

            # Start playback thread
            self.node.get_logger().info("▶️ Starting playback thread")
            self.playback_thread = threading.Thread(target=self._playback_thread_worker)
            self.playback_thread.daemon = True
            self.playback_thread.start()

            # Update UI
            if hasattr(self, 'label_status'):
                self.label_status.setText("Status: Playing")
                self.label_status.setStyleSheet("color: green; font-weight: bold;")

            self.node.get_logger().info("✅ Playback started")

        except Exception as e:
            self.node.get_logger().error(f"❌ Error starting playback: {e}")
            QMessageBox.critical(self, "Error", f"Failed to start playback:\n{e}")

    def on_pause_clicked(self):
        """Handle pause button click"""
        try:
            if self.playback_thread and self.playback_thread.is_alive():
                # Set pause flag
                self.pause_playback_flag.set()
                self.node.get_logger().info("⏸️ Playback paused")

                if hasattr(self, 'label_status'):
                    self.label_status.setText("Status: Paused")
                    self.label_status.setStyleSheet("color: orange; font-weight: bold;")
            else:
                self.node.get_logger().warn("⚠️ No playback to pause")

        except Exception as e:
            self.node.get_logger().error(f"❌ Error pausing playback: {e}")

    def on_stop_clicked(self):
        """Handle stop button click"""
        self.stop_playback()

    def stop_playback(self):
        """Stop ROS2 bag playback"""
        try:
            # Store thread reference before it gets cleared
            thread = self.playback_thread

            if thread and thread.is_alive():
                self.node.get_logger().info("🛑 Stopping playback...")

                # Set stop flag
                self.stop_playback_flag.set()

                # Clear pause flag (in case it's paused)
                self.pause_playback_flag.clear()

                # Wait for thread to finish (max 5 seconds)
                thread.join(timeout=5)

                if thread.is_alive():
                    self.node.get_logger().warn("⚠️ Playback thread did not stop gracefully")
                else:
                    self.node.get_logger().info("✅ Playback stopped")

                # Clear pause state
                self.is_paused = False

                # Update UI
                if hasattr(self, 'label_status'):
                    self.label_status.setText("Status: Stopped")
                    self.label_status.setStyleSheet("color: red; font-weight: bold;")

        except Exception as e:
            self.node.get_logger().error(f"❌ Error stopping playback: {e}")

    def closeEvent(self, event):
        """Handle window close event"""
        self.node.get_logger().info("🔚 Bag Replay Tool closing...")

        # Stop playback if running
        if self.playback_thread and self.playback_thread.is_alive():
            self.stop_playback()

        event.accept()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    node = rclpy.create_node('bag_replay_tool')

    app = QtWidgets.QApplication(sys.argv)

    try:
        window = BagReplayTool(node)

        # Create a timer to spin ROS2
        timer = QtCore.QTimer()
        timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
        timer.start(100)  # Spin every 100ms

        exit_code = app.exec_()

        node.destroy_node()
        rclpy.shutdown()

        sys.exit(exit_code)

    except Exception as e:
        node.get_logger().error(f"❌ Error in main: {e}")
        import traceback
        traceback.print_exc()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
