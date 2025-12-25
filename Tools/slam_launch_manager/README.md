# SLAM Launch Manager

ROS2 SLAM Launch Manager with Qt GUI for starting and stopping SLAM launch files.

## Features

- **DSS Controller**: Start/Stop DSS controller launch file
- **LIO-SAM**: Start/Stop LIO-SAM SLAM launch file
- **KISS-ICP**: Start/Stop KISS-ICP SLAM launch file
- **SLAM Toolbox**: Start/Stop SLAM Toolbox launch file
- **RTAB-Map**: Start/Stop RTAB-Map SLAM launch file
- **Custom Launch**: Browse and launch any custom launch file
- **Status Log**: Real-time logging of launch status
- **Stop All**: Emergency stop for all running launches

## Installation

### Prerequisites

```bash
sudo apt install python3-pyqt5 python3-pyqt5.qtwidgets
```

### Build

```bash
cd /home/amap/parking_robot_ros2_ws
colcon build --packages-select slam_launch_manager
source install/setup.bash
```

## Usage

```bash
ros2 run slam_launch_manager slam_launch_manager
```

## UI Components

- **Start/Stop Buttons**: Control individual launch files
- **Browse**: Select custom launch files from file system
- **Status Log**: View launch events and errors
- **Stop All**: Emergency stop button for all processes

## Architecture

- `slam_launch_manager_node.py`: Main ROS2 node with PyQt5 integration
- `ui/slam_launch_manager.ui`: Qt Designer UI file
- Manages subprocess for each launch file
- Auto-detects launch files in workspace

## Notes

- Launch files are terminated gracefully with SIGINT
- Force kill with SIGKILL if process doesn't respond within 5 seconds
- UI updates button states every 500ms based on process status
