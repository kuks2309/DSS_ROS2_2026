# Launch Manager

ROS2 Launch Manager with Qt GUI for starting and stopping launch files.

## Features

- **DSS Controller**: Start/Stop DSS controller launch file
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
cd /home/amap/ros2_ws
colcon build --packages-select launch_manager
source install/setup.bash
```

## Usage

```bash
ros2 run launch_manager launch_manager
```

## UI Components

- **Start/Stop Buttons**: Control individual launch files
- **Browse**: Select custom launch files from file system
- **Status Log**: View launch events and errors
- **Stop All**: Emergency stop button for all processes

## Architecture

- `launch_manager_node.py`: Main ROS2 node with PyQt5 integration
- `ui/launch_manager.ui`: Qt Designer UI file
- Manages subprocess for each launch file
- Auto-detects launch files in workspace

## Notes

- Launch files are terminated gracefully with SIGINT
- Force kill with SIGKILL if process doesn't respond within 5 seconds
- UI updates button states every 500ms based on process status
