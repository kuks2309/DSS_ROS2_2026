# ROS2 Bag Replay Tool

GUI tool for replaying ROS2 bag files with topic selection and playback control.

**Converted from ROS1 to ROS2**

## Features

- **File Selection**: Browse and select ROS2 bag directories using GUI
- **Recent Files Menu**: Quick access to recently opened bag files
  - Accessible via File menu
  - Shows up to 10 most recently used files
  - Persistent across sessions
- **Topic Selection**: View all topics in bag file and select which ones to publish
- **Playback Controls**: Play, Pause, and Stop controls
- **Playback Options**:
  - Loop playback
  - Adjustable playback rate (0.1x to 10x)
  - Publish clock time option
- **Bag Info Display**: View detailed information about the selected bag file

## Installation

```bash
cd ~/parking_robot_ros2_ws
colcon build --packages-select bag_replay_tool_ros2
source install/setup.bash
```

## Usage

### Launch the tool

```bash
ros2 launch bag_replay_tool_ros2 bag_replay_tool.launch.py
```

Or run directly:

```bash
ros2 run bag_replay_tool_ros2 bag_replay_tool_node
```

### Using the GUI

1. **Select a bag file**:
   - Click "Browse..." button to select a ROS2 bag directory, or
   - Use File → Open Bag File... (Ctrl+O), or
   - Select from File → Recent Files menu
   - Note: ROS2 bags are directories containing `metadata.yaml` and `.db3` files

2. **Review topics**: The left panel shows all topics in the bag file with checkboxes
   - Check/uncheck topics to select which ones to publish
   - By default, all topics are selected

3. **Configure playback options**:
   - Loop Playback: Enable to continuously replay the bag
   - Playback Rate: Adjust speed (default 1.0x)
   - Publish Clock Time: Enable to publish /clock topic

4. **Control playback**:
   - Click **Play** to start playback with selected topics
   - Click **Pause** to pause (can resume)
   - Click **Stop** to stop playback

## Dependencies

- rclpy
- rosbag2_py
- PyQt5
- python-yaml

### Install Python dependencies

```bash
pip3 install PyQt5
```

## Package Structure

```
bag_replay_tool_ros2/
├── bag_replay_tool_ros2/
│   ├── __init__.py
│   └── bag_replay_tool_node.py    # Main Python node
├── ui/
│   └── bag_replay_tool.ui         # Qt Designer UI file
├── launch/
│   └── bag_replay_tool.launch.py  # ROS2 launch file
├── resource/
│   └── bag_replay_tool_ros2       # Package marker
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## Changes from ROS1 Version

### Key Differences

1. **Bag Format**:
   - ROS1: Single `.bag` file
   - ROS2: Directory containing `metadata.yaml` and `.db3` SQLite database

2. **API Changes**:
   - `rospy` → `rclpy`
   - `rosbag` → `rosbag2_py` and `ros2 bag` CLI
   - Node initialization using `rclpy.create_node()`

3. **Playback Method**:
   - ROS1: Used `rosbag` Python API directly
   - ROS2: Uses `ros2 bag play` subprocess command
   - Pause/Resume using SIGSTOP/SIGCONT signals

4. **Build System**:
   - ROS1: catkin with CMakeLists.txt
   - ROS2: ament_python with setup.py

5. **File Dialog**:
   - ROS1: File selection for `.bag` files
   - ROS2: Directory selection for bag folders

## Notes

- The tool uses `ros2 bag play` command with appropriate options
- Topic selection uses the `--topics` flag to filter topics
- Pause functionality uses SIGSTOP/SIGCONT signals on the subprocess
- The tool automatically cleans up processes on exit
- Recent files list is stored in `~/.ros2/bag_replay_tool_recent.json`
- Clear recent files list via File → Clear Recent Files

## Troubleshooting

### UI file not found
Ensure the package is properly built and installed:
```bash
colcon build --packages-select bag_replay_tool_ros2 --symlink-install
source install/setup.bash
```

### PyQt5 not found
Install PyQt5:
```bash
pip3 install PyQt5
```

### Invalid bag directory
ROS2 bags are directories, not single files. Make sure you select the directory containing `metadata.yaml`.

## Migration Guide

For users migrating from ROS1 `bag_replay_tool`:

1. **Bag Format**: Convert ROS1 bags to ROS2 format:
   ```bash
   ros2 bag convert -i input.bag -o output_ros2_bag
   ```

2. **Usage**: The GUI interface remains the same, but:
   - Select directories instead of files
   - Bag info format is different
   - Recent files are stored in `~/.ros2/` instead of `~/.ros/`

## License

MIT
