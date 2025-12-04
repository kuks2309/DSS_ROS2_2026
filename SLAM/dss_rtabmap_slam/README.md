# RTAB-Map SLAM for DSS Platform

RTAB-Map (Real-Time Appearance-Based Mapping) SLAM package configured for DSS platform using LiDAR sensor.

## Features

- **LiDAR-based SLAM**: 3D mapping using point cloud data
- **ICP Odometry**: Real-time odometry estimation from LiDAR
- **Loop Closure Detection**: Automatic drift correction
- **Mapping & Localization**: Support both modes
- **Visualization**: Built-in rviz visualization

## Prerequisites

Install RTAB-Map ROS2 packages:
```bash
sudo apt install ros-humble-rtabmap-ros
```

## Package Structure

```
rtabmap_slam/
├── launch/
│   ├── rtabmap_mapping.launch.py       # Basic mapping with external odometry
│   └── rtabmap_with_odom.launch.py     # Mapping with ICP odometry
├── config/
│   └── rtabmap_params.yaml             # RTAB-Map parameters
└── README.md
```

## Launch Files

### 1. rtabmap_with_odom.launch.py (Recommended)
Complete SLAM system with ICP odometry generation from LiDAR.

```bash
ros2 launch rtabmap_slam rtabmap_with_odom.launch.py
```

**Features:**
- Generates odometry from LiDAR using ICP
- RTAB-Map SLAM mapping
- Visualization window

### 2. rtabmap_mapping.launch.py
SLAM mapping requiring external odometry source.

```bash
# Mapping mode (default)
ros2 launch rtabmap_slam rtabmap_mapping.launch.py

# Localization mode
ros2 launch rtabmap_slam rtabmap_mapping.launch.py localization:=true
```

**Parameters:**
- `use_sim_time`: Use simulation time (default: false)
- `localization`: Localization mode vs mapping mode (default: false)
- `database_path`: Path to save/load map database (default: ~/.ros/rtabmap.db)

## Topics

### Subscribed Topics
- `/dss/sensor/lidar` (sensor_msgs/PointCloud2): LiDAR point cloud
- `/odom` (nav_msgs/Odometry): Odometry (for rtabmap_mapping.launch.py)

### Published Topics
- `/map` (nav_msgs/OccupancyGrid): 2D occupancy grid map
- `/mapData` (rtabmap_ros/MapData): RTAB-Map data
- `/odom` (nav_msgs/Odometry): Odometry (from icp_odometry)
- `/tf`: Transform tree

## TF Frames

```
map -> odom -> base_link
```

- `map`: Global fixed frame
- `odom`: Odometry frame (drift may occur)
- `base_link`: Robot base frame

## Configuration

Key parameters in [rtabmap_params.yaml](config/rtabmap_params.yaml):

**ICP Parameters:**
- `Icp/VoxelSize`: Point cloud downsampling (0.1m)
- `Icp/MaxCorrespondenceDistance`: Max distance for point matching (1.0m)
- `Icp/PointToPlane`: Use point-to-plane ICP (true)

**Loop Closure:**
- `RGBD/AngularUpdate`: Min rotation for new keyframe (0.05 rad)
- `RGBD/LinearUpdate`: Min translation for new keyframe (0.05 m)

**Optimization:**
- `Optimizer/Slam2D`: 2D vs 3D SLAM (false = 3D)

## Usage with Launch Manager

Update [launch_manager_node.py](../../launch_manager/launch_manager/launch_manager_node.py) to point to RTAB-Map launch file:

```python
self.node.launch_files['rtabmap'] = '/home/amap/ros2_ws/src/SLAM/rtabmap_slam/launch/rtabmap_with_odom.launch.py'
```

## Database Management

RTAB-Map saves maps to a database file (default: `~/.ros/rtabmap.db`).

**View saved map:**
```bash
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

**Delete database (start fresh):**
```bash
rm ~/.ros/rtabmap.db
```

**Export map:**
```bash
rtabmap-export ~/.ros/rtabmap.db
```

## Troubleshooting

**No odometry received:**
- Ensure `/dss/sensor/lidar` is publishing
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify `base_link` frame exists

**Poor mapping quality:**
- Adjust `Icp/VoxelSize` (smaller = more detail)
- Increase `Icp/MaxCorrespondenceDistance` for large environments
- Tune `RGBD/LinearUpdate` and `RGBD/AngularUpdate`

**High CPU usage:**
- Increase `Icp/VoxelSize` (downsample more)
- Reduce `OdomF2M/ScanMaxSize` (limit point cloud size)

## References

- [RTAB-Map ROS Wiki](http://wiki.ros.org/rtabmap_ros)
- [RTAB-Map Documentation](https://github.com/introlab/rtabmap)
