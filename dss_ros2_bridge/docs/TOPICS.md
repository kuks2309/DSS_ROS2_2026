# DSS ROS2 Bridge - Topics

## Topic List

| Topic | Message Type | Frame ID | Description |
|-------|--------------|----------|-------------|
| `/dss/sensor/imu` | `sensor_msgs/Imu` | `imu_link` | IMU 센서 데이터 |
| `/dss/sensor/gps/fix` | `sensor_msgs/NavSatFix` | `gps_link` | GPS 위치 데이터 |
| `/dss/sensor/camera/rgb` | `sensor_msgs/Image` | `camera` | RGB 카메라 이미지 |
| `/dss/sensor/lidar3d` | `sensor_msgs/PointCloud2` | `lidar_link` | LiDAR 포인트클라우드 |
| `/clock` | `rosgraph_msgs/Clock` | - | 시뮬레이션 시간 |

---

## Architecture

```
DSS (Unreal) ──── NATS Server ──── dss_ros2_bridge ──── ROS2 Topics
```

## NATS → ROS2 Mapping

| NATS Topic | ROS2 Topic |
|------------|------------|
| `dss.sensor.imu` | `/dss/sensor/imu` |
| `dss.sensor.gps` | `/dss/sensor/gps/fix` |
| `dss.sensor.camera.rgb` | `/dss/sensor/camera/rgb` |
| `dss.sensor.lidar` | `/dss/sensor/lidar3d` |
| `dss.simTime.clock` | `/clock` |

---

## Usage

```bash
# Launch all bridge nodes
ros2 launch dss_ros2_bridge launch.py

# Check topics
ros2 topic list
ros2 topic echo /dss/sensor/imu
ros2 topic echo /dss/sensor/gps/fix
ros2 topic hz /dss/sensor/lidar3d
```

---

## Nodes

| Node | Source | Description |
|------|--------|-------------|
| `DSSToROSIMUNode` | `DSSToROSIMU.cpp` | IMU 브릿지 |
| `DSSToROSGpsNode` | `DSSToROSGps.cpp` | GPS 브릿지 |
| `DSSToROSImageNode` | `DSSToROSImage.cpp` | 카메라 브릿지 |
| `DSSToROSPointCloudNode` | `DSSToROSPointCloud.cpp` | LiDAR 브릿지 |
| `DSSToROSClockNode` | `DSSToROSClock.cpp` | Clock 브릿지 |

---

## Dependencies

- `sensor_msgs`, `rosgraph_msgs`, `std_msgs`
- NATS C++ Client, Protocol Buffers, TurboJPEG
