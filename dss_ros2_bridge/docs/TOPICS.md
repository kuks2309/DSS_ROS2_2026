# DSS ROS2 Bridge - Topics Documentation

## Overview

`dss_ros2_bridge` 패키지는 DSS(Driving Simulation System, Unreal Engine 기반)와 ROS2 간의 브릿지 역할을 합니다. NATS 메시징 시스템을 통해 DSS에서 데이터를 수신하고, 이를 ROS2 토픽으로 변환하여 퍼블리시합니다.

## Architecture

```
┌─────────────────┐      ┌──────────┐      ┌─────────────────┐      ┌─────────────┐
│  DSS (Unreal)   │ ──── │   NATS   │ ──── │ dss_ros2_bridge │ ──── │ ROS2 Topics │
│    Simulator    │      │  Server  │      │     Nodes       │      │             │
└─────────────────┘      └──────────┘      └─────────────────┘      └─────────────┘
```

## ROS2 Publishers

### 1. IMU Data

| 항목 | 내용 |
|------|------|
| **Topic** | `/imu/data` |
| **Message Type** | `sensor_msgs::msg::Imu` |
| **Node** | `DSSToROSIMUNode` |
| **Source File** | `src/DSSToROSIMU.cpp` |
| **Frame ID** | `imu_link` |
| **NATS Source** | `dss.sensor.imu` |

**Description:**
IMU 센서 데이터를 퍼블리시합니다. Orientation(quaternion), Angular Velocity, Linear Acceleration 정보를 포함합니다.

**Message Fields:**
- `header.frame_id`: "imu_link"
- `orientation`: Quaternion (x, y, z, w)
- `angular_velocity`: Vector3 (x, y, z) [rad/s]
- `linear_acceleration`: Vector3 (x, y, z) [m/s²]

---

### 2. Camera RGB Image

| 항목 | 내용 |
|------|------|
| **Topic** | `/dss/sensor/camera/rgb` |
| **Message Type** | `sensor_msgs::msg::Image` |
| **Node** | `DSSToROSImageNode` |
| **Source File** | `src/DSSToROSImage.cpp` |
| **Frame ID** | `camera` |
| **NATS Source** | `dss.sensor.camera.rgb` |

**Description:**
RGB 카메라 이미지를 퍼블리시합니다. JPEG 포맷으로 수신된 이미지를 RGB8 포맷으로 디코딩하여 전송합니다.

**Message Fields:**
- `header.frame_id`: "camera"
- `encoding`: "rgb8"
- `width`, `height`: 이미지 크기
- `data`: RGB 픽셀 데이터

---

### 3. Simulation Clock

| 항목 | 내용 |
|------|------|
| **Topic** | `/clock` |
| **Message Type** | `rosgraph_msgs::msg::Clock` |
| **Node** | `DSSToROSClockNode` |
| **Source File** | `src/DSSToROSClock.cpp` |
| **NATS Source** | `dss.simTime.clock` |

**Description:**
시뮬레이션 시간을 퍼블리시합니다. `use_sim_time` 파라미터가 `True`일 때만 활성화됩니다. 프레임 카운트를 기반으로 고정 시간 간격(0.005초/프레임)으로 시간을 계산합니다.

**Message Fields:**
- `clock`: 시뮬레이션 시간 (sec, nanosec)

**Notes:**
- 200 FPS 기준 (1 frame = 0.005 seconds)

---

### 4. GPS/GNSS Data

| 항목 | 내용 |
|------|------|
| **Topic** | `/dss/gps` |
| **Message Type** | `sensor_msgs::msg::NavSatFix` |
| **Node** | `DSSToROSGpsNode` |
| **Source File** | `src/DSSToROSGps.cpp` |
| **Frame ID** | `gps_link` |
| **NATS Source** | `dss.sensor.gps` |

**Description:**
GPS/GNSS 위치 데이터를 퍼블리시합니다. 위도, 경도, 고도 및 공분산 정보를 포함합니다.

**Message Fields:**
- `header.frame_id`: "gps_link"
- `latitude`: 위도 [degrees]
- `longitude`: 경도 [degrees]
- `altitude`: 고도 [meters]
- `position_covariance`: 위치 공분산 행렬
- `status`: GPS 상태 정보

---

### 5. LiDAR Point Cloud

| 항목 | 내용 |
|------|------|
| **Topic** | `/points` |
| **Message Type** | `sensor_msgs::msg::PointCloud2` |
| **Node** | `DSSToROSPointCloudNode` |
| **Source File** | `src/DSSToROSPointCloud.cpp` |
| **Frame ID** | `lidar_link` |
| **NATS Source** | `dss.sensor.lidar` |

**Description:**
LiDAR 포인트 클라우드 데이터를 퍼블리시합니다. 여러 양자화 포맷(16-bit, 10-byte, 14-byte)을 지원합니다.

**Message Fields:**
- `header.frame_id`: "lidar_link"
- `fields`: x, y, z, intensity, ring, time
- `point_step`: 포인트당 바이트 수
- `data`: 포인트 클라우드 데이터

**Point Fields:**
| Field | Type | Offset | Description |
|-------|------|--------|-------------|
| x | FLOAT32 | 0 | X 좌표 [m] |
| y | FLOAT32 | 4 | Y 좌표 [m] |
| z | FLOAT32 | 8 | Z 좌표 [m] |
| intensity | FLOAT32 | 12 | 반사 강도 |
| ring | UINT16 | 16 | 레이저 링 번호 |
| time | FLOAT32 | 18 | 타임스탬프 오프셋 |

---

## ROS2 Subscribers (Demo)

데모 노드 `DssDemoController`는 브릿지 노드들이 퍼블리시하는 토픽을 구독하는 예제입니다.

| Topic | Message Type | Source File |
|-------|--------------|-------------|
| `/dss/sensor/imu` | `sensor_msgs::msg::Imu` | `src/DSSDemo.cpp` |
| `/dss/sensor/lidar` | `sensor_msgs::msg::PointCloud2` | `src/DSSDemo.cpp` |
| `/dss/sensor/camera/rgb` | `sensor_msgs::msg::Image` | `src/DSSDemo.cpp` |

---

## Topic Summary Table

| Topic Name | Message Type | Direction | Node |
|------------|--------------|-----------|------|
| `/imu/data` | `sensor_msgs/Imu` | Publish | DSSToROSIMUNode |
| `/dss/sensor/camera/rgb` | `sensor_msgs/Image` | Publish | DSSToROSImageNode |
| `/clock` | `rosgraph_msgs/Clock` | Publish | DSSToROSClockNode |
| `/dss/gps` | `sensor_msgs/NavSatFix` | Publish | DSSToROSGpsNode |
| `/points` | `sensor_msgs/PointCloud2` | Publish | DSSToROSPointCloudNode |

---

## NATS Topic Mapping

| NATS Topic (DSS) | ROS2 Topic |
|------------------|------------|
| `dss.sensor.imu` | `/imu/data` |
| `dss.sensor.camera.rgb` | `/dss/sensor/camera/rgb` |
| `dss.simTime.clock` | `/clock` |
| `dss.sensor.gps` | `/dss/gps` |
| `dss.sensor.lidar` | `/points` |

---

## Heartbeat Topics (NATS)

각 노드는 NATS를 통해 하트비트 메시지를 퍼블리시하여 상태를 모니터링합니다.

| Node | Heartbeat Topic |
|------|-----------------|
| DSSToROSIMUNode | `dss.dssToROSImu.heartBeat` |
| DSSToROSImageNode | `dss.dssToROSImage.heartBeat` |
| DSSToROSClockNode | `dss.dssToROSClock.heartBeat` |
| DSSToROSGpsNode | `dss.dssToROSGps.heartBeat` |
| DSSToROSPointCloudNode | `dss.dssToROSPointCloud.heartBeat` |

---

## Launch Configuration

### Default Launch

```bash
ros2 launch dss_ros2_bridge launch.py
```

### Launch Parameters

모든 노드에 공통으로 적용되는 파라미터:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_sim_time` | `True` | 시뮬레이션 시간 사용 여부 |

### Launched Nodes

`launch/launch.py`에서 실행되는 노드 목록:
1. `DSSToROSImageNode`
2. `DSSToROSIMUNode`
3. `DSSToROSPointCloudNode`
4. `DSSToROSGpsNode`
5. `DSSToROSClockNode`

---

## Usage Examples

### Subscribe to IMU Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)

    def imu_callback(self, msg):
        self.get_logger().info(f'IMU: orientation={msg.orientation}')
```

### Subscribe to Point Cloud

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class LiDARSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/points',
            self.pointcloud_callback,
            10)

    def pointcloud_callback(self, msg):
        self.get_logger().info(f'PointCloud: {msg.width * msg.height} points')
```

### View Topics with CLI

```bash
# List all topics
ros2 topic list

# Echo IMU data
ros2 topic echo /imu/data

# Echo GPS data
ros2 topic echo /dss/gps

# Check topic frequency
ros2 topic hz /points
```

---

## Dependencies

- `sensor_msgs`
- `rosgraph_msgs`
- `std_msgs`
- NATS C++ Client Library
- Protocol Buffers (protobuf)
- TurboJPEG (for image decoding)
