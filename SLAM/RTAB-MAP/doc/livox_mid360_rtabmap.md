# Livox MID-360 + RTAB-Map SLAM 설정 가이드

이 문서는 Livox MID-360 라이다를 RTAB-Map SLAM과 함께 사용하기 위한 설정 방법을 안내합니다.

## 목차
- [하드웨어 개요](#하드웨어-개요)
- [토픽 매핑](#토픽-매핑)
- [Livox SDK2 및 드라이버 설치](#livox-sdk2-및-드라이버-설치)
- [네트워크 설정](#네트워크-설정)
- [RTAB-Map 연동 설정](#rtab-map-연동-설정)
- [TF 프레임 설정](#tf-프레임-설정)
- [Launch 파일 수정](#launch-파일-수정)
- [문제 해결](#문제-해결)

---

## 하드웨어 개요

### Livox MID-360 사양

| 항목 | 사양 |
|------|------|
| FOV | 360° (수평) × 59° (수직) |
| 측정 거리 | 40m @ 10% 반사율 |
| 포인트 레이트 | 200,000 points/s |
| 정확도 | ±2cm (0.2m ~ 20m) |
| 내장 IMU | 6축 IMU (가속도계 + 자이로) |
| 인터페이스 | 100Mbps 이더넷 |
| 전원 | 9-27V DC, 최대 9W |
| 크기 | Φ65mm × 60mm |
| 무게 | 265g |

### 기본 네트워크 정보

| 항목 | 기본값 |
|------|--------|
| LiDAR IP | 192.168.1.1xx (xx = SN 마지막 2자리) |
| Host IP | 192.168.1.50 |
| Point Cloud Port | 56100 |
| IMU Port | 56200 |

---

## 토픽 매핑

### Livox MID-360 발행 토픽

| 토픽 이름 | 메시지 타입 | 설명 |
|-----------|-------------|------|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 3D 포인트 클라우드 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 내장 IMU 데이터 |

### RTAB-Map 필요 토픽

| RTAB-Map 파라미터 | 연결할 Livox 토픽 |
|-------------------|-------------------|
| `scan_cloud` | `/livox/lidar` |
| `imu` | `/livox/imu` |

### 기존 설정과의 차이점

| 항목 | 기존 (예: Velodyne) | Livox MID-360 |
|------|---------------------|---------------|
| Point Cloud 토픽 | `/velodyne_points` | `/livox/lidar` |
| IMU 토픽 | `/dss/sensor/imu` | `/livox/imu` |
| 프레임 ID | `velodyne` | `livox_frame` |
| 스캔 패턴 | 회전식 (반복) | 비반복식 (로즈 패턴) |

---

## Livox SDK2 및 드라이버 설치

### 1. Livox SDK2 설치

```bash
# 의존성 설치
sudo apt update
sudo apt install -y cmake build-essential git

# SDK2 클론 및 빌드
cd ~/Downloads
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 2. Livox ROS2 드라이버 설치

```bash
# 워크스페이스에 클론
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# 빌드
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select livox_ros_driver2 \
    --cmake-args -DROS_EDITION="ROS2" -DHUMBLE_ROS="humble"
```

### 3. 설치 확인

```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep livox
# 출력: livox_ros_driver2
```

---

## 네트워크 설정

### 1. LiDAR 시리얼 번호 확인

LiDAR 하단의 스티커에서 시리얼 번호 마지막 2자리를 확인합니다.
예: SN이 `...47`로 끝나면 LiDAR IP는 `192.168.1.147`

### 2. 호스트 PC 네트워크 설정

```bash
# 이더넷 인터페이스 확인
ip link show

# IP 설정 (eth0을 실제 인터페이스명으로 변경)
sudo ip addr add 192.168.1.50/24 dev eth0
sudo ip link set eth0 up
```

### 3. 연결 테스트

```bash
# LiDAR ping 테스트
ping 192.168.1.1xx  # xx는 SN 마지막 2자리
```

### 4. Livox 설정 파일 수정

`~/ros2_ws/src/livox_ros_driver2/config/MID360_config.json`에서 IP 주소를 확인/수정합니다.

---

## RTAB-Map 연동 설정

### 1. 토픽 리매핑

RTAB-Map launch 파일에서 다음과 같이 토픽을 매핑합니다:

**기존 Velodyne 기준:**
```python
# 변경 전
'scan_cloud': '/velodyne_points'
'imu': '/dss/sensor/imu'
```

**Livox MID-360 기준:**
```python
# 변경 후
'scan_cloud': '/livox/lidar'
'imu': '/livox/imu'
```

### 2. ICP Odometry 파라미터 조정

Livox의 비반복 스캔 패턴에 맞게 파라미터를 조정합니다:

| 파라미터 | 권장값 | 설명 |
|----------|--------|------|
| `Icp/VoxelSize` | 0.1 ~ 0.15 | 포인트 밀도에 따라 조정 |
| `Icp/MaxCorrespondenceDistance` | 0.5 ~ 1.0 | 매칭 최대 거리 |
| `Icp/Iterations` | 30 | ICP 반복 횟수 |
| `OdomF2M/ScanMaxSize` | 20000 | MID-360의 높은 포인트 레이트 고려 |
| `Icp/PointToPlaneRadius` | 0.0 | 비활성화 (point-to-plane 미사용 시) |

### 3. 프레임 ID 설정

| 파라미터 | 값 |
|----------|-----|
| `frame_id` | `livox_frame` |
| `odom_frame_id` | `odom` |
| `map_frame_id` | `map` |

---

## TF 프레임 설정

### 프레임 구조

```
map
 └── odom
      └── base_link
           └── livox_frame
```

### Static TF 퍼블리셔 설정

LiDAR가 차량에 장착된 위치에 따라 TF를 설정합니다:

```bash
# 예: LiDAR가 base_link 기준 위로 0.5m, 앞으로 0.2m에 장착된 경우
ros2 run tf2_ros static_transform_publisher \
    --x 0.2 --y 0.0 --z 0.5 \
    --roll 0.0 --pitch 0.0 --yaw 0.0 \
    --frame-id base_link --child-frame-id livox_frame
```

---

## Launch 파일 수정

### dss_rtabmap_slam 패키지 수정 사항

기존 launch 파일에서 다음 부분을 수정해야 합니다:

### 1. 토픽 이름 변경

| 항목 | 기존 값 | 변경 값 |
|------|---------|---------|
| Point Cloud 토픽 | `/scan` 또는 `/velodyne_points` | `/livox/lidar` |
| IMU 토픽 | `/dss/sensor/imu` | `/livox/imu` |

### 2. 프레임 ID 변경

| 항목 | 기존 값 | 변경 값 |
|------|---------|---------|
| LiDAR 프레임 | `laser` 또는 `velodyne` | `livox_frame` |

### 3. 포인트 클라우드 필터링

MID-360은 포인트 밀도가 높으므로 필요시 다운샘플링:

| 파라미터 | 권장값 |
|----------|--------|
| `cloud_decimation` | 2 ~ 4 |
| `cloud_max_depth` | 30.0 |
| `cloud_min_depth` | 0.5 |

---

## 실행 순서

### 1. Livox 드라이버 실행

```bash
# 터미널 1
source ~/ros2_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 2. 토픽 확인

```bash
# 터미널 2
ros2 topic list | grep livox
# /livox/lidar
# /livox/imu

ros2 topic hz /livox/lidar
# 약 10Hz (스캔 주기)
```

### 3. RTAB-Map SLAM 실행

```bash
# 터미널 3
source ~/ros2_ws/install/setup.bash
ros2 launch dss_rtabmap_slam rtabmap_with_rviz.launch.py
```

---

## 문제 해결

### 문제 1: 포인트 클라우드가 RViz에 표시되지 않음

**확인 사항:**
1. Fixed Frame이 `livox_frame`으로 설정되어 있는지 확인
2. TF tree가 정상인지 확인: `ros2 run tf2_tools view_frames`
3. 토픽 데이터 확인: `ros2 topic echo /livox/lidar --once`

### 문제 2: ICP Odometry 실패

**증상:**
```
[WARN] ICP failed: not enough correspondences
```

**해결:**
- `Icp/MaxCorrespondenceDistance` 값 증가 (0.5 → 1.0)
- `Icp/VoxelSize` 값 감소 (더 많은 포인트 사용)
- 초기 위치에서 충분한 특징점이 있는지 확인

### 문제 3: IMU 데이터 동기화 문제

**증상:**
```
[WARN] IMU data stamp is too old
```

**해결:**
- `wait_imu_to_init` 파라미터를 `true`로 설정
- IMU 토픽 Hz 확인: `ros2 topic hz /livox/imu` (보통 200Hz)

### 문제 4: 네트워크 연결 불가

**확인 사항:**
1. 이더넷 케이블 연결 상태
2. IP 대역 일치 확인 (192.168.1.x)
3. 방화벽 설정: `sudo ufw status`
4. LiDAR 전원 LED 상태 (정상: 초록색 점등)

---

## 참고 자료

- **Livox MID-360 공식 매뉴얼**: https://www.livoxtech.com/mid-360
- **Livox ROS2 드라이버**: https://github.com/Livox-SDK/livox_ros_driver2
- **RTAB-Map 위키**: https://github.com/introlab/rtabmap/wiki
- **RTAB-Map ROS2**: https://github.com/introlab/rtabmap_ros

---

**작성일**: 2025-12-05  
**버전**: 1.0  
**관련 문서**: [installation.md](./installation.md)
