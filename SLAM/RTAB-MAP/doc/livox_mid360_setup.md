# Livox MID-360 LiDAR SLAM 설치 가이드

이 문서는 Livox MID-360 라이다를 ROS2 Humble에서 사용하기 위한 설치 및 설정 방법을 안내합니다.

## 목차
- [하드웨어 사양](#하드웨어-사양)
- [Livox SDK2 설치](#livox-sdk2-설치)
- [Livox ROS2 드라이버 설치](#livox-ros2-드라이버-설치)
- [네트워크 설정](#네트워크-설정)
- [드라이버 실행](#드라이버-실행)
- [SLAM 솔루션 선택 가이드](#slam-솔루션-선택-가이드)

---

## 하드웨어 사양

### Livox MID-360 스펙
| 항목 | 사양 |
|------|------|
| FOV | 360° × 59° |
| 측정 거리 | 40m @ 10% 반사율 |
| 포인트 레이트 | 200,000 points/s |
| 정확도 | ±2cm (0.2m ~ 20m) |
| 내장 IMU | 6축 IMU (3축 가속도계 + 3축 자이로) |
| 인터페이스 | 100Mbps 이더넷 |
| 전원 | 9-27V DC, 최대 9W |

### 기본 네트워크 설정
| 항목 | 값 |
|------|-----|
| LiDAR IP | 192.168.1.1xx (xx는 SN 마지막 2자리) |
| Host IP | 192.168.1.50 (기본값) |
| Point Cloud Port | 56000~56100 |
| IMU Port | 56200~56300 |

---

## Livox SDK2 설치

### 1. 의존성 설치

```bash
sudo apt update
sudo apt install -y \
    cmake \
    build-essential \
    git \
    libpcl-dev \
    libboost-all-dev
```

### 2. Livox SDK2 빌드

```bash
cd ~/Downloads
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 3. 설치 확인

```bash
# 라이브러리 확인
ls /usr/local/lib | grep livox
# 출력: liblivox_lidar_sdk_shared.so, liblivox_lidar_sdk_static.a

# 헤더 확인
ls /usr/local/include | grep livox
# 출력: livox_lidar_api.h, livox_lidar_def.h
```

---

## Livox ROS2 드라이버 설치

### 1. 워크스페이스에 클론

```bash
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

### 2. 드라이버 빌드

```bash
cd ~/ros2_ws

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 빌드 (humble 버전)
colcon build --packages-select livox_ros_driver2 \
    --cmake-args -DROS_EDITION="ROS2" -DHUMBLE_ROS="humble"
```

### 3. 환경 설정

```bash
source ~/ros2_ws/install/setup.bash
```

---

## 네트워크 설정

### 1. 이더넷 인터페이스 설정

MID-360의 기본 IP 대역에 맞게 호스트 PC를 설정합니다.

```bash
# 네트워크 인터페이스 확인 (보통 eth0 또는 enp로 시작)
ip link show

# 고정 IP 설정 (예: eth0 인터페이스)
sudo ip addr add 192.168.1.50/24 dev eth0
sudo ip link set eth0 up
```

### 2. 영구적 설정 (Netplan)

`/etc/netplan/01-livox-config.yaml` 파일 생성:

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:  # 실제 인터페이스명으로 변경
      addresses:
        - 192.168.1.50/24
      dhcp4: false
```

적용:
```bash
sudo netplan apply
```

### 3. 방화벽 설정 (필요시)

```bash
# UDP 포트 열기
sudo ufw allow 56000:56300/udp
```

### 4. 연결 확인

```bash
# LiDAR IP로 ping (SN 끝 2자리에 따라 다름)
ping 192.168.1.1xx

# 예: SN이 ...47로 끝나면
ping 192.168.1.147
```

---

## 드라이버 설정 파일

### MID360 설정 파일 수정

`~/ros2_ws/src/livox_ros_driver2/config/MID360_config.json`:

```json
{
  "lidar_summary_info": {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "192.168.1.50",
      "cmd_data_port": 56101,
      "push_msg_ip": "",
      "push_msg_port": 56201,
      "point_data_ip": "",
      "point_data_port": 56301,
      "imu_data_ip": "",
      "imu_data_port": 56401,
      "log_data_ip": "",
      "log_data_port": 56501
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.1xx",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
      }
    }
  ]
}
```

**주의**: `"ip": "192.168.1.1xx"` 부분을 실제 LiDAR의 IP로 변경하세요.

---

## 드라이버 실행

### 기본 실행

```bash
source ~/ros2_ws/install/setup.bash

# MID-360 드라이버 실행
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 토픽 확인

```bash
# 새 터미널에서
source ~/ros2_ws/install/setup.bash

# 토픽 목록 확인
ros2 topic list

# 예상 출력:
# /livox/lidar
# /livox/imu
# /tf_static

# 포인트 클라우드 메시지 확인
ros2 topic echo /livox/lidar --once

# IMU 데이터 확인
ros2 topic echo /livox/imu --once

# Hz 확인
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

### RViz2로 시각화

```bash
# RViz2 실행
rviz2

# 설정:
# 1. Fixed Frame: "livox_frame"
# 2. Add -> By topic -> /livox/lidar -> PointCloud2
# 3. Size를 0.02~0.05로 설정
```

---

## SLAM 솔루션 선택 가이드

Livox MID-360과 호환되는 주요 SLAM 솔루션들:

### 1. FAST-LIO2 (권장)

| 항목 | 설명 |
|------|------|
| 특징 | Livox 라이다에 최적화된 LIO |
| IMU | 필수 (MID-360 내장 IMU 사용) |
| 정확도 | 매우 높음 |
| 실시간성 | 우수 (ARM도 지원) |
| ROS2 | 지원 |

**적합한 경우**: 고정밀 매핑, 실내/실외 모두 사용

### 2. Point-LIO

| 항목 | 설명 |
|------|------|
| 특징 | Point-by-point 처리 방식 |
| IMU | 필수 |
| 정확도 | 높음 |
| 실시간성 | 매우 우수 |
| ROS2 | 지원 |

**적합한 경우**: 빠른 움직임이 있는 환경, 드론 등

### 3. LIO-SAM

| 항목 | 설명 |
|------|------|
| 특징 | Factor Graph 기반 최적화 |
| IMU | 필수 |
| GPS | 선택적 지원 |
| 루프 클로저 | 지원 |
| ROS2 | 지원 |

**적합한 경우**: 대규모 야외 매핑, GPS 융합 필요시

### 4. RTAB-Map (기존 설치됨)

| 항목 | 설명 |
|------|------|
| 특징 | 다중 센서 융합 SLAM |
| IMU | 선택 |
| 카메라 | 선택적 지원 |
| 루프 클로저 | 강력 지원 |
| 로컬라이제이션 | 지원 |

**적합한 경우**: 카메라+LiDAR 융합, 로컬라이제이션 모드 필요시

---

## 다음 단계

1. [FAST-LIO2 설치](./fast_lio2_setup.md) - Livox 최적화 SLAM
2. [Point-LIO 설치](./point_lio_setup.md) - 고속 환경용
3. [LIO-SAM 설치](./lio_sam_setup.md) - GPS 융합 지원

---

**작성일**: 2025-12-05
**버전**: 1.0
