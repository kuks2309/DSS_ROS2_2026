# RTAB-Map SLAM 설치 가이드

이 문서는 DSS 플랫폼에서 RTAB-Map SLAM을 사용하기 위한 패키지 설치 방법을 안내합니다.

## 목차
- [시스템 요구사항](#시스템-요구사항)
- [ROS2 패키지 설치](#ros2-패키지-설치)
- [libpointmatcher 설치](#libpointmatcher-설치)
- [설치 확인](#설치-확인)
- [문제 해결](#문제-해결)

---

## 시스템 요구사항

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **메모리**: 최소 4GB RAM (권장 8GB+)

---

## ROS2 패키지 설치

### 1. RTAB-Map ROS2 패키지 설치

RTAB-Map의 모든 구성 요소를 설치합니다:

```bash
sudo apt update
sudo apt install ros-humble-rtabmap-ros
```

이 명령어는 다음 패키지들을 자동으로 설치합니다:
- `rtabmap_slam` - SLAM 노드
- `rtabmap_odom` - ICP 기반 Visual/LiDAR Odometry
- `rtabmap_util` - 유틸리티 도구들
- `rtabmap_viz` - 시각화 도구

### 2. 개별 패키지 설치 (선택사항)

특정 패키지만 설치하려면:

```bash
# SLAM 패키지만
sudo apt install ros-humble-rtabmap-slam

# Odometry 패키지만
sudo apt install ros-humble-rtabmap-odom

# 유틸리티 패키지만
sudo apt install ros-humble-rtabmap-util
```

### 3. 추가 의존성 설치

RTAB-Map이 제대로 작동하기 위한 추가 패키지:

```bash
sudo apt install \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-visualization-msgs \
  ros-humble-rviz2
```

---

## libpointmatcher 설치

RTAB-Map의 ICP (Iterative Closest Point) 알고리즘은 libpointmatcher 라이브러리를 사용합니다.

### 방법 1: 패키지 관리자로 설치 (권장)

```bash
sudo apt install libpointmatcher-dev
```

### 방법 2: 소스에서 빌드

더 최신 버전이 필요하거나 커스터마이징이 필요한 경우:

#### 2.1 의존성 설치

```bash
sudo apt install \
  libeigen3-dev \
  libboost-all-dev \
  cmake \
  build-essential \
  git
```

#### 2.2 libnabo 설치 (libpointmatcher 의존성)

```bash
cd ~/Downloads
git clone https://github.com/ethz-asl/libnabo.git
cd libnabo
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

#### 2.3 libpointmatcher 빌드 및 설치

```bash
cd ~/Downloads
git clone https://github.com/ethz-asl/libpointmatcher.git
cd libpointmatcher
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

#### 2.4 라이브러리 경로 업데이트

```bash
sudo ldconfig
```

---

## 설치 확인

### 1. RTAB-Map 패키지 확인

```bash
ros2 pkg list | grep rtabmap
```

**예상 출력:**
```
rtabmap_conversions
rtabmap_demos
rtabmap_launch
rtabmap_msgs
rtabmap_odom
rtabmap_python
rtabmap_rviz_plugins
rtabmap_slam
rtabmap_sync
rtabmap_util
rtabmap_viz
```

### 2. ICP Odometry 노드 확인

```bash
ros2 run rtabmap_odom icp_odometry --ros-args -h
```

에러 없이 help 메시지가 출력되면 정상입니다.

### 3. RTAB-Map 버전 확인

```bash
ros2 run rtabmap_slam rtabmap --params | grep "RTAB-Map"
```

### 4. libpointmatcher 확인

```bash
ldconfig -p | grep pointmatcher
```

**예상 출력:**
```
libpointmatcher.so.1 (libc6,x86-64) => /usr/lib/x86_64-linux-gnu/libpointmatcher.so.1
libpointmatcher.so (libc6,x86-64) => /usr/lib/x86_64-linux-gnu/libpointmatcher.so
```

---

## 문제 해결

### 문제 1: ICP Odometry 노드가 실행되지 않음

**증상:**
```
[ERROR] Could not find library 'libpointmatcher'
```

**해결:**
```bash
# libpointmatcher 재설치
sudo apt install --reinstall libpointmatcher-dev
sudo ldconfig
```

### 문제 2: "Registration failed" 에러

**증상:**
```
[ERROR] Registration failed: "limit out of bounds"
```

**원인:** ICP 파라미터 설정이 차량 속도에 맞지 않음

**해결:** `rtabmap_with_rviz.launch.py`에서 다음 파라미터 조정:
```python
'Icp/MaxTranslation': '3.0',  # 차량 속도에 맞게 증가
'Icp/MaxRotation': '3.14',    # 180도까지 허용
'Icp/Iterations': '20',       # 실시간 성능을 위해 조정
```

### 문제 3: ROS2 패키지를 찾을 수 없음

**증상:**
```
Package 'rtabmap_slam' not found
```

**해결:**
```bash
# ROS2 환경 재설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 패키지 재설치
sudo apt update
sudo apt install ros-humble-rtabmap-ros
```

### 문제 4: 메모리 부족

**증상:**
```
Killed (메모리 부족으로 프로세스 종료)
```

**해결:**
1. Swap 메모리 증가:
```bash
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

2. RTAB-Map 메모리 파라미터 조정:
```python
'Mem/STMSize': '30',              # Short-term memory 크기 감소
'Icp/VoxelSize': '0.1',           # Voxel 크기 증가 (포인트 수 감소)
'OdomF2M/ScanMaxSize': '15000',   # 최대 포인트 수 제한
```

### 문제 5: CPU 사용률 과다

**증상:**
ICP odometry 노드가 100% 이상 CPU 사용

**해결:**
```python
# ICP iterations 감소
'Icp/Iterations': '20',  # 50 → 20으로 감소

# Voxel 크기 증가 (처리할 포인트 수 감소)
'Icp/VoxelSize': '0.15',  # 0.1 → 0.15

# 업데이트 빈도 감소
'RGBD/LinearUpdate': '0.3',   # 30cm마다 업데이트
'RGBD/AngularUpdate': '0.1',  # 0.1 rad마다 업데이트
```

---

## 참고 자료

- **RTAB-Map 공식 위키**: http://wiki.ros.org/rtabmap_ros
- **GitHub 저장소**: https://github.com/introlab/rtabmap_ros
- **libpointmatcher 문서**: https://libpointmatcher.readthedocs.io/
- **ICP 파라미터 가이드**: https://github.com/introlab/rtabmap/wiki/Kinect-mapping#parameters

---

## 추가 설정

### ROS2 환경 자동 설정

매번 source 명령을 실행하지 않도록 `~/.bashrc`에 추가:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### RTAB-Map 데이터베이스 저장 위치

기본 위치: `~/.ros/rtabmap.db`

커스텀 위치 설정:
```bash
# Launch 시 database_path 인자로 지정
ros2 launch dss_rtabmap_slam rtabmap_with_rviz.launch.py \
  database_path:=/home/amap/ros2_ws/src/SLAM/dss_rtabmap_localization/maps/my_map.db
```

---

**작성일**: 2025-12-05
**버전**: 1.0
**문의**: DSS SLAM 팀
