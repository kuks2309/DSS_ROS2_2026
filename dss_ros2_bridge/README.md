# DSS ROS2 Bridge 설치 가이드 (ROS 2 Humble)

이 문서는 **Ubuntu 22.04 / WSL2 환경에서 ROS 2 Humble + DSS ROS2 Bridge**를 설치하는 전체 과정을 정리합니다.

---

## 1. ROS 2 Humble 설치

### **1.1 기본 패키지 업데이트**

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg lsb-release
```

---

### **1.2 ROS 2 Humble 저장소 추가 및 설치**

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update

sudo apt update && sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -

sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update

sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
```

---

### **1.3 ROS 2 환경 설정**

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 2. 의존성 설치

### **2.1 필수 라이브러리 설치**

```bash
sudo apt update
sudo apt install libnats-dev
sudo apt install -y git cmake build-essential pkg-config autoconf automake libtool
sudo apt install nlohmann-json3-dev
```

---

### **2.2 Protobuf 3.15.8 설치**

```bash
cd ~
git clone https://github.com/protocolbuffers/protobuf.git
cd protobuf
git checkout v3.15.8
git submodule update --init --recursive

./autogen.sh
./configure
make -j$(nproc)

sudo make install
sudo ldconfig
```

설치 확인:

```bash
ldconfig -p | grep protobuf
```

---

## 3. 저장소 Clone

HTTPS 방식으로 저장소를 클론합니다 (SSH 공개키 필요 없음):

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/kimbyoungkyu/dss_ros2_bridge.git
```

---

## 4. 빌드하기

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

---

## 5. 실행하기(브릿지 실행)

```bash
cd ~/ros2_ws
source ./install/setup.bash
ros2 launch dss_ros2_bridge launch.py
```

---

