# dss_controller Python 패키지 설치 안내 (순수 Python 버전)

이 문서는 **C++ 빌드 없이**, 오직 **pip3 기반 Python 환경만으로**  
`dss_controller` 패키지를 설치하고 실행하는 방법을 정리한 안내서입니다.

Google Protobuf, NATS Python Client 등 필요한 Python 패키지 설치를 모두 포함합니다.

---

# 📦 1. 필요한 Python 패키지 설치

순수 Python 버전에서는 다음 라이브러리를 설치해야 합니다:

- **protobuf** (Google Protocol Buffers - Python 버전)
- **nats-py** (NATS Python 클라이언트)
- **asyncio-nats-client** 또는 **nats-py**
- **rclpy** (ROS2 Python API — *ROS2가 설치되어 있어야 함*)
- 기타 ROS 메시지 의존성

---

# 🛠 1-1. pip3 기반 의존성 설치

```
pip3 install protobuf
pip3 install nats-py
pip3 install asyncio-nats-client
pip3 install setuptools
```

---

# 🛠 1-2. ROS2 Python API 설치

ROS2 Humble이 설치된 환경에서는 다음 명령으로 Python API를 설치할 수 있습니다:

```
sudo apt install ros-humble-rclpy
sudo apt install ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-geometry-msgs
```

환경 설정:

```
source /opt/ros/humble/setup.bash
```

---

# 🏗 2. dss_controller 패키지 설치

패키지 루트에서 다음 명령 실행:

```
cd ~/ros2_ws/src/dss_controller
pip3 install .
```

또는 개발 모드 설치:

```
pip3 install -e .
```

---

# ▶️ 3. 실행 방법

```
ros2 run dss_controller dss_controller
```

또는 Python 직접 실행:

```
python3 dss_controller/dss_controller_node.py
```

(ROS2 환경변수가 설정되어 있어야 합니다.)

---

# 📁 4. 패키지 구조

```
dss_controller/
├── setup.py
├── setup.cfg
├── package.xml
├── resource/
│   └── dss_controller
├── dss_controller/
│   ├── __init__.py
│   └── dss_controller_node.py
└── .gitignore
```

---

# 📌 5. 순수 Python 버전에서 필요한 패키지 요약

| 패키지명 | 설치 방법 |
|----------|-----------|
| protobuf | `pip3 install protobuf` |
| nats-py | `pip3 install nats-py` |
| asyncio-nats-client | `pip3 install asyncio-nats-client` |
| setuptools | `pip3 install setuptools` |
| rclpy | `sudo apt install ros-humble-rclpy` |
| ROS 메시지 | `sudo apt install ros-humble-std-msgs ...` |

---

# ❗ 참고

- C++ 기반 NATS / Protobuf 설치는 필요 없음  
- pip3 기반 설치만으로 모든 기능 동작 가능  
- ROS2 기반 Python 노드 실행을 위한 rclpy만 있으면 됨

---

필요하시면  
✔ `pip3 install` 자동 설치 스크립트  
✔ NATS + Protobuf Python 샘플 코드  
✔ ROS2 Python 메시지 퍼블리셔/서브스크라이버 템플릿  
도 만들어 드립니다!
