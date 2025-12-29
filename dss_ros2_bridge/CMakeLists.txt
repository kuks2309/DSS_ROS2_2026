cmake_minimum_required(VERSION 3.5)
project(dss_ros2_bridge)

# 사용 언어
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Force Protobuf 3.15 from /usr/local
set(Protobuf_DIR "/usr/local/lib/cmake/protobuf")

# 공통 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Protobuf 3.15 REQUIRED)

# 메시지 생성기
find_package(rosidl_default_generators REQUIRED)


find_library(NATS_LIB nats PATHS /usr/local/lib)

# ===============================
# 메시지 인터페이스 생성
# ===============================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DssControl.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ===============================
# 실행 파일 생성
# ===============================

add_executable(DSSToROSImageNode src/DSSToROSImage.cpp src/dss.pb.cc  src/defaultGateway.cpp)
ament_target_dependencies(DSSToROSImageNode rclcpp sensor_msgs OpenCV)
target_link_libraries(DSSToROSImageNode
  "${cpp_typesupport_target}"
  ${NATS_LIB}
  Threads::Threads
  protobuf::libprotobuf
)

add_executable(DSSToROSPointCloudNode src/DSSToROSPointCloud.cpp src/dss.pb.cc src/defaultGateway.cpp)
ament_target_dependencies(DSSToROSPointCloudNode rclcpp sensor_msgs OpenCV)
target_link_libraries(DSSToROSPointCloudNode
  "${cpp_typesupport_target}"
  ${NATS_LIB}
  Threads::Threads
  protobuf::libprotobuf
)

add_executable(DSSToROSIMUNode src/DSSToROSIMU.cpp src/dss.pb.cc src/defaultGateway.cpp)
ament_target_dependencies(DSSToROSIMUNode rclcpp sensor_msgs OpenCV)
target_link_libraries(DSSToROSIMUNode
  "${cpp_typesupport_target}"
  ${NATS_LIB}
  Threads::Threads
  protobuf::libprotobuf
)


add_executable(DSSToROSClockNode src/DSSToROSClock.cpp src/dss.pb.cc src/defaultGateway.cpp)
ament_target_dependencies(DSSToROSClockNode rclcpp sensor_msgs OpenCV)
target_link_libraries(DSSToROSClockNode
  "${cpp_typesupport_target}"
  ${NATS_LIB}
  Threads::Threads
  protobuf::libprotobuf
)


add_executable(DSSToROSGpsNode src/DSSToROSGps.cpp src/dss.pb.cc src/defaultGateway.cpp)
ament_target_dependencies(DSSToROSGpsNode rclcpp sensor_msgs OpenCV)
target_link_libraries(DSSToROSGpsNode
  "${cpp_typesupport_target}"
  ${NATS_LIB}
  Threads::Threads
  protobuf::libprotobuf
)


add_executable(DSSDemoNode src/DSSDemo.cpp src/dss.pb.cc src/DSS.VSSClient.cpp src/defaultGateway.cpp)
ament_target_dependencies(DSSDemoNode rclcpp sensor_msgs OpenCV)
target_link_libraries(DSSDemoNode
  "${cpp_typesupport_target}"
  ${NATS_LIB}
  Threads::Threads
  protobuf::libprotobuf
)


# ===============================
# 설치
# ===============================

install(
  TARGETS
    DSSToROSImageNode
    DSSToROSClockNode
    DSSToROSPointCloudNode
    DSSToROSIMUNode
    DSSToROSGpsNode
    DSSDemoNode
    ${cpp_typesupport_target}        # ← ★ 추가! typesupport .so 설치
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# 메시지 런타임 export
ament_export_dependencies(rosidl_default_runtime)

ament_package()
