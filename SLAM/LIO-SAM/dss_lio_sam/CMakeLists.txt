cmake_minimum_required(VERSION 3.8)
project(dss_lio_sam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

# Link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

# ROS dependencies list
set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  message_filters
)

# OpenCV for range image
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# IMU Preintegration Node
add_executable(imu_preintegration_node src/imuPreintegration.cpp)
ament_target_dependencies(imu_preintegration_node ${dependencies})
target_link_libraries(imu_preintegration_node
  ${PCL_LIBRARIES}
  gtsam
)

# Image Projection Node
add_executable(image_projection_node src/imageProjection.cpp)
ament_target_dependencies(image_projection_node ${dependencies})
target_link_libraries(image_projection_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam
)

# Map Optimization Node
add_executable(map_optimization_node src/mapOptimization.cpp)
ament_target_dependencies(map_optimization_node ${dependencies})
target_link_libraries(map_optimization_node
  ${PCL_LIBRARIES}
  gtsam
)

# Install executables
install(TARGETS
  imu_preintegration_node
  image_projection_node
  map_optimization_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
