cmake_minimum_required(VERSION 3.5)
project(robot_fusion)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)

# 确保头文件能被找到
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

# Build the Gazebo plugin
add_library(robot_distance_plugin SHARED src/robot_distance_plugin.cpp)
target_link_libraries(robot_distance_plugin
  ${GAZEBO_LIBRARIES}
)
ament_target_dependencies(robot_distance_plugin
  rclcpp
  geometry_msgs
  gazebo_msgs
  gazebo_ros
)

# Build the UKF library
add_library(ukf_lib src/ukf.cpp)
target_link_libraries(ukf_lib Eigen3::Eigen)
target_include_directories(ukf_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ===== 地图融合节点（Executable 方式，ros2 run 用）=====
add_executable(map_fusion_node 
  src/map_fusion_node.cpp 
  src/map_fusion_main.cpp  # 关键：需要 main 文件
)
target_include_directories(map_fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(map_fusion_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# Build the C++ node
add_executable(distance_to_pose_wrapper src/distance_to_pose_wrapper.cpp)
ament_target_dependencies(distance_to_pose_wrapper
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Build the UKF node
add_executable(robot_ukf_node src/robot_ukf_node.cpp)
target_link_libraries(robot_ukf_node ukf_lib)
ament_target_dependencies(robot_ukf_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# Install 可执行文件
install(TARGETS map_fusion_node distance_to_pose_wrapper robot_ukf_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install 插件
install(TARGETS robot_distance_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# Install 头文件
install(DIRECTORY include/
  DESTINATION include
)

# Install launch, models, config
install(DIRECTORY launch models config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()