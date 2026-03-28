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

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${gazebo_msgs_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Link directories
link_directories(${GAZEBO_LIBRARY_DIRS})

# Build the Gazebo plugin
add_library(robot_distance_plugin SHARED src/robot_distance_plugin.cpp)
target_link_libraries(robot_distance_plugin
  ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${gazebo_msgs_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
)

# Build the UKF library
add_library(ukf_lib src/ukf.cpp)
target_link_libraries(ukf_lib
  Eigen3::Eigen
)
target_include_directories(ukf_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Build the C++ node
add_executable(distance_to_pose_wrapper src/distance_to_pose_wrapper.cpp)
target_link_libraries(distance_to_pose_wrapper
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)

# Build the UKF node
add_executable(robot_ukf_node src/robot_ukf_node.cpp)
target_link_libraries(robot_ukf_node
  ukf_lib
  ${rclcpp_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  Eigen3::Eigen
)

# Install the C++ node
install(TARGETS distance_to_pose_wrapper robot_ukf_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the plugin
install(TARGETS robot_distance_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch, models, config
install(DIRECTORY launch models config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_package()