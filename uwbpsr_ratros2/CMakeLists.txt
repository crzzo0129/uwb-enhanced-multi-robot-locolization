cmake_minimum_required(VERSION 3.5)
project(uwbpsr_ratros2)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(msg_files
  msg/LinktrackAnchorframe0.msg
  msg/LinktrackNode0.msg
  msg/LinktrackNode1.msg
  msg/LinktrackNode2.msg
  msg/LinktrackNodeframe0.msg
  msg/LinktrackNodeframe1.msg
  msg/LinktrackNodeframe2.msg
  msg/LinktrackNodeframe3.msg
  msg/LinktrackTag.msg
  msg/LinktrackTagframe0.msg
  msg/LinktrackNode4Anchor.msg
  msg/LinktrackNode4Tag.msg
  msg/LinktrackNodeframe4.msg
  msg/LinktrackNode5.msg
  msg/LinktrackNodeframe5.msg
  msg/LinktrackNode6.msg
  msg/LinktrackNodeframe6.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)


include_directories(
  extern/serial/include
  src/utils
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}/msg
)


add_subdirectory(extern/serial)


add_library(nutils
  src/utils/protocol_extracter/nprotocol_base.cpp
  src/utils/protocol_extracter/nprotocol_extracter.cpp
  src/utils/nlink_unpack/nlink_utils.c
  src/utils/nlink_unpack/nlink_linktrack_anchorframe0.c
  src/utils/nlink_unpack/nlink_linktrack_tagframe0.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe0.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe1.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe2.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe3.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe4.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe5.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe6.c
  src/utils/nlink_unpack/nlink_iot_frame0.c
  src/utils/init_serial.cpp
  src/utils/nlink_protocol.cpp
  src/utils/nutils.cpp
)

target_link_libraries(nutils
  serial
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

ament_target_dependencies(nutils
  rclcpp
  std_msgs
  geometry_msgs
)


add_executable(linktrack
  src/linktrack/init.cpp
  src/linktrack/main.cpp
  src/linktrack/protocols.cpp
)

target_link_libraries(linktrack
  nutils
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

ament_target_dependencies(linktrack
  rclcpp std_msgs geometry_msgs rosidl_default_runtime
)


add_executable(linktrack_rviz_converter
  src/linktrack/main_rviz_converter.cpp
)

target_link_libraries(linktrack_rviz_converter
  nutils
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

ament_target_dependencies(linktrack_rviz_converter
  rclcpp std_msgs geometry_msgs rosidl_default_runtime
)



# 安装
install(TARGETS
  linktrack
  linktrack_rviz_converter
  nutils
  
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}/msg
  DESTINATION include/${PROJECT_NAME}/msg
)
ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY
  launch
  src/linktrack

  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

