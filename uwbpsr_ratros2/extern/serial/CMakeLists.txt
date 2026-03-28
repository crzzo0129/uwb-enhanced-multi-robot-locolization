cmake_minimum_required(VERSION 3.5)
project(serial)

# 设置头文件路径
include_directories(include)

# 添加源文件
set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

# 平台特定代码
if(APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)
else()
    list(APPEND serial_SRCS src/impl/win.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_win.cc)
endif()

# 编译为静态库
add_library(${PROJECT_NAME} ${serial_SRCS})

# 链接平台相关库
if(APPLE)
    target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
    target_link_libraries(${PROJECT_NAME} rt pthread)
else()
    target_link_libraries(${PROJECT_NAME} setupapi)
endif()

# # 示例程序（可选）
# add_executable(serial_example examples/serial_example.cc)
# target_link_libraries(serial_example ${PROJECT_NAME})

# # 安装目标（可选）
# install(TARGETS ${PROJECT_NAME}
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
# )

install(DIRECTORY include/serial DESTINATION include)
