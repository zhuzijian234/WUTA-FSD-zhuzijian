cmake_minimum_required(VERSION 3.8)
project(path_generator)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
# 路径生成节点依赖 ROS2 和自定义消息。
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(wuta_msgs REQUIRED)
find_package(autoware_msgs REQUIRED)

# 该节点会根据 mission_mode 生成 trackdrive / skidpad / acceleration 三类路径。
add_executable(path_generator_node src/path_generator_node.cpp)

target_include_directories(path_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(path_generator_node
  rclcpp
  geometry_msgs
  wuta_msgs
  autoware_msgs
)

install(TARGETS path_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
