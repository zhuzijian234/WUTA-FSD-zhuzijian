cmake_minimum_required(VERSION 3.8)
project(controller)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
# ROS2 客户端库，用于创建节点、订阅者、发布者和定时器。
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(autoware_msgs REQUIRED)
find_package(wuta_msgs REQUIRED)

# 控制器节点由主控制逻辑和两个辅助算法文件组成。
add_executable(controller_node
  src/controller_node.cpp
  src/pure_pursuit.cpp
  src/twist_filter.cpp
)

target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(controller_node
  rclcpp
  geometry_msgs
  visualization_msgs
  autoware_msgs
  wuta_msgs
)

install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
