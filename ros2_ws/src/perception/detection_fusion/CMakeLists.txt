cmake_minimum_required(VERSION 3.8)
project(detection_fusion)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wuta_msgs REQUIRED)
# 融合节点是后续多源感知接入的扩展点，目前以接口和结构预留为主。
ament_package()
