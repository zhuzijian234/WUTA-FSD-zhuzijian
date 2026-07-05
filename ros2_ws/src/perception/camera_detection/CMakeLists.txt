cmake_minimum_required(VERSION 3.8)
project(camera_detection)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wuta_msgs REQUIRED)
# 相机检测模块目前是预留接口，用于后续接入视觉锥桶检测与多源融合。
ament_package()
