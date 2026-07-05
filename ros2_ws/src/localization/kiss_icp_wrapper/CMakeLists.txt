cmake_minimum_required(VERSION 3.8)
project(kiss_icp_wrapper)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wuta_msgs REQUIRED)
# 该包是 KISS-ICP 的配置与包装层，后续用于适配不同雷达参数和启动配置。
ament_package()
