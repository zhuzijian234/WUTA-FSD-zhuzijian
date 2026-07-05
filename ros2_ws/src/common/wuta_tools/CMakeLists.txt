cmake_minimum_required(VERSION 3.8)
project(wuta_tools)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wuta_msgs REQUIRED)
# 该包目前是工具层占位包，后续可以放置共享工具函数、辅助算法或通用节点基类。
ament_package()
