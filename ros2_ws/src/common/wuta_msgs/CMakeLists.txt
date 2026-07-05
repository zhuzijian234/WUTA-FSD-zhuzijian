cmake_minimum_required(VERSION 3.8)
project(wuta_msgs)

# 这是整个系统的消息基础层。
# 所有节点都依赖它，因此它必须先于其他包被编译和生成接口。
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成自定义消息接口，让 ROS2 能把这些 .msg 文件编译成 C++/Python 可用类型。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cone.msg"
  "msg/ConeArray.msg"
  "msg/ConeMap.msg"
  "msg/MissionState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 导出 rosidl 运行时依赖，方便其他包引用这些消息。
ament_export_dependencies(rosidl_default_runtime)
ament_package()
