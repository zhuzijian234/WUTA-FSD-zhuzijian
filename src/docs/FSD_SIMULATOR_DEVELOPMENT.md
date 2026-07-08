# WUTA-FSD 仿真模拟器开发文档

本文档基于当前仓库源码整理，面向开发与 WUTA-FSD 对接的仿真模拟器。重点说明 FSD 向外传出的数据、仿真器需要回灌的数据、ROS2 topic/message 约定、坐标系和闭环联调方式。

> 当前源码是 ROS2 工作区，目录入口为 `ros2_ws/src`。README 中部分中文因编码显示异常，以下以源码为准。

## 1. 系统数据流概览

WUTA-FSD 的主链路如下：

```text
LiDAR PointCloud2
  -> lidar_detection_node
  -> /perception/lidar/cones             wuta_msgs/ConeArray
  -> cone_map_builder
  -> /mapping/cone_map                   wuta_msgs/ConeMap
  -> boundary_detector_node
  -> /planning/centerline                autoware_msgs/Lane
  -> path_generator_node
  -> /planning/final_waypoints           autoware_msgs/Lane
  -> controller_node
  -> /control/command                    autoware_msgs/Command
```

定位与状态管理链路：

```text
/odometry/filtered 或 /ndt/pose
  -> localization_manager
  -> /localization/pose                  geometry_msgs/PoseStamped

/system/lidar_ready + /system/localization_ready + /mapping/cone_map.is_closed
  -> mission_manager
  -> /system/mission_state               wuta_msgs/MissionState
```

仿真器最重要的闭环接口是：

- 订阅 FSD 输出：`/control/command`
- 发布车辆状态回 FSD：`/localization/pose`、`/localization/velocity`
- 根据仿真层级发布环境感知输入：`/hesai/pandar`、`/perception/lidar/cones` 或 `/mapping/cone_map`
- 发布系统就绪/急停等状态：`/system/lidar_ready`、`/system/localization_ready`、`/system/emergency`

## 2. 推荐仿真接入层级

### 2.1 Level A：整栈仿真

目标：验证感知、建图、规划、控制全链路。

仿真器发布：

| Topic | Type | 说明 |
|---|---|---|
| `/hesai/pandar` | `sensor_msgs/msg/PointCloud2` | Hesai 128 线 LiDAR 点云，frame 建议为 `base_link` 或实际 LiDAR frame |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EXPLORE 模式下 localization_manager 的 EKF 输入 |
| `/localization/velocity` | `geometry_msgs/msg/TwistStamped` | 控制器用于当前速度估计 |
| `/system/lidar_ready` | `std_msgs/msg/Bool` | LiDAR 仿真就绪 |
| `/system/localization_ready` | `std_msgs/msg/Bool` | 定位仿真就绪，若使用 localization_manager 会由其发布 |
| `/tf` / `/tf_static` | `tf2_msgs/msg/TFMessage` | 至少提供 `map -> base_link`、`base_link -> lidar_frame` |

仿真器订阅：

| Topic | Type | 说明 |
|---|---|---|
| `/control/command` | `autoware_msgs/msg/Command` | FSD 输出给车辆/VCU 的控制命令 |

### 2.2 Level B：跳过点云感知

目标：验证建图、规划、控制，不开发 LiDAR 点云仿真。

仿真器发布：

| Topic | Type | 说明 |
|---|---|---|
| `/perception/lidar/cones` | `wuta_msgs/msg/ConeArray` | 车辆坐标系或 LiDAR 坐标系下的锥桶检测 |
| `/localization/pose` | `geometry_msgs/msg/PoseStamped` | 车辆在 `map` 下的位姿 |
| `/localization/velocity` | `geometry_msgs/msg/TwistStamped` | 车辆速度 |
| `/tf` / `/tf_static` | `tf2_msgs/msg/TFMessage` | `cone_map_builder` 会查 `map <- sensor_frame` 变换 |
| `/system/lidar_ready`、`/system/localization_ready` | `std_msgs/msg/Bool` | 推动 mission_manager 从 IDLE 到 READY |

### 2.3 Level C：跳过感知和建图

目标：只验证规划/控制或车辆动力学闭环。

仿真器发布：

| Topic | Type | 说明 |
|---|---|---|
| `/mapping/cone_map` | `wuta_msgs/msg/ConeMap` | 全局 `map` 坐标系锥桶地图 |
| `/localization/pose` | `geometry_msgs/msg/PoseStamped` | 车辆位姿 |
| `/localization/velocity` | `geometry_msgs/msg/TwistStamped` | 车辆速度 |
| `/system/mission_state` | `wuta_msgs/msg/MissionState` | 可绕过 mission_manager，直接置为 EXPLORE/RACE |

### 2.4 Level D：只测控制器

目标：验证 Pure Pursuit 输出与车辆模型。

仿真器发布：

| Topic | Type | 说明 |
|---|---|---|
| `/planning/final_waypoints` | `autoware_msgs/msg/Lane` | 控制器跟踪路径 |
| `/localization/pose` | `geometry_msgs/msg/PoseStamped` | 车辆位姿 |
| `/localization/velocity` | `geometry_msgs/msg/TwistStamped` | 车辆速度 |
| `/system/mission_state` | `wuta_msgs/msg/MissionState` | state 必须为 `EXPLORE` 或 `RACE`，控制器才输出非停车命令 |

仿真器订阅：

| Topic | Type | 说明 |
|---|---|---|
| `/control/command` | `autoware_msgs/msg/Command` | 速度与转角命令 |

## 3. FSD 向仿真器传出的数据

### 3.1 车辆控制命令

| Topic | Type | 发布节点 | 频率 | 用途 |
|---|---|---|---|---|
| `/control/command` | `autoware_msgs/msg/Command` | `controller_node` | 默认 50 Hz | 车辆执行命令 |

当前源码使用字段：

| 字段 | 单位 | 说明 |
|---|---:|---|
| `speed` | m/s | 目标车速，来自路径点速度并经 `TwistFilter` 平滑 |
| `angle` | degree | 前轮转角或转向命令，正值表示左转，硬限幅到 `max_steer_angle` |
| `dv_state` | enum/int | 当前使用 `4` 表示正常，`6` 表示 emergency |

控制器启用条件：

- `/system/mission_state.state == EXPLORE(3)` 或 `RACE(5)`
- 已收到 `/localization/pose`
- 已收到非空 `/planning/final_waypoints`

停车输出：

- 当 mission state 不是 `EXPLORE/RACE` 时，控制器发布 `speed=0.0`、`angle=0.0`、`dv_state=4`

### 3.2 路径与规划调试输出

| Topic | Type | 发布节点 | 说明 |
|---|---|---|---|
| `/planning/centerline` | `autoware_msgs/msg/Lane` | `boundary_detector_node` | Trackdrive 中由锥桶地图生成的中心线 |
| `/planning/final_waypoints` | `autoware_msgs/msg/Lane` | `path_generator_node` | 控制器消费的最终路径 |
| `/planning/centerline_viz` | `visualization_msgs/msg/MarkerArray` | `boundary_detector_node` | RViz 中心线可视化 |
| `/control/target_viz` | `visualization_msgs/msg/MarkerArray` | `controller_node` | Pure Pursuit 目标点和前视圆 |

`autoware_msgs/msg/Lane` 当前源码使用字段：

- `header.stamp`
- `header.frame_id = "map"`
- `waypoints[]`
- 每个 `waypoint.pose.pose.position.{x,y,z}`
- 每个 `waypoint.pose.pose.orientation.w`
- 每个 `waypoint.twist.twist.linear.x`，单位 m/s

### 3.3 感知与地图输出

| Topic | Type | 发布节点 | 频率/触发 | 说明 |
|---|---|---|---|---|
| `/perception/lidar/cones` | `wuta_msgs/msg/ConeArray` | `lidar_detection_node` | 跟随点云输入 | LiDAR 锥桶检测结果 |
| `/perception/lidar/cones_viz` | `visualization_msgs/msg/MarkerArray` | `lidar_detection_node` | 有订阅者时 | 检测锥桶可视化 |
| `/mapping/cone_map` | `wuta_msgs/msg/ConeMap` | `cone_map_builder` | 5 Hz | 全局锥桶地图 |
| `/mapping/cone_map_viz` | `visualization_msgs/msg/MarkerArray` | `cone_map_builder` | 有订阅者时 | 地图可视化 |

### 3.4 定位与系统状态输出

| Topic | Type | 发布节点 | 说明 |
|---|---|---|---|
| `/localization/pose` | `geometry_msgs/msg/PoseStamped` | `localization_manager` | 下游规划控制统一定位输入 |
| `/system/localization_ready` | `std_msgs/msg/Bool` | `localization_manager` | 收到有效定位后发布 true |
| `/system/mission_state` | `wuta_msgs/msg/MissionState` | `mission_manager` | 10 Hz 状态广播 |
| `/system/inspection_result` | `std_msgs/msg/String` | `mission_manager` | 车检接口预留，当前返回 `INSPECTION_NOT_IMPLEMENTED` |
| `/ndt/pose` | `geometry_msgs/msg/PoseStamped` | `ndt_localization` | RACE 模式 NDT 位姿 |
| `/ndt/path` | `nav_msgs/msg/Path` | `ndt_localization` | 最近 500 个 NDT 位姿 |
| `/ndt/aligned_cloud` | `sensor_msgs/msg/PointCloud2` | `ndt_localization` | NDT 对齐点云调试 |
| `/ndt/map_ready` | `std_msgs/msg/Bool` | `map_saver` | NDT 点云地图保存完成 |

## 4. 自定义消息定义

### 4.1 `wuta_msgs/msg/Cone`

```text
uint8 COLOR_UNKNOWN = 0
uint8 COLOR_BLUE    = 1
uint8 COLOR_YELLOW  = 2
uint8 COLOR_ORANGE  = 3

geometry_msgs/Point position
uint8 color
float32 confidence
```

约定：

- `position` 是三维点，坐标系由所在消息的 `header.frame_id` 决定
- LiDAR detector 当前输出 `color=COLOR_UNKNOWN`、`confidence=1.0`
- `cone_map_builder` 可按车辆朝向把左侧分为蓝色、右侧分为黄色

### 4.2 `wuta_msgs/msg/ConeArray`

```text
std_msgs/Header header
Cone[] cones
```

约定：

- `header.frame_id` 必须可通过 TF 转换到 `map`
- 用于感知检测输出时，通常是 LiDAR frame 或 `base_link`

### 4.3 `wuta_msgs/msg/ConeMap`

```text
std_msgs/Header header
Cone[] blue_cones
Cone[] yellow_cones
Cone[] orange_cones
Cone[] unknown_cones
bool is_closed
```

约定：

- `header.frame_id = "map"`
- `is_closed=true` 表示赛道闭环完成，mission_manager 会从 `EXPLORE` 切到 `MAPPING_DONE`
- 当前源码尚未自动从 `MAPPING_DONE` 切到 `RACE`，注释中标记为 TODO

### 4.4 `wuta_msgs/msg/MissionState`

状态常量：

| 名称 | 值 | 说明 |
|---|---:|---|
| `IDLE` | 0 | 初始化 |
| `READY` | 1 | 传感器/定位就绪，等待开始 |
| `INSPECTION` | 2 | 车检预留 |
| `EXPLORE` | 3 | 探索/建图圈 |
| `MAPPING_DONE` | 4 | 地图完成 |
| `RACE` | 5 | 高速循迹 |
| `FINISH` | 6 | 任务完成 |
| `EMERGENCY` | 7 | 急停或故障 |

任务模式：

| 名称 | 值 |
|---|---:|
| `MISSION_TRACKDRIVE` | 0 |
| `MISSION_SKIDPAD` | 1 |
| `MISSION_ACCELERATION` | 2 |

定位模式：

| 名称 | 值 |
|---|---:|
| `LOC_KISS_ICP` | 0 |
| `LOC_NDT` | 1 |

字段：

```text
std_msgs/Header header
uint8 state
uint8 mission_mode
uint8 localization_mode
string description
```

## 5. 仿真器需要发布给 FSD 的关键数据

### 5.1 车辆位姿

| Topic | Type | 坐标系 | 建议频率 |
|---|---|---|---:|
| `/localization/pose` | `geometry_msgs/msg/PoseStamped` | `map` | 50 Hz |

字段要求：

- `header.frame_id = "map"`
- `pose.position.{x,y,z}` 为车辆参考点位置，建议用后轴中心或 `base_link` 原点，与车辆模型统一
- `pose.orientation` 使用 quaternion，yaw 表示车辆航向

如果使用 `localization_manager`，则不要直接发布 `/localization/pose`，而是发布：

- EXPLORE：`/odometry/filtered`，`nav_msgs/msg/Odometry`
- RACE：`/ndt/pose`，`geometry_msgs/msg/PoseStamped`

### 5.2 车辆速度

| Topic | Type | 坐标系 | 建议频率 |
|---|---|---|---:|
| `/localization/velocity` | `geometry_msgs/msg/TwistStamped` | `base_link` 或 `map` | 50 Hz |

当前控制器只使用：

```text
sqrt(twist.linear.x^2 + twist.linear.y^2)
```

### 5.3 锥桶检测

如果跳过点云感知，发布：

| Topic | Type | 坐标系 | 建议频率 |
|---|---|---|---:|
| `/perception/lidar/cones` | `wuta_msgs/msg/ConeArray` | LiDAR frame 或 `base_link` | 10-20 Hz |

要求：

- 每个 cone 的 `position` 是传感器/车辆坐标系下位置
- `header.stamp` 与 TF 时间匹配
- `header.frame_id` 能被 TF2 转到 `map`
- `confidence` 建议 0.0-1.0
- 若不提供颜色，设 `COLOR_UNKNOWN`

### 5.4 全局锥桶地图

如果跳过感知和建图，发布：

| Topic | Type | 坐标系 | 建议频率 |
|---|---|---|---:|
| `/mapping/cone_map` | `wuta_msgs/msg/ConeMap` | `map` | 5-10 Hz |

要求：

- `blue_cones` 为左边界
- `yellow_cones` 为右边界
- `orange_cones` 可用于起终点线
- `unknown_cones` 也会被 boundary_detector 使用
- Trackdrive 中需要至少 4 个有效锥桶才会尝试 Delaunay 中心线

### 5.5 系统状态辅助 topic

| Topic | Type | 方向 | 说明 |
|---|---|---|---|
| `/system/lidar_ready` | `std_msgs/msg/Bool` | 仿真器 -> FSD | true 后 mission_manager 记录 LiDAR 就绪 |
| `/system/localization_ready` | `std_msgs/msg/Bool` | 仿真器/定位 -> FSD | true 后 mission_manager 记录定位就绪 |
| `/system/emergency` | `std_msgs/msg/Bool` | 仿真器 -> FSD | true 会切到 EMERGENCY |
| `/system/mission_mode_cmd` | `std_msgs/msg/String` | 仿真器/人工 -> FSD | `trackdrive`、`skidpad`、`acceleration` |
| `/system/inspection_trigger` | `std_msgs/msg/Bool` | 仿真器/人工 -> FSD | 车检预留，仅 IDLE/READY 有效 |

注意：当前 `mission_manager` 从 IDLE 自动切 READY，但没有外部 start topic 自动切 EXPLORE。源码中 `transitionTo(EXPLORE)` 只在内部函数可用，当前没有订阅 start 命令。因此联调时建议：

1. 临时由仿真器直接发布 `/system/mission_state` 置为 `EXPLORE`/`RACE`；或
2. 给 `mission_manager` 增加 `/system/start` topic；或
3. 在测试阶段只启动下游节点，不启动 `mission_manager`。

## 6. 坐标系与单位

推荐 TF 树：

```text
map
  -> odom
    -> base_link
      -> lidar
```

最小可运行约定：

- 路径、锥桶地图、全局位姿统一使用 `map`
- 点云和 `ConeArray` 可使用 `base_link` 或 LiDAR frame，但必须提供到 `map` 的 TF
- 距离单位：m
- 速度单位：m/s
- yaw：rad
- `/control/command.angle`：degree，正值左转

`cone_map_builder` 会使用 TF2 查询：

```text
target_frame = "map"
source_frame = ConeArray.header.frame_id
time = ConeArray.header.stamp
timeout = 0.1 s
```

## 7. 当前节点接口清单

| 节点 | 订阅 | 发布 |
|---|---|---|
| `lidar_detection_node` | `/hesai/pandar` `sensor_msgs/PointCloud2` | `/perception/lidar/cones` `ConeArray`；`/perception/lidar/cones_viz` |
| `cone_map_builder` | `/perception/lidar/cones` `ConeArray`；`/localization/pose` `PoseStamped` | `/mapping/cone_map` `ConeMap`；`/mapping/cone_map_viz` |
| `boundary_detector_node` | `/mapping/cone_map` `ConeMap`；`/localization/pose` `PoseStamped`；`/system/mission_state` | `/planning/centerline` `Lane`；`/planning/centerline_viz` |
| `path_generator_node` | `/planning/centerline` `Lane`；`/localization/pose`；`/system/mission_state` | `/planning/final_waypoints` `Lane` |
| `controller_node` | `/planning/final_waypoints` `Lane`；`/localization/pose`；`/localization/velocity`；`/system/mission_state` | `/control/command` `Command`；`/control/target_viz` |
| `localization_manager` | `/system/mission_state`；`/odometry/filtered`；`/ndt/pose` | `/localization/pose`；`/system/localization_ready` |
| `mission_manager` | `/mapping/cone_map`；`/system/emergency`；`/system/mission_mode_cmd`；`/system/lidar_ready`；`/system/localization_ready`；`/system/inspection_trigger` | `/system/mission_state`；`/system/inspection_result` |
| `ndt_localization` | `/hesai/pandar`；`/initialpose`；`/system/mission_state` | `/ndt/pose`；`/ndt/path`；`/ndt/aligned_cloud` |
| `map_saver` | `/hesai/pandar`；`/kiss/odometry`；`/system/mission_state` | `/ndt/map_ready` |

## 8. 默认参数

### 8.1 LiDAR 检测

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `input_topic` | `/hesai/pandar` | 输入点云 |
| `output_topic` | `/perception/lidar/cones` | 输出锥桶 |
| `detector_type` | `traditional` | `dl` 当前会抛出未实现异常 |
| `max_detection_range` | 20.0 m | 最大检测距离 |
| `cluster_tolerance` | 0.4 m | 聚类距离 |
| `min_cluster_size` | 3 | 最小点数 |
| `max_cluster_size` | 200 | 最大点数 |
| `max_cone_width` | 0.5 m | 锥桶宽度过滤 |
| `min_cone_height` | 0.1 m | 最小高度 |
| `max_cone_height` | 0.6 m | 最大高度 |

### 8.2 建图

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `merge_distance` | 0.5 m | 合并同一物理锥桶 |
| `min_hit_count` | 2 | 至少观测次数 |
| `loop_closure_distance` | 3.0 m | 回到起点距离阈值 |
| `min_cones_for_closure` | 10 | 闭环所需最小锥桶数 |
| `start_skip_distance` | 5.0 m | 行驶一定距离后才检查闭环 |
| `assign_colors` | true | 按左右自动分配蓝/黄 |
| `map_save_path` | `/tmp/wuta_cone_map.yaml` | 闭环后保存锥桶 YAML |

### 8.3 规划与控制

| 模块 | 参数 | 默认值 |
|---|---|---:|
| boundary_detector | `lookahead_distance` | 15.0 m |
| boundary_detector | `desired_velocity` | 7.0 m/s |
| path_generator | `trackdrive_velocity` | 7.0 m/s |
| path_generator | `skidpad_radius` | 9.125 m |
| path_generator | `skidpad_velocity` | 5.0 m/s |
| path_generator | `skidpad_points` | 72 |
| path_generator | `acceleration_length` | 75.0 m |
| path_generator | `acceleration_velocity` | 15.0 m/s |
| controller | `wheel_base` | 1.53 m |
| controller | `max_steer_angle` | 25.0 deg |
| controller | `ld_ratio` | 2.0 |
| controller | `min_lookahead` | 2.0 m |
| controller | `max_lookahead` | 20.0 m |
| controller | `control_rate_hz` | 50 Hz |

## 9. 仿真器实现建议

### 9.1 车辆动力学

将 `/control/command` 转成自行车模型输入：

```text
v_cmd = command.speed
delta = command.angle * pi / 180
L = 1.53

x_dot   = v * cos(yaw)
y_dot   = v * sin(yaw)
yaw_dot = v / L * tan(delta)
```

建议加入：

- 速度一阶滞后或加速度限制
- 转角速率限制
- 急停：`dv_state == 6` 或 `/system/emergency=true` 时立即制动

### 9.2 Trackdrive 环境

锥桶地图建议数据结构：

```text
left_boundary:  blue cones
right_boundary: yellow cones
start_finish:   orange cones
```

如果发布 `/mapping/cone_map`：

- 全部点使用 `map` 坐标
- 左右边界要与行驶方向一致
- `is_closed=false` 可保持 EXPLORE；`is_closed=true` 会触发 MAPPING_DONE

如果发布 `/perception/lidar/cones`：

- 根据车辆 pose 把全局锥桶转换到传感器坐标
- 只发布可见范围内的锥桶
- 增加遮挡、漏检和测量噪声更接近真实情况

### 9.3 Skidpad / Acceleration

这两种模式由 `path_generator_node` 自己根据当前 pose 生成路径，不依赖 `boundary_detector`：

- `MISSION_SKIDPAD`：生成左右两个半径 9.125 m 的圆，各两圈
- `MISSION_ACCELERATION`：沿当前航向生成 75 m 直线，最后 10 m 降速

仿真器只需保证：

- `/system/mission_state.mission_mode` 正确
- `/system/mission_state.state` 为 `EXPLORE` 或 `RACE`
- `/localization/pose` 已发布

## 10. 最小闭环测试流程

### 10.1 只测控制闭环

1. 启动 `controller_node`
2. 仿真器发布：
   - `/system/mission_state`: `state=3`，`mission_mode=0`，`localization_mode=0`
   - `/localization/pose`
   - `/localization/velocity`
   - `/planning/final_waypoints`
3. 仿真器订阅 `/control/command`
4. 用命令更新车辆模型，再回写 pose/velocity

验收：

- `/control/command.speed` 跟随路径点速度
- `/control/command.angle` 在 `[-25, 25]` deg 内
- 车辆能够沿路径收敛

### 10.2 测规划 + 控制

1. 启动 `boundary_detector_node`、`path_generator_node`、`controller_node`
2. 仿真器发布：
   - `/mapping/cone_map`
   - `/localization/pose`
   - `/localization/velocity`
   - `/system/mission_state`
3. 订阅：
   - `/planning/centerline`
   - `/planning/final_waypoints`
   - `/control/command`

验收：

- Trackdrive 下 `/planning/centerline` 有 waypoint
- `/planning/final_waypoints` 速度为 `trackdrive_velocity`
- 控制输出稳定

### 10.3 测建图 + 规划 + 控制

1. 启动 `cone_map_builder`、`boundary_detector_node`、`path_generator_node`、`controller_node`
2. 仿真器发布：
   - `/perception/lidar/cones`
   - `/tf`
   - `/localization/pose`
   - `/localization/velocity`
   - `/system/mission_state`
3. 订阅：
   - `/mapping/cone_map`
   - `/planning/final_waypoints`
   - `/control/command`

验收：

- `/mapping/cone_map` 中锥桶数量随行驶增长
- 行驶回起点后 `is_closed=true`
- 闭环后保存 `/tmp/wuta_cone_map.yaml`

## 11. 当前源码中的注意点

1. `camera_detection` 与 `detection_fusion` 目前只有包骨架，没有节点实现。
2. `DLDetector` 当前未实现，配置 `detector_type=dl` 会抛异常。
3. `mission_manager` 当前没有 start topic，READY 不会自动进入 EXPLORE，需要仿真器直接发布 mission_state 或补充启动接口。
4. `mission_manager` 在 `MAPPING_DONE` 后没有自动等待 `/ndt/map_ready` 并切入 `RACE`，源码标记为 TODO。
5. `map_saver` 当前注释说明暂时假设点云已在 map frame，TF 变换逻辑未完成。
6. 控制器订阅 `/localization/velocity`，但 `localization_manager` 当前只发布 `/localization/pose`，仿真器需要补足 velocity。
7. `autoware_msgs/msg/Command`、`Lane` 来自外部依赖；当前文档只列出源码实际读写字段。

## 12. 建议仿真器模块划分

```text
sim_world
  - 赛道/锥桶地图加载
  - 可见性、噪声、漏检模型

sim_vehicle
  - 订阅 /control/command
  - 自行车模型积分
  - 发布 /localization/pose 和 /localization/velocity

sim_sensors
  - Level A: 发布 /hesai/pandar
  - Level B: 发布 /perception/lidar/cones
  - Level C: 发布 /mapping/cone_map

sim_system
  - 发布 mission_state 或 ready/emergency/mode command
  - 管理 trackdrive/skidpad/acceleration 场景
```

## 13. 推荐首版仿真器目标

第一版建议实现 Level D + Level C：

1. 先闭环 `/planning/final_waypoints -> /control/command -> vehicle model -> /localization/pose`
2. 再发布 `/mapping/cone_map`，接入 `boundary_detector` 和 `path_generator`
3. 最后再做 `/perception/lidar/cones` 或 `/hesai/pandar`

这样可以最快确认 FSD 控制输出、车辆动力学和规划接口是否一致，再逐步增加传感器真实度。

## 14. 参考方案补充：不改 FSD，只替换传感器输入层

参考 `D:\track_loader\WUTA-FSD 仿真器开发方案.md`，推荐第一原则是：尽量不修改现有 FSD 算法节点，只用仿真器替换真实传感器和车辆反馈。

整栈闭环中仿真器主要负责 3 类输入：

| 虚拟传感器/反馈 | Topic | Type | 主要消费者 | 建议频率 |
|---|---|---|---|---:|
| Hesai 128 LiDAR | `/hesai/pandar` | `sensor_msgs/msg/PointCloud2` | `lidar_detection_node`、KISS-ICP、NDT | 10 Hz |
| CG-410 INS | `/cg410/odometry` | `nav_msgs/msg/Odometry` | `ekf_node` | 20 Hz |
| 车速/CAN 反馈 | `/localization/velocity` | `geometry_msgs/msg/TwistStamped` | `controller_node` | 50 Hz |

闭环顺序：

```text
仿真器发布传感器数据
  -> FSD 感知/定位/建图/规划/控制
  -> FSD 发布 /control/command
  -> 仿真器车辆模型积分更新 ground truth
  -> 生成下一帧传感器数据
```

## 15. 推荐新增 ROS2 包结构

建议在工作区内新增 Python 包，便于快速迭代，无需 C++ 编译：

```text
ros2_ws/src/simulation/wuta_simulator/
  package.xml
  setup.py
  config/
    simulator.yaml
  launch/
    simulator.launch.py
  wuta_simulator/
    __init__.py
    vehicle_model.py
    lidar_simulator.py
    ins_simulator.py
    can_simulator.py
    sim_mission_manager.py
  tracks/
    trackdrive.yaml
    skidpad.yaml
    acceleration.yaml
```

模块职责：

| 模块 | 输入 | 输出 | 说明 |
|---|---|---|---|
| `vehicle_model.py` | `/control/command` | `/sim/ground_truth`、可选 `/localization/pose` | 自行车运动学模型 |
| `lidar_simulator.py` | `/sim/ground_truth` + 赛道 YAML | `/hesai/pandar` | 直接读取 YAML，完成可见性判断、遮挡和点云生成 |
| `ins_simulator.py` | `/sim/ground_truth` | `/cg410/odometry` | 加噪 GNSS/INS 融合位姿 |
| `can_simulator.py` | `/sim/ground_truth` | `/localization/velocity` | 发布实际车速反馈 |
| `sim_mission_manager.py` | 配置/键盘/测试脚本 | `/system/mission_state` | 测试阶段可绕过现有 mission_manager |

## 16. 赛道 YAML 建议格式

建议与 `cone_map_builder` 保存的锥桶地图风格保持兼容，同时保留赛道类型和起点位姿：

```yaml
track:
  type: trackdrive          # trackdrive / skidpad / acceleration
  start_pose:
    x: 0.0
    y: 0.0
    yaw: 0.0                # rad
  blue_cones:
    - [1.5, 5.0, 0.0]
    - [2.0, 5.5, 0.0]
  yellow_cones:
    - [1.5, -5.0, 0.0]
  orange_cones:
    - [10.0, 0.0, 0.0]
```

手工生成规则：

- Trackdrive：两列锥桶间距建议 3-5 m，包含直道和弯道。
- Skidpad：两个半径 9.125 m 的圆，圆心距 18.25 m。
- Acceleration：75 m 直线赛道。
- `start_pose` 必须与车辆模型初始位姿严格一致，否则规划路径和车辆反馈会一开始就错位。

## 17. 各仿真模块实现细节

### 17.1 Vehicle Model

建议 50 Hz 积分，`dt=0.02 s`：

```text
x, y, yaw = start_pose
L = 1.53

steer = clamp(command.angle, -25 deg, 25 deg)
x     += speed * cos(yaw) * dt
y     += speed * sin(yaw) * dt
yaw   += speed * tan(steer) / L * dt
```

第一版可直接令 `speed = command.speed`。第二版建议加入加速度限制、转角速率限制和执行延迟。

### 17.2 LiDAR Simulator

简化点云即可满足当前 `traditional_detector` 的聚类需求，不必模拟完整 128 线扫描。

建议逻辑：

```text
对每个锥桶：
  1. 转到车体/LiDAR 坐标系
  2. 判断是否在视场内，例如 120 deg
  3. 判断距离范围，例如 1.5-50 m
  4. 可选遮挡检测
  5. 生成 12-16 个锥桶表面点，叠加高斯噪声 sigma=0.02 m

额外生成 200-500 个随机地面点，z 约为 LiDAR 高度下方
```

地面点很重要：当前 `lidar_detection_node` 的传统检测会做 RANSAC/高度阈值地面分割，没有地面输入时，检测效果和真实链路会有差异。

### 17.3 INS Simulator

输入 ground truth，输出 `/cg410/odometry`：

```text
position = gt.position + N(0, 0.05 m)
yaw      = gt.yaw + N(0, 0.5 deg)
```

如果要验证 EKF 漂移校正，可进一步加入低频 GNSS 噪声、短时漂移和偶发跳变。

### 17.4 CAN Simulator

输入 ground truth speed，输出 `/localization/velocity`：

```text
twist.linear.x = gt.speed
twist.linear.y = 0.0
```

真实车上该数据来自轮速/CAN；仿真中直接发布车辆模型实际速度即可。

## 18. A/B 两阶段运行模式

### 模式 A：跳过定位，优先验证规划控制

不启动 KISS-ICP + EKF，直接把 ground truth 位姿发布到 `/localization/pose`：

```text
/control/command -> VehicleModel -> /sim/ground_truth
                                  -> /localization/pose
                                  -> /localization/velocity

tracks/*.yaml -> LiDAR Simulator -> /hesai/pandar

FSD:
  lidar_detection -> cone_map_builder -> boundary_detector
  -> path_generator -> controller -> /control/command
```

优点：避开 KISS-ICP 在简化点云上可能不收敛的问题，最快验证感知、建图、规划、控制是否能跑通。

### 模式 B：完整定位链路

加入 KISS-ICP + EKF：

```text
/control/command -> VehicleModel -> /sim/ground_truth
                                  -> LiDAR Simulator -> /hesai/pandar
                                  -> INS Simulator   -> /cg410/odometry
                                  -> CAN Simulator   -> /localization/velocity

FSD:
  kiss_icp -> ekf -> localization_manager -> /localization/pose
  lidar_detection -> cone_map_builder -> boundary_detector
  -> path_generator -> controller
```

模式 B 更接近实车，但依赖点云质量、TF、EKF 配置和 KISS-ICP 收敛情况。建议在模式 A 稳定后再进入。

## 19. 仿真器内部与外部 topic 总表

| # | Topic | Type | 频率 | 发布者 | 订阅者 | 归属 |
|---:|---|---|---:|---|---|---|
| 1 | `/sim/ground_truth` | `nav_msgs/msg/Odometry` | 50 Hz | `vehicle_model` | `lidar_simulator`、`ins_simulator`、RViz | 仿真器内部 |
| 2 | `/hesai/pandar` | `sensor_msgs/msg/PointCloud2` | 10 Hz | `lidar_simulator` | FSD LiDAR/KISS/NDT 节点 | 仿真器 -> FSD |
| 3 | `/cg410/odometry` | `nav_msgs/msg/Odometry` | 20 Hz | `ins_simulator` | `ekf_node` | 仿真器 -> FSD |
| 4 | `/localization/velocity` | `geometry_msgs/msg/TwistStamped` | 50 Hz | `can_simulator` | `controller_node` | 仿真器 -> FSD |
| 5 | `/localization/pose` | `geometry_msgs/msg/PoseStamped` | 50 Hz | 模式 A: 仿真器；模式 B: FSD | 控制/规划/建图节点 | 混合 |
| 6 | `/control/command` | `autoware_msgs/msg/Command` | 50 Hz | `controller_node` | `vehicle_model` | FSD -> 仿真器 |
| 7 | `/system/mission_state` | `wuta_msgs/msg/MissionState` | 10-50 Hz | `mission_manager` 或 `sim_mission_manager` | FSD 全部关键节点 | 状态管理 |
| 8 | `/mapping/cone_map` | `wuta_msgs/msg/ConeMap` | 5 Hz | `cone_map_builder` | `mission_manager`、`boundary_detector` | FSD |
| 9 | `/sim/track_viz` | `visualization_msgs/msg/MarkerArray` | 1-5 Hz | `lidar_simulator`/可视化节点 | RViz | 仿真器调试 |

## 20. RViz2 可视化建议

RViz2 的 Fixed Frame 设为 `map`。

FSD 已有可视化：

| Topic | 内容 |
|---|---|
| `/mapping/cone_map_viz` | FSD 建出的锥桶地图 |
| `/planning/centerline_viz` | 规划中心线 |
| `/control/target_viz` | Pure Pursuit 目标点和前视圆 |

仿真器建议额外发布：

| Topic | 内容 |
|---|---|
| `/sim/ground_truth` | 真实车辆位姿，用于和定位结果比较 |
| `/sim/track_viz` | 真实赛道锥桶，用于和 FSD 建图结果比较 |
| `/sim/trajectory_viz` | 车辆历史轨迹，可选 |

## 21. 暑期加工计划与验收标准

参考 `D:\track_loader\暑期加工计划.md`，建议按两阶段推进。

### 阶段一：各组独立完成模块设计与验收

目标日期：7 月 13 日前。

分工：

| 小组 | 模块 |
|---|---|
| 感知组 | `lidar_simulator`（直接读取赛道 YAML） |
| 定位建图组 | `ins_simulator`、模式 B 定位链路对接 |
| 规控组 | `vehicle_model`、`can_simulator`、控制闭环 |

交付物：

- 每组 1-2 页模块设计文档
- 模块 workflow
- 输入/输出 topic 表
- 单模块可运行 demo

### 阶段二：合并代码，完成全链路仿真

目标日期：7 月 14 日-7 月 20 日。

集成重点：

- 各组统一坐标系和 `start_pose`
- 统一 topic 名称和频率
- 统一随机种子，保证可复现
- 感知组检查地面分割和锥桶检出率，避免漏检过高
- 建图组对比 FSD 建图结果与赛道 YAML 真值
- 规控组验证轨迹合理性，不蛇形、不切弯、不冲出赛道

## 22. 建议量化指标

### 22.1 全链路实时性

从 `/hesai/pandar.header.stamp` 到对应 `/control/command` 输出的端到端延迟：

```text
目标：总耗时 < 100 ms
```

实现建议：

- 在仿真器发布点云时记录 timestamp
- 在收到 `/control/command` 时计算差值
- 输出 `latency.csv`

### 22.2 可复现性

相同 random seed 连续运行两次：

```text
trajectory.csv 逐帧逐值一致
```

需要固定：

- Python `random.seed`
- NumPy seed
- 赛道 YAML
- 初始 pose
- 控制周期 dt

### 22.3 建图质量

对比 `/mapping/cone_map` 与赛道 YAML 真值：

| 指标 | 目标 |
|---|---:|
| 锥桶数量误差 | 尽量接近 0 |
| 平均位置误差 | 建议 < 0.3 m |
| 漏检率 | 按场景统计，越低越好 |
| 误检率 | 按场景统计，越低越好 |

### 22.4 规划控制质量

逐帧计算车辆到中心线的横向偏差：

| 指标 | 目标 |
|---|---:|
| 平均横向偏差 | < 0.3 m |
| 最大横向偏差 | < 1.0 m，弯道可适当放宽 |
| 弯道通过质量 | 弯道横向偏差 < 1.0 m |
| 弯后收敛 | 过弯后 5 m 内回到中心线附近 |

最终建议输出：

- `trajectory.csv`
- `latency.csv`
- `mapping_error.csv`
- RViz 截图或轨迹可视化图片
- 一份整体仿真器可行性分析
