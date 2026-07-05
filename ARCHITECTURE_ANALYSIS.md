# WUTA-FSD 整体架构分析

> 武汉理工大学无人驾驶方程式赛车自动驾驶算法栈
> 分支 `wuta0318`：适配地平线征程 J6 域控制器的 ROS2 Humble 重构版本

---

## 一、项目总览

### 1.1 硬件平台

| 设备 | 型号 | 关键参数 |
|------|------|----------|
| 域控制器 | 地平线征程 J6 | 128 TOPS BPU，CPU 137K DMIPS |
| 激光雷达 | 禾赛 128线 | 替换原 Velodyne |
| 组合导航 | 华测 CG-410 | GNSS + IMU 内部融合 |
| 相机 | 待定 | 接口已预留 |

### 1.2 技术选型

| 模块 | 方案 | 说明 |
|------|------|------|
| 中间层 | ROS2 Humble | J6 不支持 ROS1 |
| 探索定位 | KISS-ICP + EKF (robot_localization) | 无 IMU 依赖 |
| 竞速定位 | NDT 地图匹配 | 高精度高速循迹 |
| 锥桶检测 | PCL 传统聚类 (DL 接口预留) | 可切换至 PointPillars/CenterPoint |
| 路径规划 | Delaunay 三角剖分 (参考 HRT-D) | 替代原 Voronoi |
| 控制 | Pure Pursuit (参考 HRT-D Control) | 无 HPL 私有框架依赖 |
| 消息格式 | autoware_msgs + 自定义 wuta_msgs | 生态兼容 + 定制需求 |

---

## 二、系统分层架构

```
┌──────────────────────────────────────────────────────────────────┐
│                     【系统管理层】                                  │
│                  mission_manager (状态机)                          │
│     IDLE → READY → INSPECTION → EXPLORE → MAPPING_DONE →         │
│                          RACE → FINISH                            │
│                      ↕ EMERGENCY (任意状态可触发)                   │
└──────┬────────────────┬──────────────────┬───────────────────────┘
       │                │                  │
       ▼                ▼                  ▼
┌──────────────┐ ┌──────────────┐ ┌──────────────────┐
│  【感知层】    │ │  【定位层】    │ │    【建图层】       │
│              │ │              │ │                  │
│ lidar_       │ │ kiss_icp +   │ │ cone_map_        │
│ detection    │ │ EKF (EXPLORE)│ │ builder          │
│ (PCL/DL双后端)│ │              │ │ (TF变换+去重合并   │
│              │ │ NDT 地图匹配   │ │  +闭环检测)       │
│ camera_      │ │ (RACE)       │ │                  │
│ detection(预留)│ │              │ │                  │
│              │ │ localization_│ │                  │
│ detection_   │ │ manager      │ │                  │
│ fusion(预留)  │ │ (双模式切换)   │ │                  │
└──────┬───────┘ └──────┬───────┘ └────────┬─────────┘
       │                │                  │
       │   ConeArray    │  /localization/  │  ConeMap
       │                │     pose         │
       └────────────────┼──────────────────┘
                        │
                        ▼
              ┌─────────────────┐
              │   【规划层】      │
              │                 │
              │ boundary_       │
              │ detector        │
              │ (Delaunay中心线) │
              │        ↓        │
              │ path_generator  │
              │ (三模式路径生成)  │
              └────────┬────────┘
                       │
                       ▼
              ┌─────────────────┐
              │   【控制层】      │
              │                 │
              │ controller      │
              │ (Pure Pursuit   │
              │  + TwistFilter) │
              │        ↓        │
              │ /control/command│
              │   → VCU (CAN)   │
              └─────────────────┘
```

---

## 三、自定义消息定义 (wuta_msgs)

### 3.1 Cone.msg — 单个锥桶检测结果
```
几何: geometry_msgs/Point position  # 传感器/地图坐标系下的3D位置
分类: uint8 color                   # COLOR_UNKNOWN=0, BLUE=1, YELLOW=2, ORANGE=3
置信: float32 confidence            # [0.0, 1.0]
```

### 3.2 ConeArray.msg — 单帧锥桶检测列表
```
头帧: std_msgs/Header header        # frame_id 指示坐标系(sensor/map)
列表: Cone[] cones
```
- 由 LiDAR 检测节点或相机检测节点发布
- 每帧点云/图像产生一帧 ConeArray

### 3.3 ConeMap.msg — 全局锥桶地图
```
头帧: std_msgs/Header header        # frame_id: "map"
按颜色分组:
  blue_cones[]    — 左侧边界 (蓝色锥桶)
  yellow_cones[]  — 右侧边界 (黄色锥桶)
  orange_cones[]  — 起止线 (橙色锥桶)
  unknown_cones[] — 未分类锥桶
闭环标志: bool is_closed            # true 表示赛道已闭环
```
- 由 cone_map_builder 发布，供规划和建图模块使用

### 3.4 MissionState.msg — 系统任务状态
```
状态常量:
  IDLE=0 → READY=1 → INSPECTION=2 → EXPLORE=3 → MAPPING_DONE=4 → RACE=5 → FINISH=6
  EMERGENCY=7 (任意时刻可触发)

任务模式常量: TRACKDRIVE=0, SKIDPAD=1, ACCELERATION=2
定位模式常量: LOC_KISS_ICP=0, LOC_NDT=1

消息字段: state | mission_mode | localization_mode | description
```
- 由 mission_manager 以 10Hz 频率广播
- 所有模块通过此消息获知当前系统阶段

---

## 四、完整话题 (Topic) 拓扑图

```
                        ┌──────────────────────────────────────────────┐
                        │              系统管理话题                      │
                        │                                              │
                        │  /system/mission_state  ← MissionManager     │
                        │  /system/emergency      → MissionManager     │
                        │  /system/mission_mode_cmd → MissionManager   │
                        │  /system/lidar_ready     → MissionManager    │
                        │  /system/localization_ready → MissionManager │
                        │  /system/inspection_trigger → MissionManager │
                        │  /system/inspection_result ← MissionManager  │
                        └──────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                              数据流拓扑                                       │
│                                                                             │
│  禾赛128线                                                                    │
│  /hesai/pandar (PointCloud2)                                                │
│      │                                                                      │
│      ├──→ lidar_detection_node                                              │
│      │       │                                                              │
│      │       └──→ /perception/lidar/cones (ConeArray)                       │
│      │                │                                                     │
│      │                └──→ cone_map_builder ──→ /mapping/cone_map (ConeMap) │
│      │                                                     │                │
│      ├──→ kiss_icp_node ──→ /kiss/odometry (Odometry)     │                │
│      │       │                     │                       │                │
│      │       │              ┌──────┘                       │                │
│      │       │              ▼                              │                │
│      │       │       ekf_node (robot_localization)         │                │
│      │       │              │                              │                │
│      │       │              ▼                              │                │
│      │       │       /odometry/filtered (Odometry)         │                │
│      │       │              │                              │                │
│      │       │              ▼                              │                │
│      │       │       localization_manager                  │                │
│      │       │              │                              │                │
│      │       │              └──→ /localization/pose (PoseStamped)           │
│      │       │                          │                  │                │
│      │       │              ┌───────────┴──────────┐       │                │
│      │       │              │                      │       │                │
│      │       │              ▼                      ▼       ▼                │
│      │       │       boundary_detector    path_generator   mission_manager  │
│      │       │              │                      │                        │
│      │       │              ▼                      │                        │
│      │       │       /planning/centerline (Lane)   │                        │
│      │       │              │                      │                        │
│      │       │              └──────────────────────┤                        │
│      │       │                                     ▼                        │
│      │       │       /planning/final_waypoints (Lane)                       │
│      │       │                              │                               │
│      │       │                              ▼                               │
│      │       │                       controller_node                        │
│      │       │                              │                               │
│      │       │                              ▼                               │
│      │       │                       /control/command (Command)             │
│      │       │                              │                               │
│      │       │                              └──→ VCU (CAN总线)               │
│      │       │                                                              │
│      ├──→ ndt_localization ──→ /ndt/pose (PoseStamped)                     │
│      │       │                         │                                    │
│      │       │                         └──→ localization_manager (RACE模式)  │
│      │       │                                                              │
│      └──→ map_saver (EXPLORE阶段积累点云 → 保存PCD地图)                      │
│                                                                             │
│  定位速度:  /localization/velocity (TwistStamped) ──→ controller_node       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 话题速查表

| 话题名 | 消息类型 | 流向 | 说明 |
|--------|----------|------|------|
| `/hesai/pandar` | `PointCloud2` | 传感器→多节点 | 禾赛128线原始点云 |
| `/perception/lidar/cones` | `ConeArray` | LiDAR检测→建图 | LiDAR锥桶检测结果 |
| `/perception/lidar/cones_viz` | `MarkerArray` | LiDAR检测→RViz | 检测可视化 |
| `/kiss/odometry` | `Odometry` | KISS-ICP→EKF | 激光里程计 |
| `/odometry/filtered` | `Odometry` | EKF→定位管理器 | EKF融合后里程计 |
| `/localization/pose` | `PoseStamped` | 定位管理器→规划/控制/建图 | **统一位姿输出** |
| `/localization/velocity` | `TwistStamped` | 定位→控制 | 车辆速度 |
| `/ndt/pose` | `PoseStamped` | NDT→定位管理器 | NDT匹配位姿(RACE模式) |
| `/ndt/path` | `Path` | NDT→RViz | NDT定位轨迹 |
| `/ndt/map_ready` | `Bool` | map_saver→系统 | NDT地图就绪信号 |
| `/mapping/cone_map` | `ConeMap` | 建图→规划/任务管理 | 全局锥桶地图 |
| `/mapping/cone_map_viz` | `MarkerArray` | 建图→RViz | 锥桶地图可视化 |
| `/planning/centerline` | `Lane` | 边界检测→路径生成 | Delaunay中心线 |
| `/planning/centerline_viz` | `MarkerArray` | 边界检测→RViz | 中心线可视化 |
| `/planning/final_waypoints` | `Lane` | 路径生成→控制 | **最终参考路径** |
| `/control/command` | `Command` | 控制→VCU | **最终控制指令** |
| `/control/target_viz` | `MarkerArray` | 控制→RViz | 目标点+前视圈可视化 |
| `/system/mission_state` | `MissionState` | 任务管理→所有模块 | **全局状态广播(10Hz)** |
| `/system/emergency` | `Bool` | 任意→任务管理 | 紧急停机触发 |
| `/system/mission_mode_cmd` | `String` | 操作者→任务管理 | 运行时模式切换命令 |
| `/system/lidar_ready` | `Bool` | LiDAR→任务管理 | LiDAR就绪状态 |
| `/system/localization_ready` | `Bool` | 定位管理器→任务管理 | 定位就绪状态 |
| `/system/inspection_trigger` | `Bool` | 操作者→任务管理 | 车检触发(预留) |
| `/system/inspection_result` | `String` | 任务管理→操作者 | 车检结果(预留) |

---

## 五、节点详细分析

### 5.1 感知层 — lidar_detection

```
┌─────────────────────────────────────────┐
│         LidarDetectionNode               │
│                                          │
│  参数: detector_type ("traditional"/"dl") │
│                                          │
│  ┌─────────────────────────────────┐    │
│  │  IDetector (抽象接口)              │    │
│  │    detect(PointCloud) → ConeArray│    │
│  └──────────┬──────────────────────┘    │
│             │                            │
│     ┌───────┴────────┐                  │
│     ▼                ▼                   │
│  TraditionalDetector  DLDetector        │
│  ┌──────────────┐   ┌──────────────┐    │
│  │1.范围滤波     │   │ 模型推理       │    │
│  │2.地面去除     │   │ (ONNX/BPU/   │    │
│  │  (RANSAC/阈值)│   │  TensorRT)   │    │
│  │3.体素下采样   │   │              │    │
│  │4.欧氏聚类     │   │ [待实现]      │    │
│  │5.锥桶形状筛选 │   │              │    │
│  └──────────────┘   └──────────────┘    │
└─────────────────────────────────────────┘
```

**设计要点:**
- **策略模式**: `IDetector` 抽象接口，运行时通过参数 `detector_type` 切换后端
- **传统PCL管线**: 范围滤波 → 地面去除(RANSAC平面拟合/高度阈值) → 体素下采样 → 欧氏聚类 → 尺寸筛选
- **DL接口预留**: `DLDetector` 类已定义，支持 PointPillars/CenterPoint，待模型就绪后填充 `detect()` 实现
- **颜色分配后移**: 检测阶段不区分颜色(COLOR_UNKNOWN)，颜色由下游 cone_map_builder 根据左右位置分配

### 5.2 定位层 — 双模式定位

```
        EXPLORE (探索圈)              RACE (竞速圈)
   ┌─────────────────────┐    ┌──────────────────────┐
   │  /hesai/pandar       │    │  /hesai/pandar        │
   │      │               │    │      │                │
   │      ▼               │    │      ▼                │
   │  kiss_icp_node       │    │  ndt_localization     │
   │  (KISS-ICP里程计)     │    │  (PCL NDT配准)        │
   │      │               │    │      │                │
   │      ▼               │    │      │                │
   │  /kiss/odometry      │    │  /ndt/pose            │
   │      │               │    │      │                │
   │      ▼               │    │      │                │
   │  ekf_node            │    │      │                │
   │  (robot_localization)│    │      │                │
   │  + CG-410 GNSS       │    │      │                │
   │      │               │    │      │                │
   │      ▼               │    │      │                │
   │  /odometry/filtered  │    │      │                │
   └──────────┬───────────┘    └───────┬──────────────┘
              │                        │
              └────────┬───────────────┘
                       ▼
            ┌─────────────────────┐
            │ localization_manager │
            │                     │
            │ 根据 mission_state   │
            │ .localization_mode  │
            │ 选择输出源并统一发布  │
            │                     │
            │ → /localization/pose│
            └─────────────────────┘
```

**定位管理器核心逻辑:**
1. 订阅 `/system/mission_state`，检测 `localization_mode` 字段
2. EXPLORE 阶段 (`LOC_KISS_ICP`): 转发 `/odometry/filtered` → `/localization/pose`
3. RACE 阶段 (`LOC_NDT`): 转发 `/ndt/pose` → `/localization/pose`
4. 模式切换由 `mission_manager` 自动控制 (进入RACE自动切NDT，进入EXPLORE自动切KISS-ICP)
5. 对外统一接口: `/localization/pose`，规划和控制层无需感知底层切换

**NDT定位工作流程:**
1. 启动时等待 RACE 模式激活
2. 激活后从磁盘加载 PCD 地图 (`map_saver` 在 EXPLORE 阶段保存)
3. 接收 `/initialpose` 作为初始匹配位姿
4. 每帧点云: ROS→PCL转换 → 体素降采样 → NDT配准 → 收敛检查 → 发布位姿
5. 同时发布对齐点云 (`/ndt/aligned_cloud`) 和轨迹历史 (`/ndt/path`)

### 5.3 建图层 — cone_map_builder

```
┌────────────────────────────────────────────────┐
│              ConeMapBuilder                     │
│                                                 │
│  输入: /perception/lidar/cones (ConeArray)       │
│        /localization/pose (PoseStamped)          │
│                                                 │
│  处理流程:                                       │
│  1. TF2: sensor_frame → map_frame 坐标变换       │
│  2. 锥桶去重: 新检测与已有锥桶做距离匹配           │
│     - 距离 < merge_distance(0.5m) → 融合(滑动平均) │
│     - hit_count 作为置信度代理                    │
│  3. 颜色分配: 根据车头方向+叉积判断左蓝右黄        │
│  4. 闭环检测:                                     │
│     - 条件: 已确认锥桶数 ≥ min_cones_for_closure  │
│     - 条件: 已行驶距离 ≥ start_skip_distance      │
│     - 条件: 当前位置距起点 < loop_closure_distance │
│     - 触发: 标记 loop_closed_，保存YAML地图        │
│  5. 发布 ConeMap (5Hz) + 可视化                    │
│                                                 │
│  输出: /mapping/cone_map (ConeMap)               │
│        /mapping/cone_map_viz (MarkerArray)       │
└────────────────────────────────────────────────┘
```

**关键设计:**
- 多线程执行器: 位姿回调和锥桶回调使用独立的 CallbackGroup，避免高频位姿被慢速锥桶处理阻塞
- 闭环后锁定: `loop_closed_=true` 后停止更新，地图不再变化
- YAML持久化: 闭环时自动保存地图到 `map_save_path_`，供后续比赛复用

### 5.4 规划层 — boundary_detector + path_generator

```
┌──────────────────────────────────────────────────────┐
│                 规划层双节点结构                        │
│                                                      │
│  ┌─────────────────────┐   ┌─────────────────────┐   │
│  │ boundary_detector    │   │  path_generator      │   │
│  │                     │   │                     │   │
│  │ 输入: ConeMap + pose │   │ 三模式:              │   │
│  │                     │   │                     │   │
│  │ 处理:                │   │ TRACKDRIVE:         │   │
│  │ 1.锥桶→Point2d点集   │   │  转发centerline     │   │
│  │ 2.Delaunay三角剖分   │──→│  设置目标速度        │   │
│  │ 3.搜索中轴中点序列   │   │                     │   │
│  │ 4.→ centerline (Lane)│   │ SKIDPAD:            │   │
│  │                     │   │  生成8字路径          │   │
│  │ 输出:                 │   │  (双圆r=9.125m)      │   │
│  │ /planning/centerline │   │                     │   │
│  └─────────────────────┘   │ ACCELERATION:        │   │
│                            │  生成直线加速路径     │   │
│                            │  (75m,末尾10m减速)    │   │
│                            │                     │   │
│                            │ 输出:                │   │
│                            │ /planning/           │   │
│                            │   final_waypoints    │   │
│                            └─────────────────────┘   │
└──────────────────────────────────────────────────────┘
```

**边界检测 (boundary_detector):**
- 仅在 TRACKDRIVE 模式下激活
- 从 ConeMap 中提取前视范围内的蓝/黄锥桶，转换为 Point2d 点集
- 调用 HRT-D pathplanning 库进行 Delaunay 三角剖分
- 搜索左右锥桶之间的中点序列作为赛道中心线
- 使用上一帧的中点序列作为当前帧搜索的初始值(时序连续性)

**路径生成 (path_generator):**
- 三种比赛模式:
  - **Trackdrive**: 透传 boundary_detector 的 centerline，统一设置目标速度
  - **Skidpad**: 根据当前位姿生成 FSG 标准 8 字路径(右圆 2 圈 + 左圆 2 圈，半径 9.125m)
  - **Acceleration**: 沿当前航向生成 75m 直线路径(末尾 10m 线性减速至 0)
- 统一输出到 `/planning/final_waypoints`

### 5.5 控制层 — controller

```
┌──────────────────────────────────────────────────────┐
│                 ControllerNode                        │
│                                                      │
│  输入: /localization/pose (PoseStamped)               │
│        /localization/velocity (TwistStamped)          │
│        /planning/final_waypoints (Lane)               │
│        /system/mission_state (MissionState)           │
│                                                      │
│  ┌─────────────────────────────────────────────┐     │
│  │ controlLoop() @ 50Hz                         │     │
│  │                                              │     │
│  │  [VehicleState]                              │     │
│  │   x, y, yaw, velocity                        │     │
│  │        │                                     │     │
│  │        ▼                                     │     │
│  │  ┌──────────────┐                            │     │
│  │  │ PurePursuit   │                            │     │
│  │  │              │                            │     │
│  │  │ 1.计算前视距离 │  lookahead = clamp(        │     │
│  │  │              │    |v|*ld_ratio,            │     │
│  │  │              │    min_lookahead,           │     │
│  │  │              │    max_lookahead)           │     │
│  │  │              │                            │     │
│  │  │ 2.找目标点    │  从路径尾向前扫描，第一个     │     │
│  │  │              │  在前视距离内的点             │     │
│  │  │              │                            │     │
│  │  │ 3.横向偏移    │  x_body = 车体坐标系横向偏移   │     │
│  │  │              │                            │     │
│  │  │ 4.曲率→转角   │  δ = atan(WB * κ)         │     │
│  │  │              │  κ = 2*x_body / dist²      │     │
│  │  │              │                            │     │
│  │  │ 5.速度参考    │  直接取路径点的速度属性       │     │
│  │  └──────┬───────┘                            │     │
│  │         │ raw_angle, raw_velocity            │     │
│  │         ▼                                    │     │
│  │  ┌──────────────┐                            │     │
│  │  │ TwistFilter   │                            │     │
│  │  │              │                            │     │
│  │  │ 紧急: 速度=0  │                            │     │
│  │  │ 加速: 0.9旧+  │  缓慢上升，防打滑            │     │
│  │  │      0.1新   │                            │     │
│  │  │ 减速: 0.3旧+  │  快速响应，保安全            │     │
│  │  │      0.7新   │                            │     │
│  │  │ 转向限幅:     │  clamp(±max_steer_angle)   │     │
│  │  └──────┬───────┘                            │     │
│  │         │ filtered_angle, filtered_velocity   │     │
│  └──────────┼──────────────────────────────────┘     │
│             ▼                                        │
│  输出: /control/command (Command)                     │
│        /control/target_viz (MarkerArray)              │
└──────────────────────────────────────────────────────┘
```

**Pure Pursuit 算法核心:**
- 前视距离自适应: `lookahead = clamp(|v| × ld_ratio, 2.0m, 20.0m)`
- 目标点选择: 从路径尾向前扫描，第一个进入前视圆的点
- 横向偏移计算: 目标点变换到车体坐标系，`x_body` 即横向偏差
- 曲率公式: `κ = 2 * x_body / dist²` (纯几何推导)
- 转向角换算: `δ = atan(wheel_base * κ)` (自行车模型)

**TwistFilter 安全保护:**
- 紧急状态: 直接输出零速零转角
- 加速平滑: EMA 系数 0.1 (慢加速防打滑)
- 减速快速: EMA 系数 0.7 (快速响应保安全)
- 转角限幅: `±max_steer_angle` (默认±25°)

### 5.6 系统管理层 — mission_manager

```
              ┌──────────────────────────────────┐
              │        MissionManager             │
              │                                   │
              │  状态机转换图:                      │
              │                                   │
              │  IDLE ──────────────────────────┐ │
              │   │ 传感器就绪                    │ │
              │   ▼                              │ │
              │  READY ──────→ INSPECTION ──→ READY  │
              │   │ 开始探索      ↑ 车检完成       │ │
              │   ▼                              │ │
              │  EXPLORE ──→ MAPPING_DONE        │ │
              │   │ KISS-ICP    │ 地图闭环        │ │
              │   │ +建图       ▼                │ │
              │   │          RACE                │ │
              │   │           │ NDT定位           │ │
              │   │           ▼                  │ │
              │   │          FINISH              │ │
              │   │                              │ │
              │   └──── EMERGENCY ←─────────────┘ │
              │           紧急停机(任意状态可触发)    │
              │                                   │
              │  输出: /system/mission_state (10Hz) │
              └──────────────────────────────────┘
```

**状态迁移触发条件:**
| 迁移 | 触发条件 |
|------|----------|
| IDLE → READY | LiDAR就绪 AND 定位就绪 |
| READY → EXPLORE | 手动触发(或自动) |
| EXPLORE → MAPPING_DONE | cone_map.is_closed == true |
| MAPPING_DONE → RACE | NDT地图就绪后自动切换 |
| RACE → FINISH | 完成比赛(手动/检测) |
| ANY → EMERGENCY | `/system/emergency` 收到 true |

**定位模式自动切换:**
- 进入 EXPLORE: `localization_mode = LOC_KISS_ICP`
- 进入 RACE: `localization_mode = LOC_NDT`

---

## 六、launch 文件结构

```
localization.launch.py
├── DeclareLaunchArgument: pointcloud_topic (default: /hesai/pandar)
├── Node: kiss_icp_node (KISS-ICP激光里程计)
│   └── config: kiss_icp_hesai128.yaml (禾赛128线参数)
├── Node: ekf_node (robot_localization EKF融合)
│   └── config: ekf.yaml (EKF参数)
└── Node: localization_manager (双模式切换器)
```

---

## 七、第三方依赖库

### 7.1 HRT-D pathplanning (Delaunay 算法库)
```
boundary_detector/thirdparty/pathplanning/
├── BowyerWatson.cpp    — Bowyer-Watson 增量Delaunay算法
├── DelaunayTriangle.cpp — Delaunay 三角形数据结构
├── Divide.cpp          — 区域划分
├── Evaluation.cpp      — 路径代价评估
├── GlobalVariables.cpp — 全局变量
├── Line.cpp            — 线段几何
├── MidPoint.cpp        — 中点(左右锥桶对的中点)
├── Path.cpp            — 路径数据结构
├── PathSearch.cpp      — Delaunay图中的路径搜索
├── Point2d.cpp         — 2D点
├── Tool.cpp            — 工具函数
├── Triangle.cpp        — 三角形几何
└── Vect.cpp            — 向量运算
```
纯C++实现，无ROS依赖，通过 ROS2 wrapper (boundary_detector_node) 接入系统。

### 7.2 外部 ROS2 包
| 包 | 用途 |
|----|------|
| `kiss-icp` (submodule) | 激光里程计 |
| `robot_localization` (submodule) | EKF/UKF 多传感器融合 |
| `pcl_ros` | PCL 点云库 ROS2 桥接 |
| `tf2_ros` | 坐标变换 |
| `autoware_msgs` | Lane, Waypoint, Command 消息类型 |

---

## 八、代码注释状态评估

项目中所有源文件已包含较为详细的中文注释。以下是注释覆盖情况：

| 文件 | 注释质量 | 说明 |
|------|---------|------|
| 所有 `.msg` 文件 | ✅ 完善 | 每个字段都有中文说明 |
| 所有 `.hpp` 头文件 | ✅ 完善 | 类/结构体/函数都有 Doxygen 风格中文注释 |
| `controller_node.cpp` | ✅ 完善 | 逐行中文注释，逻辑清晰 |
| `pure_pursuit.cpp` | ✅ 完善 | 算法每一步都有说明 |
| `twist_filter.cpp` | ✅ 完善 | 滤波器逻辑完整注释 |
| `localization_manager.cpp` | ✅ 完善 | 模式切换逻辑清晰 |
| `ndt_localization.cpp` | ✅ 完善 | NDT配准流程完整注释 |
| `map_saver.cpp` | ✅ 完善 | 点云积累和保存逻辑清晰 |
| `cone_map_builder.cpp` | ✅ 完善 | 建图全流程完整注释 |
| `boundary_detector_node.cpp` | ✅ 良好 | Delaunay调用流程清晰 |
| `path_generator_node.cpp` | ✅ 良好 | 三模式路径生成清晰 |
| `mission_manager.cpp` | ✅ 完善 | 状态机逻辑完整注释 |
| `lidar_detection_node.cpp` | ✅ 良好 | 检测器调度逻辑清晰 |
| `traditional_detector.cpp` | ✅ 完善 | 传统PCL管线每步注释 |
| `dl_detector.cpp` | ✅ 良好 | TODO标记清晰，待实现 |
| `detector_base.hpp` | ✅ 完善 | 抽象接口设计意图明确 |

---

## 九、关键设计原则总结

1. **感知解耦**: LiDAR 和 Camera backbone 完全独立节点，通过抽象接口支持多后端切换
2. **定位双模式**: EXPLORE(KISS-ICP+EKF) 和 RACE(NDT) 分离，localization_manager 统一对外接口
3. **无私有框架依赖**: Control 层直接使用 rclcpp，不引入 HPL 等私有框架
4. **状态机驱动**: MissionManager 作为全局状态总线，所有模块根据状态决定行为
5. **地图持久化**: 探索圈结束后保存锥桶地图(YAML)和点云地图(PCD)，供后续复用
6. **统一时间戳**: 所有节点使用同一时钟源，避免 SLAM 融合时序问题
7. **策略模式**: 检测器和路径生成均采用策略模式，便于切换和扩展
8. **安全滤波**: TwistFilter 在控制输出前做平滑和限幅，防止激进控制

---

## 十、待完成工作

| 优先级 | 模块 | 内容 |
|--------|------|------|
| 🔴 高 | ndt_localization | 实车标定 NDT 参数 (resolution, step_size) |
| 🔴 高 | controller | `/control/command` → VCU CAN 帧格式对接 |
| 🔴 高 | 全系统 | LiDAR → base_link 外参标定 |
| 🟡 中 | lidar_detection | DL 后端实现 (PointPillars/CenterPoint) |
| 🟡 中 | camera_detection | 相机型号确认后实现检测节点 |
| 🟡 中 | detection_fusion | LiDAR+Camera 融合策略 |
| 🟡 中 | mission_manager | INSPECTION 车检流程实现 |
| 🟢 低 | map_saver | TF2 坐标系变换完善 |
| 🟢 低 | 全系统 | CG-410 驱动 topic 名确认和 ekf.yaml 更新 |
