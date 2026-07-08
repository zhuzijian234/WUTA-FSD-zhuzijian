# WUTA\-FSD 仿真器开发方案

## 目标

开发一个轻量级 ROS 2 仿真器，在不依赖任何硬件的情况下，闭环验证完整算法链路。

## 核心思路：不修改现有代码，只替换传感器输入层

现有管线的传感器边界是 3 个话题。仿真器只需发布这 3 个话题，整条 FSD 管线无需任何改动即可运行。

```Plain Text

  传感器: Hesai 128 LiDAR
  话题: /hesai/pandar
  消息类型: PointCloud2
  消费者: kiss_icp_node、lidar_detection_node、ndt_localization_node
  说明: 三个节点共用一个话题
  ────────────────────────────────────────
  传感器: CG-410 INS
  话题: /cg410/odometry
  消息类型: Odometry
  消费者: ekf_node
  说明: GNSS RTK + IMU 融合位姿，校正 KISS-ICP 漂移
  ────────────────────────────────────────
  传感器: 车速（CAN）
  话题: /localization/velocity  
  消息类型: TwistStamped
  消费者: controller_node
  说明: 车辆当前速度，Pure Pursuit 计算前视距离用
  
  
                    仿真器                          现有 FSD 管线
            ┌──────────────────┐          ┌──────────────────────────┐
            │  Track Loader    │          │                          │
            │  (cone_map.yaml) │          │  lidar_detection ──┐     │
            │        │         │          │       │            │     │
            │        ▼         │    ┌─────┤  cone_map_builder  │     │
            │  LiDAR Sim  ─────┼───→│     │       │            │     │
            │  (射线投射)       │    │     │  boundary_detector │     │
            │        │         │    │     │       │            │     │
            │        ▼         │    │     │  path_generator    │     │
            │  Vehicle Model  ←┼────┤     │       │            │     │
            │  (自行车模型)     │    │     │  controller ──────┼──┐  │
            │   │   │   │      │    │     └───────────────────┘  │  │
            └───┼───┼───┼──────┘    └───────────────────────────┼──┘
                │   │   │                                         │
        /cg410  │   │   /localization/velocity                    │
        /odometry   │                                             │
                    │                                             │
            /hesai/pandar                                  /control/command
                                                              (闭环!)
```

闭环流程：仿真器发布传感器数据 → FSD 管线处理 → 输出 /control/command → 仿真器用控制指令更新车辆位置 → 新的传感器数据 → \.\.\.

---

## 架构设计

新建 1 个包，所有脚本用 Python 3 实现（快速开发，不编译）。

```Plain Text
src/simulation/wuta_simulator/
├── package.xml
├── setup.py
├── config/
│   └── simulator.yaml
├── launch/
│   └── simulator.launch.py     # 一键启动仿真器 + FSD 管线
├── wuta_simulator/
│   ├── __init__.py
│   ├── vehicle_model.py        # 自行车运动学模型
│   ├── lidar_simulator.py      # 直接读取赛道 YAML，射线投射模拟 LiDAR
│   ├── ins_simulator.py        # 模拟 CG-410 INS
│   └── can_simulator.py        # 模拟 CAN 车速上报
└── tracks/
    ├── trackdrive.yaml         # 赛道追逐赛道定义
    ├── skidpad.yaml            # 八字绕桩赛道定义
    └── acceleration.yaml       # 直线加速赛道定义
```

---

## 各模块详细设计

### 赛道加载器

输入：YAML 赛道文件

输出：锥筒全局坐标列表（blue\_cones, yellow\_cones, orange\_cones）

YAML 格式（与 cone\_map\_builder 保存的格式兼容）：

```Plain Text
track:
  type: trackdrive         # trackdrive / skidpad / acceleration
  start_pose:
    x: 0.0
    y: 0.0
    yaw: 0.0               # radians
  blue_cones:
    - [1.5, 5.0, 0.0]
    - [2.0, 5.5, 0.0]
  yellow_cones:
    - [1.5, -5.0, 0.0]
  orange_cones:
    - [10.0, 0.0, 0.0]
```

手动生成赛道规则：

- Trackdrive：按 FSG 规则，两列锥筒间距 3\-5m，直道 \+ 弯道

- Skidpad：两个圆（半径 9\.125m），圆心距 18\.25m

- Acceleration：直线 75m

后期可加 RViz 交互工具点选锥筒位置生成 YAML。

### 车辆运动学模型

- 输入：/control/command \(speed m/s, angle degrees\)

- 输出：ground truth 位姿

- 频率：50Hz

- start\_pose必须严格对齐赛道yaml文件的start\_pose

```Plain Text
x, y, yaw = start_pose
wheelbase = 1.53  # m, dt = 0.02s

steer = clamp(command.angle, -25°, 25°)
x     += command.speed * cos(yaw) * dt
y     += command.speed * sin(yaw) * dt
yaw   += command.speed * tan(steer) / wheelbase * dt
```

### LiDAR 模拟器

- 输入：车辆 ground truth \+ 赛道锥筒坐标

- 输出：/hesai/pandar \(PointCloud2\)

- 频率：10Hz

核心算法：

```Plain Text
对每个锥筒：
  → 是否在 LiDAR 120° 视场内
  → 是否在有效距离内（1.5m ~ 50m）
  → 是否有遮挡
  → 生成 12-16 个点模拟锥筒表面反射（加高斯噪声 σ=0.02m）

生成 200-500 个随机地面点（LiDAR z=-1.0 附近）
  → 让 RANSAC 地面分割有输入可处理
```

不模拟 128 线扫描，不加载动态物体。10Hz × 几百个点，Python 完全够用。

### INS 模拟器

- 输入：车辆 ground truth

- 输出：/cg410/odometry \(Odometry\)

- 频率：20Hz

```Plain Text
odom.position = gt.position + noise(σ=0.05m)
odom.orientation = yaw_to_quat(gt.yaw + noise(σ=0.5°))
```

### CAN 模拟器

- 输入：车辆 ground truth speed

- 输出：/localization/velocity \(TwistStamped\)

- 频率：50Hz

- 发布实际车速，供纯跟踪算法计算前视距离。真车上，轮速传感器测得实际车速 → CAN 总线 → CAN 驱动 → /localization/velocity，这里直接采用vehicle\_model发布的仿真实际车速。

```Plain Text
twist.linear.x = gt.speed
```

---

## 运行模式

先进行A，确认规划链路无误后，再进行B，验证定位建图。

### 模式 A：跳过定位（第一期）

不启动 KISS\-ICP \+ EKF，直接把 ground truth 位姿发到 /localization/pose：

```Plain Text
/control/command → VehicleModel → GT pose → /localization/pose
                                            → /cg410/odometry
                                            → /localization/velocity
                    tracks/*.yaml → LiDAR Sim → /hesai/pandar

FSD: lidar_detection → cone_map_builder → boundary → path_gen → controller
                                                                    ↑
                                                  /localization/pose ─┘
```

优点：绕过 KISS\-ICP 可能不收敛的问题，快速验证规划控制链路。

### 模式 B：全仿真（第二期）

加上 KISS\-ICP \+ EKF 完整定位链：

```Plain Text
/control/command → VehicleModel → GT pose → LiDAR Sim → /hesai/pandar
                                            → INS Sim   → /cg410/odometry
                                            → CAN Sim   → /localization/velocity

FSD: kiss_icp → ekf → localization_manager → /localization/pose
      lidar_detection → cone_map_builder → boundary → path_gen → controller
```

---

## 可视化（RViz2）

FSD 管线已有：

|话题|内容|
|---|---|
|/mapping/cone\_map\_viz|锥筒地图（彩色圆柱体）|
|/planning/centerline\_viz|中心线（绿色线段）|
|/control/target\_viz|控制目标点（红色箭头）|

仿真器额外发布：

|话题|内容|
|---|---|
|/sim/ground\_truth \(Odometry\)|真实位姿（与定位估计对比）|
|/sim/track\_viz \(MarkerArray\)|真实锥筒位置（与建图结果对比）|

RViz2 中 Fixed Frame 设为 map，就能看到完整仿真画面：赛道、车辆、中心线、控制目标点。

---

## 开发计划

### 

1. 创建 wuta\_simulator 包骨架

2. 写 [vehicle\_model\.py](http://vehicle_model.py) — 自行车模型 \+ 发布 GT

3. 写 [ins\_simulator\.py](http://ins_simulator.py) — 发布 /cg410/odometry

4. 写 [can\_simulator\.py](http://can_simulator.py) — 发布 /localization/velocity

5. 写 [lidar\_simulator\.py](http://lidar_simulator.py) — 直接加载 YAML 地图，射线投射 \+ 地面点

7. LiDAR 仿真数据喂给 lidar\_detection\_node，跑完整感知链路

8. 接入 KISS\-ICP（模式 B），验证定位 \+ 建图 \+ 规划 \+ 控制全闭环

9. 生成 3 个 FSG 标准赛道 YAML

10. 赛道可视化节点

11. 指标统计（跑完用时、路径偏差、是否撞锥筒）

---


