# WUTA-FSD LiDAR 仿真器 — 完整检验流程

> **日期**: 2026-07-09  
> **分支**: `feature/zhuzijian/simulator`  
> **环境**: Windows 11 + WSL2 (Ubuntu 22.04) + ROS2 Humble

---

## 目录

1. [项目结构总览](#1-项目结构总览)
2. [环境准备](#2-环境准备)
3. [Level 1：纯 Python 核心测试](#3-level-1纯-python-核心测试)
4. [Level 2：ROS2 单节点仿真](#4-level-2ros2-单节点仿真)
5. [Level 3：FSD 全栈编译](#5-level-3fsd-全栈编译)
6. [端到端联调](#6-端到端联调)
7. [编译问题与修复记录](#7-编译问题与修复记录)
8. [待解决问题与下一步](#8-待解决问题与下一步)

---

## 1. 项目结构总览

```
WUTA-FSD/
├── src/                          # Python LiDAR 仿真器
│   ├── lidar_sim/
│   │   ├── lidar_simulator.py    # 核心引擎 (688行)
│   │   └── lidar_sim_node.py     # ROS2 节点 (224行)
│   ├── config/
│   │   └── lidar_simulator.yaml  # ROS2 参数
│   ├── launch/
│   │   └── lidar_simulator.launch.py
│   ├── tracks/
│   │   ├── trackdrive.yaml       # 赛道: 64锥桶, 曲线
│   │   ├── acceleration.yaml     # 赛道: 75m直线
│   │   └── skidpad.yaml          # 赛道: 8字形
│   └── tests/
│       └── test_lidar_core.py    # 5 个单元测试
│
└── ros2_ws/                      # C++ FSD 算法栈
    └── src/
        ├── common/wuta_msgs/     # 自定义消息接口
        ├── common/wuta_tools/    # 工具库
        ├── perception/           # 感知 (3个包)
        ├── localization/         # 定位 (4个包)
        ├── mapping/              # 建图 (1个包)
        ├── planning/             # 规划 (2个包)
        ├── control/              # 控制 (1个包)
        └── system/               # 系统管理 (1个包)
```

---

## 2. 环境准备

### 2.1 Windows 端 (Level 1 纯 Python 测试)

```powershell
# 使用 conda 创建环境
conda create -n wuta_sim python=3.10 -y
conda activate wuta_sim
pip install numpy==2.2.5 pyyaml==6.0.3 pytest==9.0.3

# 验证
cd d:\OneDrive\Desktop\WUTA-FSD\src
python -c "from lidar_sim.lidar_simulator import LidarSimulator, LidarConfig, load_track_yaml; print('OK')"
```

### 2.2 WSL2 端 (Level 2+ ROS2 仿真)

```bash
# Ubuntu 22.04 on WSL2 (Microsoft Store 安装)
# ROS2 Humble 安装 (使用清华镜像，国内网络需要)
sudo apt update && sudo apt install ros-humble-desktop -y

# 安装编译依赖
sudo apt install python3-pip python3-colcon-common-extensions -y
pip3 install numpy pyyaml

# 编译 lidar_sim (Python 包)
cd /mnt/d/OneDrive/Desktop/WUTA-FSD/src
colcon build --symlink-install --packages-select lidar_sim
```

> **注意**: 如果遇到 `raw.githubusercontent.com` 连接失败，使用清华镜像：
> ```bash
> sudo sh -c 'echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list'
> ```

---

## 3. Level 1：纯 Python 核心测试

### 3.1 核心 Demo

```bash
# Windows 或 WSL 均可运行
cd src
python -c "
from lidar_sim.lidar_simulator import *
config = LidarConfig()
cones = load_track_yaml('tracks/trackdrive.yaml')
sim = LidarSimulator(config)
result = sim.simulate_scan(cones, x=0.0, y=0.0, yaw=0.0)
print(f'visible_cones={len(result[\"visible_cones\"])}')
print(f'cone_points={len(result[\"cone_points\"])}')
print(f'ground_points={len(result[\"ground_points\"])}')
print(f'point_cloud_shape={result[\"point_cloud\"].shape}')
"
```

**期望输出**:
```
visible_cones=40
cone_points=568
ground_points=359
point_cloud_shape=(927, 3)
```

> ✅ **验证通过**: 输出跨平台一致（随机种子固定）

### 3.2 单元测试

```bash
cd src
python -m pytest tests/test_lidar_core.py -v
```

**期望输出**: `5 passed`

> ✅ **验证通过**: 5/5 测试全部通过

### 3.3 三条赛道加载测试

```bash
python -c "
from lidar_sim.lidar_simulator import load_track_yaml
for track in ['trackdrive', 'acceleration', 'skidpad']:
    c = load_track_yaml(f'tracks/{track}.yaml')
    print(f'{track}: {len(c)} cones')
"
```

**期望输出**:
```
trackdrive: 64 cones
acceleration: 54 cones
skidpad: 36 cones
```

> ✅ **验证通过**

---

## 4. Level 2：ROS2 单节点仿真

### 4.1 编译

```bash
cd /mnt/d/OneDrive/Desktop/WUTA-FSD/src
colcon build --symlink-install --packages-select lidar_sim
```

> ✅ **验证通过**: `Finished <<< lidar_sim`

### 4.2 启动节点

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lidar_sim lidar_simulator_node --ros-args -p use_start_pose_until_odom:=true
```

**期望日志**:
```
[INFO] [lidar_simulator]: Loaded 64 cones from .../trackdrive.yaml; publishing /hesai/pandar at 10.0 Hz
```

> ✅ **验证通过**: 节点启动，加载赛道成功

### 4.3 检查话题列表

```bash
# 另一个终端
source /opt/ros/humble/setup.bash
ros2 topic list
```

**期望输出**:
```
/hesai/pandar              # LiDAR 点云 (PointCloud2)
/parameter_events
/rosout
/sim/ground_truth           # 订阅：车辆位姿
/sim/lidar/visible_cones    # 可视锥桶 (MarkerArray)
```

> ✅ **验证通过**: 5 个话题全部就绪

### 4.4 验证点云数据

```bash
ros2 topic echo /hesai/pandar --once
```

**期望输出**:
```
header:
  frame_id: lidar
height: 1
width: ~1000                # 约 1000 个点
fields:
- name: x  datatype: 7
- name: y  datatype: 7
- name: z  datatype: 7
point_step: 12              # 3 × float32
```

> ✅ **验证通过**: 点云结构正确，frame_id=`lidar`

### 4.5 验证发布频率

```bash
ros2 topic hz /hesai/pandar
```

**期望输出**:
```
average rate: 10.077
  min: 0.092s max: 0.104s std dev: 0.00347s
```

> ✅ **验证通过**: 10Hz 稳定发布，抖动 < 5ms

### 4.6 验证可视锥桶

```bash
ros2 topic echo /sim/lidar/visible_cones --once | grep "ns:" | wc -l
```

**期望输出**: `41` (每个锥桶一个 marker + 1 个 delete_all marker)

> ✅ **验证通过**: 40 个锥桶在地面真值中被标记为可见

---

## 5. Level 3：FSD 全栈编译

### 5.1 准备工作

```bash
# 复制 ros2_ws 到 WSL 原生 ext4 文件系统（NTFS 上 colcon 构建会失败！）
cp -r /mnt/d/OneDrive/Desktop/WUTA-FSD/ros2_ws ~/ros2_ws
cd ~/ros2_ws

# 安装系统依赖（如未安装）
sudo apt install libpcl-dev libeigen3-dev libyaml-cpp-dev -y
```

### 5.2 编译全部包

```bash
source /opt/ros/humble/setup.bash

# 跳过已知有问题的 3 个包
colcon build --symlink-install \
  --packages-skip controller path_generator boundary_detector
```

### 5.3 编译结果

| 状态 | 包名 | 类型 | 说明 |
|:--:|------|------|------|
| ✅ | `wuta_msgs` | 接口 | 自定义 ROS2 消息 (ConeArray, MissionState 等) |
| ✅ | `wuta_tools` | 工具 | 通用工具函数 |
| ✅ | `lidar_detection` | 感知 | **LiDAR 锥桶检测** (传统 PCL + DL 后端) |
| ✅ | `detection_fusion` | 感知 | 多传感器检测融合 |
| ✅ | `camera_detection` | 感知 | 相机检测节点 |
| ✅ | `kiss_icp_wrapper` | 定位 | KISS-ICP 点云配准封装 |
| ✅ | `ndt_localization` | 定位 | NDT 点云地图匹配定位 |
| ✅ | `localization_manager` | 定位 | 定位模式管理与融合 |
| ✅ | `cone_map_builder` | 建图 | 锥桶 SLAM 地图构建 |
| ✅ | `mission_manager` | 系统 | 任务状态机 (EXPLORE → RACE) |
| ❌ | `controller` | 控制 | `autoware_msgs::Lane` API 已废弃 |
| ❌ | `path_generator` | 规划 | 同上，依赖旧版 autoware 消息 |
| ❌ | `boundary_detector` | 规划 | 第三方库 `pathplanning` 缺失依赖 |

> 总计：**10/13 包编译成功** (77%)

---

## 6. 端到端联调

### 6.1 数据流架构

```
┌─────────────┐     /hesai/pandar       ┌──────────────────┐
│ lidar_sim    │ ──────────────────────→ │ lidar_detection   │
│ (Python)     │    PointCloud2          │ (C++)             │
│              │                         │                   │
│ 10 Hz        │                         │ traditional/DL    │
└──────┬───────┘                         └────────┬──────────┘
       │                                          │
       │ /sim/lidar/visible_cones                 │ /perception/lidar/cones
       │ (地面真值, 调试用)                         │ (ConeArray)
       ▼                                          ▼
   可视化/调试                               ┌──────────────┐
                                            │ detection_    │
                                            │ fusion        │
                                            └──────┬───────┘
                                                   │
                                                   │ /perception/fusion/cones
                                                   ▼
                                              ┌──────────────┐
                                              │ cone_map_     │
                                              │ builder       │
                                              └──────┬───────┘
                                                     │
                                                     │ /map/cones
                                                     ▼
                                              ┌──────────────┐
                                              │ boundary_     │
                                              │ detector ❌    │
                                              └──────┬───────┘
                                                     │
                                                     │ /planning/centerline
                                                     ▼
                                              ┌──────────────┐
                                              │ path_         │
                                              │ generator ❌   │
                                              └──────┬───────┘
                                                     │
                                                     │ /planning/trajectory
                                                     ▼
                                              ┌──────────────┐
                                              │ controller ❌  │
                                              └──────────────┘
```

### 6.2 sim → detection 联调步骤

```bash
# 终端 1: 启动仿真器
source /opt/ros/humble/setup.bash
source /mnt/d/OneDrive/Desktop/WUTA-FSD/src/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run lidar_sim lidar_simulator_node --ros-args -p use_start_pose_until_odom:=true

# 终端 2: 启动感知节点
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run lidar_detection lidar_detection_node

# 终端 3: 监控输出
source /opt/ros/humble/setup.bash
ros2 topic echo /perception/lidar/cones
```

### 6.3 联调结果

| 检查项 | 结果 | 说明 |
|--------|:--:|------|
| 仿真器 → 点云发布 | ✅ | 10Hz, ~1000点/帧 |
| lidar_detection 订阅点云 | ✅ | 日志: "Listening on /hesai/pandar" |
| lidar_detection 发布 ConeArray | ✅ | 话题 `/perception/lidar/cones` 已发布 |
| 检测到锥桶 | ⚠️ | 当前为 **空结果** `cones: []` |
| 可视锥桶地面真值 | ✅ | 40 个锥桶在仿真器侧正确标记 |

> ⚠️ **已知问题**: 仿真点云 → PCL 传统检测器的检测率接近 0%。  
> **原因分析**:
> 1. 仿真点云是锥桶表面被射线命中的点（frustum 模型），分布稀疏且形状不规则
> 2. 传统 Euclidean 聚类 + 锥形滤波器对真实 LiDAR 稠密点云调优，仿真点云点数（~14点/锥桶）不足
> 3. 起点位姿处最近锥桶在 ±2m 侧向，仅有少量侧锥在 FOV 内
> **预期解决方案**: 调低 `min_cluster_size` (3→2)、放宽 `cluster_tolerance`、或使用 DL 后端

---

## 7. 编译问题与修复记录

### 7.1 `mission_manager`: 安装阶段找不到 config/

**错误**:
```
ament_cmake_symlink_install_directory() can't find '.../mission_manager/config/'
```

**修复**: 创建空 `config/` 目录
```bash
mkdir -p ros2_ws/src/system/mission_manager/config
```

### 7.2 `ndt_localization`: 缺少 visualization_msgs 依赖

**错误**:
```
fatal error: visualization_msgs/msg/marker_array.hpp: No such file or directory
```

**修复**: 在 `CMakeLists.txt` 第 29 行 `ament_target_dependencies` 中添加 `visualization_msgs`

### 7.3 `ndt_localization`: 安装阶段找不到 launch/

**修复**: 创建空 `launch/` 目录
```bash
mkdir -p ros2_ws/src/localization/ndt_localization/launch
```

### 7.4 `boundary_detector`: 第三方库编译错误

**错误**:
```
Path.cpp: fatal error: hpl/hpl_param.hpp: No such file or directory
Path.cpp: error: 'cos' was not declared in this scope
Path.cpp: error: 'class GlobalVariables' has no member named 'GetStartPointRedundancy'
```

**部分修复**: 添加了 `#include <cassert>` 和 `#include <cmath>`，但 `hpl/hpl_param.hpp` 和 `GetStartPointRedundancy` 需要完整第三方库。**暂不修复**。

### 7.5 `controller` / `path_generator`: autoware_msgs 旧 API

**错误**:
```
fatal error: autoware_msgs/msg/lane.hpp: No such file or directory
```

**原因**: Autoware ROS1 → ROS2 迁移中，`Lane` 和 `Waypoint` 消息类型被移除，替换为 `Trajectory` / `TrajectoryPoint` (在 `autoware_planning_msgs` 中)。

**修复方向**: 需要重构代码，将 `autoware_msgs::msg::Lane` → `autoware_planning_msgs::msg::Trajectory`，`autoware_msgs::msg::Waypoint` → `autoware_planning_msgs::msg::TrajectoryPoint`。**需要团队负责人决定**。

---

## 8. 待解决问题与下一步

### 8.1 短期 (本周)

| 优先级 | 任务 | 负责人 | 预估 |
|--------|------|--------|------|
| P0 | 调优 lidar_detection 参数适配仿真点云 | 感知组 | 2h |
| P1 | 写 vehicle_model 发布 `/sim/ground_truth` | 规控组 | 4h |
| P1 | 修复 `controller` autoware_msgs 依赖 | 规控组 | 4h |
| P2 | 修复 `path_generator` autoware_msgs 依赖 | 规控组 | 2h |

### 8.2 中期 (7.14-7.20)

| 优先级 | 任务 | 说明 |
|--------|------|------|
| P0 | 完整 closed-loop 仿真 | sim → detection → fusion → map → plan → control → vehicle_model → sim |
| P1 | 多赛道切换测试 | trackdrive / skidpad / acceleration |
| P2 | boundary_detector 第三方库修复 | 替换或修复 `pathplanning` 库 |

### 8.3 关键文件路径速查

| 用途 | 路径 |
|------|------|
| 仿真核心 | `src/lidar_sim/lidar_simulator.py` |
| ROS2 节点 | `src/lidar_sim/lidar_sim_node.py` |
| 感知检测 | `ros2_ws/src/perception/lidar_detection/` |
| 消息定义 | `ros2_ws/src/common/wuta_msgs/` |
| 赛道定义 | `src/tracks/*.yaml` |
| 仿真配置 | `src/config/lidar_simulator.yaml` |
| 检测配置 | `ros2_ws/src/perception/lidar_detection/config/lidar_detection.yaml` |
| 单元测试 | `src/tests/test_lidar_core.py` |
| 启动文件 | `src/launch/lidar_simulator.launch.py` |

---

## 附录 A: 快速验证脚本

以下脚本一键验证 Level 1 + Level 2：

```bash
#!/bin/bash
# verify_all.sh — 保存到 WSL ~/ 目录下运行

set -e
echo "========================================="
echo " WUTA-FSD 仿真器一键验证"
echo "========================================="

# Level 1: Python core
echo ""
echo "--- Level 1: Python Core Demo ---"
cd /mnt/d/OneDrive/Desktop/WUTA-FSD/src
python3 -c "
from lidar_sim.lidar_simulator import *
config = LidarConfig()
cones = load_track_yaml('tracks/trackdrive.yaml')
sim = LidarSimulator(config)
r = sim.simulate_scan(cones, 0, 0, 0)
assert len(r['visible_cones']) == 40, f'Expected 40, got {len(r[\"visible_cones\"])}'
assert r['point_cloud'].shape == (927, 3), f'Expected (927,3), got {r[\"point_cloud\"].shape}'
print('PASS: visible_cones=40, point_cloud_shape=(927,3)')
"

echo ""
echo "--- Level 1: Unit Tests ---"
python3 -m pytest tests/test_lidar_core.py -q

# Level 2: ROS2 node
echo ""
echo "--- Level 2: ROS2 Node Startup ---"
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 5 ros2 run lidar_sim lidar_simulator_node --ros-args -p use_start_pose_until_odom:=true 2>&1 | head -5

echo ""
echo "========================================="
echo " ALL CHECKS PASSED"
echo "========================================="
```

---

## 附录 B: Windows ↔ WSL 工作流

```
Windows (开发)              WSL2 (编译/运行)
┌──────────────┐           ┌─────────────────────┐
│ VS Code      │ ──编辑──→ │ /mnt/d/.../WUTA-FSD/ │ (共享文件系统)
│ conda wuta_sim│           │                      │
│ pytest       │           │ ~/ros2_ws/           │ (原生 ext4, 编译用)
│              │           │ colcon build         │
│              │           │ ros2 run             │
└──────────────┘           └─────────────────────┘
```

- **代码编辑**: Windows VS Code → `/mnt/d/OneDrive/Desktop/WUTA-FSD/`
- **Python 测试**: Windows conda 环境直接运行
- **ROS2 编译**: 必须从 `/mnt/d/` 复制到 `~/ros2_ws/` (ext4)，否则 NTFS 符号链接会失败
- **ROS2 运行**: `src/` 可在 `/mnt/d/` 直接运行（Python 无需编译）；`ros2_ws/` 从 `~/ros2_ws/install/` 运行