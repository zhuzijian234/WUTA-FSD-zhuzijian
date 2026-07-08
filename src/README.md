# track_loader

## ROS2 LiDAR simulator

This repository now contains a Python ROS2 package named `lidar_sim`.
Design notes are collected in `docs/`.

### Ubuntu / ROS2 build

Use this repository as a ROS2 workspace root, then run:

```bash
cd ~/track_loader
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select lidar_sim
source install/setup.bash
```

Quick core check without ROS:

```bash
python3 -m pytest tests/test_lidar_core.py
```

### Run

```bash
ros2 launch lidar_sim lidar_simulator.launch.py
```

Default behavior:

- subscribes: `/sim/ground_truth` (`nav_msgs/msg/Odometry`)
- publishes: `/hesai/pandar` (`sensor_msgs/msg/PointCloud2`)
- publishes debug markers: `/sim/lidar/visible_cones` (`visualization_msgs/msg/MarkerArray`)
- loads track YAML directly from: `share/lidar_sim/tracks/trackdrive.yaml`

Override the track file:

```bash
ros2 launch lidar_sim lidar_simulator.launch.py track_file:=/path/to/track.yaml
```

Track YAML files live in `tracks/`; there is no separate `track_loader` layer.

Core-only demo:

```bash
ros2 run lidar_sim lidar_simulator_demo
```
