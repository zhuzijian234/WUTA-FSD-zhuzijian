from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy 
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Point                     # 用于 Marker 的3D点
from nav_msgs.msg import Odometry                       # 里程计消息（ground truth 订阅）
from rclpy.node import Node                             # ROS2 Node 基类
from sensor_msgs_py import point_cloud2                 # PointCloud2 构造工具
from sensor_msgs.msg import PointCloud2                 # 点云消息
from std_msgs.msg import Header                         # 消息头（时间戳+坐标系）
from visualization_msgs.msg import Marker, MarkerArray  # RViz 可视化

from .lidar_simulator import LidarConfig, LidarSimulator, load_track_yaml

#三个工具函数
#四元数转偏航角
def _quaternion_to_yaw(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

#找yamal文件路径
def _default_track_file() -> str:
    try:
        share_dir = Path(get_package_share_directory("lidar_sim"))
        installed_track = share_dir / "tracks" / "trackdrive.yaml"
        if installed_track.exists():
            return str(installed_track)
    except PackageNotFoundError:
        pass

    source_track = (
        Path(__file__).resolve().parents[1]
        / "tracks"
        / "trackdrive.yaml"
    )
    return str(source_track)

#根据颜色名称返回RGBA颜色值
def _marker_color(color: str) -> Tuple[float, float, float, float]:
    colors = {
        "blue": (0.05, 0.25, 1.0, 0.95),
        "yellow": (1.0, 0.85, 0.05, 0.95),
        "orange": (1.0, 0.35, 0.0, 0.95),
        "red": (1.0, 0.0, 0.0, 0.95),
        "unknown": (0.8, 0.8, 0.8, 0.8),
    }
    return colors.get(color, colors["unknown"])

#ros2节点类
class LidarSimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_simulator")
#参数声明
        self.declare_parameter("track_file", _default_track_file()) #yaml路径
        self.declare_parameter("ground_truth_topic", "/sim/ground_truth") #地面真值话题
        self.declare_parameter("pointcloud_topic", "/hesai/pandar") #点云话题
        self.declare_parameter("visible_markers_topic", "/sim/lidar/visible_cones") #可见锥桶话题
        self.declare_parameter("publish_rate_hz", 10.0) #发布频率
        self.declare_parameter("frame_id", "lidar")  #坐标系
        self.declare_parameter("use_start_pose_until_odom", True) #使用起始位姿直到接收到里程计
        self.declare_parameter("random_seed", 42) #随机种子
        self.declare_parameter("fov_deg", 120.0) #视场角
        self.declare_parameter("min_range", 1.5) #最小范围
        self.declare_parameter("max_range", 50.0) #最大范围
        self.declare_parameter("points_per_cone_min", 12)  #每个锥桶最少点数
        self.declare_parameter("points_per_cone_max", 16)  #每个锥桶最多点数
        self.declare_parameter("surface_noise_std", 0.02)  #表面噪声标准差
        self.declare_parameter("center_noise_std", 0.02)  #中心噪声标准差
        self.declare_parameter("ground_points_min", 200)   #地面点数最小值
        self.declare_parameter("ground_points_max", 500)   #地面点数最大值
        self.declare_parameter("ground_z_std", 0.05)       #地面Z轴标准差
        self.declare_parameter("lidar_height", 1.0)        #激光雷达高度
        self.declare_parameter("lidar_offset_x", 0.0)      #激光雷达偏移X
        self.declare_parameter("lidar_offset_y", 0.0)      #激光雷达偏移Y
        self.declare_parameter("lidar_offset_z", 1.0)      #激光雷达偏移Z
        self.declare_parameter("detection_probability", 1.0) #检测概率
        self.declare_parameter("include_ground", True)       #是否包含地面点
        self.declare_parameter("enable_occlusion", True)     #是否启用遮挡检测
 #加载赛道
        track_file = str(self.get_parameter("track_file").value)
        if not track_file:
            track_file = _default_track_file()
        self.cones, self.start_pose = load_track_yaml(track_file)
        self.current_pose: Optional[List[float]] = None
#配置激光雷达模拟器
        config = LidarConfig(
            fov_deg=float(self.get_parameter("fov_deg").value),
            min_range=float(self.get_parameter("min_range").value),
            max_range=float(self.get_parameter("max_range").value),
            points_per_cone_min=int(self.get_parameter("points_per_cone_min").value),
            points_per_cone_max=int(self.get_parameter("points_per_cone_max").value),
            surface_noise_std=float(self.get_parameter("surface_noise_std").value),
            center_noise_std=float(self.get_parameter("center_noise_std").value),
            ground_points_min=int(self.get_parameter("ground_points_min").value),
            ground_points_max=int(self.get_parameter("ground_points_max").value),
            ground_z_std=float(self.get_parameter("ground_z_std").value),
            lidar_height=float(self.get_parameter("lidar_height").value),
            lidar_offset=(
                float(self.get_parameter("lidar_offset_x").value),
                float(self.get_parameter("lidar_offset_y").value),
                float(self.get_parameter("lidar_offset_z").value),
            ),
            detection_probability=float(self.get_parameter("detection_probability").value),
            include_ground=bool(self.get_parameter("include_ground").value),
            enable_occlusion=bool(self.get_parameter("enable_occlusion").value),
        )
        self.simulator = LidarSimulator(config, seed=int(self.get_parameter("random_seed").value))

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.use_start_pose_until_odom = bool(self.get_parameter("use_start_pose_until_odom").value)

        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("pointcloud_topic").value),
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("visible_markers_topic").value),
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("ground_truth_topic").value),
            self._on_ground_truth,
            10,
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive.")
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_scan)

        self.get_logger().info(
            f"Loaded {len(self.cones)} cones from {track_file}; "
            f"publishing {self.get_parameter('pointcloud_topic').value} at {publish_rate_hz:.1f} Hz"
        )
#里程计回调
    def _on_ground_truth(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.current_pose = [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            _quaternion_to_yaw(pose.orientation),
        ]
#位姿选择
    def _active_pose(self) -> Optional[List[float]]:
        if self.current_pose is not None: 
            return self.current_pose   #优先：收到过里程计
        if self.use_start_pose_until_odom:
            return self.start_pose #回退：赛道起始位姿
        return None    # 都没有：跳过发布
#主循环
    def _publish_scan(self) -> None:
        vehicle_pose = self._active_pose()
        if vehicle_pose is None:
            self.get_logger().debug("Waiting for ground truth odometry.")
            return

        scan = self.simulator.simulate_scan(self.cones, vehicle_pose)
        stamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = self.frame_id

        cloud_points = np.asarray(scan["point_cloud"], dtype=np.float32)
        msg = point_cloud2.create_cloud_xyz32(header, cloud_points.tolist())
        self.pointcloud_pub.publish(msg) #
        self.marker_pub.publish(self._make_visible_markers(scan["visible_cones"], header)) #

    def _make_visible_markers(self, visible_cones: List[Dict[str, Any]], header: Header) -> MarkerArray:
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header = header
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for idx, cone in enumerate(visible_cones):
            marker = Marker()
            marker.header = header
            marker.ns = "visible_cones"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(
                x=float(cone["position"][0]),
                y=float(cone["position"][1]),
                z=float(cone["position"][2]),
            )
            marker.pose.orientation.w = 1.0
            size = cone["size"]
            marker.scale.x = float(size[0])
            marker.scale.y = float(size[1])
            marker.scale.z = float(size[2])
            r, g, b, a = _marker_color(str(cone["color"]))
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a
            marker_array.markers.append(marker)

        return marker_array

##spin() 的阻塞循环内部：
#while rclpy.ok():
#   检查订阅队列 → 有新 Odometry → 调用 _on_ground_truth()
# 检查 Timer → 到期 → 调用 _publish_scan()
# 检查其他回调...
# #sleep(微小间隔) 
def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LidarSimulatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
#节点一边听车辆模型的位姿变化，一边以 10Hz 的频率"替"禾赛雷达生成当前视角下的点云，
# 在 FSD 算法栈看来和真实雷达别无二致。
if __name__ == "__main__":
    main()
