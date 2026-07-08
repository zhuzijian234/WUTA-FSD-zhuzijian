from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from .lidar_simulator import LidarConfig, LidarSimulator, load_track_yaml


def _quaternion_to_yaw(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


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


def _marker_color(color: str) -> Tuple[float, float, float, float]:
    colors = {
        "blue": (0.05, 0.25, 1.0, 0.95),
        "yellow": (1.0, 0.85, 0.05, 0.95),
        "orange": (1.0, 0.35, 0.0, 0.95),
        "red": (1.0, 0.0, 0.0, 0.95),
        "unknown": (0.8, 0.8, 0.8, 0.8),
    }
    return colors.get(color, colors["unknown"])


class LidarSimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_simulator")

        self.declare_parameter("track_file", _default_track_file())
        self.declare_parameter("ground_truth_topic", "/sim/ground_truth")
        self.declare_parameter("pointcloud_topic", "/hesai/pandar")
        self.declare_parameter("visible_markers_topic", "/sim/lidar/visible_cones")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("frame_id", "lidar")
        self.declare_parameter("use_start_pose_until_odom", True)
        self.declare_parameter("random_seed", 42)
        self.declare_parameter("fov_deg", 120.0)
        self.declare_parameter("min_range", 1.5)
        self.declare_parameter("max_range", 50.0)
        self.declare_parameter("points_per_cone_min", 12)
        self.declare_parameter("points_per_cone_max", 16)
        self.declare_parameter("surface_noise_std", 0.02)
        self.declare_parameter("center_noise_std", 0.02)
        self.declare_parameter("ground_points_min", 200)
        self.declare_parameter("ground_points_max", 500)
        self.declare_parameter("ground_z_std", 0.05)
        self.declare_parameter("lidar_height", 1.0)
        self.declare_parameter("lidar_offset_x", 0.0)
        self.declare_parameter("lidar_offset_y", 0.0)
        self.declare_parameter("lidar_offset_z", 1.0)
        self.declare_parameter("detection_probability", 1.0)
        self.declare_parameter("include_ground", True)
        self.declare_parameter("enable_occlusion", True)

        track_file = str(self.get_parameter("track_file").value)
        if not track_file:
            track_file = _default_track_file()
        self.cones, self.start_pose = load_track_yaml(track_file)
        self.current_pose: Optional[List[float]] = None

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

    def _on_ground_truth(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.current_pose = [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            _quaternion_to_yaw(pose.orientation),
        ]

    def _active_pose(self) -> Optional[List[float]]:
        if self.current_pose is not None:
            return self.current_pose
        if self.use_start_pose_until_odom:
            return self.start_pose
        return None

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
        self.pointcloud_pub.publish(msg)
        self.marker_pub.publish(self._make_visible_markers(scan["visible_cones"], header))

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


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LidarSimulatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
