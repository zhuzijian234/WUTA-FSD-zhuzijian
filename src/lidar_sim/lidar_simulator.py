from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple

import numpy as np
import yaml


COLOR_UNKNOWN = 0
COLOR_BLUE = 1
COLOR_YELLOW = 2
COLOR_ORANGE = 3
COLOR_RED = 4


@dataclass(frozen=True)
class ConeSpec:
    name: str
    color: str
    color_id: int
    size: Tuple[float, float, float]  # width, depth, height in meters
    aliases: Tuple[str, ...]


CONE_SPECS: Dict[str, ConeSpec] = {
    "large_orange": ConeSpec(
        name="large_orange",
        color="orange",
        color_id=COLOR_ORANGE,
        size=(0.35, 0.35, 0.70),
        aliases=("large_orange", "orange", "big_orange"),
    ),
    "small_red": ConeSpec(
        name="small_red",
        color="red",
        color_id=COLOR_RED,
        size=(0.20, 0.20, 0.30),
        aliases=("small_red", "red"),
    ),
    "small_yellow": ConeSpec(
        name="small_yellow",
        color="yellow",
        color_id=COLOR_YELLOW,
        size=(0.20, 0.20, 0.30),
        aliases=("small_yellow", "yellow"),
    ),
    "small_blue": ConeSpec(
        name="small_blue",
        color="blue",
        color_id=COLOR_BLUE,
        size=(0.20, 0.20, 0.30),
        aliases=("small_blue", "blue"),
    ),
}

DEFAULT_CONE_TYPE = "small_blue"


@dataclass(frozen=True)
class LidarConfig:
    fov_deg: float = 120.0
    min_range: float = 1.5
    max_range: float = 50.0
    points_per_cone_min: int = 12
    points_per_cone_max: int = 16
    surface_noise_std: float = 0.02
    center_noise_std: float = 0.02
    ground_points_min: int = 200
    ground_points_max: int = 500
    ground_z_std: float = 0.05
    lidar_height: float = 1.0
    lidar_offset: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    detection_probability: float = 1.0
    include_ground: bool = True
    enable_occlusion: bool = True


@dataclass
class ConeRecord:
    position: np.ndarray
    color: str
    color_id: int
    cone_type: str
    size: np.ndarray
    confidence: float = 1.0


def _as_vector3(value: Any, name: str = "vector") -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape[0] < 3:
        raise ValueError(f"{name} must contain at least 3 values.")
    return arr[:3].copy()


def _yaw_rotation(yaw: float) -> np.ndarray:
    c = float(np.cos(yaw))
    s = float(np.sin(yaw))
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def parse_pose(pose: Any) -> Tuple[np.ndarray, float]:
    """Return base position [x, y, z] and yaw from dict or array-like pose."""
    if isinstance(pose, dict):
        if "position" in pose:
            position = _as_vector3(pose["position"], "pose position")
        else:
            position = np.array(
                [
                    float(pose.get("x", 0.0)),
                    float(pose.get("y", 0.0)),
                    float(pose.get("z", 0.0)),
                ],
                dtype=float,
            )
        yaw = float(pose.get("yaw", pose.get("heading", 0.0)))
        return position, yaw

    values = np.asarray(pose, dtype=float).reshape(-1)
    if values.shape[0] == 3:
        return np.array([values[0], values[1], 0.0], dtype=float), float(values[2])
    if values.shape[0] >= 4:
        return values[:3].copy(), float(values[3])
    raise ValueError("Pose must be [x, y, yaw], [x, y, z, yaw], or a dict.")


def _cone_spec_from_kind(kind: Optional[Any]) -> ConeSpec:
    if kind is None:
        return CONE_SPECS[DEFAULT_CONE_TYPE]

    normalized = str(kind).strip().lower()
    for spec in CONE_SPECS.values():
        if normalized == spec.name or normalized in spec.aliases:
            return spec
    raise ValueError(f"Unknown cone type/color: {kind!r}")


def _spec_from_color_and_type(color: Optional[Any], cone_type: Optional[Any]) -> ConeSpec:
    if cone_type is not None:
        return _cone_spec_from_kind(cone_type)
    if color is not None:
        return _cone_spec_from_kind(color)
    return _cone_spec_from_kind(DEFAULT_CONE_TYPE)


def normalize_cone(cone: Any, default_type: str = DEFAULT_CONE_TYPE) -> ConeRecord:
    """Normalize dict/array cone data to a ConeRecord."""
    if isinstance(cone, ConeRecord):
        return cone

    if isinstance(cone, dict):
        raw_position = cone.get("position", cone.get("center"))
        if raw_position is None and {"x", "y"}.issubset(cone.keys()):
            raw_position = [cone["x"], cone["y"], cone.get("z", 0.0)]
        if raw_position is None:
            raise ValueError("Cone dict requires 'position', 'center', or x/y fields.")

        cone_type = cone.get("type", cone.get("name"))
        color = cone.get("color")
        spec = _spec_from_color_and_type(color, cone_type or default_type)
        if "size" in cone or "dimensions" in cone:
            size = _as_vector3(cone.get("size", cone.get("dimensions")), "cone size")
        else:
            size = np.asarray(spec.size, dtype=float)

        return ConeRecord(
            position=_as_vector3(raw_position, "cone position"),
            color=str(color or spec.color),
            color_id=int(cone.get("color_id", spec.color_id)),
            cone_type=str(cone_type or spec.name),
            size=size,
            confidence=float(cone.get("confidence", 1.0)),
        )

    values = np.asarray(cone, dtype=float).reshape(-1)
    if values.shape[0] < 3:
        raise ValueError("Cone array must be [x, y, z] or [x, y, z, w, d, h].")

    spec = _cone_spec_from_kind(default_type)
    size = values[3:6].copy() if values.shape[0] >= 6 else np.asarray(spec.size, dtype=float)
    return ConeRecord(
        position=values[:3].copy(),
        color=spec.color,
        color_id=spec.color_id,
        cone_type=spec.name,
        size=size,
    )


def load_track_yaml(track_file: Any) -> Tuple[list, list]:
    """
    Load a simulator track YAML directly into cone dictionaries and start pose.

    Returns:
        cones: list of dictionaries accepted by LidarSimulator
        start_pose: [x, y, z, yaw]
    """
    path = Path(track_file).expanduser()
    if not path.exists():
        raise FileNotFoundError(f"Track file not found: {path}")

    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    track = data.get("track", data)
    start_pose_data = track.get("start_pose", {})
    start_pose = [
        float(start_pose_data.get("x", 0.0)),
        float(start_pose_data.get("y", 0.0)),
        float(start_pose_data.get("z", 0.0)),
        float(start_pose_data.get("yaw", 0.0)),
    ]

    cones = []
    cone_groups = (
        ("blue_cones", "blue", "small_blue"),
        ("yellow_cones", "yellow", "small_yellow"),
        ("orange_cones", "orange", "large_orange"),
        ("red_cones", "red", "small_red"),
        ("unknown_cones", "unknown", DEFAULT_CONE_TYPE),
    )
    for key, color, cone_type in cone_groups:
        for entry in track.get(key, []) or []:
            if isinstance(entry, dict):
                position = [
                    float(entry.get("x", 0.0)),
                    float(entry.get("y", 0.0)),
                    float(entry.get("z", 0.0)),
                ]
            else:
                values = np.asarray(entry, dtype=float).reshape(-1)
                if values.shape[0] < 2:
                    raise ValueError(f"Invalid cone entry in {path}: {entry!r}")
                position = [
                    float(values[0]),
                    float(values[1]),
                    float(values[2]) if values.shape[0] > 2 else 0.0,
                ]

            cones.append({"position": position, "color": color, "type": cone_type})

    return cones, start_pose


def lidar_origin_from_pose(
    vehicle_pose: Any,
    lidar_offset: Iterable[float] = (0.0, 0.0, 1.0),
) -> Tuple[np.ndarray, float]:
    base_position, yaw = parse_pose(vehicle_pose)
    offset = _as_vector3(lidar_offset, "lidar_offset")
    lidar_origin = base_position + _yaw_rotation(yaw) @ offset
    return lidar_origin, yaw


def world_to_lidar(
    point_world: Any,
    vehicle_pose: Any,
    lidar_offset: Iterable[float] = (0.0, 0.0, 1.0),
) -> np.ndarray:
    """Transform a map/world point into the LiDAR frame."""
    point_world = _as_vector3(point_world, "point_world")
    lidar_origin, yaw = lidar_origin_from_pose(vehicle_pose, lidar_offset)
    return _yaw_rotation(-yaw) @ (point_world - lidar_origin)


def lidar_to_world(
    point_lidar: Any,
    vehicle_pose: Any,
    lidar_offset: Iterable[float] = (0.0, 0.0, 1.0),
) -> np.ndarray:
    """Transform a LiDAR-frame point into the map/world frame."""
    point_lidar = _as_vector3(point_lidar, "point_lidar")
    lidar_origin, yaw = lidar_origin_from_pose(vehicle_pose, lidar_offset)
    return lidar_origin + _yaw_rotation(yaw) @ point_lidar


def front(car_position: Any, car_heading: float, cone_position: Any) -> bool:
    """Return True when the cone is in front of the vehicle/LiDAR."""
    car_position = _as_vector3(car_position, "car_position")
    cone_position = _as_vector3(cone_position, "cone_position")
    delta = cone_position[:2] - car_position[:2]
    x_forward = delta[0] * np.cos(car_heading) + delta[1] * np.sin(car_heading)
    return bool(x_forward > 0.0)


def angle_judge(
    car_position: Any,
    car_heading: float,
    cone_position: Any,
    fov_deg: float = 120.0,
) -> bool:
    """Return True when the cone is inside the horizontal LiDAR FOV."""
    car_position = _as_vector3(car_position, "car_position")
    cone_position = _as_vector3(cone_position, "cone_position")
    dx = cone_position[0] - car_position[0]
    dy = cone_position[1] - car_position[1]
    angle_to_cone = np.arctan2(dy, dx)
    relative_angle = angle_to_cone - car_heading
    relative_angle = (relative_angle + np.pi) % (2 * np.pi) - np.pi
    half_fov = np.radians(fov_deg / 2.0)
    return bool(abs(relative_angle) <= half_fov)


def distance_judge(
    car_position: Any,
    cone_position: Any,
    min_dist: float = 1.5,
    max_dist: float = 50.0,
) -> bool:
    """Return True when the cone is inside the valid horizontal LiDAR range."""
    car_position = _as_vector3(car_position, "car_position")
    cone_position = _as_vector3(cone_position, "cone_position")
    dist = np.linalg.norm(cone_position[:2] - car_position[:2])
    return bool(min_dist <= dist <= max_dist)


def _parse_obstacle_box(obstacle: Any) -> Tuple[np.ndarray, np.ndarray]:
    """Normalize a cone obstacle to an axis-aligned bounding box."""
    if isinstance(obstacle, dict) and "bbox_min" in obstacle and "bbox_max" in obstacle:
        bbox_min = _as_vector3(obstacle["bbox_min"], "bbox_min")
        bbox_max = _as_vector3(obstacle["bbox_max"], "bbox_max")
        return np.minimum(bbox_min, bbox_max), np.maximum(bbox_min, bbox_max)

    cone = normalize_cone(obstacle)
    half_xy = cone.size[:2] / 2.0
    bbox_min = np.array(
        [cone.position[0] - half_xy[0], cone.position[1] - half_xy[1], cone.position[2]],
        dtype=float,
    )
    bbox_max = np.array(
        [cone.position[0] + half_xy[0], cone.position[1] + half_xy[1], cone.position[2] + cone.size[2]],
        dtype=float,
    )
    return bbox_min, bbox_max


def _segment_intersects_aabb(
    start: Any,
    end: Any,
    bbox_min: Any,
    bbox_max: Any,
    eps: float = 1e-9,
) -> bool:
    """Slab test for an open line segment intersecting an AABB."""
    start = _as_vector3(start, "start")
    end = _as_vector3(end, "end")
    bbox_min = _as_vector3(bbox_min, "bbox_min")
    bbox_max = _as_vector3(bbox_max, "bbox_max")
    direction = end - start
    t_min = 0.0
    t_max = 1.0

    for axis in range(3):
        if abs(direction[axis]) <= eps:
            if start[axis] < bbox_min[axis] or start[axis] > bbox_max[axis]:
                return False
            continue

        inv_dir = 1.0 / direction[axis]
        t1 = (bbox_min[axis] - start[axis]) * inv_dir
        t2 = (bbox_max[axis] - start[axis]) * inv_dir
        if t1 > t2:
            t1, t2 = t2, t1
        t_min = max(t_min, t1)
        t_max = min(t_max, t2)
        if t_min > t_max:
            return False

    return bool(t_max > eps and t_min < 1.0 - eps)


def occlusion_judge(
    car_position: Any,
    cone_position: Any,
    obstacles: Optional[Iterable[Any]],
    eps: float = 1e-9,
) -> bool:
    """
    Judge whether the line of sight from LiDAR to cone is blocked.

    Obstacles are modeled from the cone size table as vertical AABBs. Returns
    True when any cone box intersects the open segment from car_position to
    cone_position.
    """
    if not obstacles:
        return False

    car_position = _as_vector3(car_position, "car_position")
    cone_position = _as_vector3(cone_position, "cone_position")
    if np.dot(cone_position - car_position, cone_position - car_position) <= eps:
        return False

    for obstacle in obstacles:
        obstacle_record = normalize_cone(obstacle)
        if np.allclose(obstacle_record.position, cone_position, atol=eps):
            continue
        bbox_min, bbox_max = _parse_obstacle_box(obstacle_record)
        if _segment_intersects_aabb(car_position, cone_position, bbox_min, bbox_max, eps):
            return True

    return False


def judge(
    car_position: Any,
    car_heading: float,
    cone_position: Any,
    obstacles: Optional[Iterable[Any]] = None,
    fov_deg: float = 120.0,
    min_dist: float = 1.5,
    max_dist: float = 50.0,
) -> bool:
    """Return True when a cone can be scanned by the LiDAR."""
    if not front(car_position, car_heading, cone_position):
        return False
    if not angle_judge(car_position, car_heading, cone_position, fov_deg):
        return False
    if not distance_judge(car_position, cone_position, min_dist, max_dist):
        return False
    if occlusion_judge(car_position, cone_position, obstacles):
        return False
    return True


def generate_cone_surface_points(
    cone_position: Any,
    cone_type: Optional[str] = None,
    color: Optional[str] = None,
    size: Optional[Iterable[float]] = None,
    n_points: Optional[int] = None,
    noise_std: float = 0.02,
    rng: Optional[np.random.Generator] = None,
) -> np.ndarray:
    """Generate noisy LiDAR returns on a cone frustum surface."""
    rng = rng or np.random.default_rng()
    position = _as_vector3(cone_position, "cone_position")
    spec = _spec_from_color_and_type(color, cone_type)
    cone_size = np.asarray(size if size is not None else spec.size, dtype=float)[:3]

    if n_points is None:
        n_points = int(rng.integers(12, 17))
    if n_points <= 0:
        return np.empty((0, 3), dtype=float)

    width, depth, height = cone_size
    bottom_radius = max(width, depth) / 2.0
    top_radius = bottom_radius * 0.3

    heights = rng.uniform(0.0, height, n_points)
    angles = rng.uniform(0.0, 2.0 * np.pi, n_points)
    radii = bottom_radius * (1.0 - heights / height) + top_radius * (heights / height)

    points = np.column_stack(
        [
            position[0] + radii * np.cos(angles),
            position[1] + radii * np.sin(angles),
            position[2] + heights,
        ]
    )

    if noise_std > 0.0:
        points += rng.normal(0.0, noise_std, points.shape)
    return points


def generate_ground_points(
    n_points: int,
    fov_deg: float = 120.0,
    min_range: float = 1.5,
    max_range: float = 50.0,
    ground_z: float = -1.0,
    ground_z_std: float = 0.05,
    rng: Optional[np.random.Generator] = None,
) -> np.ndarray:
    """Generate random ground points in the LiDAR frame."""
    rng = rng or np.random.default_rng()
    if n_points <= 0:
        return np.empty((0, 3), dtype=float)

    half_fov = np.radians(fov_deg / 2.0)
    angles = rng.uniform(-half_fov, half_fov, n_points)
    ranges = rng.uniform(min_range, max_range, n_points)
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    z = rng.normal(ground_z, ground_z_std, n_points)
    return np.column_stack([x, y, z])


def point_make_and_map(
    cone_position: Any,
    lidar_height: float = 1.0,
    global_map: Optional[np.ndarray] = None,
    rng: Optional[np.random.Generator] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Backward-compatible helper: generate cone surface points and ground points.
    """
    rng = rng or np.random.default_rng()
    cone_points = generate_cone_surface_points(cone_position, rng=rng)
    n_ground = int(rng.integers(200, 501))
    ground_points = generate_ground_points(n_ground, ground_z=-float(lidar_height), rng=rng)

    if global_map is not None:
        generated = np.vstack([cone_points, ground_points])
        if isinstance(global_map, list):
            global_map.extend(generated.tolist())

    return cone_points, ground_points


def plane_make(
    n_points: int = 300,
    lidar_height: float = 1.0,
    rng: Optional[np.random.Generator] = None,
) -> np.ndarray:
    """Backward-compatible helper for simulated ground generation."""
    return generate_ground_points(n_points, ground_z=-float(lidar_height), rng=rng)


class LidarSimulator:
    """Pure Python LiDAR simulator without ROS publish/subscribe code."""

    def __init__(self, config: Optional[LidarConfig] = None, seed: Optional[int] = None):
        self.config = config or LidarConfig()
        self.rng = np.random.default_rng(seed)

    def set_seed(self, seed: Optional[int]) -> None:
        self.rng = np.random.default_rng(seed)

    def transform_cones_to_lidar(self, cones: Iterable[Any], vehicle_pose: Any):
        lidar_cones = []
        for cone in cones:
            record = normalize_cone(cone)
            lidar_position = world_to_lidar(
                record.position,
                vehicle_pose,
                self.config.lidar_offset,
            )
            lidar_cones.append(
                ConeRecord(
                    position=lidar_position,
                    color=record.color,
                    color_id=record.color_id,
                    cone_type=record.cone_type,
                    size=record.size.copy(),
                    confidence=record.confidence,
                )
            )
        return lidar_cones

    def visible_cones(self, cones: Iterable[Any], vehicle_pose: Any):
        """Return visible cones in LiDAR coordinates with noisy measured centers."""
        lidar_cones = self.transform_cones_to_lidar(cones, vehicle_pose)
        visible = []
        origin = np.zeros(3, dtype=float)

        for index, cone in enumerate(lidar_cones):
            if not judge(
                origin,
                0.0,
                cone.position,
                None,
                self.config.fov_deg,
                self.config.min_range,
                self.config.max_range,
            ):
                continue

            if self.config.enable_occlusion:
                obstacles = [other for other_index, other in enumerate(lidar_cones) if other_index != index]
                if occlusion_judge(origin, cone.position, obstacles):
                    continue

            if self.config.detection_probability < 1.0:
                if self.rng.random() > self.config.detection_probability:
                    continue

            measured_position = cone.position.copy()
            if self.config.center_noise_std > 0.0:
                measured_position += self.rng.normal(0.0, self.config.center_noise_std, 3)

            visible.append(
                {
                    "index": index,
                    "position": measured_position,
                    "true_position": cone.position.copy(),
                    "color": cone.color,
                    "color_id": cone.color_id,
                    "type": cone.cone_type,
                    "size": cone.size.copy(),
                    "confidence": cone.confidence,
                    "distance": float(np.linalg.norm(cone.position[:2])),
                }
            )

        visible.sort(key=lambda item: item["distance"])
        return visible

    def simulate_scan(self, cones: Iterable[Any], vehicle_pose: Any) -> Dict[str, Any]:
        """Generate one LiDAR scan in the LiDAR frame."""
        visible = self.visible_cones(cones, vehicle_pose)
        cone_clouds = []

        for cone in visible:
            n_points = int(
                self.rng.integers(
                    self.config.points_per_cone_min,
                    self.config.points_per_cone_max + 1,
                )
            )
            cone_clouds.append(
                generate_cone_surface_points(
                    cone["true_position"],
                    cone_type=cone["type"],
                    color=cone["color"],
                    size=cone["size"],
                    n_points=n_points,
                    noise_std=self.config.surface_noise_std,
                    rng=self.rng,
                )
            )

        cone_points = (
            np.vstack(cone_clouds)
            if cone_clouds
            else np.empty((0, 3), dtype=float)
        )

        if self.config.include_ground:
            n_ground = int(
                self.rng.integers(
                    self.config.ground_points_min,
                    self.config.ground_points_max + 1,
                )
            )
            ground_points = generate_ground_points(
                n_ground,
                self.config.fov_deg,
                self.config.min_range,
                self.config.max_range,
                -self.config.lidar_height,
                self.config.ground_z_std,
                self.rng,
            )
        else:
            ground_points = np.empty((0, 3), dtype=float)

        point_cloud = np.vstack([cone_points, ground_points])
        return {
            "point_cloud": point_cloud,
            "cone_points": cone_points,
            "ground_points": ground_points,
            "visible_cones": visible,
            "frame_id": "lidar",
        }


def main() -> None:
    default_track = Path(__file__).resolve().parents[1] / "tracks" / "trackdrive.yaml"
    cones, vehicle_pose = load_track_yaml(default_track)
    simulator = LidarSimulator(seed=42)
    scan = simulator.simulate_scan(cones, vehicle_pose)
    print(f"visible_cones={len(scan['visible_cones'])}")
    print(f"cone_points={len(scan['cone_points'])}")
    print(f"ground_points={len(scan['ground_points'])}")
    print(f"point_cloud_shape={scan['point_cloud'].shape}")


if __name__ == "__main__":
    main()
