import numpy as np

from lidar_sim.lidar_simulator import (
    LidarConfig,
    LidarSimulator,
    angle_judge,
    distance_judge,
    front,
    lidar_to_world,
    load_track_yaml,
    occlusion_judge,
    world_to_lidar,
)


def test_coordinate_round_trip():
    pose = [1.0, 2.0, 0.0, np.pi / 4.0]
    point = np.array([5.0, 3.0, 0.0])

    lidar_point = world_to_lidar(point, pose)
    assert np.allclose(lidar_to_world(lidar_point, pose), point)


def test_visibility_gates():
    origin = np.zeros(3)

    assert front(origin, 0.0, [5.0, 0.0, 0.0]) is True
    assert angle_judge(origin, 0.0, [5.0, 0.0, 0.0]) is True
    assert angle_judge(origin, 0.0, [0.0, 5.0, 0.0]) is False
    assert distance_judge(origin, [5.0, 0.0, 0.0]) is True
    assert distance_judge(origin, [1.0, 0.0, 0.0]) is False


def test_cone_aabb_occlusion():
    origin = np.zeros(3)

    assert (
        occlusion_judge(
            origin,
            [5.0, 0.0, -2.0],
            [{"position": [2.5, 0.0, -1.1], "color": "yellow"}],
        )
        is True
    )
    assert (
        occlusion_judge(
            origin,
            [5.0, 0.0, -2.0],
            [{"position": [2.5, 0.2, -1.1], "color": "yellow"}],
        )
        is False
    )


def test_scan_is_reproducible():
    cones = [
        {"position": [5.0, 0.0, 0.0], "color": "yellow"},
        {"position": [8.0, 1.0, 0.0], "color": "blue"},
        {"position": [-5.0, 0.0, 0.0], "color": "red"},
    ]
    config = LidarConfig(ground_points_min=10, ground_points_max=10)

    scan1 = LidarSimulator(config, seed=7).simulate_scan(cones, [0.0, 0.0, 0.0])
    scan2 = LidarSimulator(config, seed=7).simulate_scan(cones, [0.0, 0.0, 0.0])

    assert len(scan1["visible_cones"]) == 2
    assert scan1["point_cloud"].shape[1] == 3
    assert np.allclose(scan1["point_cloud"], scan2["point_cloud"])


def test_load_track_yaml_directly():
    cones, start_pose = load_track_yaml("tracks/acceleration.yaml")

    assert start_pose == [0.0, 0.0, 0.0, 0.0]
    assert len(cones) == 54
    assert cones[0]["color"] == "blue"
