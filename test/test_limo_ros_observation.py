# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

from types import SimpleNamespace

import numpy as np

from vlfm.mapping.obstacle_map import ObstacleMap
from vlfm.mapping.value_map import ValueMap
from vlfm.ros.limo_vlfm_node import (
    _build_annotated_rgb,
    _build_object_map,
    _build_value_map,
    build_limo_observation_cache,
    normalize_depth_array,
    quaternion_from_yaw,
    yaw_from_quaternion,
)


def test_normalize_depth_converts_16uc1_mm_to_unit_range() -> None:
    depth_mm = np.array([[0, 300, 1650, 3000, 5000]], dtype=np.uint16)

    normalized = normalize_depth_array(depth_mm, "16UC1", min_depth=0.3, max_depth=3.0)

    np.testing.assert_allclose(normalized[0, 1:4], [0.0, 0.5, 1.0], atol=1e-6)
    assert normalized[0, 0] == 1.0
    assert normalized[0, 4] == 1.0


def test_normalize_depth_keeps_32fc1_meters_and_invalid_far() -> None:
    depth_m = np.array([[np.nan, -1.0, 0.3, 1.65, 3.0]], dtype=np.float32)

    normalized = normalize_depth_array(depth_m, "32FC1", min_depth=0.3, max_depth=3.0)

    assert normalized[0, 0] == 1.0
    assert normalized[0, 1] == 1.0
    np.testing.assert_allclose(normalized[0, 2:], [0.0, 0.5, 1.0], atol=1e-6)


def test_yaw_quaternion_roundtrip_without_y_flip() -> None:
    q = quaternion_from_yaw(np.pi / 2)
    if isinstance(q, dict):
        q = SimpleNamespace(**q)

    assert np.isclose(yaw_from_quaternion(q), np.pi / 2)


def test_build_limo_observation_cache_contract() -> None:
    obstacle_map = ObstacleMap(
        min_height=0.0,
        max_height=1.0,
        agent_radius=0.05,
        area_thresh=0.05,
        size=100,
        pixels_per_meter=20,
    )
    grid = np.full((100, 100), -1, dtype=np.int8)
    grid[30:70, 30:70] = 0
    grid[50, 40:60] = 100
    rgb = np.zeros((6, 8, 3), dtype=np.uint8)
    depth_mm = np.full((6, 8), 1000, dtype=np.uint16)

    obs = build_limo_observation_cache(
        rgb=rgb,
        depth_raw=depth_mm,
        depth_encoding="16UC1",
        fx=100.0,
        fy=100.0,
        grid=grid,
        grid_resolution=0.05,
        grid_origin_xy=np.array([-2.5, -2.5]),
        base_xyz=np.array([0.1, 0.2, 0.0]),
        base_yaw=0.3,
        cam_xyz=np.array([0.1, 0.2, 0.5]),
        cam_yaw=0.4,
        obstacle_map=obstacle_map,
        min_depth=0.3,
        max_depth=3.0,
    )

    assert set(obs) == {
        "frontier_sensor",
        "robot_xy",
        "robot_heading",
        "nav_depth",
        "object_map_rgbd",
        "value_map_rgbd",
    }
    np.testing.assert_allclose(obs["robot_xy"], [0.1, 0.2])
    assert obs["nav_depth"].shape == (6, 8)
    assert obs["nav_depth"].min() >= 0.0
    assert obs["nav_depth"].max() <= 1.0
    assert len(obs["frontier_sensor"]) > 0
    tf_cam2map = obs["object_map_rgbd"][0][2]
    assert tf_cam2map.shape == (4, 4)
    np.testing.assert_allclose(tf_cam2map[:3, :3].T @ tf_cam2map[:3, :3], np.eye(3), atol=1e-6)
    assert np.isclose(np.linalg.det(tf_cam2map[:3, :3]), 1.0)


def test_limo_debug_visual_panels_are_uint8_bgr() -> None:
    obstacle_map = ObstacleMap(
        min_height=0.0,
        max_height=1.0,
        agent_radius=0.05,
        area_thresh=0.05,
        size=100,
        pixels_per_meter=20,
    )
    grid = np.full((100, 100), -1, dtype=np.int8)
    grid[35:65, 35:65] = 0
    grid[50, 45:55] = 100
    obstacle_map.update_from_occupancy_grid(grid, 0.05, np.array([-2.5, -2.5]))
    obstacle_map.update_agent_traj(np.array([0.0, 0.0]), 0.0)

    value_map = ValueMap(value_channels=1, size=100)
    value_map.update_agent_traj(np.array([0.0, 0.0]), 0.0)
    policy = SimpleNamespace(
        _object_masks=np.zeros((8, 10), dtype=np.uint8),
        _last_target_bbox=np.array([1, 2, 5, 6]),
        _value_map=value_map,
        _circle_marker_radius=5,
        _circle_marker_thickness=2,
        _frontier_color=(0, 0, 255),
        _selected__frontier_color=(0, 255, 255),
        _target_object="bed",
        _object_map=SimpleNamespace(
            clouds={"bed": np.array([[0.1, 0.2, 0.0, 1.0]])},
            last_target_coord=np.array([0.1, 0.2]),
        ),
    )
    policy._object_masks[2:5, 3:7] = 1
    obs_cache = {"frontier_sensor": np.array([[0.5, 0.5]])}
    goal = {"xy": np.array([0.5, 0.5])}

    panels = [
        _build_annotated_rgb(np.zeros((8, 10, 3), dtype=np.uint8), policy),
        obstacle_map.visualize(),
        _build_value_map(policy, obs_cache, goal),
        _build_object_map(obstacle_map, policy, goal),
    ]

    for panel in panels:
        assert panel is not None
        assert panel.dtype == np.uint8
        assert panel.ndim == 3
        assert panel.shape[2] == 3
