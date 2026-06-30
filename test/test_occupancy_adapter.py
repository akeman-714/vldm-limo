# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import numpy as np

from vlfm.mapping.obstacle_map import ObstacleMap


def _make_obstacle_map(size: int = 1000) -> ObstacleMap:
    return ObstacleMap(
        min_height=0.0,
        max_height=1.0,
        agent_radius=0.05,
        area_thresh=0.05,
        size=size,
        pixels_per_meter=20,
    )


def _xy_to_rc(obs_map: ObstacleMap, xy: np.ndarray) -> tuple[int, int]:
    px = obs_map._xy_to_px(np.asarray(xy, dtype=np.float64).reshape(1, 2))[0]
    return int(px[1]), int(px[0])


def _grid_center_xy(row: int, col: int, resolution: float, origin_xy: np.ndarray) -> np.ndarray:
    return np.array(
        [
            origin_xy[0] + (col + 0.5) * resolution,
            origin_xy[1] + (row + 0.5) * resolution,
        ],
        dtype=np.float64,
    )


def _xy_to_grid_rc(xy: np.ndarray, resolution: float, origin_xy: np.ndarray) -> tuple[int, int]:
    col = int(np.floor((xy[0] - origin_xy[0]) / resolution))
    row = int(np.floor((xy[1] - origin_xy[1]) / resolution))
    return row, col


def test_occupancy_grid_updates_obstacles_and_explored_area() -> None:
    obs_map = _make_obstacle_map()
    resolution = 0.05
    origin_xy = np.array([-25.0, -25.0], dtype=np.float64)
    grid = np.full((1000, 1000), -1, dtype=np.int8)
    grid[480:521, 480:521] = 0
    grid[500, 490:511] = 100

    obs_map.update_from_occupancy_grid(grid, resolution, origin_xy)

    wall_rc = _xy_to_rc(obs_map, _grid_center_xy(500, 500, resolution, origin_xy))
    free_rc = _xy_to_rc(obs_map, _grid_center_xy(490, 500, resolution, origin_xy))
    unknown_rc = _xy_to_rc(obs_map, _grid_center_xy(470, 500, resolution, origin_xy))

    assert bool(obs_map._map[wall_rc]) is True
    assert bool(obs_map._map[free_rc]) is False
    assert bool(obs_map.explored_area[free_rc]) is True
    assert bool(obs_map.explored_area[unknown_rc]) is False


def test_occupancy_grid_frontiers_fall_on_free_unknown_boundary() -> None:
    obs_map = _make_obstacle_map()
    resolution = 0.05
    origin_xy = np.array([-25.0, -25.0], dtype=np.float64)
    grid = np.full((1000, 1000), -1, dtype=np.int8)
    grid[460:541, 460:541] = 0
    grid[500, 470:531] = 100

    obs_map.update_from_occupancy_grid(grid, resolution, origin_xy)

    assert len(obs_map.frontiers) > 0
    known = grid != -1
    for xy in obs_map.frontiers:
        row, col = _xy_to_grid_rc(xy, resolution, origin_xy)
        # ObstacleMap._get_frontiers dilates explored_area by 5x5 before calling
        # frontier_exploration, so the waypoint may sit just outside the raw
        # OccupancyGrid known/free mask while still marking the same boundary.
        row0, row1 = max(row - 3, 0), min(row + 4, grid.shape[0])
        col0, col1 = max(col - 3, 0), min(col + 4, grid.shape[1])
        patch = known[row0:row1, col0:col1]
        assert 0 <= row < grid.shape[0]
        assert 0 <= col < grid.shape[1]
        assert patch.any()
        assert not patch.all()


def test_occupancy_grid_xy_pixel_roundtrip_within_one_cell() -> None:
    obs_map = _make_obstacle_map()
    samples = np.array(
        [
            [0.0, 0.0],
            [1.25, -2.5],
            [-4.0, 3.75],
        ],
        dtype=np.float64,
    )

    roundtrip = obs_map._px_to_xy(obs_map._xy_to_px(samples))

    np.testing.assert_allclose(roundtrip, samples, atol=0.05)


def _run_all() -> None:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"PASS {t.__name__}")
    print(f"\n{len(tests)} passed")


if __name__ == "__main__":
    _run_all()
