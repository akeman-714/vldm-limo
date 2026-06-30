# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from vlfm.policy.limo_policy import LimoITMPolicy, should_preempt, yaw_towards
from vlfm.utils.object_memory import recall_object, remember_object
from vlfm.mapping.object_point_cloud_map import ObjectPointCloudMap


def _make_policy() -> LimoITMPolicy:
    policy = LimoITMPolicy.__new__(LimoITMPolicy)
    policy._pointnav_stop_radius = 0.9
    policy._remembered_goal = None
    policy._blocked_frontiers = []
    policy._num_steps = 0
    policy._last_goal = np.zeros(2)
    policy._target_object = "chair"
    policy._memory_written = set()
    policy._called_stop = False
    policy._last_verify_result = ""
    policy._attribute_verified = False
    policy._observations_cache = {}
    policy._policy_info = {}
    policy._update_object_maps_from_cache = lambda obs_cache: None
    policy._update_value_map = lambda: None
    policy._get_target_object_location = lambda robot_xy: None
    policy._get_best_frontier = lambda observations, frontiers: (frontiers[0], 0.42)
    policy._reset_per_goal_nav = lambda: None

    def maybe_remember(goal):
        path = os.environ.get("VLFM_OBJECT_MEMORY_PATH")
        if path and goal is not None:
            remember_object(path, policy._target_object, np.asarray(goal)[:2])

    policy._maybe_remember_object = maybe_remember
    return policy


def _obs(frontiers: np.ndarray) -> dict:
    return {
        "frontier_sensor": frontiers,
        "robot_xy": np.array([0.0, 0.0]),
        "robot_heading": 0.0,
        "nav_depth": np.zeros((4, 4), dtype=np.float32),
        "object_map_rgbd": [],
        "value_map_rgbd": [],
    }


def test_decide_goal_returns_explore_frontier() -> None:
    policy = _make_policy()

    goal = policy.decide_goal(_obs(np.array([[1.0, 0.0], [0.0, 2.0]])))

    assert goal["mode"] == "explore"
    np.testing.assert_allclose(goal["xy"], [1.0, 0.0])
    assert goal["value"] == 0.42
    assert goal["yaw_hint"] == 0.0


def test_decide_goal_prefers_detected_object_over_frontier() -> None:
    policy = _make_policy()
    policy._get_target_object_location = lambda robot_xy: np.array([0.0, 2.0, 0.3])

    goal = policy.decide_goal(_obs(np.array([[1.0, 0.0]])))

    assert goal["mode"] == "navigate"
    np.testing.assert_allclose(goal["xy"], [0.0, 1.1])
    assert np.isclose(goal["yaw_hint"], np.pi / 2)


def test_decide_goal_uses_memory_before_exploration() -> None:
    policy = _make_policy()
    policy._remembered_goal = np.array([-1.0, 0.0])

    goal = policy.decide_goal(_obs(np.array([[1.0, 0.0]])))

    assert goal["mode"] == "navigate-memory"
    np.testing.assert_allclose(goal["xy"], [-1.0, 0.0])


def test_unreachable_frontier_is_blocked_from_next_decision() -> None:
    policy = _make_policy()
    blocked = {"mode": "explore", "xy": np.array([1.0, 0.0])}

    policy.on_goal_unreachable(blocked)
    goal = policy.decide_goal(_obs(np.array([[1.0, 0.0], [2.0, 0.0]])))

    np.testing.assert_allclose(goal["xy"], [2.0, 0.0])


def test_reached_frontier_is_blocked_from_next_decision() -> None:
    policy = _make_policy()
    reached = {"mode": "explore", "xy": np.array([1.0, 0.0])}

    verdict = policy.on_goal_reached(_obs(np.empty((0, 2))), reached)
    goal = policy.decide_goal(_obs(np.array([[1.0, 0.0], [2.0, 0.0]])))

    assert verdict == {"accepted": True, "reason": "frontier reached", "next": "explore"}
    np.testing.assert_allclose(goal["xy"], [2.0, 0.0])


def test_on_goal_reached_rejects_attribute_mismatch() -> None:
    policy = _make_policy()
    rejected = []
    policy._object_map = SimpleNamespace(
        reject_region=lambda name, xy, radius=0.5: rejected.append((name, np.asarray(xy), radius))
    )
    policy._attribute_match = lambda obs_cache, robot_xy: False

    verdict = policy.on_goal_reached(_obs(np.empty((0, 2))), {"mode": "navigate", "xy": np.array([1.0, 2.0])})

    assert verdict == {"accepted": False, "reason": "attr mismatch -> reject", "next": "explore"}
    assert rejected[0][0] == "chair"
    np.testing.assert_allclose(rejected[0][1], [1.0, 2.0])


def test_on_goal_reached_accepts_and_writes_memory(tmp_path: Path, monkeypatch) -> None:
    policy = _make_policy()
    path = tmp_path / "memory.json"
    monkeypatch.setenv("VLFM_OBJECT_MEMORY_PATH", str(path))
    policy._object_map = SimpleNamespace()
    policy._attribute_match = lambda obs_cache, robot_xy: True

    verdict = policy.on_goal_reached(_obs(np.empty((0, 2))), {"mode": "navigate", "xy": np.array([1.25, -2.5])})

    assert verdict["accepted"] is True
    assert verdict["next"] == "done"
    np.testing.assert_allclose(recall_object(path, "chair"), [1.25, -2.5])


def test_preemption_rules() -> None:
    current = {"mode": "explore", "xy": np.array([0.0, 0.0])}

    assert should_preempt(None, current)
    assert should_preempt(current, {"mode": "navigate", "xy": np.array([0.1, 0.0])})
    assert not should_preempt(current, {"mode": "explore", "xy": np.array([0.2, 0.0])}, preempt_dist=0.5)
    assert should_preempt(current, {"mode": "explore", "xy": np.array([0.6, 0.0])}, preempt_dist=0.5)
    assert not should_preempt(current, {"mode": "done", "xy": None})


def test_yaw_towards_uses_map_xy_without_y_flip() -> None:
    assert np.isclose(yaw_towards(np.array([0.0, 0.0]), np.array([0.0, 1.0])), np.pi / 2)


def test_object_point_cloud_map_clouds_are_per_instance() -> None:
    first = ObjectPointCloudMap(erosion_size=1)
    second = ObjectPointCloudMap(erosion_size=1)

    first.clouds["chair"] = np.ones((1, 4))

    assert "chair" not in second.clouds
