# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import math
import os
from typing import Any, Dict, List, Optional, Union

import numpy as np

try:
    from vlfm.policy.itm_policy import ITMPolicyV2
except ModuleNotFoundError as e:  # pragma: no cover - only for lightweight offline tests
    if e.name != "torch":
        raise

    class ITMPolicyV2:  # type: ignore[no-redef]
        pass


GoalDecision = Dict[str, Any]


def yaw_towards(source_xy: np.ndarray, target_xy: np.ndarray) -> float:
    """Yaw in the map/episodic frame that faces ``target_xy`` from ``source_xy``."""
    source = np.asarray(source_xy, dtype=np.float64)[:2]
    target = np.asarray(target_xy, dtype=np.float64)[:2]
    delta = target - source
    return float(math.atan2(delta[1], delta[0]))


def should_preempt(
    current: Optional[GoalDecision],
    candidate: GoalDecision,
    preempt_dist: float = 0.5,
) -> bool:
    """M5 preemption policy: object goals beat exploration; frontier drift is damped."""
    if candidate.get("mode") == "done":
        return False
    if current is None:
        return True
    if candidate.get("xy") is None:
        return False
    if candidate["mode"] in ("navigate", "navigate-memory") and current.get("mode") == "explore":
        return True
    if candidate["mode"] == "explore" and current.get("mode") == "explore":
        cur_xy = np.asarray(current["xy"], dtype=np.float64)[:2]
        new_xy = np.asarray(candidate["xy"], dtype=np.float64)[:2]
        return bool(np.linalg.norm(new_xy - cur_xy) > preempt_dist)
    return False


class LimoITMPolicy(ITMPolicyV2):
    """ITM policy wrapper for the Limo/Nav2 coupling.

    This class keeps the VLFM perception, value map, object memory, attribute
    verification, and reject-region logic, but never emits PointNav/Habitat
    actions. Its public API returns map-frame goals for Nav2.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        kwargs.setdefault("compute_frontiers", False)
        kwargs.setdefault("load_pointnav_policy", False)
        super().__init__(*args, **kwargs)
        self._blocked_frontiers: List[np.ndarray] = []

    def _cache_observations(self, observations: Any) -> None:
        self._observations_cache = observations

    def _infer_depth(self, rgb: np.ndarray, min_depth: float, max_depth: float) -> np.ndarray:
        height, width = rgb.shape[:2]
        return np.ones((height, width), dtype=np.float32)

    def reset_episode(self, target_object: str, query: str = "") -> None:
        self._reset()
        if query:
            os.environ["VLFM_NAV_QUERY"] = query
        self._target_object = target_object
        self._configure_attribute_query(target_object)
        self._maybe_recall_object_memory()
        self._blocked_frontiers = []
        self._num_steps = 0
        self._did_reset = True

    def decide_goal(self, obs_cache: Dict[str, Any]) -> GoalDecision:
        self._observations_cache = obs_cache
        self._policy_info = {}

        self._update_object_maps_from_cache(obs_cache)
        if obs_cache.get("value_map_rgbd"):
            self._update_value_map()

        robot_xy = np.asarray(obs_cache["robot_xy"], dtype=np.float64)[:2]
        object_goal = self._get_target_object_location(robot_xy)
        if object_goal is not None:
            nav_goal = self._object_approach_goal(robot_xy, object_goal[:2])
            return self._finish_goal(self._make_goal("navigate", nav_goal, robot_xy, value=1.0))

        if self._remembered_goal is not None:
            return self._finish_goal(
                self._make_goal("navigate-memory", self._remembered_goal[:2], robot_xy, value=0.5)
            )

        frontiers = self._filter_blocked_frontiers(obs_cache.get("frontier_sensor"))
        if len(frontiers) == 0:
            return self._finish_goal(
                {
                    "mode": "done",
                    "xy": None,
                    "yaw_hint": 0.0,
                    "value": 0.0,
                    "stop_radius": 0.0,
                }
            )

        best_frontier, best_value = self._get_best_frontier(obs_cache, frontiers)
        return self._finish_goal(
            self._make_goal("explore", best_frontier[:2], robot_xy, value=float(best_value))
        )

    def on_goal_reached(self, obs_cache: Dict[str, Any], goal: GoalDecision) -> Dict[str, Any]:
        self._observations_cache = obs_cache
        mode = goal.get("mode", "")
        if mode == "explore":
            self._block_frontier(goal)
            return {"accepted": True, "reason": "frontier reached", "next": "explore"}

        if mode not in ("navigate", "navigate-memory"):
            return {"accepted": True, "reason": f"{mode or 'goal'} reached", "next": "explore"}

        robot_xy = np.asarray(obs_cache["robot_xy"], dtype=np.float64)[:2]
        verdict = self._attribute_match(obs_cache, robot_xy)
        if verdict is False:
            self._reject_goal_region(goal)
            self._reset_per_goal_nav()
            self._last_verify_result = "attr mismatch -> reject"
            return {"accepted": False, "reason": self._last_verify_result, "next": "explore"}

        self._attribute_verified = True
        self._called_stop = True
        loc = self._get_target_object_location(robot_xy)
        goal_xy = np.asarray(goal["xy"], dtype=np.float64)[:2]
        self._maybe_remember_object(loc if loc is not None else goal_xy)
        return {"accepted": True, "reason": self._last_verify_result or "accepted", "next": "done"}

    def on_goal_unreachable(self, goal: GoalDecision) -> None:
        if goal.get("xy") is None:
            return
        if goal.get("mode") in ("navigate", "navigate-memory"):
            self._reject_goal_region(goal)
        else:
            self._block_frontier(goal)
        self._reset_per_goal_nav()

    def _block_frontier(self, goal: GoalDecision) -> None:
        if goal.get("xy") is None:
            return
        self._blocked_frontiers.append(np.asarray(goal["xy"], dtype=np.float64)[:2].copy())

    def _update_object_maps_from_cache(self, obs_cache: Dict[str, Any]) -> None:
        for rgb, depth, tf_cam2map, min_depth, max_depth, fx, fy in obs_cache.get("object_map_rgbd", []):
            self._update_object_map(rgb, depth, tf_cam2map, min_depth, max_depth, fx, fy)

    def _filter_blocked_frontiers(self, frontiers: Union[np.ndarray, None]) -> np.ndarray:
        if frontiers is None:
            return np.empty((0, 2), dtype=np.float64)
        arr = np.asarray(frontiers, dtype=np.float64)
        if arr.size == 0:
            return np.empty((0, 2), dtype=np.float64)
        arr = arr.reshape(-1, arr.shape[-1])[:, :2]
        if not self._blocked_frontiers:
            return arr
        block_radius = float(os.environ.get("VLFM_FRONTIER_BLOCK_RADIUS", "0.5"))
        keep = np.ones(len(arr), dtype=bool)
        for blocked in self._blocked_frontiers:
            keep &= np.linalg.norm(arr - blocked[:2], axis=1) > block_radius
        return arr[keep]

    def _make_goal(self, mode: str, xy: np.ndarray, robot_xy: np.ndarray, value: float) -> GoalDecision:
        xy = np.asarray(xy, dtype=np.float64)[:2]
        self._last_goal = xy.copy()
        return {
            "mode": mode,
            "xy": xy,
            "yaw_hint": yaw_towards(robot_xy, xy),
            "value": float(value),
            "stop_radius": float(self._pointnav_stop_radius),
        }

    def _object_approach_goal(self, robot_xy: np.ndarray, object_xy: np.ndarray) -> np.ndarray:
        """Back off from the object center so Nav2 targets reachable floor near it."""
        robot_xy = np.asarray(robot_xy, dtype=np.float64)[:2]
        object_xy = np.asarray(object_xy, dtype=np.float64)[:2]
        delta = object_xy - robot_xy
        dist = float(np.linalg.norm(delta))
        approach = float(os.environ.get("VLFM_OBJECT_APPROACH_DIST", str(self._pointnav_stop_radius)))
        if dist <= max(approach, 1e-6):
            return object_xy
        return object_xy - delta / dist * approach

    def _finish_goal(self, goal: GoalDecision) -> GoalDecision:
        xy = goal.get("xy")
        xy_text = None if xy is None else np.round(np.asarray(xy, dtype=np.float64)[:2], 3).tolist()
        print(
            f"[limo] step={self._num_steps} mode={goal['mode']} xy={xy_text} "
            f"value={goal['value']:.3f} yaw={goal['yaw_hint']:.3f}",
            flush=True,
        )
        self._num_steps += 1
        return goal

    def _reject_goal_region(self, goal: GoalDecision) -> None:
        if goal.get("xy") is None or not self._target_object:
            return
        radius = float(os.environ.get("VLFM_ATTR_REJECT_RADIUS", "0.6"))
        xy = np.asarray(goal["xy"], dtype=np.float64)[:2]
        self._object_map.reject_region(self._target_object, xy, radius=radius)
        print(
            f"[attr] reject {self._target_object!r} around {np.round(xy, 3).tolist()} r={radius:.2f}",
            flush=True,
        )
