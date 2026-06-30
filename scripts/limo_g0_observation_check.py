#!/usr/bin/env python3
# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter

from vlfm.policy.limo_policy import yaw_towards
from vlfm.ros.limo_vlfm_node import LimoVLFMNode


def main() -> None:
    rclpy.init()
    # Defaults for mentor's limo_pro_sim workspace.
    defaults = {
        "rgb_topic": "/camera/image",
        "depth_topic": "/camera/depth_image",
        "camera_info_topic": "/camera/camera_info",
        "map_topic": "/map",
        "map_frame": "map",
        "base_frame": "base_footprint",
        "camera_frame": "depth_link",
        "min_depth": 0.3,
        "max_depth": 8.0,
        "use_sim_time": True,
    }
    overrides = [Parameter(name, value=value) for name, value in defaults.items()]
    node = LimoVLFMNode(parameter_overrides=overrides)

    last_log = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.time()
            if now - last_log < 1.0:
                continue
            last_log = now
            try:
                obs = node.build_observation()
            except Exception as e:
                node.get_logger().info(f"[g0] waiting for complete obs: {e}")
                continue

            robot_xy = np.asarray(obs["robot_xy"], dtype=np.float64)[:2]
            frontiers = np.asarray(obs["frontier_sensor"], dtype=np.float64)
            depth = obs["nav_depth"]
            valid = float(np.mean(depth < 1.0))
            if frontiers.size == 0:
                node.get_logger().info(
                    f"[g0] xy={np.round(robot_xy, 3).tolist()} yaw={obs['robot_heading']:.3f} "
                    f"frontiers=0 depth_valid={valid:.2f}"
                )
                continue

            frontiers = frontiers.reshape(-1, frontiers.shape[-1])[:, :2]
            node.publish_frontier_markers(frontiers)
            idx = int(np.argmin(np.linalg.norm(frontiers - robot_xy, axis=1)))
            goal_xy = frontiers[idx]
            goal = {
                "mode": "explore",
                "xy": goal_xy,
                "yaw_hint": yaw_towards(robot_xy, goal_xy),
                "value": 0.0,
                "stop_radius": 0.0,
            }
            node.publish_goal_marker(goal)
            node.get_logger().info(
                f"[g0] xy={np.round(robot_xy, 3).tolist()} yaw={obs['robot_heading']:.3f} "
                f"frontiers={len(frontiers)} nearest={np.round(goal_xy, 3).tolist()} "
                f"goal_yaw={goal['yaw_hint']:.3f} depth_valid={valid:.2f}"
            )
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
