#!/usr/bin/env python3
# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.
"""G2/G3 mission runner: VLFM brain + Nav2 driver on the Limo sim.

This is the acceptance entry point for the full ``decide_goal`` loop (value map,
object detection, navigate preemption, arrival verify, reject, FOUND & VERIFIED,
memory). It wires the real ``LimoITMPolicy`` to ``LimoVLFMNode`` and hands both
to ``run_nav2_mission`` (the preemption loop from notes_zh/24).

Prereqs (the integration process runs the policy AND rclpy in one process):
  * source scripts/source_limo_ros_env.sh   (ROS Jazzy + mentor ws + venv)
  * torch importable in that env             (policy import needs it)
  * VLM servers up: ITM(12182) + detector(12184/12181); MobileSAM(12183) for the
    object-navigate path; BLIP2-VQA(12185) or attr-verifier(12186) for true G3.
Run scripts/limo_preflight.py first to see what is achievable on this box.

Config via env (or CLI: limo_mission.py <target_object> ["natural language query"]):
  VLFM_TARGET_OBJECT   target noun, e.g. "chair" (default)
  VLFM_NAV_QUERY       optional attribute instruction, e.g. "the black office chair"
  VLFM_USE_VQA         "1" to confirm detections with BLIP2-VQA (needs :12185)
  VLFM_DECIDE_HZ       re-decide rate during nav (default 1.0)
  VLFM_PREEMPT_DIST    frontier drift (m) before re-targeting (default 0.5)
  VLFM_OBJECT_MEMORY_PATH  json file for recall/remember across episodes
"""

import os
import sys

# Mentor limo_pro_sim topic/frame defaults (see notes_zh/25 and limo_g0_observation_check.py).
NODE_DEFAULTS = {
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

# LimoITMPolicy construction, mirroring config/experiments/reality.yaml.
POLICY_KWARGS = dict(
    text_prompt="Seems like there is a target_object ahead.",
    use_max_confidence=False,
    pointnav_policy_path="",  # unused: load_pointnav_policy=False inside LimoITMPolicy
    depth_image_shape=(212, 240),
    pointnav_stop_radius=0.9,
    object_map_erosion_size=5,
    obstacle_map_area_threshold=1.5,
    min_obstacle_height=0.1,
    max_obstacle_height=1.5,
    agent_radius=0.2,
    coco_threshold=float(os.environ.get("VLFM_COCO_THRESHOLD", "0.3")),
    visualize=False,
)


def _preflight_gate(target_is_coco: bool) -> None:
    """Refuse to start (with guidance) unless value-map exploration is runnable."""
    from limo_preflight import ATTR, GDINO, ITM, SAM, YOLO, check_imports, probe_port, summarize_scope

    ports_up = {p: probe_port(p) for p in (ITM, YOLO, SAM, GDINO, ATTR)}
    imports = check_imports()
    scope = summarize_scope(
        ports_up,
        torch_ok=imports["torch"],
        rclpy_ok=imports["rclpy"],
        nav2_ok=imports["nav2_simple_commander"],
        target_is_coco=target_is_coco,
    )
    print(f"[mission] achievable: {scope['level']}", flush=True)
    for item in scope["missing"]:  # type: ignore[union-attr]
        print(f"[mission]   missing: {item}", flush=True)
    if not scope["value_map_explore"]:
        print("[mission] aborting: the real policy cannot run here yet.", flush=True)
        sys.exit(1)
    if not scope["object_navigate"]:
        print("[mission] NOTE: MobileSAM down -> object-navigate disabled; "
              "exploration-only run (target detections cannot be localized).", flush=True)
    elif not scope["attribute_verify"]:
        print("[mission] NOTE: no verify server -> arrival verify fails open (accept w/o check).", flush=True)


def main() -> None:
    target = os.environ.get("VLFM_TARGET_OBJECT", "chair")
    query = os.environ.get("VLFM_NAV_QUERY", "")
    if len(sys.argv) > 1:
        target = sys.argv[1]
    if len(sys.argv) > 2:
        query = sys.argv[2]

    try:
        from vlfm.vlm.coco_classes import COCO_CLASSES

        target_is_coco = any(part in COCO_CLASSES for part in target.split("|"))
    except Exception:
        target_is_coco = True

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # find limo_preflight
    _preflight_gate(target_is_coco)

    # Imports deferred until after the gate: LimoITMPolicy pulls in torch.
    import rclpy
    from rclpy.parameter import Parameter

    from vlfm.policy.limo_policy import LimoITMPolicy
    from vlfm.ros.limo_vlfm_node import LimoVLFMNode, run_nav2_mission

    use_vqa = os.environ.get("VLFM_USE_VQA", "0") == "1"
    decide_hz = float(os.environ.get("VLFM_DECIDE_HZ", "1.0"))
    preempt_dist = float(os.environ.get("VLFM_PREEMPT_DIST", "0.5"))

    rclpy.init()
    overrides = [Parameter(name, value=value) for name, value in NODE_DEFAULTS.items()]
    node = LimoVLFMNode(parameter_overrides=overrides)
    policy = LimoITMPolicy(use_vqa=use_vqa, **POLICY_KWARGS)

    node.announce(f"[mission] target={target!r} query={query!r} use_vqa={use_vqa} "
                  f"decide_hz={decide_hz} preempt_dist={preempt_dist}")
    try:
        run_nav2_mission(
            node,
            policy,
            target_object=target,
            query=query,
            decide_hz=decide_hz,
            preempt_dist=preempt_dist,
        )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
