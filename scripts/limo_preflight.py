#!/usr/bin/env python3
# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.
"""Preflight check for the Limo/Nav2 VLFM mission (G2/G3).

Run this BEFORE ``limo_mission.py`` to learn, in one shot, exactly which part of
the pipeline is runnable on the current box:

  * which VLM servers are up (ITM / YOLO / SAM / GroundingDINO / VQA / attr),
  * whether the *integration process* env has what it needs (rclpy + torch +
    nav2_simple_commander + the vlfm package),
  * and therefore which milestone is achievable right now:

      value-map exploration  -> needs ITM(12182) + YOLO(12184) + torch + rclpy
      object navigate (COCO)  -> + MobileSAM(12183)
      attribute verify (G3)   -> + attr-verifier 12186 (Bailian/cloud VLM)

The scope logic is pure (``summarize_scope``) so it can be unit-tested without
ROS or torch installed.
"""

import os
import socket
import sys
from typing import Dict, List

# server_wrapper ports (defaults mirror vlfm.policy.* and launch_vlm_servers_*.sh)
PORTS = {
    "ITM (SigLIP2/BLIP2, value map)": int(os.environ.get("BLIP2ITM_PORT", "12182")),
    "YOLO26 (COCO detection)": int(os.environ.get("YOLOV7_PORT", "12184")),
    "MobileSAM (object mask -> object navigate)": int(os.environ.get("SAM_PORT", "12183")),
    "GroundingDINO (non-COCO det)": int(os.environ.get("GROUNDING_DINO_PORT", "12181")),
    "AttrVerifier (cloud VLM verify)": int(os.environ.get("ATTR_VERIFIER_PORT", "12186")),
    "BLIP2-VQA (legacy optional det confirm)": int(os.environ.get("BLIP2_PORT", "12185")),
}

# Friendly key -> port, used by summarize_scope (decoupled from display labels).
ITM, YOLO, SAM, GDINO, VQA, ATTR = 12182, 12184, 12183, 12181, 12185, 12186


def probe_port(port: int, host: str = "127.0.0.1", timeout: float = 1.0) -> bool:
    """True if something is listening on ``host:port``."""
    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def check_imports() -> Dict[str, bool]:
    """Which integration-process Python deps are importable here."""
    names = {
        "rclpy": "rclpy",
        "cv_bridge": "cv_bridge",
        "nav2_simple_commander": "nav2_simple_commander.robot_navigator",
        "torch": "torch",
        "frontier_exploration": "frontier_exploration",
        "vlfm": "vlfm.ros.limo_vlfm_node",
    }
    out: Dict[str, bool] = {}
    for label, module in names.items():
        try:
            __import__(module)
            out[label] = True
        except Exception:
            out[label] = False
    return out


def summarize_scope(
    ports_up: Dict[int, bool],
    torch_ok: bool,
    rclpy_ok: bool,
    nav2_ok: bool,
    target_is_coco: bool = True,
) -> Dict[str, object]:
    """Pure decision: which milestone is runnable given servers + deps.

    Returns a dict with per-level booleans, the highest achievable ``level``,
    and the ``missing`` items blocking the next level up.
    """
    process_ok = torch_ok and rclpy_ok and nav2_ok
    detector_up = ports_up.get(YOLO, False) if target_is_coco else ports_up.get(GDINO, False)

    value_map_explore = process_ok and ports_up.get(ITM, False) and detector_up
    object_navigate = value_map_explore and ports_up.get(SAM, False)
    # Real cloud VLM verify uses the attr-verifier (:12186). Without it the policy
    # still runs the verify/reject/FOUND flow via a LOCAL heuristic fallback, so
    # object_navigate alone already exercises the full G3 path (weaker verdict).
    attribute_verify = object_navigate and ports_up.get(ATTR, False)

    if object_navigate and attribute_verify:
        level = "G3: full object navigate + cloud attribute verify"
    elif object_navigate:
        level = "G3 (heuristic verify): object navigate + reject/found flow; :12186 down -> local heuristic verdict"
    elif value_map_explore:
        level = "G2-lite: value-map-guided exploration only (no object navigate)"
    else:
        level = "blocked: cannot run the real policy"

    missing: List[str] = []
    if not process_ok:
        if not torch_ok:
            missing.append("torch (integration process needs it to import the policy)")
        if not rclpy_ok:
            missing.append("rclpy")
        if not nav2_ok:
            missing.append("nav2_simple_commander")
    if not ports_up.get(ITM, False):
        missing.append(f"ITM server :{ITM}")
    if not detector_up:
        missing.append(f"detector server :{YOLO if target_is_coco else GDINO}")
    if value_map_explore and not object_navigate:
        missing.append(f"MobileSAM :{SAM} (required to localize a detected object -> object navigate)")
    if object_navigate and not attribute_verify:
        missing.append(f"AttrVerifier :{ATTR} (cloud VLM verify; without it verify uses a local heuristic)")

    return {
        "value_map_explore": value_map_explore,
        "object_navigate": object_navigate,
        "attribute_verify": attribute_verify,
        "level": level,
        "missing": missing,
    }


def main() -> int:
    target = os.environ.get("VLFM_TARGET_OBJECT", "chair")
    # Treat anything in the COCO set as COCO; cheap heuristic without importing torch.
    try:
        from vlfm.vlm.coco_classes import COCO_CLASSES

        target_is_coco = any(part in COCO_CLASSES for part in target.split("|"))
    except Exception:
        target_is_coco = True

    print("=== Limo/Nav2 VLFM preflight ===")
    print(f"target object: {target!r}  (COCO={target_is_coco})\n")

    print("VLM servers:")
    ports_up: Dict[int, bool] = {}
    for label, port in PORTS.items():
        up = probe_port(port)
        ports_up[port] = up
        print(f"  [{'UP  ' if up else 'DOWN'}] :{port}  {label}")

    print("\nIntegration-process Python deps:")
    imports = check_imports()
    for label, ok in imports.items():
        print(f"  [{'OK  ' if ok else 'MISS'}] {label}")

    scope = summarize_scope(
        ports_up,
        torch_ok=imports["torch"],
        rclpy_ok=imports["rclpy"],
        nav2_ok=imports["nav2_simple_commander"],
        target_is_coco=target_is_coco,
    )

    print(f"\n>>> Achievable now: {scope['level']}")
    if scope["missing"]:
        print("    To reach the next level, bring up / install:")
        for item in scope["missing"]:  # type: ignore[union-attr]
            print(f"      - {item}")

    return 0 if scope["value_map_explore"] else 1


if __name__ == "__main__":
    sys.exit(main())
