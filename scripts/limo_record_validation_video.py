#!/usr/bin/env python3
# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.
"""Record a compact visual proof video for the Limo VLFM online validation.

The video is intentionally self-contained: it combines the robot RGB camera,
VLFM-native debug panels (annotated RGB, obstacle map, value map, object cloud
map), a small odom trace, and mission log text. It does not command the robot;
it only subscribes.
"""

from __future__ import annotations

import argparse
import math
import os
import textwrap
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image


KEY_PATTERNS = (
    "[mission]",
    "[limo]",
    "Navigating to goal",
    "Reached the goal",
    "Goal succeeded",
    "[attr]",
    "[arrive]",
    "FOUND & VERIFIED",
    "FAILED",
    "CANCELED",
)


PANEL_BG = (18, 18, 18)
PANEL_FG = (240, 240, 240)
PANEL_MUTED = (180, 210, 235)


def _yaw_from_quat(q: object) -> float:
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    return float(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _tail_key_lines(path: Optional[Path], max_lines: int = 7) -> list[str]:
    if path is None or not path.exists():
        return []
    try:
        text = path.read_text(errors="replace")
    except OSError:
        return []
    lines = []
    for line in text.splitlines():
        clean = line.replace("\x1b[0m", "").strip()
        if any(pat in clean for pat in KEY_PATTERNS):
            lines.append(clean)
    return lines[-max_lines:]


def _log_contains(path: Optional[Path], needle: str) -> bool:
    if path is None or not path.exists():
        return False
    try:
        return needle in path.read_text(errors="replace")
    except OSError:
        return False


def _draw_label(img: np.ndarray, label: str) -> None:
    cv2.rectangle(img, (0, 0), (img.shape[1], 30), (0, 0, 0), -1)
    cv2.putText(img, label, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (245, 245, 245), 1, cv2.LINE_AA)


def _placeholder(size: tuple[int, int], text: str) -> np.ndarray:
    w, h = size
    img = np.full((h, w, 3), PANEL_BG, dtype=np.uint8)
    cv2.putText(img, text, (18, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (210, 210, 210), 1, cv2.LINE_AA)
    return img


def _fit_panel(img: Optional[np.ndarray], size: tuple[int, int], label: str, placeholder: str) -> np.ndarray:
    w, h = size
    if img is None:
        panel = _placeholder(size, placeholder)
        _draw_label(panel, label)
        return panel

    arr = np.asarray(img)
    if arr.ndim == 2:
        arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)

    ih, iw = arr.shape[:2]
    scale = min(w / max(iw, 1), h / max(ih, 1))
    nw, nh = max(1, int(iw * scale)), max(1, int(ih * scale))
    resized = cv2.resize(arr, (nw, nh), interpolation=cv2.INTER_AREA)
    panel = np.full((h, w, 3), PANEL_BG, dtype=np.uint8)
    x0 = (w - nw) // 2
    y0 = (h - nh) // 2
    panel[y0 : y0 + nh, x0 : x0 + nw] = resized
    _draw_label(panel, label)
    return panel


def _draw_wrapped_lines(
    panel: np.ndarray,
    lines: list[str],
    x: int,
    y: int,
    width_chars: int,
    line_height: int,
    max_rows: int,
) -> None:
    rows: list[str] = []
    for line in lines:
        rows.extend(textwrap.wrap(line, width=width_chars) or [""])
    for row in rows[-max_rows:]:
        cv2.putText(panel, row[-width_chars:], (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (230, 230, 230), 1, cv2.LINE_AA)
        y += line_height


class LimoValidationRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("limo_validation_recorder")
        self.args = args
        self.bridge = CvBridge()
        self.rgb: Optional[np.ndarray] = None
        self.odom_xy: Optional[np.ndarray] = None
        self.odom_yaw = 0.0
        self.goal_xy: Optional[np.ndarray] = None
        self.goal_yaw = 0.0
        self.cmd = Twist()
        self.path_xy: list[np.ndarray] = []
        self.annotated_rgb: Optional[np.ndarray] = None
        self.obstacle_map: Optional[np.ndarray] = None
        self.value_map: Optional[np.ndarray] = None
        self.object_map: Optional[np.ndarray] = None

        self.create_subscription(Image, args.rgb_topic, self._rgb_cb, 5)
        self.create_subscription(Odometry, args.odom_topic, self._odom_cb, 20)
        self.create_subscription(PoseStamped, args.goal_topic, self._goal_cb, 10)
        self.create_subscription(Twist, args.cmd_topic, self._cmd_cb, 20)
        self.create_subscription(Image, args.annotated_rgb_topic, self._vis_cb("annotated_rgb"), 5)
        self.create_subscription(Image, args.obstacle_map_topic, self._vis_cb("obstacle_map"), 5)
        self.create_subscription(Image, args.value_map_topic, self._vis_cb("value_map"), 5)
        self.create_subscription(Image, args.object_map_topic, self._vis_cb("object_map"), 5)

    def _rgb_cb(self, msg: Image) -> None:
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.odom_xy = np.array([float(p.x), float(p.y)], dtype=np.float64)
        self.odom_yaw = _yaw_from_quat(msg.pose.pose.orientation)
        if not self.path_xy or np.linalg.norm(self.odom_xy - self.path_xy[-1]) > 0.03:
            self.path_xy.append(self.odom_xy.copy())

    def _goal_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.goal_xy = np.array([float(p.x), float(p.y)], dtype=np.float64)
        self.goal_yaw = _yaw_from_quat(msg.pose.orientation)

    def _cmd_cb(self, msg: Twist) -> None:
        self.cmd = msg

    def _vis_cb(self, name: str):
        def cb(msg: Image) -> None:
            setattr(self, name, self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"))

        return cb

    def frame(self, start_time: float, mission_log: Optional[Path]) -> np.ndarray:
        canvas = np.full((720, 1280, 3), 10, dtype=np.uint8)
        canvas[0:288, 0:512] = self._camera_panel((512, 288))
        canvas[288:576, 0:512] = _fit_panel(
            self.annotated_rgb,
            (512, 288),
            "VLFM annotated RGB (YOLO/SAM masks)",
            "Waiting for /vlfm/vis/annotated_rgb",
        )
        canvas[576:720, 0:512] = self._status_panel((512, 144), start_time, mission_log)

        canvas[0:360, 512:896] = _fit_panel(
            self.obstacle_map,
            (384, 360),
            "VLFM obstacle map + frontiers",
            "Waiting for /vlfm/vis/obstacle_map",
        )
        canvas[0:360, 896:1280] = _fit_panel(
            self.value_map,
            (384, 360),
            "VLFM value map",
            "Waiting for /vlfm/vis/value_map",
        )
        canvas[360:720, 512:896] = _fit_panel(
            self.object_map,
            (384, 360),
            "VLFM object cloud / selected goal",
            "Waiting for /vlfm/vis/object_map",
        )
        canvas[360:720, 896:1280] = self._odom_panel((384, 360))
        return canvas

    def _camera_panel(self, size: tuple[int, int]) -> np.ndarray:
        w, h = size
        if self.rgb is None:
            panel = _placeholder(size, "Waiting for /camera/image")
            _draw_label(panel, "Robot RGB camera")
            return panel
        return _fit_panel(cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR), size, "Robot RGB camera", "")

    def _status_panel(self, size: tuple[int, int], start_time: float, mission_log: Optional[Path]) -> np.ndarray:
        w, h = size
        panel = np.full((h, w, 3), (25, 25, 25), dtype=np.uint8)
        cv2.putText(panel, "Limo VLFM validation", (16, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, PANEL_FG, 1, cv2.LINE_AA)
        elapsed = time.time() - start_time
        cv2.putText(panel, f"t={elapsed:5.1f}s", (420, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, PANEL_MUTED, 1, cv2.LINE_AA)
        y = 52
        if self.odom_xy is not None:
            cv2.putText(panel, f"odom=({self.odom_xy[0]:.2f},{self.odom_xy[1]:.2f}) yaw={self.odom_yaw:.2f}", (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.43, PANEL_FG, 1, cv2.LINE_AA)
        else:
            cv2.putText(panel, "odom=(waiting)", (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.43, PANEL_FG, 1, cv2.LINE_AA)
        y += 20
        if self.goal_xy is not None:
            cv2.putText(panel, f"goal=({self.goal_xy[0]:.2f},{self.goal_xy[1]:.2f}) yaw={self.goal_yaw:.2f}", (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.43, PANEL_FG, 1, cv2.LINE_AA)
        else:
            cv2.putText(panel, "goal=(waiting)", (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.43, PANEL_FG, 1, cv2.LINE_AA)
        y += 20
        cv2.putText(panel, f"cmd=({self.cmd.linear.x:.2f}, {self.cmd.angular.z:.2f})", (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.43, PANEL_FG, 1, cv2.LINE_AA)
        _draw_wrapped_lines(panel, _tail_key_lines(mission_log, max_lines=4), 16, 112, 72, 16, 2)
        return panel

    def _odom_panel(self, size: tuple[int, int]) -> np.ndarray:
        w, h = size
        panel = np.full((h, w, 3), (238, 238, 238), dtype=np.uint8)
        _draw_label(panel, "ROS odom trace + /vlfm/goal")
        margin = 34
        x0, y0 = margin, 42
        map_w, map_h = w - 2 * margin, h - 58
        cv2.rectangle(panel, (x0, y0), (x0 + map_w, y0 + map_h), (248, 248, 248), -1)
        cv2.rectangle(panel, (x0, y0), (x0 + map_w, y0 + map_h), (80, 80, 80), 1)

        points = list(self.path_xy)
        if self.goal_xy is not None:
            points.append(self.goal_xy)
        if self.odom_xy is not None:
            points.append(self.odom_xy)
        if not points:
            cv2.putText(panel, "waiting for /odom", (x0 + 55, y0 + map_h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (80, 80, 80), 1)
            return panel

        arr = np.vstack(points)
        center = arr.mean(axis=0)
        span = np.maximum(arr.max(axis=0) - arr.min(axis=0), 4.0)
        scale = 0.82 * min(map_w, map_h) / float(max(span[0], span[1]))

        def to_px(xy: np.ndarray) -> tuple[int, int]:
            dx, dy = (xy - center) * scale
            return int(x0 + map_w / 2 + dx), int(y0 + map_h / 2 - dy)

        if len(self.path_xy) >= 2:
            pts = np.array([to_px(p) for p in self.path_xy], dtype=np.int32)
            cv2.polylines(panel, [pts], False, (30, 130, 220), 2)
        for tick in range(-4, 5, 2):
            cx = int(x0 + map_w / 2 + tick * scale)
            cy = int(y0 + map_h / 2 - tick * scale)
            if x0 <= cx <= x0 + map_w:
                cv2.line(panel, (cx, y0), (cx, y0 + map_h), (222, 222, 222), 1)
            if y0 <= cy <= y0 + map_h:
                cv2.line(panel, (x0, cy), (x0 + map_w, cy), (222, 222, 222), 1)

        if self.goal_xy is not None:
            gx, gy = to_px(self.goal_xy)
            cv2.drawMarker(panel, (gx, gy), (30, 30, 230), cv2.MARKER_TILTED_CROSS, 22, 2)
            cv2.putText(panel, "goal", (gx + 8, gy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (30, 30, 180), 1)
        if self.odom_xy is not None:
            rx, ry = to_px(self.odom_xy)
            cv2.circle(panel, (rx, ry), 7, (30, 160, 30), -1)
            hx = int(rx + 18 * math.cos(self.odom_yaw))
            hy = int(ry - 18 * math.sin(self.odom_yaw))
            cv2.line(panel, (rx, ry), (hx, hy), (20, 100, 20), 2)
            cv2.putText(panel, "robot", (rx + 8, ry + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 100, 20), 1)
        return panel


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="outputs/limo_g3_validation.mp4")
    parser.add_argument("--mission-log", default="")
    parser.add_argument("--duration", type=float, default=240.0)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--speedup", type=float, default=4.0, help="Playback speed multiplier; 4.0 makes a 4x video.")
    parser.add_argument("--rgb-topic", default="/camera/image")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--goal-topic", default="/vlfm/goal")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--annotated-rgb-topic", default="/vlfm/vis/annotated_rgb")
    parser.add_argument("--obstacle-map-topic", default="/vlfm/vis/obstacle_map")
    parser.add_argument("--value-map-topic", default="/vlfm/vis/value_map")
    parser.add_argument("--object-map-topic", default="/vlfm/vis/object_map")
    parser.add_argument("--stop-on-found", action="store_true")
    args = parser.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    mission_log = Path(args.mission_log) if args.mission_log else None

    rclpy.init()
    node = LimoValidationRecorder(args)
    writer_fps = args.fps * max(args.speedup, 1e-6)
    writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), writer_fps, (1280, 720))
    if not writer.isOpened():
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(f"Could not open video writer for {out_path}")

    start = time.time()
    frame_period = 1.0 / max(args.fps, 1e-6)
    next_frame = start
    try:
        while rclpy.ok() and time.time() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.time()
            if now >= next_frame:
                writer.write(node.frame(start, mission_log))
                next_frame += frame_period
            if args.stop_on_found and _log_contains(mission_log, "FOUND & VERIFIED"):
                # Keep a short tail after the success line so it is visible.
                tail_until = time.time() + 3.0
                while rclpy.ok() and time.time() < tail_until:
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if time.time() >= next_frame:
                        writer.write(node.frame(start, mission_log))
                        next_frame += frame_period
                break
    except KeyboardInterrupt:
        pass
    finally:
        writer.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
