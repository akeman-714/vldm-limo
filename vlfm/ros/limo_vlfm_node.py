# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import math
import os
import time
from typing import Any, Optional

import cv2
import numpy as np

from vlfm.mapping.obstacle_map import ObstacleMap
from vlfm.policy.limo_policy import should_preempt
from vlfm.utils.geometry_utils import get_fov, xyz_yaw_to_tf_matrix

try:
    import message_filters
    import rclpy
    import tf2_ros
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseStamped, Quaternion
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from visualization_msgs.msg import Marker, MarkerArray
except Exception as e:  # pragma: no cover - exercised on ROS machines
    message_filters = None
    rclpy = None
    tf2_ros = None
    CvBridge = None
    PoseStamped = None
    Quaternion = None
    BasicNavigator = None
    TaskResult = None
    OccupancyGrid = None
    Odometry = None
    QoSProfile = None
    ReliabilityPolicy = None
    DurabilityPolicy = None
    CameraInfo = None
    Image = None
    Marker = None
    MarkerArray = None
    Node = object
    _ROS_IMPORT_ERROR: Optional[Exception] = e
else:  # pragma: no cover - exercised on ROS machines
    _ROS_IMPORT_ERROR = None


def normalize_depth_array(
    depth_raw: np.ndarray,
    encoding: str,
    min_depth: float,
    max_depth: float,
) -> np.ndarray:
    d_m = np.asarray(depth_raw).astype(np.float32)
    if encoding == "16UC1" or np.issubdtype(np.asarray(depth_raw).dtype, np.integer):
        d_m *= 0.001
    d = (d_m - float(min_depth)) / (float(max_depth) - float(min_depth))
    d = np.clip(d, 0.0, 1.0)
    invalid = (d_m <= 0) | ~np.isfinite(d_m)
    d[invalid] = 1.0
    return d.astype(np.float32)


def yaw_from_quaternion(q: Any) -> float:
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(math.atan2(siny_cosp, cosy_cosp))


def quaternion_from_yaw(yaw: float) -> Any:
    if Quaternion is None:
        return {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(float(yaw) / 2.0),
            "w": math.cos(float(yaw) / 2.0),
        }
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(float(yaw) / 2.0)
    q.w = math.cos(float(yaw) / 2.0)
    return q


def transform_to_matrix(tf_msg: Any) -> np.ndarray:
    """Convert a ROS TransformStamped-like object to a 4x4 matrix."""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    return translation_rotation_to_matrix(t, q)


def pose_to_matrix(pose_msg: Any) -> np.ndarray:
    """Convert a ROS Pose-like object to a 4x4 matrix."""
    return translation_rotation_to_matrix(pose_msg.position, pose_msg.orientation)


def translation_rotation_to_matrix(translation: Any, rotation: Any) -> np.ndarray:
    t = translation
    q = rotation
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        x = y = z = 0.0
        w = 1.0
    else:
        x /= norm
        y /= norm
        z /= norm
        w /= norm

    mat = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), float(t.x)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), float(t.y)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), float(t.z)],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return mat


def pose_from_matrix(tf_matrix: np.ndarray) -> tuple[np.ndarray, float]:
    xyz = np.asarray(tf_matrix[:3, 3], dtype=np.float64)
    yaw = float(math.atan2(tf_matrix[1, 0], tf_matrix[0, 0]))
    return xyz, yaw


def build_limo_observation_cache(
    rgb: np.ndarray,
    depth_raw: np.ndarray,
    depth_encoding: str,
    fx: float,
    fy: float,
    grid: np.ndarray,
    grid_resolution: float,
    grid_origin_xy: np.ndarray,
    base_xyz: np.ndarray,
    base_yaw: float,
    cam_xyz: np.ndarray,
    cam_yaw: float,
    obstacle_map: ObstacleMap,
    min_depth: float,
    max_depth: float,
) -> dict:
    depth = normalize_depth_array(depth_raw, depth_encoding, min_depth, max_depth)
    # depth_link is a ROS optical frame (z forward, x right, y down), while
    # geometry_utils.get_point_cloud() returns VLFM/Habitat-style points
    # (x forward, y left, z up).  Use the robot/body yaw for that corrected
    # point cloud; using the optical frame yaw rotates object goals by the
    # camera optical-frame offset.
    tf_cam2map = xyz_yaw_to_tf_matrix(np.asarray(cam_xyz, dtype=np.float64), base_yaw)
    obstacle_map.update_from_occupancy_grid(
        np.asarray(grid, dtype=np.int8),
        grid_resolution,
        np.asarray(grid_origin_xy, dtype=np.float64),
    )
    obstacle_map.update_agent_traj(np.asarray(base_xyz, dtype=np.float64)[:2], base_yaw)
    fov = get_fov(float(fx), depth.shape[1])
    return {
        "frontier_sensor": obstacle_map.frontiers,
        "robot_xy": np.asarray(base_xyz, dtype=np.float64)[:2],
        "robot_heading": float(base_yaw),
        "nav_depth": depth,
        "object_map_rgbd": [(rgb, depth, tf_cam2map, min_depth, max_depth, fx, fy)],
        "value_map_rgbd": [(rgb, depth, tf_cam2map, min_depth, max_depth, fov)],
    }


def _as_bgr_uint8(img: Any) -> Optional[np.ndarray]:
    if img is None:
        return None
    arr = np.asarray(img)
    if arr.ndim == 2:
        arr = cv2.cvtColor(arr.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    elif arr.ndim == 3 and arr.shape[2] == 4:
        arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    elif arr.ndim != 3 or arr.shape[2] != 3:
        return None
    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(arr)


def _xy_to_flipped_vis_px(obstacle_map: ObstacleMap, xy: np.ndarray) -> tuple[int, int]:
    px = obstacle_map._xy_to_px(np.asarray(xy, dtype=np.float64).reshape(1, 2))[0]
    return int(px[0]), int(obstacle_map.size - 1 - px[1])


def _draw_goal_marker(img: np.ndarray, obstacle_map: ObstacleMap, goal: dict, color: tuple[int, int, int]) -> None:
    if goal.get("xy") is None:
        return
    x, y = _xy_to_flipped_vis_px(obstacle_map, np.asarray(goal["xy"], dtype=np.float64)[:2])
    if 0 <= x < img.shape[1] and 0 <= y < img.shape[0]:
        cv2.drawMarker(img, (x, y), color, cv2.MARKER_TILTED_CROSS, 22, 2)


def _build_annotated_rgb(rgb: Optional[np.ndarray], policy: Any) -> Optional[np.ndarray]:
    if rgb is None:
        return None
    img = cv2.cvtColor(np.asarray(rgb, dtype=np.uint8), cv2.COLOR_RGB2BGR)
    masks = getattr(policy, "_object_masks", None)
    if masks is not None and np.asarray(masks).size and np.asarray(masks).sum() > 0:
        contours, _ = cv2.findContours(np.asarray(masks, dtype=np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, (255, 0, 0), 2)
    bbox = getattr(policy, "_last_target_bbox", None)
    if bbox is not None:
        x1, y1, x2, y2 = [int(v) for v in np.asarray(bbox).reshape(-1)[:4]]
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 220, 255), 2)
    return img


def _build_value_map(policy: Any, obs_cache: dict, goal: dict) -> Optional[np.ndarray]:
    value_map = getattr(policy, "_value_map", None)
    if value_map is None:
        return None

    markers = []
    frontiers = np.asarray(obs_cache.get("frontier_sensor", []), dtype=np.float64)
    if frontiers.size:
        frontiers = frontiers.reshape(-1, frontiers.shape[-1])[:, :2]
        for frontier in frontiers:
            markers.append(
                (
                    frontier,
                    {
                        "radius": getattr(policy, "_circle_marker_radius", 5),
                        "thickness": getattr(policy, "_circle_marker_thickness", 2),
                        "color": getattr(policy, "_frontier_color", (0, 0, 255)),
                    },
                )
            )
    if goal.get("xy") is not None:
        markers.append(
            (
                np.asarray(goal["xy"], dtype=np.float64)[:2],
                {
                    "radius": getattr(policy, "_circle_marker_radius", 5),
                    "thickness": getattr(policy, "_circle_marker_thickness", 2),
                    "color": getattr(policy, "_selected__frontier_color", (0, 255, 255)),
                },
            )
        )

    reduce_fn = getattr(policy, "_vis_reduce_fn", lambda i: np.max(i, axis=-1))
    return _as_bgr_uint8(value_map.visualize(markers, reduce_fn=reduce_fn))


def _build_object_map(obstacle_map: ObstacleMap, policy: Any, goal: dict) -> Optional[np.ndarray]:
    img = _as_bgr_uint8(obstacle_map.visualize())
    if img is None:
        return None

    object_map = getattr(policy, "_object_map", None)
    target_object = getattr(policy, "_target_object", "")
    clouds = getattr(object_map, "clouds", {}) if object_map is not None else {}
    target_names = [name for name in target_object.split("|") if name] or list(clouds.keys())
    for name in target_names:
        cloud = np.asarray(clouds.get(name, []), dtype=np.float64)
        if cloud.size == 0:
            continue
        cloud = cloud.reshape(-1, cloud.shape[-1])
        step = max(1, len(cloud) // 500)
        for xy in cloud[::step, :2]:
            x, y = _xy_to_flipped_vis_px(obstacle_map, xy)
            if 0 <= x < img.shape[1] and 0 <= y < img.shape[0]:
                cv2.circle(img, (x, y), 2, (0, 165, 255), -1)

    last_coord = getattr(object_map, "last_target_coord", None) if object_map is not None else None
    if last_coord is not None:
        _draw_goal_marker(img, obstacle_map, {"xy": np.asarray(last_coord)[:2]}, (0, 220, 255))
    _draw_goal_marker(img, obstacle_map, goal, (0, 255, 0))
    return img


class LimoVLFMNode(Node):  # type: ignore[misc]
    """ROS2 observation adapter for the Limo/Nav2 VLFM coupling."""

    def __init__(self, parameter_overrides: Optional[list[Any]] = None) -> None:
        if _ROS_IMPORT_ERROR is not None:
            raise ImportError(f"ROS2 dependencies are not available: {_ROS_IMPORT_ERROR}") from _ROS_IMPORT_ERROR
        super().__init__("limo_vlfm", parameter_overrides=parameter_overrides or [])

        self.rgb_topic = self._declare("rgb_topic", "/camera/color/image_raw")
        self.depth_topic = self._declare("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.camera_info_topic = self._declare("camera_info_topic", "/camera/color/camera_info")
        self.map_topic = self._declare("map_topic", "/map")
        self.odom_topic = self._declare("odom_topic", "/odom")
        self.map_frame = self._declare("map_frame", "map")
        self.odom_frame = self._declare("odom_frame", "odom")
        self.base_frame = self._declare("base_frame", "base_link")
        self.camera_frame = self._declare("camera_frame", "camera_color_optical_frame")
        self.pose_lookup_mode = str(self._declare("pose_lookup_mode", "chain_fallback"))
        self.min_depth = float(self._declare("min_depth", 0.3))
        self.max_depth = float(self._declare("max_depth", 3.0))
        self.map_size = int(self._declare("map_size", 1000))
        self.pixels_per_meter = int(self._declare("pixels_per_meter", 20))
        self.agent_radius = float(self._declare("agent_radius", 0.18))
        self.min_obstacle_height = float(self._declare("min_obstacle_height", 0.0))
        self.max_obstacle_height = float(self._declare("max_obstacle_height", 1.0))
        self.obstacle_map_area_threshold = float(self._declare("obstacle_map_area_threshold", 1.5))
        self.publish_debug_images_enabled = bool(self._declare("publish_debug_images", True))
        self.debug_image_period = float(self._declare("debug_image_period", 0.5))
        self.log_tf_pose = bool(self._declare("log_tf_pose", True))

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.obstacle_map = ObstacleMap(
            min_height=self.min_obstacle_height,
            max_height=self.max_obstacle_height,
            area_thresh=self.obstacle_map_area_threshold,
            agent_radius=self.agent_radius,
            size=self.map_size,
            pixels_per_meter=self.pixels_per_meter,
        )

        self.latest_rgb: Optional[np.ndarray] = None
        self.latest_depth_raw: Optional[np.ndarray] = None
        self.latest_depth_encoding = ""
        self.latest_map: Optional[Any] = None
        self.latest_odom: Optional[Any] = None
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None

        rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
        sync.registerCallback(self._rgbd_cb)
        self._camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 1)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, map_qos)
        self._odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self._goal_pub = self.create_publisher(PoseStamped, "/vlfm/goal", 10)
        self._frontier_pub = self.create_publisher(MarkerArray, "/vlfm/frontiers", 1)
        self._debug_image_pubs = {
            "annotated_rgb": self.create_publisher(Image, "/vlfm/vis/annotated_rgb", 1),
            "obstacle_map": self.create_publisher(Image, "/vlfm/vis/obstacle_map", 1),
            "value_map": self.create_publisher(Image, "/vlfm/vis/value_map", 1),
            "object_map": self.create_publisher(Image, "/vlfm/vis/object_map", 1),
        }
        self._last_debug_image_time = 0.0
        self._obs_step = 0
        self._last_logged_base_xyz: Optional[np.ndarray] = None
        self._last_logged_base_yaw: Optional[float] = None
        self._last_pose_fallback_log_time = 0.0

    def _declare(self, name: str, default: Any) -> Any:
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _rgbd_cb(self, rgb_msg: Any, depth_msg: Any) -> None:
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        self.latest_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        self.latest_depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        self.latest_depth_encoding = depth_msg.encoding

    def _camera_info_cb(self, msg: Any) -> None:
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])

    def _map_cb(self, msg: Any) -> None:
        self.latest_map = msg

    def _odom_cb(self, msg: Any) -> None:
        self.latest_odom = msg

    def lookup_pose(self, target_frame: str) -> tuple[np.ndarray, float]:
        stamp = rclpy.time.Time() if rclpy is not None else None
        tf = self.tf_buffer.lookup_transform(self.map_frame, target_frame, stamp)
        t = tf.transform.translation
        xyz = np.array([t.x, t.y, t.z], dtype=np.float64)
        yaw = yaw_from_quaternion(tf.transform.rotation)
        return xyz, yaw

    def lookup_transform_matrix(self, target_frame: str, source_frame: str) -> np.ndarray:
        stamp = rclpy.time.Time() if rclpy is not None else None
        tf = self.tf_buffer.lookup_transform(target_frame, source_frame, stamp)
        return transform_to_matrix(tf)

    def lookup_observation_poses(self) -> tuple[np.ndarray, float, np.ndarray, float, str]:
        if self.pose_lookup_mode == "direct":
            base_xyz, base_yaw = self.lookup_pose(self.base_frame)
            cam_xyz, cam_yaw = self.lookup_pose(self.camera_frame)
            return base_xyz, base_yaw, cam_xyz, cam_yaw, "direct"
        if self.pose_lookup_mode == "odom_topic":
            return self.lookup_observation_poses_from_odom_topic()

        try:
            tf_map_odom = self.lookup_transform_matrix(self.map_frame, self.odom_frame)
            tf_odom_base = self.lookup_transform_matrix(self.odom_frame, self.base_frame)
            tf_base_cam = self.lookup_transform_matrix(self.base_frame, self.camera_frame)
            tf_map_base = tf_map_odom @ tf_odom_base
            tf_map_cam = tf_map_base @ tf_base_cam
            base_xyz, base_yaw = pose_from_matrix(tf_map_base)
            cam_xyz, cam_yaw = pose_from_matrix(tf_map_cam)
            return base_xyz, base_yaw, cam_xyz, cam_yaw, "chain"
        except Exception as e:
            if self.pose_lookup_mode == "chain":
                raise
            now = time.time()
            if now - self._last_pose_fallback_log_time > 5.0:
                self.get_logger().warn(f"Explicit TF chain failed ({e}); falling back to direct map lookups.")
                self._last_pose_fallback_log_time = now
            base_xyz, base_yaw = self.lookup_pose(self.base_frame)
            cam_xyz, cam_yaw = self.lookup_pose(self.camera_frame)
            return base_xyz, base_yaw, cam_xyz, cam_yaw, "direct-fallback"

    def lookup_observation_poses_from_odom_topic(self) -> tuple[np.ndarray, float, np.ndarray, float, str]:
        if self.latest_odom is None:
            raise RuntimeError("Odometry has not arrived yet.")
        tf_map_odom = self.lookup_transform_matrix(self.map_frame, self.odom_frame)
        tf_odom_base = pose_to_matrix(self.latest_odom.pose.pose)
        tf_base_cam = self.lookup_transform_matrix(self.base_frame, self.camera_frame)
        tf_map_base = tf_map_odom @ tf_odom_base
        tf_map_cam = tf_map_base @ tf_base_cam
        base_xyz, base_yaw = pose_from_matrix(tf_map_base)
        cam_xyz, cam_yaw = pose_from_matrix(tf_map_cam)
        return base_xyz, base_yaw, cam_xyz, cam_yaw, "odom_topic"

    def log_observation_tf_pose(
        self,
        base_xyz: np.ndarray,
        base_yaw: float,
        cam_xyz: np.ndarray,
        cam_yaw: float,
        pose_source: str,
    ) -> None:
        if not self.log_tf_pose:
            return

        if self._last_logged_base_xyz is None or self._last_logged_base_yaw is None:
            dxy_text = "nan"
            dyaw_text = "nan"
        else:
            dxy = float(np.linalg.norm(np.asarray(base_xyz)[:2] - self._last_logged_base_xyz[:2]))
            dyaw = math.atan2(
                math.sin(float(base_yaw) - self._last_logged_base_yaw),
                math.cos(float(base_yaw) - self._last_logged_base_yaw),
            )
            dxy_text = f"{dxy:.4f}"
            dyaw_text = f"{dyaw:.4f}"

        print(
            f"[limo] obs={self._obs_step} pose={pose_source} "
            f"tf_base(map) xyz={np.round(base_xyz, 3).tolist()} "
            f"yaw={float(base_yaw):.3f} dxy={dxy_text} dyaw={dyaw_text} "
            f"tf_cam(map) xyz={np.round(cam_xyz, 3).tolist()} yaw={float(cam_yaw):.3f}",
            flush=True,
        )
        self._obs_step += 1
        self._last_logged_base_xyz = np.asarray(base_xyz, dtype=np.float64).copy()
        self._last_logged_base_yaw = float(base_yaw)

    def build_observation(self) -> dict:
        if self.latest_rgb is None or self.latest_depth_raw is None:
            raise RuntimeError("RGB-D has not arrived yet.")
        if self.fx is None or self.fy is None:
            raise RuntimeError("CameraInfo has not arrived yet.")
        if self.latest_map is None:
            raise RuntimeError("OccupancyGrid has not arrived yet.")
        if self.pose_lookup_mode == "odom_topic" and self.latest_odom is None:
            raise RuntimeError("Odometry has not arrived yet.")

        base_xyz, base_yaw, cam_xyz, cam_yaw, pose_source = self.lookup_observation_poses()
        self.log_observation_tf_pose(base_xyz, base_yaw, cam_xyz, cam_yaw, pose_source)

        grid_msg = self.latest_map
        grid = np.asarray(grid_msg.data, dtype=np.int8).reshape(grid_msg.info.height, grid_msg.info.width)
        origin_xy = np.array([grid_msg.info.origin.position.x, grid_msg.info.origin.position.y], dtype=np.float64)
        return build_limo_observation_cache(
            self.latest_rgb,
            self.latest_depth_raw,
            self.latest_depth_encoding,
            float(self.fx),
            float(self.fy),
            grid,
            grid_msg.info.resolution,
            origin_xy,
            base_xyz,
            base_yaw,
            cam_xyz,
            cam_yaw,
            self.obstacle_map,
            self.min_depth,
            self.max_depth,
        )

    def to_pose(self, goal: dict) -> Any:
        if goal.get("xy") is None:
            raise ValueError("Cannot convert a done/no-op goal to PoseStamped.")
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(goal["xy"][0])
        pose.pose.position.y = float(goal["xy"][1])
        pose.pose.orientation = quaternion_from_yaw(float(goal["yaw_hint"]))
        return pose

    def publish_goal_marker(self, goal: dict) -> None:
        self._goal_pub.publish(self.to_pose(goal))

    def publish_frontier_markers(self, frontiers: np.ndarray) -> None:
        """Publish all frontiers as a sphere MarkerArray on /vlfm/frontiers.

        Lets you eyeball in RViz whether the frontier dots hug the /map
        known/unknown boundary (the one manual G0 check still pending).
        """
        arr = np.asarray(frontiers, dtype=np.float64)
        markers = MarkerArray()
        if arr.size:
            arr = arr.reshape(-1, arr.shape[-1])[:, :2]
        for idx, xy in enumerate(arr):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "vlfm_frontiers"
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(xy[0])
            m.pose.position.y = float(xy[1])
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.4, 1.0, 0.9
            markers.markers.append(m)
        # A leading DELETEALL clears stale frontiers from the previous publish.
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.ns = "vlfm_frontiers"
        clear.action = Marker.DELETEALL
        markers.markers.insert(0, clear)
        self._frontier_pub.publish(markers)

    def publish_debug_images(self, policy: Any, obs_cache: dict, goal: dict) -> None:
        """Publish VLFM-native visual panels used by the validation recorder."""
        if not self.publish_debug_images_enabled:
            return
        now = time.time()
        if now - self._last_debug_image_time < self.debug_image_period:
            return
        self._last_debug_image_time = now

        images = {
            "annotated_rgb": _build_annotated_rgb(self.latest_rgb, policy),
            "obstacle_map": _as_bgr_uint8(self.obstacle_map.visualize()),
            "value_map": _build_value_map(policy, obs_cache, goal),
            "object_map": _build_object_map(self.obstacle_map, policy, goal),
        }
        stamp = self.get_clock().now().to_msg()
        for key, img in images.items():
            if img is None:
                continue
            msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self.map_frame
            self._debug_image_pubs[key].publish(msg)

    def announce(self, message: str) -> None:
        self.get_logger().info(message)


def run_nav2_mission(
    node: LimoVLFMNode,
    policy: Any,
    target_object: str,
    query: str = "",
    decide_hz: float = 1.0,
    preempt_dist: float = 0.5,
    initial_spin: bool = True,
) -> None:  # pragma: no cover - integration entry point
    if BasicNavigator is None or TaskResult is None or rclpy is None:
        raise ImportError("Nav2 simple commander is not available in this environment.")

    def pump_node(max_callbacks: int = 20, timeout_sec: float = 0.0) -> None:
        # A single spin_once can be monopolized by high-rate RGB-D/TF callbacks.
        # Drain a small batch so slower-but-critical state like /odom is fresh
        # before build_observation() snapshots the cache.
        for _ in range(max_callbacks):
            rclpy.spin_once(node, timeout_sec=timeout_sec)

    def wait_for_observation(label: str, timeout_sec: float = 30.0) -> dict:
        start = time.time()
        last_log = 0.0
        last_error = ""
        while rclpy.ok() and time.time() - start < timeout_sec:
            pump_node(max_callbacks=20, timeout_sec=0.01)
            try:
                return node.build_observation()
            except Exception as e:
                last_error = str(e)
                if time.time() - last_log > 1.0:
                    node.get_logger().info(f"[{label}] waiting for complete obs: {last_error}")
                    last_log = time.time()
        raise RuntimeError(f"{label}: observation did not become ready within {timeout_sec:.1f}s ({last_error})")

    navigator = BasicNavigator()
    nav2_localizer = os.environ.get("VLFM_NAV2_LOCALIZER", "robot_localization")
    navigator.waitUntilNav2Active(localizer=nav2_localizer)
    policy.reset_episode(target_object, query=query)
    wait_for_observation("mission-start")

    if initial_spin:
        # Spin in place and keep pumping RGB-D so the value/object maps fill in
        # before the first frontier is chosen (otherwise the first pick is blind).
        navigator.spin(spin_dist=2.0 * math.pi)
        warm = 0.0
        while not navigator.isTaskComplete():
            pump_node(max_callbacks=20, timeout_sec=0.0)
            if time.time() - warm < 0.5:
                continue
            warm = time.time()
            try:
                policy.decide_goal(node.build_observation())  # warm maps; goal ignored
            except Exception as e:  # obs not ready yet, or a perception server hiccup
                node.get_logger().info(f"[spin-warmup] {e}")
        wait_for_observation("post-spin")

    current = None
    while rclpy.ok():
        pump_node(max_callbacks=20, timeout_sec=0.0)
        obs = node.build_observation()
        node.publish_frontier_markers(obs["frontier_sensor"])
        candidate = policy.decide_goal(obs)
        node.publish_debug_images(policy, obs, candidate)
        if candidate["mode"] == "done":
            node.announce("explore exhausted")
            break
        if should_preempt(current, candidate, preempt_dist=preempt_dist):
            navigator.goToPose(node.to_pose(candidate))
            node.publish_goal_marker(candidate)
            current = candidate

        last_decide = time.time()
        while not navigator.isTaskComplete():
            pump_node(max_callbacks=20, timeout_sec=0.0)
            if time.time() - last_decide < 1.0 / max(decide_hz, 1e-6):
                continue
            last_decide = time.time()
            obs = node.build_observation()
            candidate = policy.decide_goal(obs)
            node.publish_debug_images(policy, obs, candidate)
            if should_preempt(current, candidate, preempt_dist=preempt_dist):
                navigator.goToPose(node.to_pose(candidate))
                node.publish_goal_marker(candidate)
                current = candidate

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            pump_node(max_callbacks=20, timeout_sec=0.0)
            verdict = policy.on_goal_reached(node.build_observation(), current)
            node.announce(f"[arrive] {current['mode']} accepted={verdict['accepted']} {verdict['reason']}")
            if verdict["accepted"] and verdict["next"] == "done":
                node.announce("FOUND & VERIFIED")
                break
        elif result in (TaskResult.FAILED, TaskResult.CANCELED):
            policy.on_goal_unreachable(current)
            node.get_logger().warn(f"[unreachable] {current['mode']} {np.round(current['xy'], 2).tolist()}")
        current = None
