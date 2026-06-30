# 21 · M2 观测插线层：RGBD + TF + `/map` → observation cache

> 目标：写 Limo ROS2 节点的「插线层」，把传感器变成 VLFM 大脑唯一吃的 `obs_cache`（[19 §C](19_Nav2耦合_接口契约.md)）。它是 `HabitatMixin._cache_observations` / Spot `ObjectNavEnv._get_obs` 的**第三个兄弟**。
> 依赖：[19](19_Nav2耦合_接口契约.md)、[20](20_Nav2耦合_M1_occupancy_grid适配.md)。范例：`vlfm/reality/objectnav_env.py:118`、`vlfm/policy/habitat_policies.py:185`。

## 1. 规划

节点订阅 RGB / Depth / CameraInfo / `/map` / TF，用 `message_filters` 近似同步 RGB+Depth，按 [19 §C] 组装 `obs_cache`。**value map 与 object map 是 VLFM 命根子、ROS 无等价物，必须本层从 RGB-D 喂入**；`/map` 只供障碍/frontier（M1）。

## 2. 接口

- 产出：`build_observation() -> dict`（[19 §C] 的 7 键）。
- 节点参数：`rgb_topic/depth_topic/camera_info_topic/map_topic/map_frame/base_frame/camera_frame/min_depth/max_depth`。

## 3. 详细步骤

**3.1 节点骨架**（建议 `vlfm_ros/limo_vlfm_node.py`，纯 rclpy，不依赖 habitat）：
```python
class LimoVLFMNode(Node):
    def __init__(self):
        # 参数声明（见 §2），cv_bridge，tf2_ros.Buffer/TransformListener
        # message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue, slop=0.1)
        # CameraInfo 订阅一次取 fx,fy 后可退订
        # /map 订阅，回调存最新 OccupancyGrid
        # 共享一个 ObstacleMap(size, pixels_per_meter=20, agent_radius=机器人半径, ...)
```

**3.2 深度归一化（口径唯一，见 [19 §C/E]）**：
```python
def normalize_depth(depth_raw):
    d_m = depth_raw.astype(np.float32)
    if depth_msg_is_mm: d_m *= 0.001            # 16UC1 毫米 → 米
    d = (d_m - self.min_depth) / (self.max_depth - self.min_depth)
    d = np.clip(d, 0.0, 1.0)
    d[(d_m <= 0) | ~np.isfinite(d_m)] = 1.0     # 无效深度填 1（远），与 obstacle/value 约定一致
    return d  # HxW float[0,1]
```

**3.3 取位姿（map 帧，TF）**：
```python
def lookup_pose(self, target_frame):  # base_frame 或 camera_frame
    tf = self.tf_buffer.lookup_transform(self.map_frame, target_frame, rclpy.time.Time())
    x = tf.transform.translation.x; y = tf.transform.translation.y; z = tf.transform.translation.z
    yaw = yaw_from_quat(tf.transform.rotation)   # 仅取 yaw
    return np.array([x, y, z]), yaw              # ★map 帧原值，不翻 y
```

**3.4 组装 obs_cache**：
```python
def build_observation(self):
    rgb = self.latest_rgb                       # HxWx3 uint8 RGB（cv_bridge: bgr8→转RGB）
    depth = self.normalize_depth(self.latest_depth)
    (rx, ry, _), ryaw = self.lookup_pose(self.base_frame)
    cam_xyz, cyaw = self.lookup_pose(self.camera_frame)
    tf_cam2map = xyz_yaw_to_tf_matrix(cam_xyz, cyaw)   # vlfm.utils.geometry_utils

    # M1：/map 注入 → frontier（map 帧）
    g = self.latest_map
    grid = np.asarray(g.data, np.int8).reshape(g.info.height, g.info.width)
    self.obstacle_map.update_from_occupancy_grid(
        grid, g.info.resolution, np.array([g.info.origin.position.x, g.info.origin.position.y]))
    frontiers = self.obstacle_map.frontiers
    self.obstacle_map.update_agent_traj(np.array([rx, ry]), ryaw)  # 仅可视化

    fov = get_fov(self.fx, depth.shape[1])      # 弧度
    return {
        "frontier_sensor": frontiers,
        "robot_xy": np.array([rx, ry]),
        "robot_heading": ryaw,
        "nav_depth": depth,                     # 方案B不驾驶，占位即可
        "object_map_rgbd": [(rgb, depth, tf_cam2map, self.min_depth, self.max_depth, self.fx, self.fy)],
        "value_map_rgbd":  [(rgb, depth, tf_cam2map, self.min_depth, self.max_depth, fov)],
    }
```

**3.5 关键：`camera_frame` 用光学帧**（`*_color_optical_frame`，z 朝前、x 朝右、y 朝下），因为 VLFM 的 `get_point_cloud`/`tf_camera_to_episodic` 按光学约定。用 `base_link` 当相机帧会让点云投影全错。

## 4. 注意点

- **同步**：RGB/Depth 用 `ApproximateTimeSynchronizer(slop≈0.1s)`；TF 用「最近可用」（`Time()`）或按图像 stamp 查（更准但要等 TF）。先用最近可用跑通 G0。
- **对齐深度**：尽量用 `aligned_depth_to_color`，保证 RGB 与 Depth 同内参同像素；否则 object mask 与点云对不上。
- **fx/fy 来源**：`CameraInfo.k`（行主序 3x3），`fx=k[0], fy=k[4]`。**用 depth 对齐后的彩色内参**。
- **`min/max_depth`**：按相机量程定（如 RealSense 0.3–3.0 m / 0.1–10 m）。一处定义，全局引用。
- **RGB 顺序**：cv_bridge 默认出 `bgr8`，VLFM 期望 RGB，转一次 `cv2.cvtColor(..., BGR2RGB)`。

## 5. 可能问题 + 解法

| # | 现象 | 根因 | 解法 |
|---|---|---|---|
| 1 | `lookup_transform` 抛 `ExtrapolationException` | TF 没到 / 时间戳太旧 | 用 `rclpy.time.Time()`（最新）；起 `TransformListener` 后等几帧；确认 RTAB-Map 在发 `map→base_link` 全链 |
| 2 | 点云/object map 明显歪斜 | 用了 `base_link` 而非光学帧；或没转 RGB | `camera_frame` 改 `*_color_optical_frame`；BGR→RGB |
| 3 | value map 一片空白 | depth 没归一化 / 全填 1 | 检查 `normalize_depth`：有效深度比例、mm↔m；echo depth 数值范围 |
| 4 | RGB 与 depth 错位、mask 飘 | 未用 aligned depth | 启用相机 `align_depth`，或自己按外参对齐 |
| 5 | 节点卡顿 | 每帧重建图 + VLM 推理同步阻塞 | M5：建图/感知放低频（~1–2Hz）；VLM 客户端调用做超时 |

## 6. 验收标准

**6.1 离线**（录一段 rosbag：RGB+Depth+CameraInfo+/map+TF）：
- 回放 bag，调 `build_observation()`，断言 7 键齐全、形状/类型符合 [19 §C]。
- `tf_cam2map` 是 4x4、`det≈1`、左上 3x3 正交（合法刚体变换）。
- `depth ∈ [0,1]`，有效像素占比 > 阈值（如 > 30%）。

**6.2 在线**：
- `ros2 topic hz`：节点能稳定产出 obs（≥1Hz）。
- 把 `robot_xy`/`robot_heading` 发成 `/vlfm/pose` Marker，在 RViz 与机器人 TF 重合（验证「不翻 y、map 帧原值」）。
- （联合 M1）`/vlfm/frontiers` 贴在 `/map` 边界（M1 §6.2）。
- **关键人工核对**：机器人朝向 marker 与真实朝向一致；frontier 在机器人前方未探索处而非身后——**若反向，先查 y 取反与 frame 名**（[19 §E]）。

> 过了 6.1+6.2，配合 M1 即满足 **G0 帧验证**（此时 decide_goal 尚未接，仅打印/可视化）。
