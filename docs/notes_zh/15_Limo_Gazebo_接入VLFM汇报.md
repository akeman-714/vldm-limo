# 15 · Limo Gazebo 仿真接入 VLFM 汇报

## 0. 当前访问情况与已确认信息

mentor 给的工作区是:

```text
10.89.4.4:/home/xiaowei.du/projects/robot/ROS2/limo_nav_ws
```

我已经能 SSH 登录到 `10.89.4.4`。普通 `asong` 用户不能直接读 `/home/xiaowei.du`，因为目录权限是 `750`，但 `asong` 在 `sudo` 组里，所以我用 sudo 做了只读查看。

mentor 的工作区结构已确认:

```text
limo_nav_ws/
  build/
  install/
  log/
  src/
    limo_pro_sim/
      config/bridge.yaml
      config/nav2_params.yaml
      launch/demo.launch.py
      launch/sim.launch.py
      launch/slam_3d.launch.py
      launch/nav2.launch.py
      launch/mission.launch.py
      scripts/mission_node.py
      urdf/limo_pro.urdf.xacro
      urdf/limo_pro.gazebo.xacro
      worlds/small_house.sdf
      worlds/home.sdf
      worlds/indoor.sdf
```

所以这份汇报现在是基于本地 `vlfm` 代码 + mentor 实际 `limo_pro_sim` 工作区整理的。

---

## 1. 结论先讲

VLFM 接到 Limo Gazebo 的核心工作不是重写 VLFM 算法，而是补一个 **ROS2/Gazebo Robot 适配层**。

现在仓库里已经有一套真机 Spot 适配:

- `vlfm/reality/robots/base_robot.py`: 抽象机器人接口。
- `vlfm/reality/robots/bdsw_robot.py`: Spot 的具体实现。
- `vlfm/reality/objectnav_env.py`: 把机器人传感器数据整理成 VLFM policy 要吃的 observation。
- `vlfm/policy/reality_policies.py`: 把 VLFM policy 输出转换成机器人动作字典。
- `vlfm/reality/run_bdsw_objnav_env.py`: 运行闭环。

接 Limo Gazebo 时，应该照这个结构在 VLFM 侧新增:

```text
vlfm/reality/robots/limo_ros2_robot.py   # ROS2 topic/tf/action 适配器
vlfm/reality/limo_objectnav_env.py       # 如果传感器布局和 Spot 差异较大，单独写 env
vlfm/reality/run_limo_objnav_env.py      # 启动入口
config/experiments/limo_gazebo.yaml      # Limo 仿真配置
```

mentor 的 `limo_pro_sim` 本身已经把 Gazebo、SLAM、Nav2、RViz、Mission Node 都搭好了，VLFM 侧原则上不需要先改他的 Gazebo 模型。

---

## 2. VLFM policy 真正需要什么输入

`ObjectNavEnv._get_obs()` 返回的是 VLFM 的核心 observation:

```python
{
    "nav_depth": nav_depth,
    "robot_xy": robot_xy,
    "robot_heading": robot_heading,
    "objectgoal": self.target_object,
    "obstacle_map_depths": obstacle_map_depths,
    "value_map_rgbd": value_map_rgbd,
    "object_map_rgbd": object_map_rgbd,
}
```

逐项解释:

| 字段 | VLFM 用途 | Limo/Gazebo 侧怎么给 |
|---|---|---|
| `nav_depth` | PointNav 局部避障/到点导航 | 前向 depth image，归一化到 0-1 |
| `robot_xy` | 建图和目标相对位置 | 从 `/odom` 或 TF `map/odom -> base_link` 取 x,y |
| `robot_heading` | 机器人朝向 | 从 odom/tf 四元数转 yaw |
| `objectgoal` | 目标类别文字 | 配置里给，比如 `"chair"` |
| `obstacle_map_depths` | 更新障碍图和 frontier | depth + camera->episode transform + fx/fy/fov |
| `value_map_rgbd` | 语义价值图，判断哪里更像目标 | RGB + depth + camera pose |
| `object_map_rgbd` | 检测到目标后反投影成目标点 | RGB + depth + camera pose + intrinsics |

一句话: **VLFM 只要求 RGB、Depth、机器人位姿、相机外参/内参、目标文本**。它不关心这些来自 Habitat、Spot 还是真实/仿真的 Limo。

---

## 3. VLFM 输出什么动作

现实版 policy 输出的是动作字典:

```python
{
    "angular": float,  # [-1, 1]
    "linear": float,   # [-1, 1]
    "arm_yaw": -1,
    "info": {...},
}
```

`PointNavEnv._compute_displacements()` 会把它缩放成单步位移:

- `linear * max_lin_dist`
- `angular * max_ang_dist`

Spot 版最后调用的是 `spot.set_base_position(...)`。Limo/Gazebo 版更自然的落点应该是发布 `/cmd_vel`:

```text
geometry_msgs/Twist
linear.x  = linear_velocity
angular.z = angular_velocity
```

这里有一个设计选择:

1. 保持 VLFM 原来的“单步位移”语义，把 `max_lin_dist/time_step` 和 `max_ang_dist/time_step` 转成速度，持续发布 `time_step` 秒。
2. 或者改成直接速度控制，但要小心 PointNav 原来学到的是离散时间步行为，不建议一上来大改。

我建议先用方案 1，最保守，和现有 `config/experiments/reality.yaml` 语义一致。

---

## 4. mentor 工作区里的实际 topic/frame

`config/bridge.yaml` 已经桥接了 VLFM 最小闭环需要的 topic:

| ROS2 topic | 类型 | 方向 | VLFM 用途 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS -> Gazebo | 控制 Limo 底盘 |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo -> ROS | 可读轮速里程计 |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo -> ROS | TF |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo -> ROS | 可选，用现有 SLAM/costmap |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo -> ROS | SLAM/odom 已用 |
| `/camera/image` | `sensor_msgs/msg/Image` | Gazebo -> ROS | VLFM RGB |
| `/camera/depth_image` | `sensor_msgs/msg/Image` | Gazebo -> ROS | VLFM depth |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo -> ROS | 相机内参 fx/fy |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | Gazebo -> ROS | 可选，不是 VLFM 必需 |

URDF/README 里确认的 TF 链:

```text
map -> odom -> base_footprint -> base_link -> depth_camera_link -> depth_link
```

其中 `depth_link` 是 REP-103 optical frame: `z` forward, `x` right, `y` down。VLFM 适配器应该优先用 TF 查 `depth_link` 到 `map` 或 `odom` 的变换。

RGB-D 相机参数来自 `urdf/limo_pro.gazebo.xacro`:

| 参数 | 值 |
|---|---|
| sensor name | `rgbd_camera` |
| topic root | `camera` |
| frame | `depth_link` |
| update rate | `15 Hz` |
| image size | `640 x 480` |
| format | `R8G8B8` |
| horizontal FOV | `1.2 rad` |
| RGB clip | `0.1 - 15.0 m` |
| depth clip | `0.2 - 10.0 m` |

如果后面要在远端运行仿真，可以用这些命令现场确认:

```bash
source install/setup.bash
ros2 topic list
ros2 topic info /cmd_vel
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames
ros2 topic list | grep -E "image|camera|depth|rgb"
ros2 topic list | grep -E "camera_info"
```

如果 mentor 的仿真已经跑 Nav2，还可能有:

- `/map`
- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/scan`

这些不是 VLFM 的硬需求，但如果已有 costmap，可以考虑直接喂给障碍图，减少从 depth 重建障碍的工作。

实际查看后确认: mentor 的 `slam_3d.launch.py` 已经用 RTAB-Map 生成 `/map` 和 `/cloud_map`，Nav2 也会有 costmap；但第一版 VLFM 接入建议先只用 RGB-D + TF/Odom + `/cmd_vel`，闭环跑通后再考虑复用 `/map` 或 costmap。

---

## 5. 适配器应该长什么样

需要实现一个类似 `BDSWRobot` 的类，但底层用 `rclpy` 订阅/发布:

```python
class LimoROS2Robot(BaseRobot):
    @property
    def xy_yaw(self):
        # 从 /odom 或 tf 读 base_link 位姿
        return np.array([x, y]), yaw

    def get_camera_images(self, camera_source):
        # 返回 {source: np.ndarray}
        # RGB: uint8 HxWx3
        # Depth: uint16 mm 或 float32 m，后续要统一

    def get_camera_data(self, srcs):
        # 返回 image, fx, fy, tf_camera_to_global

    def command_base_velocity(self, ang_vel, lin_vel):
        # 发布 geometry_msgs/Twist 到 /cmd_vel

    def get_transform(self, frame="base_link"):
        # 从 tf2 buffer 查 transform，转 4x4 numpy
```

如果 Limo 没有机械臂/腕部相机，要处理掉 Spot 版里的 `arm_yaw` 初始化扫描:

- 简单方案: Limo 原地旋转若干角度来代替 arm scan。
- 更快方案: 直接把 `_done_initializing=True`，不做初始化扫描，先跑通闭环。
- 推荐第一阶段用“跳过 arm scan”，因为 Gazebo 接入先验证数据链和动作闭环。

对这个 Limo 仿真，推荐先跳过 `arm_yaw` 初始化扫描。因为它只有固定前向 RGB-D 相机，没有 Spot 那种腕部相机环视。

---

## 6. mentor 现有 launch 怎么跑

`README.md` 里给出的主入口是:

```bash
cd ~/projects/robot/ROS2/limo_nav_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch limo_pro_sim demo.launch.py
```

`demo.launch.py` 会按顺序启动:

```text
sim.launch.py      # Gazebo + robot_state_publisher + ros_gz_bridge
slam_3d.launch.py  # RTAB-Map RGB-D + LiDAR SLAM
rviz2
nav2.launch.py     # Nav2 planner/controller/costmap
mission.launch.py  # 固定航点任务
```

对接 VLFM 时建议这样启动 mentor 仿真:

```bash
ros2 launch limo_pro_sim demo.launch.py mission:=false
```

原因: `mission_node.py` 和 VLFM 都属于“决定去哪”的上层任务逻辑。Mission Node 会向 Nav2 发固定 A/B/C/Home 航点；VLFM 则会根据语义和 frontier 自己选目标。如果两个同时跑，行为会互相干扰。

如果 VLFM 第一版直接发布 `/cmd_vel`，还要注意不要让 Nav2 的 `velocity_smoother` 同时占用 `/cmd_vel`。更稳的做法是第一阶段只启动 `sim.launch.py` + 必要 TF/SLAM，不启动 Nav2；或者启动 demo 时 `mission:=false`，并确认没有 Nav2 在持续发速度。

---

## 7. 坐标系要特别小心

VLFM 的 `ObjectNavEnv.reset()` 会把启动时机器人位置作为 episodic 原点:

```python
self.tf_episodic_to_global = self.robot.get_transform()
self.tf_global_to_episodic = np.linalg.inv(self.tf_episodic_to_global)
self.episodic_start_yaw = self.robot.xy_yaw[1]
```

因此 ROS2 侧只要稳定提供 `map/odom` 坐标下的机器人和相机姿态，VLFM 内部会转到 episode 坐标。mentor 这套仿真里，RTAB-Map 发布 `map->odom`，ICP odometry 发布 `odom->base_footprint`，robot_state_publisher 发布 `base_footprint->base_link->depth_link`。

最容易错的是相机坐标轴。Spot 版有一段相机坐标转换:

```python
rotation_matrix = np.array([
    [0, -1, 0, 0],
    [0, 0, -1, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1],
])
```

这说明 VLFM mapping 代码期待的是自己的 camera xyz 约定。Limo/Gazebo 接入时必须用可视化检查:

- 障碍是否投到机器人前方，而不是后方/侧方。
- robot trail 是否和 `/odom` 轨迹方向一致。
- value map 的扇形是否跟相机视野一致。

---

## 8. 推荐落地顺序

第一阶段: 只跑通传感器链。

- 启动 Gazebo。
- 写 `LimoROS2Robot`，能读 RGB、Depth、CameraInfo、TF、Odom。
- 保存一帧 observation 到 `.npz`，检查 shape、dtype、深度单位、fx/fy、tf。

第二阶段: 跑通 VLFM 单步推理。

- 启动 VLM servers: `./scripts/upstream/launch_vlm_servers.sh` 或项目里现有的替代脚本。
- 用固定目标 `"chair"`。
- 调 `policy.get_action(obs, mask)`，先不要发 `/cmd_vel`，只打印动作。

第三阶段: 接 `/cmd_vel` 闭环。

- 把 `angular/linear` 转成 `Twist`。
- 限速，例如 `linear.x <= 0.2 m/s`，`angular.z <= 0.5 rad/s`。
- 每次 action 发布 `time_step` 秒，然后停一下取新观测。

第四阶段: 语义和目标检测验证。

- 确认 value map 能随 RGB 变化。
- 确认检测到目标后 `object_map` 能给出合理 `(x,y)`。
- 再调 frontier / stop radius / max depth。

---

## 9. 给 mentor 的一句话汇报

我已经看了 `10.89.4.4:/home/xiaowei.du/projects/robot/ROS2/limo_nav_ws`，mentor 的包叫 `limo_pro_sim`，Gazebo/ROS2 已经桥好了 `/camera/image`、`/camera/depth_image`、`/camera/camera_info`、`/tf`、`/odom`、`/cmd_vel`，TF 是 `map -> odom -> base_footprint -> base_link -> depth_camera_link -> depth_link`。所以 VLFM 接入 Limo Gazebo 的关键不是重写算法，而是在 VLFM 侧写一个 ROS2 适配层: 从这些 topic/TF 取 RGB-D、内参、位姿，整理成 `ObjectNavEnv` observation；VLFM policy 输出 `linear/angular` 后转成 `/cmd_vel`。第一版建议 `ros2 launch limo_pro_sim demo.launch.py mission:=false`，先关掉固定航点 Mission Node，避免它和 VLFM 抢控制。
