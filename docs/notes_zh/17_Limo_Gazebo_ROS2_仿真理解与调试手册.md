# 17 · Limo Gazebo / ROS2 仿真理解与调试手册

## 0. 这份文档解决什么问题

目标不是立刻接 VLFM，而是让你知道 mentor 的 Gazebo/ROS2 仿真到底怎么运转、每个核心部件负责什么、怎么通过命令、打印、RViz/Gazebo 可视化去验证它。

读完后你应该能回答:

- Gazebo 在干什么？
- ROS2 topic 是什么？
- `/tf` 为什么重要？
- `ros_gz_bridge` 是谁？
- RTAB-Map、Nav2、Mission Node 各自负责什么？
- 我怎么确认相机、深度、里程计、地图、控制都正常？
- VLFM 后面到底应该接在哪一层？

当前仿真工作区:

```text
10.89.4.4:/home/xiaowei.du/projects/robot/ROS2/limo_nav_ws
```

主要包:

```text
src/limo_pro_sim
```

---

## 1. 一句话理解这套系统

这套仿真可以理解成:

```text
Gazebo 负责“模拟世界和机器人”
ROS2 负责“把传感器、位姿、控制命令变成 topic/TF”
RTAB-Map 负责“用传感器建图和定位”
Nav2 负责“给定目标点后规划并控制机器人过去”
Mission Node 负责“给 Nav2 发固定航点任务”
VLFM 以后要替换或绕过 Mission Node，自己决定去哪里
```

更具体:

```text
Gazebo 小房子世界
  -> 生成 RGB-D / LiDAR / IMU / Odom
  -> ros_gz_bridge 桥到 ROS2 topic
  -> RTAB-Map 建 /map 和 map->odom
  -> Nav2 用 /map /scan /tf 规划
  -> Mission Node 发 A/B/C/Home 航点
  -> Nav2 输出 /cmd_vel
  -> ros_gz_bridge 桥回 Gazebo 驱动机器人
```

---

## 2. 核心文件在哪里

mentor 包的关键文件:

```text
limo_pro_sim/
  README.md
  config/
    bridge.yaml          # Gazebo <-> ROS2 topic 桥接
    nav2_params.yaml     # Nav2 参数
  launch/
    demo.launch.py       # 一键启动全流程
    sim.launch.py        # Gazebo + robot_state_publisher + bridge
    slam_3d.launch.py    # RTAB-Map / ICP odometry
    nav2.launch.py       # Nav2
    mission.launch.py    # 固定航点 Mission Node
  scripts/
    mission_node.py      # A -> B -> C -> Home
  urdf/
    limo_pro.urdf.xacro
    limo_pro.gazebo.xacro
  worlds/
    small_house.sdf
    home.sdf
    indoor.sdf
  rviz/
    demo.rviz
```

你理解系统时优先看这几个:

1. `README.md`: 总体说明。
2. `config/bridge.yaml`: 哪些 Gazebo 数据变成 ROS2 topic。
3. `launch/demo.launch.py`: 一键启动顺序。
4. `launch/sim.launch.py`: Gazebo 和 bridge 怎么起来。
5. `urdf/limo_pro.gazebo.xacro`: 传感器和 `/cmd_vel` 插件。
6. `launch/slam_3d.launch.py`: 谁发布 `map->odom`。
7. `launch/nav2.launch.py`: Nav2 怎么输出速度。
8. `scripts/mission_node.py`: 固定航点怎么发。

---

## 3. 怎么启动

进入远端:

```bash
ssh asong@10.89.4.4
```

进入工作区:

```bash
cd /home/xiaowei.du/projects/robot/ROS2/limo_nav_ws
source install/setup.bash
```

### 方式 A: 启动全流程，但关闭固定任务

这是接 VLFM 最推荐的启动方式:

```bash
ros2 launch limo_pro_sim demo.launch.py mission:=false
```

含义:

- 启动 Gazebo。
- 启动 RGB-D / LiDAR / IMU / Odom bridge。
- 启动 RTAB-Map。
- 启动 RViz。
- 启动 Nav2。
- 不启动固定 A/B/C/Home Mission Node。

为什么关 Mission Node:

Mission Node 和 VLFM 都属于“决定去哪里”的上层逻辑。如果它开着，VLFM 后面也在控制机器人，两边会打架。

### 方式 B: 只启动仿真和传感器

如果只想看 Gazebo、相机、topic:

```bash
ros2 launch limo_pro_sim sim.launch.py
```

这时通常只有:

- Gazebo
- robot_state_publisher
- ros_gz_bridge
- 基础 `/camera/*`、`/scan`、`/odom`、`/tf`、`/cmd_vel`

不一定有:

- `/map`
- `map->odom`
- Nav2 costmap

### 方式 C: 分步启动

便于你理解每层:

```bash
# 终端 1: Gazebo + bridge
ros2 launch limo_pro_sim sim.launch.py

# 终端 2: RTAB-Map SLAM
ros2 launch limo_pro_sim slam_3d.launch.py

# 终端 3: Nav2
ros2 launch limo_pro_sim nav2.launch.py

# 终端 4: RViz
rviz2 -d $(ros2 pkg prefix limo_pro_sim)/share/limo_pro_sim/rviz/demo.rviz
```

---

## 4. Gazebo 是什么，怎么看它正常

Gazebo 是物理仿真器。这里它负责:

- 加载小房子世界 `small_house.sdf`
- 生成 Limo 机器人模型
- 模拟轮子、碰撞、摩擦
- 模拟 RGB-D 相机、2D LiDAR、IMU、轮速里程计
- 接收 `/cmd_vel` 后让机器人运动

### Gazebo 相关文件

世界:

```text
worlds/small_house.sdf
```

机器人结构:

```text
urdf/limo_pro.urdf.xacro
```

Gazebo 插件和传感器:

```text
urdf/limo_pro.gazebo.xacro
```

### Gazebo 相机配置

从 `limo_pro.gazebo.xacro` 可知:

```text
sensor name: rgbd_camera
topic root: camera
frame: depth_link
update_rate: 15 Hz
image: 640 x 480
format: R8G8B8
horizontal_fov: 1.2 rad
depth range: 0.2 - 10.0 m
```

### 验证 Gazebo 正常

看 topic:

```bash
ros2 topic list
```

你至少应该看到:

```text
/camera/image
/camera/depth_image
/camera/camera_info
/scan
/imu
/odom
/tf
/cmd_vel
```

看相机频率:

```bash
ros2 topic hz /camera/image
ros2 topic hz /camera/depth_image
```

预期:

```text
接近 15 Hz
```

看 LiDAR:

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

看 odom:

```bash
ros2 topic echo /odom --once
```

---

## 5. ROS2 topic 是什么，怎么用

ROS2 topic 可以理解成“命名的数据流”。

例如:

```text
/camera/image       连续发布 RGB 图像
/camera/depth_image 连续发布深度图
/odom               连续发布机器人里程计
/cmd_vel            接收速度命令
```

常用命令:

```bash
ros2 topic list
ros2 topic info /camera/image
ros2 topic hz /camera/image
ros2 topic echo /odom --once
```

### 看 topic 类型

```bash
ros2 topic info /camera/image
```

你会看到类似:

```text
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: ...
```

这说明 `/camera/image` 发布的是 ROS2 标准 Image 消息。

### 看一帧消息

```bash
ros2 topic echo /camera/camera_info --once
```

重点看:

```text
k:
  - fx
  - 0
  - cx
  - 0
  - fy
  - cy
  - 0
  - 0
  - 1
```

VLFM 需要的是:

```text
fx = k[0]
fy = k[4]
```

---

## 6. ros_gz_bridge 是什么

Gazebo 自己内部有一套 topic，ROS2 也有一套 topic。`ros_gz_bridge` 就是翻译器。

mentor 的桥接配置在:

```text
config/bridge.yaml
```

里面确认了:

```text
Gazebo /camera/image       -> ROS2 /camera/image
Gazebo /camera/depth_image -> ROS2 /camera/depth_image
Gazebo /odom               -> ROS2 /odom
ROS2 /cmd_vel              -> Gazebo /cmd_vel
```

特别注意方向:

```text
GZ_TO_ROS:
  Gazebo 产生，ROS2 读取

ROS_TO_GZ:
  ROS2 发布，Gazebo 执行
```

`/cmd_vel` 是 `ROS_TO_GZ`，所以你在 ROS2 里发速度，Gazebo 里的车会动。

---

## 7. TF 是什么，为什么关键

TF 是坐标系关系。

对于机器人导航，光知道“有一张图”不够，还要知道:

- 机器人底盘在哪里
- 相机在哪里
- 相机相对机器人朝哪边
- 地图坐标和里程计坐标怎么对齐

这套仿真的 TF 链是:

```text
map -> odom -> base_footprint -> base_link -> depth_camera_link -> depth_link
```

含义:

| frame | 含义 |
|---|---|
| `map` | 全局地图坐标 |
| `odom` | 连续里程计坐标 |
| `base_footprint` | 机器人地面中心 |
| `base_link` | 机器人本体 |
| `depth_camera_link` | 相机实体安装 frame |
| `depth_link` | 相机 optical frame |

VLFM 最关心:

```text
base_footprint / base_link 的位置
depth_link 的相机位姿
```

### 查看 TF

查机器人到相机:

```bash
ros2 run tf2_ros tf2_echo base_footprint depth_link
```

查地图到机器人:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

查地图到相机:

```bash
ros2 run tf2_ros tf2_echo map depth_link
```

如果只启动了 `sim.launch.py`，可能没有 `map`，这时查:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint depth_link
```

### TF 可视化

生成 TF 图:

```bash
ros2 run tf2_tools view_frames
```

它会生成类似:

```text
frames.pdf
```

你重点看有没有:

```text
map
odom
base_footprint
base_link
depth_link
```

---

## 8. RTAB-Map 在这里干什么

RTAB-Map 负责 SLAM，也就是边走边建图和定位。

在 mentor 仿真里:

```text
/scan + /imu -> icp_odometry -> /icp_odom + odom->base_footprint
/camera/image + /camera/depth_image + /camera/camera_info + /scan + /imu + /icp_odom
  -> rtabmap
  -> /map + /cloud_map + map->odom
```

你可以理解成:

- ICP odometry 用 LiDAR 做比较稳的短期 odom。
- RTAB-Map 用 RGB-D、LiDAR、IMU 建 3D/2D 地图。
- `/map` 是 2D 栅格地图。
- `/cloud_map` 是 3D 点云地图。
- `map->odom` 是全局定位修正。

### 验证 RTAB-Map 正常

看 `/map`:

```bash
ros2 topic echo /map --once
```

看频率:

```bash
ros2 topic hz /map
```

看点云:

```bash
ros2 topic hz /cloud_map
```

查 TF:

```bash
ros2 run tf2_ros tf2_echo map odom
```

如果查不到 `map -> odom`，说明 RTAB-Map 没起来或还没发布。

---

## 9. Nav2 在这里干什么

Nav2 是传统导航栈。

它负责:

- 接收目标点 `NavigateToPose`
- 用 `/map` 做全局规划
- 用 `/scan` / costmap 做局部避障
- 输出速度命令

mentor 的控制链:

```text
Nav2 controller
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel
  -> ros_gz_bridge
  -> Gazebo
```

重点:

VLFM 如果直接发 `/cmd_vel`，就要避免 Nav2 也在发 `/cmd_vel`。

### 看 `/cmd_vel` 有没有多个发布者

```bash
ros2 topic info /cmd_vel
```

如果 `Publisher count` 大于 1，就要小心。

### 手动给 Nav2 发目标

最常用是在 RViz 里点 `2D Goal Pose`。

命令行也可以，但对新手不如 RViz 直观。

---

## 10. Mission Node 在这里干什么

`scripts/mission_node.py` 做的事很简单:

```text
等待 Nav2 active
发送 Waypoint A
发送 Waypoint B
发送 Waypoint C
发送 Return Home
```

默认航点在 `map` 坐标系。

它不是底层控制器，而是任务脚本。

对 VLFM 来说:

```text
Mission Node 的角色 ~= VLFM 高层决策
```

所以接 VLFM 时建议:

```bash
ros2 launch limo_pro_sim demo.launch.py mission:=false
```

---

## 11. RViz 怎么看

RViz 是 ROS2 可视化工具。

启动:

```bash
rviz2 -d $(ros2 pkg prefix limo_pro_sim)/share/limo_pro_sim/rviz/demo.rviz
```

或者用:

```bash
ros2 launch limo_pro_sim demo.launch.py mission:=false
```

它会自动开 RViz。

你重点看:

| RViz 显示项 | 看什么 |
|---|---|
| TF | 坐标系树是否完整 |
| RobotModel | 机器人模型姿态是否正常 |
| LaserScan | `/scan` 是否围绕机器人 |
| Image | `/camera/image` 是否正常 |
| Map | `/map` 是否在增长 |
| PointCloud2 | `/cloud_map` 是否正常 |
| Path | Nav2 规划路径 |
| Costmap | 障碍物代价地图 |

### 新手最推荐的观察顺序

1. 先看 RobotModel 是否出现。
2. 再看 LaserScan 是否有一圈扫描。
3. 再看 Camera Image 是否有画面。
4. 再看 Map 是否逐渐出现。
5. 最后看 Nav2 Path 和 Costmap。

---

## 12. 怎么手动玩机器人

### 方法 1: RViz 发目标

启动:

```bash
ros2 launch limo_pro_sim demo.launch.py mission:=false
```

在 RViz 中用:

```text
2D Goal Pose
```

点一个目标，看 Nav2 是否规划并移动。

### 方法 2: 键盘遥控

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

它会发布 `/cmd_vel`。

注意:

如果 Nav2 也在发 `/cmd_vel`，键盘和 Nav2 会抢控制。调试时尽量只保留一个控制源。

### 方法 3: 直接发一条速度

安全起见先发零速:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

短暂前进:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}"
```

停止:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

不要长时间无人看管地发非零速度。

---

## 13. VLFM 应该接在哪里

VLFM 有两种接法。

### 接法 A: VLFM 直接发 `/cmd_vel`

数据流:

```text
Gazebo RGB-D / TF / Odom
  -> LimoROS2Robot
  -> VLFM observation
  -> VLFM policy
  -> /cmd_vel
  -> Gazebo
```

优点:

- 最直接。
- 不依赖 Nav2 action。
- 和 VLFM 原来的 real-world policy 输出 `linear/angular` 接近。

缺点:

- 要自己保证避障和安全停止。
- 要避免 Nav2 抢 `/cmd_vel`。

适合:

- 第一版闭环实验。

### 接法 B: VLFM 只决定目标点，交给 Nav2 走

数据流:

```text
VLFM value map / frontier
  -> 选一个目标点
  -> NavigateToPose
  -> Nav2
  -> /cmd_vel
  -> Gazebo
```

优点:

- Nav2 负责局部控制、避障、速度平滑。
- 更像工程系统。

缺点:

- VLFM 原始 PointNav 头会被绕过。
- 需要把 VLFM 的目标点转到 `map` frame。
- 行为和原 VLFM 论文/代码路径差异更大。

适合:

- 后续稳定版本。

当前推荐:

```text
先接法 A dry-run 和短闭环。
确认坐标系、depth、动作都对后，再评估接法 B。
```

---

## 14. 常用检查命令清单

### Topic 总览

```bash
ros2 topic list
```

### Topic 类型和发布者

```bash
ros2 topic info /camera/image
ros2 topic info /cmd_vel
```

### 频率

```bash
ros2 topic hz /camera/image
ros2 topic hz /camera/depth_image
ros2 topic hz /scan
```

### 看一帧消息

```bash
ros2 topic echo /camera/camera_info --once
ros2 topic echo /odom --once
ros2 topic echo /map --once
```

### TF

```bash
ros2 run tf2_ros tf2_echo base_footprint depth_link
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_tools view_frames
```

### 控制

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

---

## 15. 你怎么通过实验判断“我理解了”

按下面顺序做，每一步都能解释清楚，就说明你已经摸到核心了。

### 实验 1: 只看传感器

启动:

```bash
ros2 launch limo_pro_sim sim.launch.py
```

验证:

```bash
ros2 topic hz /camera/image
ros2 topic hz /camera/depth_image
ros2 topic echo /camera/camera_info --once
ros2 topic hz /scan
```

你应该能说清:

- RGB-D 是 Gazebo 传感器产生的。
- `ros_gz_bridge` 把它桥成 ROS2 topic。
- VLFM 后面会读这些 topic。

### 实验 2: 看 TF

命令:

```bash
ros2 run tf2_ros tf2_echo base_footprint depth_link
```

你应该能说清:

- 相机装在机器人哪里。
- `depth_link` 是 optical frame。
- VLFM 需要它把 depth 投到地图里。

### 实验 3: 看 SLAM

启动:

```bash
ros2 launch limo_pro_sim demo.launch.py mission:=false
```

验证:

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
```

你应该能说清:

- RTAB-Map 发布 `/map`。
- RTAB-Map 发布 `map->odom`。
- 没有 SLAM 时未必有 `map`。

### 实验 4: 看控制

先看 `/cmd_vel`:

```bash
ros2 topic info /cmd_vel
```

发零速:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

你应该能说清:

- `/cmd_vel` 是 ROS2 发给 Gazebo 的控制入口。
- 如果多个 publisher 同时发，会抢控制。

### 实验 5: 看 Nav2 和 Mission 的区别

启动:

```bash
ros2 launch limo_pro_sim demo.launch.py mission:=false
```

用 RViz 的 `2D Goal Pose` 发目标。

你应该能说清:

- Nav2 是接目标点后规划控制。
- Mission Node 只是自动发固定目标点。
- VLFM 后面也会成为一种“发目标/发动作”的上层逻辑。

---

## 16. 接 VLFM 前必须确认的接口

接 VLFM 前至少确认:

```text
/camera/image:
  type = sensor_msgs/msg/Image
  encoding 正常
  shape = 640x480

/camera/depth_image:
  type = sensor_msgs/msg/Image
  dtype / encoding 明确
  单位明确，最好统一成 meters

/camera/camera_info:
  fx/fy 可读

TF:
  base_footprint -> depth_link 可查
  map/odom -> base_footprint 可查

/cmd_vel:
  能发布零速
  没有多个控制源抢占
```

这些通过后，VLFM adapter 才有稳定输入输出。

