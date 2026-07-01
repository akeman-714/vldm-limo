# 27 · ROS2 + Gazebo + VLFM 玩法教学（从零到能动手）

> 这份文档解决一件事：你"大概知道每个东西干啥，但一点不懂具体"。
> 所以它不堆理论。每个概念都是三段式：
>
> 1. **一句话**：这个名词到底是什么（用生活比喻）。
> 2. **敲一条命令**：在你这台 DGX 上真能敲、敲完亲眼看到。
> 3. **你应该能回答**：能答上来就说明你"具体"懂了。
>
> 配套可运行代码：[`scripts/ros2_learn/talker.py`](../../scripts/ros2_learn/talker.py)、[`listener.py`](../../scripts/ros2_learn/listener.py)。
> 相关文档：操作验证看 [17](./17_Limo_Gazebo_ROS2_仿真理解与调试手册.md)，最终接口看 [26](./26_Limo_Gazebo_VLFM_最终耦合说明与接口.md)，数据形态查 [05](./05_数据形态速查表.md)。

---

## 0. 准备：在这台 DGX 上让 ros2 命令可用

这台机器装了 ROS2 **Jazzy**（在 `/opt/ros/jazzy`），但默认不在 PATH 里。每开一个新终端，先：

```bash
cd /home/asong/vlfm
source scripts/source_limo_ros_env.sh
```

这一条等于做了三件事：`source` ROS2 Jazzy + mentor 的仿真工作区 + 你的 `vlfm_ros312` venv。之后 `ros2` 命令、`rclpy`、VLFM 代码都能用了。

验证：

```bash
ros2 --help          # 有输出就说明 ROS2 进环境了
echo $ROS_DISTRO     # 应该打印 jazzy
```

> 整份文档约定：凡是要敲 ros2 命令的终端，**都先 source 这一行**。后面不再重复提醒。

---

## 1. 一张图看懂整条链路

先把全貌钉在脑子里，后面每一节都是在放大这张图的某一块。

```text
            ┌─────────────────────────────────────────────────────────────┐
            │  Gazebo（物理世界模拟器）                                      │
            │  小房子 + Limo 小车 + 轮子/碰撞 + 相机/激光/IMU 传感器          │
            └───────┬──────────────────────────────────────────▲──────────┘
   传感器数据(GZ内部) │                                            │ /cmd_vel(让车动)
                     ▼                                            │
            ┌─────────────────────────────────────────────────────────────┐
            │  ros_gz_bridge（翻译官：Gazebo话 ⇄ ROS2话）                    │
            └───────┬──────────────────────────────────────────▲──────────┘
                     ▼  这下面全是"ROS2 topic"，是你能用命令看的东西        │
   /camera/image  /camera/depth_image  /camera/camera_info  /scan  /imu  /odom  /tf
                     │                                            │
        ┌────────────┴───────────────┐                           │
        ▼                            ▼                            │
 ┌──────────────┐            ┌──────────────────┐                 │
 │ RTAB-Map     │  建图+定位  │  Nav2            │  规划+避障+控制   │
 │ 出 /map      │──────────▶ │  收"目标点"       │─────────────────┘
 │ 出 map→odom  │  /map,/tf  │  出 /cmd_vel      │
 └──────────────┘            └────────▲─────────┘
                                      │ 发"去这个点"(NavigateToPose)
                          ┌───────────┴────────────┐
                          │ "决定去哪"的上层大脑      │   ← 这一层是关键！
                          │  ① mission_node(固定航点)  或                │
                          │  ② VLFM(看图找目标自己决定) │
                          └────────────────────────┘
```

**五个角色，一句话各自的活：**

| 角色 | 它的活 | 类比 |
|---|---|---|
| Gazebo | 模拟一个有物理的世界和小车，产生传感器数据，接受速度让车动 | 一个带物理引擎的电子游戏 |
| ros_gz_bridge | 把 Gazebo 内部数据翻译成 ROS2 topic（反向把 /cmd_vel 翻回去） | 同声传译 |
| RTAB-Map | 边走边建地图、算自己在哪（SLAM） | 一边走一边画地图的人 |
| Nav2 | 给它一个目标点，它负责规划路径、避障、发速度把车开过去 | 老司机 + 导航软件 |
| 大脑（mission / **VLFM**） | 决定"现在去哪个点" | 坐在后座说"我们去找椅子"的人 |

> **VLFM 的定位一句话**：它替换最上面那个"决定去哪"的大脑。底下的 Gazebo/桥/SLAM/Nav2 一概不动，VLFM 只负责看着相机和地图，**决定下一个目标点**，交给 Nav2 去走。

---

## 2. ROS2 五个核心名词（你点名要学的）

这一章把你说的"节点、订阅、输出数据格式、链路"逐个拆开。每个都给一条命令让你**当场看到**。

### 2.1 节点 node = 一个"上了车"的程序

**一句话**：节点就是一个正在运行的小程序，它加入了 ROS2 这条"总线"，于是能和别的节点收发数据。Gazebo 是节点、RTAB-Map 是节点、Nav2 里有一堆节点、你写的 talker 也是节点。

**敲命令**（需要先把仿真起起来，见第 3 章；或先用第 2.6 节的 talker）：

```bash
ros2 node list              # 列出当前所有"上了车"的程序
ros2 node info /controller_server   # 看某个节点：它订阅啥、发布啥、提供啥服务
```

**你应该能回答**：节点 ≈ 进程；一个程序里可以有一个或多个节点；它们彼此不用知道对方是谁，只靠 topic 名字对接。

### 2.2 topic = 一条"有名字的数据流"

**一句话**：topic 是一条命了名的广播频道，比如 `/camera/image`。谁都可以往这条频道"发"，谁都可以"听"。名字以 `/` 开头。

**敲命令**：

```bash
ros2 topic list                 # 当前所有频道（数据流）
ros2 topic info /camera/image   # 这条频道是什么类型、几个发布者几个订阅者
ros2 topic hz /camera/image     # 这条频道每秒来几帧（频率）
```

**你应该能回答**：`/camera/image` 是相机图像的流；`hz` 接近 15 说明相机在正常出图；一条 topic 同时可以有多个发布者和多个订阅者。

### 2.3 发布 / 订阅（publisher / subscriber）= ROS 的核心玩法

**一句话**：
- **发布者(publisher)**：往某条 topic 上"丢数据"的节点。
- **订阅者(subscriber)**：登记"我要听某条 topic"，之后每来一条数据，ROS 自动调用你的回调函数把数据递给你。

这是整个 ROS 最重要的一个模式。关键点：**发布方和订阅方互不认识**，一个发一个收，全靠 topic 名字对上。Gazebo 不知道 VLFM 存在，但 VLFM 订阅了 `/camera/image`，就能收到 Gazebo 产生的图。

**敲命令**（手动当一回"订阅者"看一条数据）：

```bash
ros2 topic echo /odom --once    # echo = 临时订阅，把收到的一条消息打印出来
```

**你应该能回答**：订阅不是"我主动去拿"，而是"我登记好，数据来了 ROS 推给我"（回调）。这就是为什么 VLFM 节点里全是 `_rgbd_cb`、`_map_cb` 这种 callback 函数——见 [limo_vlfm_node.py:304](../../vlfm/ros/limo_vlfm_node.py#L304)。

### 2.4 消息类型 = 数据格式（你最想搞懂的"输出数据长啥样"）

**一句话**：每条 topic 上跑的不是随便的字节，而是一种**有固定结构的消息类型**，比如 `sensor_msgs/msg/Image`。这就是你说的"输出数据格式"。知道类型，就知道里面有哪些字段、每个字段什么意思。

**怎么查一条 topic 是什么类型，以及那个类型有哪些字段：**

```bash
ros2 topic type /camera/image                    # 先问：这条流是什么类型？
#   sensor_msgs/msg/Image
ros2 interface show sensor_msgs/msg/Image         # 再问：这个类型里有哪些字段？
```

下面把**你这套系统里最常见的几个数据格式**拆给你看（这就是"具体"）：

#### `sensor_msgs/msg/Image`（相机图 / 深度图）
```text
std_msgs/Header header     # 时间戳 stamp + 坐标系名 frame_id（这条数据是哪个相机、什么时刻的）
uint32 height              # 高，比如 480
uint32 width               # 宽，比如 640
string encoding            # 像素格式：rgb8 / bgr8（彩色）或 16UC1 / 32FC1（深度）
uint32 step                # 一行多少字节
uint8[] data               # 真正的像素，一长串字节
```
- 彩色图：`encoding=rgb8`，`data` 是 640×480×3 个 uint8。
- 深度图：`encoding=16UC1` 表示每像素一个 16 位整数、**单位毫米**；`32FC1` 表示 float、**单位米**。
  → 这就是为什么 VLFM 节点里有一段把毫米 ×0.001 转成米的代码：[limo_vlfm_node.py:57](../../vlfm/ros/limo_vlfm_node.py#L57)。单位搞错，整个地图就崩。

#### `sensor_msgs/msg/CameraInfo`（相机内参，VLFM 必需）
```text
uint32 height, width
float64[9] k   # 3x3 内参矩阵摊平：[fx, 0, cx,  0, fy, cy,  0, 0, 1]
```
- VLFM 只取两个数：`fx = k[0]`，`fy = k[4]`，用来把深度图反投影成 3D 点。对应代码 [limo_vlfm_node.py:311](../../vlfm/ros/limo_vlfm_node.py#L311)。
- 看一眼真实数值：`ros2 topic echo /camera/camera_info --once`，重点看 `k:` 那一行。

#### `geometry_msgs/msg/Twist`（速度命令，`/cmd_vel` 用）
```text
Vector3 linear    # x 前进速度(m/s)、y、z
Vector3 angular   # x、y、z 转向角速度(rad/s)
```
- 差速小车只用 `linear.x`（前后）+ `angular.z`（左右转）。其余填 0。
- 这是你能**手动开车**的入口（第 4 章关卡 4）。

#### `nav_msgs/msg/Odometry`（里程计：我在哪、走多快）
```text
Header header
string child_frame_id
PoseWithCovariance pose    # pose.pose.position(x,y,z) + pose.pose.orientation(四元数 x,y,z,w)
TwistWithCovariance twist  # 当前速度
```
- 注意朝向是**四元数**（x,y,z,w 四个数），不是角度。VLFM 里 [`yaw_from_quaternion`](../../vlfm/ros/limo_vlfm_node.py#L66) 就是把四元数转成一个偏航角 yaw。

#### `nav_msgs/msg/OccupancyGrid`（2D 栅格地图，`/map`）
```text
Header header
MapMetaData info   # resolution(每格多少米) + width + height + origin(地图左下角在世界的位姿)
int8[] data        # 一长串格子值：-1=未知, 0=空闲可走, 100=被占(墙/障碍)
```
- VLFM 把它读进来变成 ObstacleMap、再算 frontier（已知/未知的边界）。对应 [limo_vlfm_node.py:336](../../vlfm/ros/limo_vlfm_node.py#L336)。

#### `geometry_msgs/msg/PoseStamped`（一个带坐标系的目标点）
```text
Header header     # 关键是 header.frame_id，比如 "map"——"这个点是相对哪个坐标系说的"
Pose pose         # position(x,y,z) + orientation(四元数)
```
- **VLFM 的输出就是它**：算出"下一个该去的点"，包成 PoseStamped 发给 Nav2。见 [`to_pose`](../../vlfm/ros/limo_vlfm_node.py#L356)。

**你应该能回答**："输出数据格式"= 消息类型；`ros2 interface show <类型>` 能拆开看字段；图像看 encoding（彩色还是深度、单位是毫米还是米），位姿看是四元数还是角度，地图看 resolution/origin/data。

### 2.5 链路 = 数据怎么从一个节点流到另一个

**一句话**：链路就是"谁发布 → 哪条 topic → 谁订阅"串起来的一条数据路径。整套系统就是很多条这样的链拼起来的。

举一条真实的链（相机到 VLFM）：

```text
Gazebo[发布] → /camera/image → ros_gz_bridge[转发] → /camera/image(ROS2)
   → LimoVLFMNode[订阅,_rgbd_cb] → 攒成 observation → policy 决策 → /vlfm/goal → Nav2
```

**敲命令**（把整张"谁连谁"的图画出来）：

```bash
ros2 run rqt_graph rqt_graph      # 弹出图形界面：椭圆=节点，方框/箭头=topic，一眼看清链路
```

如果没有图形界面，纯文字也能反推：

```bash
ros2 topic info /cmd_vel -v       # -v 会列出这条 topic 的"发布者是谁、订阅者是谁"
```

**你应该能回答**：要追一条数据走到哪了，就顺着"它订阅了谁、又发布给谁"一节节查；`rqt_graph` 是看全局链路最快的工具。

### 2.6 先别管仿真，自己跑两个节点把 2.1~2.5 串一遍（10 分钟）

不依赖 Gazebo，纯靠你刚 source 好的 ROS2 就能玩。开**两个终端**，都先 `source scripts/source_limo_ros_env.sh`。

```bash
# 终端 A：发布者
python3 scripts/ros2_learn/talker.py
# 每秒打印 "发出: hello ros2 #N"

# 终端 B：订阅者
python3 scripts/ros2_learn/listener.py
# 每秒打印 "收到: hello ros2 #N"   ← A 发的，B 收到了
```

再开**第三个终端**，不写一行代码，用纯命令行同时验证 2.1~2.5：

```bash
ros2 node list                       # 看到 /learn_talker /learn_listener —— 这就是"节点"(2.1)
ros2 topic list | grep learn         # 看到 /learn/chatter —— 这就是"topic"(2.2)
ros2 topic type /learn/chatter       # std_msgs/msg/String —— 这就是"数据格式"(2.4)
ros2 topic echo /learn/chatter       # 你自己也当一回订阅者，看到数据流过 —— "订阅"(2.3)
ros2 topic hz /learn/chatter         # ~1.0 Hz —— 频率
ros2 run rqt_graph rqt_graph         # 看到 talker → /learn/chatter → listener —— "链路"(2.5)
```

读一遍 `talker.py`：`create_publisher` 那行就是 2.3 的"成为发布者"，`msg.data=...; self.pub.publish(msg)` 就是 2.4 的"造一条数据并发出"。读 `listener.py`：`create_subscription` 那行登记订阅，`on_msg` 就是"数据来了 ROS 自动调的回调"。

**做完这一步，你对 ROS2 的"具体"已经懂一大半了。** 后面所有复杂节点（Gazebo、Nav2、VLFM）只是同一套机制的放大版。

### 2.7 还有两个你迟早撞上的概念（先混个脸熟）

- **TF（坐标变换）**：机器人有一堆坐标系——地图、车身、相机……TF 就是"任意两个坐标系之间的相对位姿"这张实时更新的表。为什么重要：相机看到"前方 2 米有个椅子"，要知道它在**地图**里的坐标，就得用 TF 把"相机坐标"换算到"地图坐标"。这套系统的 TF 链是：
  ```text
  map → odom → base_footprint → base_link → depth_camera_link → depth_link
  ```
  查任意两个之间的关系：
  ```bash
  ros2 run tf2_ros tf2_echo map base_footprint   # 车在地图里的位姿
  ros2 run tf2_ros tf2_echo base_footprint depth_link   # 相机装在车的哪
  ```
  VLFM 节点里 [`lookup_pose`](../../vlfm/ros/limo_vlfm_node.py#L317) 就是在查 TF，拿车和相机在 map 里的位姿。

- **QoS（服务质量）**：发布/订阅可以配"可靠性"和"是否保留最后一条"。99% 情况你不用管。但有一个坑要知道：`/map` 用的是 `TRANSIENT_LOCAL`（晚来的订阅者也能补到最后一帧地图）。如果你订阅 `/map` 收不到，多半是 QoS 没对上——VLFM 节点专门为此配了 QoS，见 [limo_vlfm_node.py:284](../../vlfm/ros/limo_vlfm_node.py#L284)。

---

## 3. Gazebo 是什么 & 这套仿真怎么起

### 3.1 Gazebo 干的活

Gazebo 是**物理仿真器**。在这套系统里它负责：加载小房子世界、放一台 Limo 小车、模拟轮子/碰撞/摩擦、模拟 RGB-D 相机 + 2D 激光 + IMU + 轮速里程计，并且接收 `/cmd_vel` 让车真的动起来。

它产生的数据先在 Gazebo "内部话"里，靠 **ros_gz_bridge** 翻成 ROS2 topic 你才看得到。桥是**双向**的：传感器数据 `Gazebo→ROS2`，速度命令 `/cmd_vel` 是 `ROS2→Gazebo`。

### 3.2 在你这台 DGX 上起仿真（用现成脚本，别手敲 launch）

你已经有一个管理脚本 [`scripts/limo_sim.sh`](../../scripts/limo_sim.sh)，专治"同一台机器叠开多套仿真互相抢资源"的坑。三条命令：

```bash
bash scripts/limo_sim.sh status   # 看现在有没有仿真在跑、VLM 服务端口起没起
bash scripts/limo_sim.sh up       # 干净地起一套（已经有就拒绝，避免叠开）
bash scripts/limo_sim.sh down     # 彻底拆掉仿真（但不碰 VLFM 的 VLM 服务）
```

`up` 实际跑的是 mentor 的 `ros2 launch limo_pro_sim demo.launch.py mission:=false`：起 Gazebo + 桥 + RTAB-Map + Nav2 + RViz，**但不起固定航点 mission**——因为那一层要留给 VLFM。

> **第一坑**：千万别连着敲两次 `up`。两套仿真在同一个域里抢 `/map`、`/cmd_vel`、`/tf`，相机会掉到 2Hz、Nav2 永远到不了点。先 `status` 看，要重来先 `down`。

起好后回到第 2 章的命令，这次 `ros2 topic list` 会一下多出几十条真实 topic。

---

## 4. 顺着真实链路走一遍（闯关式玩法）

仿真起好后（`bash scripts/limo_sim.sh up`），按顺序闯关。每关都是"敲命令 → 看预期 → 你学到了啥"。全部通关，你就把这条链摸透了。

### 关卡 1 · 看传感器（链路最上游）
```bash
ros2 topic hz /camera/image          # 预期 ≈15 Hz
ros2 topic hz /camera/depth_image    # 预期 ≈15 Hz
ros2 topic echo /camera/camera_info --once   # 看 k: 里的 fx,fy
ros2 topic hz /scan                  # 激光
```
学到：这些数据是 Gazebo 产生、桥翻译成 topic 的。VLFM 后面就吃这几条。

### 关卡 2 · 看 TF（数据能不能落到地图里全靠它）
```bash
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo base_footprint depth_link
ros2 run tf2_tools view_frames      # 生成 frames.pdf，看坐标系树是否完整
```
学到：有了 TF，才能把"相机里看到的东西"换算成"地图里的坐标"。

### 关卡 3 · 看 SLAM（地图是 RTAB-Map 建的）
```bash
ros2 topic hz /map                  # 地图在更新
ros2 topic echo /map --once         # 看 info.resolution / info.width / data 的值
ros2 run tf2_ros tf2_echo map odom  # 这条 TF 是 RTAB-Map 发的全局定位修正
```
学到：`/map` 是 OccupancyGrid（2.4 拆过）；没有 SLAM 就没有 `/map`，VLFM 也就没法算 frontier。

### 关卡 4 · 看控制（你亲手开一下车）
```bash
ros2 topic info /cmd_vel -v         # 先看有几个发布者（多于 1 就会抢控制）
# 让车慢慢前进（rate 10 = 每秒发 10 条，停发车就停）：
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
# Ctrl-C 停下，再补一条零速保险：
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```
学到：`/cmd_vel` 是控制入口，数据格式是 Twist（2.4）。**别让两个东西同时发 `/cmd_vel`**——你手动发的时候，别让 Nav2 也在发。

### 关卡 5 · 看 Nav2（给个目标点，它自己开过去）
在 RViz 里点工具栏的 **2D Goal Pose**，在地图上点一个点。看车自己规划路径、避障、开过去。

学到：Nav2 = "你给目标点，我负责走到"。**VLFM 要做的，就是不用你手点、由算法自动产生这个目标点。**

> 想要每关更细的预期输出和排障，看 [17 号文档](./17_Limo_Gazebo_ROS2_仿真理解与调试手册.md) 第 4~12 节。

---

## 5. VLFM 玩法（接到你真实的节点）

前面四章你已经能"读"整套系统了。这一章讲 VLFM 怎么插进去——而且全部对着你仓库里的真实代码。

### 5.1 VLFM 在链路里的位置

回到第 1 章那张图最底下的"大脑"。固定航点 `mission_node` 是把死的航点喂给 Nav2；**VLFM 把这个大脑换成"看图找目标、自己决定下一个点"**：

```text
/camera/image + /camera/depth_image + /camera/camera_info + /map + TF
        │
        ▼  (订阅、攒齐、转格式)
   LimoVLFMNode  ──> 一份 observation(obs_cache)
        │
        ▼  (VLFM 策略：value map 找"最可能有目标的方向" + 物体检测)
   policy.decide_goal(obs)  ──> 一个目标点 {xy, yaw}
        │
        ▼  (包成 PoseStamped)
   /vlfm/goal ──> Nav2.goToPose ──> /cmd_vel ──> 车动
```

底层 Gazebo / 桥 / RTAB-Map / Nav2 **完全不动**，只有"决定去哪"这一层被 VLFM 接管。

### 5.2 LimoVLFMNode 订阅什么、发布什么（对着代码看）

这是整个耦合的核心节点：[`vlfm/ros/limo_vlfm_node.py`](../../vlfm/ros/limo_vlfm_node.py)。

**它订阅（输入）：**

| topic | 类型 | 它拿来干嘛 | 代码 |
|---|---|---|---|
| `/camera/image` | `sensor_msgs/Image` | 彩色图，喂检测 + value map | [L279](../../vlfm/ros/limo_vlfm_node.py#L279) |
| `/camera/depth_image` | `sensor_msgs/Image` | 深度图，反投影成 3D | [L280](../../vlfm/ros/limo_vlfm_node.py#L280) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 取 fx, fy | [L283](../../vlfm/ros/limo_vlfm_node.py#L283) |
| `/map` | `nav_msgs/OccupancyGrid` | 算障碍/frontier | [L289](../../vlfm/ros/limo_vlfm_node.py#L289) |
| TF（map→base、map→camera） | tf2 | 把观测落到地图坐标 | [L317](../../vlfm/ros/limo_vlfm_node.py#L317) |

> 注意一个细节：彩色图和深度图必须是"同一时刻"的一对，所以用了 `message_filters.ApproximateTimeSynchronizer`（[L281](../../vlfm/ros/limo_vlfm_node.py#L281)）把两条流按时间戳配对，配上了才触发 `_rgbd_cb`。这是处理"多路传感器要对齐"的标准做法。
>
> 另一个细节：代码里订阅的**默认** topic 名是 RealSense 真机风格（`/camera/color/image_raw`）。跑 mentor 仿真时，[`limo_mission.py`](../../scripts/limo_mission.py) 里的 `NODE_DEFAULTS` 把它们改写成仿真的名字（`/camera/image` 等）。所以"同一个节点，喂不同 topic 名就能接真机或仿真"。

**它发布（输出）：**

| topic | 类型 | 是什么 | 代码 |
|---|---|---|---|
| `/vlfm/goal` | `geometry_msgs/PoseStamped` | VLFM 选定的下一个目标点（也直接喂给 Nav2） | [L290](../../vlfm/ros/limo_vlfm_node.py#L290) |
| `/vlfm/frontiers` | `visualization_msgs/MarkerArray` | 所有候选 frontier 点，RViz 里一堆小球 | [L291](../../vlfm/ros/limo_vlfm_node.py#L291) |
| `/vlfm/vis/annotated_rgb` | `sensor_msgs/Image` | 原图叠检测框/掩码 | [L293](../../vlfm/ros/limo_vlfm_node.py#L293) |
| `/vlfm/vis/value_map` | `sensor_msgs/Image` | "哪个方向最可能有目标"的热力图 | [L295](../../vlfm/ros/limo_vlfm_node.py#L295) |
| `/vlfm/vis/object_map` | `sensor_msgs/Image` | 障碍地图 + 目标点云 | [L296](../../vlfm/ros/limo_vlfm_node.py#L296) |
| `/vlfm/vis/obstacle_map` | `sensor_msgs/Image` | 障碍/已探索/frontier | [L294](../../vlfm/ros/limo_vlfm_node.py#L294) |

这些 `/vlfm/vis/*` 不影响导航，纯粹是**让你能在 RViz 里看懂 VLFM 脑子里在想什么**。

### 5.3 数据格式怎么一步步变（你最关心的"链路里数据变化"）

```text
ROS2 消息              →   节点里转成的中间格式            →   策略吃的格式
─────────────────────────────────────────────────────────────────────
Image(深度,16UC1,mm)   →   normalize_depth_array() 归一化   →   float32 [0,1] 深度
Image(彩色,bgr8)       →   转 RGB ndarray                  →   H×W×3 uint8
CameraInfo.k[0],k[4]   →   fx, fy                          →   get_fov() 算视场角
OccupancyGrid.data     →   reshape 成 H×W int8 网格          →   ObstacleMap → frontiers
TF map→base/camera     →   xyz + yaw                        →   tf_cam2map 4x4 矩阵
                       ↓
              build_limo_observation_cache() 把上面打包成一个 dict：
              { frontier_sensor, robot_xy, robot_heading,
                nav_depth, object_map_rgbd, value_map_rgbd }   ← 这就是 "observation"
                       ↓
              policy.decide_goal(obs) → { mode, xy, yaw_hint } → to_pose() → PoseStamped
```

打包那一步就是 [`build_limo_observation_cache`](../../vlfm/ros/limo_vlfm_node.py#L92)（L92），决策那一步在 [`run_nav2_mission`](../../vlfm/ros/limo_vlfm_node.py#L431) 的循环里（L431 起）。想看每个张量的精确 shape/单位，配合 [05 号速查表](./05_数据形态速查表.md) 和接口契约 [19 号](./19_Nav2耦合_接口契约.md)。

### 5.4 怎么把 VLFM 跑起来

VLFM 比纯导航多一个依赖：它要调用几个**视觉大模型服务**（检测、图文匹配 ITM、分割 SAM、问答 VQA），跑在本机端口 12181~12186。所以顺序是：

```bash
# 0) 先 source
source scripts/source_limo_ros_env.sh

# 1) 体检：看这台机器现在能跑到 VLFM 的哪一关（torch 在不在、哪些 VLM 服务起了）
python3 scripts/limo_preflight.py

# 2) 起仿真（如前述，别叠开）
bash scripts/limo_sim.sh up        # 另开终端，或 GUI=false 跑无头

# 3) 跑 VLFM 大脑 + Nav2 驾驶。target 通过环境变量给：
VLFM_TARGET_OBJECT=chair python3 scripts/limo_mission.py
#   或者直接命令行： python3 scripts/limo_mission.py chair "the black office chair"
```

`limo_mission.py` 干的事：构造真正的 `LimoITMPolicy` 策略 + `LimoVLFMNode` 节点，交给 [`run_nav2_mission`](../../vlfm/ros/limo_vlfm_node.py#L431) 跑闭环——先原地转一圈把 value map 填起来（[L464](../../vlfm/ros/limo_vlfm_node.py#L464)），然后反复"看→选点→发给 Nav2→走→到点验证"，直到 `FOUND & VERIFIED` 或探索穷尽。

> 这台 DGX 不是每个 VLM 服务都常驻。先看 `limo_preflight.py` 的输出：能到的最小关是"纯探索"，要做到真正"找到并确认目标"需要 ITM + 检测器 + SAM（+VQA）。这部分的关卡说明见 [26 号](./26_Limo_Gazebo_VLFM_最终耦合说明与接口.md)。

### 5.5 在 RViz 里看懂 VLFM 在想什么

跑起来后，在 RViz 里 **Add → By topic**，加这几个看：

| 看什么 | topic | 你能判断 |
|---|---|---|
| 候选边界点 | `/vlfm/frontiers` (MarkerArray) | 蓝色小球是否贴着"已知/未知"的边界 |
| 选中的目标 | `/vlfm/goal` (PoseStamped) | 箭头指向 VLFM 当前要去的点 |
| 价值热力图 | `/vlfm/vis/value_map` (Image) | 亮的地方=VLFM 觉得"那边更可能有目标" |
| 检测叠加 | `/vlfm/vis/annotated_rgb` (Image) | 目标被框出来没有 |

或者更省事，命令行直接看输出点：
```bash
ros2 topic echo /vlfm/goal           # 看 VLFM 实时选的目标点坐标
ros2 topic hz /vlfm/vis/value_map    # 看可视化在不在更新
```

---

## 6. 输出数据格式速查表（贴墙版）

| topic | 消息类型 | 关键字段 / 单位 / shape | 谁发 → 谁收 |
|---|---|---|---|
| `/camera/image` | sensor_msgs/Image | encoding=rgb8/bgr8；640×480×3 uint8 | Gazebo→VLFM |
| `/camera/depth_image` | sensor_msgs/Image | encoding=16UC1(mm) 或 32FC1(m)；640×480 | Gazebo→VLFM |
| `/camera/camera_info` | sensor_msgs/CameraInfo | k[0]=fx, k[4]=fy | Gazebo→VLFM |
| `/scan` | sensor_msgs/LaserScan | ranges[]（米）, angle_min/max/increment | Gazebo→Nav2/RTAB |
| `/odom` | nav_msgs/Odometry | pose.position + orientation(四元数), twist | Gazebo→RTAB/Nav2 |
| `/map` | nav_msgs/OccupancyGrid | resolution(m/格), origin, data∈{-1,0,100} | RTAB→Nav2/VLFM |
| `/tf` | tf2_msgs/TFMessage | 各坐标系相对位姿（含四元数） | 多方→所有人 |
| `/cmd_vel` | geometry_msgs/Twist | linear.x(m/s), angular.z(rad/s) | Nav2→Gazebo |
| `/vlfm/goal` | geometry_msgs/PoseStamped | frame_id=map, position + 四元数 | VLFM→Nav2 |
| `/vlfm/frontiers` | visualization_msgs/MarkerArray | 一组 SPHERE Marker | VLFM→RViz |
| `/vlfm/vis/*` | sensor_msgs/Image | bgr8 调试画面 | VLFM→RViz |

记不住字段就一条命令现查：`ros2 interface show <类型>`。

---

## 附录 A · 命令小抄

```bash
# 环境
source scripts/source_limo_ros_env.sh

# 节点
ros2 node list / ros2 node info <node>

# topic
ros2 topic list
ros2 topic info <topic> -v        # 类型 + 发布者/订阅者
ros2 topic type <topic>           # 只看类型
ros2 topic hz <topic>             # 频率
ros2 topic echo <topic> --once    # 看一条数据
ros2 interface show <类型>         # 拆解数据格式字段

# 链路
ros2 run rqt_graph rqt_graph

# TF
ros2 run tf2_ros tf2_echo <frameA> <frameB>
ros2 run tf2_tools view_frames

# 控制（小心抢 /cmd_vel）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

# 仿真栈
bash scripts/limo_sim.sh status|up|down

# 自己写的最小节点（学习用）
python3 scripts/ros2_learn/talker.py
python3 scripts/ros2_learn/listener.py

# VLFM
python3 scripts/limo_preflight.py
VLFM_TARGET_OBJECT=chair python3 scripts/limo_mission.py
```

## 附录 B · 新手最容易踩的 5 个坑

1. **新终端 ros2 命令找不到** → 没 `source scripts/source_limo_ros_env.sh`。
2. **相机掉到 2Hz、Nav2 永远到不了点** → 叠开了多套仿真。`limo_sim.sh status` 看，`down` 再 `up`。
3. **订阅 `/map` 收不到数据** → QoS 没对上（要 TRANSIENT_LOCAL）；命令行 `ros2 topic echo /map --once` 默认能收是因为它会自适应。
4. **车乱动 / 不动** → 多个发布者抢 `/cmd_vel`。`ros2 topic info /cmd_vel -v` 看发布者数；手动开车时别同时开 Nav2/VLFM。
5. **深度全错、地图鬼影** → 深度单位搞反（毫米 vs 米）。先 `ros2 topic echo /camera/depth_image --once` 看 encoding。

---

读完并把第 2.6 节、第 4 章亲手敲过一遍，你对"ROS2 节点/订阅/数据格式/链路 + Gazebo + VLFM 接在哪"就从"大概"变成"具体"了。下一步深入代码，按 README 的顺序读 20~24 号。
