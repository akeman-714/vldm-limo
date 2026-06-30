# 25 · DGX 同机 mentor Gazebo / ROS2 环境接入手册

> 目标：把这次“同一台 DGX 上两个账号，如何复用 mentor 的 Limo Gazebo + ROS2 + Nav2 环境”沉淀成可重复流程。
> 当前已验证场景：`asong` 账号访问 `xiaowei.du` 账号下的 `/home/xiaowei.du/projects/robot/ROS2/limo_nav_ws`，使用 ROS2 Jazzy + Gazebo Harmonic + RTAB-Map + Nav2。
>
> 状态提示（2026-06-30）：环境接入和仿真管理工具仍以本文为参考；VLFM 最终耦合架构、接口和 G2/G3 验收结果见 [26_Limo_Gazebo_VLFM_最终耦合说明与接口.md](./26_Limo_Gazebo_VLFM_最终耦合说明与接口.md)。

## 1. 规划

mentor 给的信息通常长这样：

```text
10.89.4.4:/home/xiaowei.du/projects/robot/ROS2/limo_nav_ws
```

含义：

| 字段 | 含义 |
| --- | --- |
| `10.89.4.4` | DGX 的内网 IP。若 `hostname -I` 里也有这个 IP，说明你和 mentor 在同一台机器上 |
| `/home/xiaowei.du/.../limo_nav_ws` | mentor 账号里的 ROS2 workspace |
| `limo_nav_ws/install/setup.bash` | build 后的 ROS2 overlay，source 后能看到 `limo_pro_sim` |

这类环境不一定需要复制。推荐顺序：

1. 先确认同机和权限。
2. 让你的账号获得只读/执行 ACL。
3. 直接 source mentor 的 workspace。
4. 用你自己的 venv 跑 VLFM ROS 节点和检查脚本。
5. 只有需要改 mentor 的 ROS 包时，才复制到你自己的目录重新 build。

安全原则：

- 不要在聊天窗口、命令行或脚本里明文写 sudo 密码。
- 不要直接改 mentor 的文件，除非提前说好。
- 默认只给 `rx` 权限，够 source、launch、读 config/URDF/RViz。
- 如果要修改，复制到 `/home/asong/...` 后自己 `chown` 和 build。

## 2. 详细步骤

### 2.1 确认是不是同一台 DGX

```bash
hostname -I
```

如果输出含：

```text
10.89.4.4
```

就说明 mentor 给的 IP 是当前机器，你可以直接访问本机路径，不需要 ssh。

### 2.2 确认系统 ROS2 / Gazebo

```bash
ls /opt/ros
source /opt/ros/jazzy/setup.bash
which ros2
which rviz2
which gz
```

本次验证结果：

```text
/opt/ros/jazzy/bin/ros2
/opt/ros/jazzy/bin/rviz2
/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz
```

### 2.3 给你的账号开 mentor workspace 读权限

如果你看到：

```text
Permission denied
```

让有权限的人在终端手动执行以下命令。注意：sudo 密码只在本地终端输入，不写进脚本。

```bash
sudo setfacl -m u:asong:x /home/xiaowei.du
sudo setfacl -m u:asong:x /home/xiaowei.du/projects
sudo setfacl -m u:asong:x /home/xiaowei.du/projects/robot
sudo setfacl -m u:asong:x /home/xiaowei.du/projects/robot/ROS2
sudo setfacl -R -m u:asong:rx /home/xiaowei.du/projects/robot/ROS2/limo_nav_ws
```

验证：

```bash
ls /home/xiaowei.du/projects/robot/ROS2/limo_nav_ws
```

期望：

```text
build  install  log  src
```

### 2.4 直接 source mentor 的 ROS2 workspace

```bash
source /opt/ros/jazzy/setup.bash
source /home/xiaowei.du/projects/robot/ROS2/limo_nav_ws/install/setup.bash
ros2 pkg list | grep -i limo
```

期望能看到：

```text
limo_pro_sim
```

### 2.5 准备 VLFM + ROS2 Python 环境

ROS2 Jazzy 使用系统 Python 3.12；普通 VLFM conda 环境未必能 import `rclpy/cv_bridge`。本次采用独立 venv，继承系统 ROS2 包：

```bash
python3 -m venv --system-site-packages /home/asong/venvs/vlfm_ros312
source /home/asong/venvs/vlfm_ros312/bin/activate
python -m pip install --upgrade pip
python -m pip install git+https://github.com/naokiyokoyama/frontier_exploration.git
python -m pip install numba 'coverage>=7.6.0'
```

本仓库已提供复用脚本：

```bash
source /home/asong/vlfm/scripts/source_limo_ros_env.sh
```

它会做：

```bash
source /opt/ros/jazzy/setup.bash
source /home/xiaowei.du/projects/robot/ROS2/limo_nav_ws/install/setup.bash
source /home/asong/venvs/vlfm_ros312/bin/activate
export PYTHONPATH="/home/asong/vlfm:${PYTHONPATH:-}"
```

验证：

```bash
source /home/asong/vlfm/scripts/source_limo_ros_env.sh
python - <<'PY'
import rclpy, cv_bridge, frontier_exploration
import vlfm.ros.limo_vlfm_node as node
print("env OK", node.__file__)
PY
```

期望：

```text
env OK /home/asong/vlfm/vlfm/ros/limo_vlfm_node.py
```

### 2.6 启动 mentor 的 Limo 仿真栈

VLFM 验收时不要启动 mentor 的固定 waypoint mission，因为 VLFM 应该是唯一 goal 发送者。

无头自动验收：

```bash
source /home/asong/vlfm/scripts/source_limo_ros_env.sh
ros2 launch limo_pro_sim demo.launch.py mission:=false rviz:=false gui:=false
```

带 RViz/Gazebo GUI：

```bash
source /home/asong/vlfm/scripts/source_limo_ros_env.sh
ros2 launch limo_pro_sim demo.launch.py mission:=false rviz:=true gui:=true
```

如果你的账号要打开 mentor 图形会话里的 RViz/Gazebo，mentor 可能需要在图形终端执行：

```bash
xhost +SI:localuser:asong
```

然后你设置：

```bash
export DISPLAY=:1
rviz2
```

### 2.7 G0 自动检查

本仓库提供：

```bash
source /home/asong/vlfm/scripts/source_limo_ros_env.sh
python /home/asong/vlfm/scripts/limo_g0_observation_check.py
```

它使用 mentor 仿真的默认 topic/frame：

| 数据 | topic / frame |
| --- | --- |
| RGB | `/camera/image` |
| Depth | `/camera/depth_image` |
| CameraInfo | `/camera/camera_info` |
| Map | `/map` |
| base frame | `base_footprint` |
| camera optical frame | `depth_link` |

通过日志示例：

```text
[g0] xy=[-0.0, -0.0] yaw=0.000 frontiers=15 nearest=[-1.185, 0.785] goal_yaw=2.556 depth_valid=0.66
```

这说明：

- RGB-D 已到。
- TF `map -> base_footprint`、`map -> depth_link` 可用。
- `/map` 已到。
- M1 的 `/map -> ObstacleMap -> frontier` 已跑通。
- `/vlfm/goal` 已发布一个 dry-run frontier goal。

RViz 人工核对：

- `/map` 有 2D occupancy grid。
- `/vlfm/goal` 箭头在 frontier 附近。
- frontier/goal 的 `xy` 与 RViz 量取一致，误差约一个 cell，即 `0.05m`。

### 2.8 G1 Nav2 最小闭环检查

检查 action：

```bash
source /home/asong/vlfm/scripts/source_limo_ros_env.sh
ros2 action list | grep navigate_to_pose
```

检查唯一司机：

```bash
ros2 topic info /cmd_vel -v
```

期望：

```text
Publisher count: 1
Node name: velocity_smoother
Subscription count: 1
Node name: ros_gz_bridge
```

发送一个 G0 日志里的 frontier goal：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -1.185, y: 0.785, z: 0.0}, orientation: {z: 0.957, w: 0.289}}}}" \
  --feedback
```

通过标准：

```text
Goal accepted
Goal finished with status: SUCCEEDED
```

本次实际跑通：机器人从原点附近移动到 `(-1.04, 0.67)` 附近，`navigate_to_pose` 返回 `SUCCEEDED`。

## 3. 接口

mentor 仿真接口和 19 号契约的差异：

| 19 默认 | mentor `limo_pro_sim` 实际 |
| --- | --- |
| `/camera/color/image_raw` | `/camera/image` |
| `/camera/aligned_depth_to_color/image_raw` | `/camera/depth_image` |
| `/camera/color/camera_info` | `/camera/camera_info` |
| `base_link` | `base_footprint` 更适合 Nav2/RTAB-Map |
| `camera_color_optical_frame` | `depth_link` |
| `/map` | `/map` |
| `navigate_to_pose` | `/navigate_to_pose` |

VLFM 节点参数应按实际系统传：

```bash
--ros-args \
  -p rgb_topic:=/camera/image \
  -p depth_topic:=/camera/depth_image \
  -p camera_info_topic:=/camera/camera_info \
  -p map_topic:=/map \
  -p map_frame:=map \
  -p base_frame:=base_footprint \
  -p camera_frame:=depth_link \
  -p min_depth:=0.3 \
  -p max_depth:=8.0
```

## 4. 注意点

1. **不要把密码发给 AI 或写进命令**
   sudo 需要密码时，在本地终端手动输入。屏幕无回显是正常的。

2. **`^[[200~sudo` 是粘贴控制字符问题**
   如果命令前出现 `^[[200~`，shell 会把它当成命令内容，导致 `sudo: 未找到命令`。重新手敲或清理粘贴前缀。

3. **同机不等于同权限**
   即使 IP 是同一台机器，只要 `/home/xiaowei.du` 是 `750`，`asong` 仍然进不去。必须 ACL 或复制。

4. **source 顺序固定**
   先 ROS2，再 mentor workspace，再 venv，再 `PYTHONPATH`：

   ```bash
   source /opt/ros/jazzy/setup.bash
   source /home/xiaowei.du/projects/robot/ROS2/limo_nav_ws/install/setup.bash
   source /home/asong/venvs/vlfm_ros312/bin/activate
   export PYTHONPATH=/home/asong/vlfm:$PYTHONPATH
   ```

5. **不要用普通 VLFM conda 直接跑 ROS2 Jazzy 节点**
   Jazzy 的 `rclpy/cv_bridge` 是 Python 3.12 系统包；Python 3.11 conda 环境容易出现 `rclpy`、`yaml`、`numpy ABI`、`cv_bridge` 不兼容。

6. **`mission:=false` 很重要**
   mentor 的 `demo.launch.py` 默认会启动 fixed waypoint mission。VLFM 接管时必须关掉，否则会有两个 goal 写者。

7. **GUI 不是必须，但 RViz 最终要看**
   自动检查能证明 topic/TF/cache/frontier/Nav2 action 通；frontier 是否“贴边界”最好 RViz 肉眼验一次。

8. **单套仿真原则（最容易踩的坑，详见 §8）**
   同机一次只允许一套仿真。起仿真前先 `bash scripts/limo_sim.sh status`，`gz sim server` 必须是 `0`。多套 `demo.launch.py` 叠在同一个 `ROS_DOMAIN_ID` 上会抢 `/map`、`/cmd_vel`、`/tf` 并共享 GPU，直接导致相机掉到 ~2Hz、Nav2 到达永远不 SUCCEEDED。**不要手敲 `pkill -f "gz sim"`** 去清理（会自匹配打断自己，还会误杀别人的仿真），统一用 `bash scripts/limo_sim.sh down`。

## 5. 可能问题 + 解法

| # | 现象 | 根因 | 解法 |
| --- | --- | --- | --- |
| 1 | `Permission denied` | mentor home/workspace 没给 ACL | 跑 §2.3 的 `setfacl` |
| 2 | `sudo: 未找到命令` | 粘贴带了 `^[[200~` 控制字符 | 重新手敲命令，从 `sudo` 开始 |
| 3 | `ros2` 找不到 | 没 source `/opt/ros/jazzy/setup.bash` | 先 source ROS2 |
| 4 | `limo_pro_sim` 找不到 | 没 source mentor workspace | source `.../limo_nav_ws/install/setup.bash` |
| 5 | `ModuleNotFoundError: frontier_exploration` | ROS2 Python 环境没有 VLFM frontier 包 | 用 §2.5 的 venv，或 source `scripts/source_limo_ros_env.sh` |
| 6 | `coverage.types.Tracer` 报错 | `numba` 和系统 `coverage` 版本不匹配 | 在 venv 装 `coverage>=7.6.0` |
| 7 | G0 一直 `RGB-D has not arrived yet` | 相机 topic 未发布或 headless 渲染未就绪 | 等几秒；查 `ros2 topic hz /camera/image`；必要时 `gui:=true` |
| 8 | G0 一直 `OccupancyGrid has not arrived yet` | RTAB-Map `/map` 未发布 | 查 `ros2 topic hz /map`、`ros2 node list | grep rtabmap` |
| 9 | Nav2 卡 `base_footprint to odom` | ICP odometry 还没初始化 TF | 等 `/scan` + `/imu`；查 `tf2_echo odom base_footprint` |
| 10 | `/cmd_vel` 有多个 publisher | mission/teleop/VLFM 同时开车 | 关 `mission`，关 teleop；保持 Nav2 唯一 `/cmd_vel` 发布链 |
| 11 | RViz 开不了 | DISPLAY/xhost 权限 | mentor 跑 `xhost +SI:localuser:asong`，你设置 `DISPLAY=:1` |
| 12 | 复制 workspace 后 launch 仍指向 mentor 路径 | ROS2 `install/` 里有绝对路径 | 复制后重新 `colcon build --symlink-install` |
| 13 | 相机掉到 ~2Hz、Nav2 到达永远不 SUCCEEDED、底盘乱转 | 同机叠了多套仿真，抢 GPU + 抢 `/map`/`/cmd_vel`/`/tf` | `bash scripts/limo_sim.sh status` 看是否 >1 套；`down` 清干净后只起一套再测，**别先归因到策略逻辑**（见 §8） |
| 14 | `gz sim` 进程在 launch 退出后还残留（孤儿） | launch 父进程死了但 gz sim 被 `systemd --user` 收养 | `bash scripts/limo_sim.sh down`（按字样匹配清孤儿，且保 VLM 服务） |
| 15 | 手敲 `pkill -f "gz sim"` 把自己/别人的会话也打断 | pattern 自匹配当前命令行 + 无差别杀别人的仿真 | 永远用 `scripts/limo_sim.sh down`（字样写在脚本文件里，命令行不含，故不自匹配） |

## 6. 具体验收

### 离线/环境验收

```bash
source /home/asong/vlfm/scripts/source_limo_ros_env.sh
python - <<'PY'
import rclpy, cv_bridge, frontier_exploration
import vlfm.ros.limo_vlfm_node as node
print("env OK", node.__file__)
PY
```

通过：

```text
env OK /home/asong/vlfm/vlfm/ros/limo_vlfm_node.py
```

### ROS graph 验收

启动：

```bash
source /home/asong/vlfm/scripts/source_limo_ros_env.sh
ros2 launch limo_pro_sim demo.launch.py mission:=false rviz:=false gui:=false
```

检查：

```bash
ros2 topic list | grep -E '^/map$|^/scan$|^/imu$|^/camera/image$|^/camera/depth_image$|^/tf$'
ros2 action list | grep navigate_to_pose
ros2 topic info /cmd_vel -v
```

通过关键点：

- `/map`、`/scan`、`/imu`、`/camera/image`、`/camera/depth_image`、`/tf` 都存在。
- `/navigate_to_pose` 存在。
- `/cmd_vel` publisher 只有 `velocity_smoother`。

### G0 验收

```bash
source /home/asong/vlfm/scripts/source_limo_ros_env.sh
python /home/asong/vlfm/scripts/limo_g0_observation_check.py
```

通过日志形态：

```text
[g0] xy=[...] yaw=... frontiers=N nearest=[...] goal_yaw=... depth_valid=...
```

本次实际通过：

```text
[g0] xy=[-0.0, -0.0] yaw=0.000 frontiers=15 nearest=[-1.185, 0.785] goal_yaw=2.556 depth_valid=0.66
```

### G1 最小 Nav2 验收

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -1.185, y: 0.785, z: 0.0}, orientation: {z: 0.957, w: 0.289}}}}" \
  --feedback
```

通过日志：

```text
Goal accepted
Goal finished with status: SUCCEEDED
```

本次实际已通过。

### 关停

启动仿真的终端按：

```text
Ctrl-C
```

确认没有残留：

```bash
source /home/asong/vlfm/scripts/source_limo_ros_env.sh
ros2 node list
```

期望为空，或只剩无关临时节点。

## 7 · G2/G3 验收（本机现状与最小可跑路径）

> G0/G1 验收的是「观测链 + Nav2 唯一司机」。G2/G3 要把真正的 `LimoITMPolicy.decide_goal`（value map / 物体检测 / 抢占 / 到达验证 / reject / 记忆）插进这个闭环。本机有两个硬约束，先认清，再跑。

### 7.1 两个硬约束（本机实测）

**约束 A：联调进程必须同时有 `torch` + `rclpy`。（已解决 ✅）**
`run_nav2_mission(node, policy)` 把 ROS 节点和策略放在**同一个进程**。`vlfm_ros312` 有 `rclpy`，原来**没有 torch**，`LimoITMPolicy` 父类 `ITMPolicyV2` 顶部 `import torch`，缺 torch 会静默退化成 `pass` 空壳。**已往 `vlfm_ros312` 装 CPU 栈解决**（重活仍在 VLM 服务进程，policy 进程只需能 import）：

```bash
# torch/torchvision 走 CPU 源，避免拖入多 GB CUDA 栈；numpy 保持 1.26.4 不被顶掉
pip install --index-url https://download.pytorch.org/whl/cpu torch torchvision
pip install hydra-core flask requests scikit-learn scikit-image
```

附带两处可移植性改动（Habitat 机器有 open3d/gym 时行为不变）：① `base_objectnav_policy` 把 `WrappedPointNavResNetPolicy` 改成**惰性 import**（Limo 路 `load_pointnav_policy=False`，不再为导入而装 gym/habitat）；② `object_point_cloud_map` 的 `open3d` 改成**可选**，缺时用 `sklearn` DBSCAN 兜底（open3d 无 aarch64/py3.12 轮子）。验证：`import LimoITMPolicy` 后 `_update_value_map` 是真方法（非 stub），构造 + `reset_episode` 通过，并用合成观测对 **live ITM(12182)+YOLO(12184)** 跑通一次 `decide_goal`（返回 `mode=explore value=...`）。

**约束 B：VLM 服务子集决定能跑到哪个 Gate。**
本机 `scripts/dgx/launch_vlm_servers_dgx.sh` 只起 **SigLIP2-ITM(12182) + YOLO26(12184)**；其余按需补。各服务对应能力：

| 服务 | 端口 | 缺了会怎样 |
| --- | --- | --- |
| ITM (SigLIP2) | 12182 | value map 不更新 → frontier 排序退化，**G2-lite 都跑不了** |
| YOLO26 (COCO 检测) | 12184 | COCO 目标检不到（非 COCO 走 GDINO 12181） |
| MobileSAM | 12183 | 检测框无法分割成物体点云 → `_get_target_object_location` 永远 None → **不会切 `navigate`**，到不了 G2 物体导航 |
| AttrVerifier(云) | 12186 | G3 到点验证退化成**本地启发式**（`heuristic_verify`，弱）；起了才是云端 VLM 真验证 `verify[...] match=` |
| BLIP2-VQA | 12185 | **可选**，仅 `use_vqa=1` 时用来二次确认检测；与到点属性验证无关 |

> 关键澄清：到点属性验证走的是 **AttrVerifier(12186) → 云 API**，缺它会 fallback 到本地 `heuristic_verify`（仍产出 verdict / reject / FOUND&VERIFIED，只是判得糙），**不是非要 BLIP2**。12186 需阿里云 MaaS 的 API key（`attribute_verifier.py`）。

### 7.2 一条命令看清现状

```bash
source /home/asong/vlfm/scripts/source_limo_ros_env.sh
python /home/asong/vlfm/scripts/limo_preflight.py
```

它探 6 个端口 + 查进程依赖（rclpy/torch/nav2/vlfm），最后打印 `Achievable now: ...`。**本机当前实测（torch 已装、SAM 未起）**：

```text
[UP] 12182 ITM   [UP] 12184 YOLO   [DOWN] 12183 SAM/12181 GDINO/12186 attr/12185 VQA
[OK] rclpy/cv_bridge/nav2/torch/vlfm
>>> Achievable now: G2-lite: value-map-guided exploration only (no object navigate)
    - MobileSAM :12183 (required to localize a detected object -> object navigate)
```

含义：**G2-lite（value-map 引导探索）现在就能跑**；要 G2 物体导航 + G3 验证，只差起 MobileSAM(12183)。

### 7.3 各 Gate 的开跑前提（约束 A 已解决）

| 目标 | 还需要 | 怎么补 |
| --- | --- | --- |
| **G2-lite**：value map 引导探索（朝目标方向更积极，但不切物体导航） | ——（torch 已装） | 现在就能跑；起仿真 + `limo_mission.py` |
| **G2**：检测到目标 → 抢占 → 开过去 | + MobileSAM(12183) | 见下 7.3.1（CPU SAM，跑在 `vlfm_ros312`，不碰 yolo_trt/siglip2 工作 env） |
| **G3**：到点二次确认 / 属性不符 reject / FOUND & VERIFIED | + AttrVerifier(12186) **可选** | 12186 起了是云端 VLM 真验证；不起则本地 `heuristic_verify` 兜底，流程一样跑通 |

#### 7.3.1 起 MobileSAM（一次性安装 + 启动器）

`mobile_sam` 是 git 装的外部代码，**你自己在终端跑这条一次性安装**（agent 不替你装外部代码）：

```bash
source /home/asong/vlfm/scripts/source_limo_ros_env.sh
pip install --no-deps "git+https://github.com/ChaoningZhang/MobileSAM.git" timm
```

装好后用仓库启动器（CPU 跑，端口 12183；慢但够验收）：

```bash
bash /home/asong/vlfm/scripts/limo_sam_server.sh start    # stop / status 同理
python /home/asong/vlfm/scripts/limo_preflight.py         # 应变成 G3 (heuristic verify)
```

> 想要 SAM 跑 GPU（更快）：在有 cu128 torch 的 env 里装 `mobile_sam` 再 `python -m vlfm.vlm.sam --port 12183`；但别往 `yolo_trt`/`siglip2_itm` 里塞，免得弄坏在跑的 YOLO/ITM。

### 7.4 跑 mission（前提满足后）

```bash
source /home/asong/vlfm/scripts/source_limo_ros_env.sh
# 先开 mentor 仿真（VLFM 是唯一发目标者）：
ros2 launch limo_pro_sim demo.launch.py mission:=false rviz:=true gui:=true
# 另开终端确认 VLM 服务在跑：
bash /home/asong/vlfm/scripts/dgx/launch_vlm_servers_dgx.sh status
# 跑闭环（找 chair；带属性查询则第二个参数给自然语言）：
python /home/asong/vlfm/scripts/limo_mission.py chair
# 例：python /home/asong/vlfm/scripts/limo_mission.py chair "the black office chair"
```

`limo_mission.py` 启动会先做一次 preflight 自检：缺 torch/ITM/YOLO 直接拒绝启动并告诉你缺什么；SAM/VQA 缺只警告并降级（探索-only / 验证 fail-open），不会跑到一半莫名崩。它用 mentor 的 topic/frame（同 §3）和 `reality.yaml` 的策略参数构造 `LimoITMPolicy`，再交给 `run_nav2_mission` 的抢占主循环（notes_zh/24 §2）。

### 7.5 Gazebo / RViz 重点看什么（对照 24 §5 验收表）

开 `rviz:=true gui:=true`，按阶段盯：

| 阶段 | RViz/Gazebo 看点 | 终端日志判定 |
| --- | --- | --- |
| 初始 spin | 机器人原地转一圈，`/map` 边界向外铺开 | `[spin-warmup]` 不再刷错；value map 已有内容 |
| 探索 | `/vlfm/goal` 箭头落在**已知-未知边界**的 frontier 上；Nav2 把车开过去、会避障；到点不对墙 | `[limo] step=.. mode=explore xy=[..] value=0.xx`，value 随推进上升 |
| 唯一司机 | —— | `ros2 topic info /cmd_vel -v` 仍只有 `velocity_smoother` 发布 |
| 发现（需 SAM） | goal 箭头从 frontier **跳到物体本体**，车转向物体 | `mode=explore` → `mode=navigate`；抢占日志 |
| 确认（需 VQA） | 车停在物体前看清 | `[attr] verify[...] match=True/False` |
| 拒绝（需 VQA） | 车不再回那个物体，转去别处 | `[attr] reject 'chair' around [..]` + `attr mismatch -> reject` |
| 成功 | 车停下 | `FOUND & VERIFIED`，`$VLFM_OBJECT_MEMORY_PATH` 文件被写 |
| 复用 | 重启同图再跑 | 首个 goal `mode=navigate-memory` |

**人工必看（G0 起就欠的一项）**：frontier 圆点 / `/vlfm/goal` 箭头是否**肉眼贴在 `/map` 已知-未知边界**，误差约一个 cell（`0.05m`）。自动日志已证明 frontier 生成，但「贴边界」要 RViz 看一眼才算数。

**硬性指标（同 24 §5，无则不算过）**：整局无 `IndexError: edge of map`；无两 frontier 间反复横跳；无到点对墙、value 不涨、探索停滞；`/cmd_vel` 全程仅 Nav2 发布。

### 7.6 排障补充（叠加 24 §6）

| 症状 | 根因 | 解法 |
| --- | --- | --- |
| `limo_mission.py` 启动即 abort，说缺 torch | venv 没 torch | 给 `vlfm_ros312` 装 CPU torch（约束 A） |
| 永远 `mode=explore`，检测到也不切 navigate | SAM(12183) 没起，物体定位为 None | 起 MobileSAM；或确认 `limo_preflight` 里 SAM=UP |
| 到点 `verify` 永远 match=True / 不打印 | 无 VQA/attr 服务，fail-open | 起 12185/12186；调 `VLFM_ATTR_FAIL_OPEN=0` 仅用于看 reject 路径 |
| value map 一直空 / frontier 排序乱 | ITM(12182) 没起 | `launch_vlm_servers_dgx.sh status` 确认 ITM=UP |
| 抢占太频、节点卡 | `decide_goal` 含 VLM 推理 | 降 `VLFM_DECIDE_HZ`（默认 1.0）、调大 `VLFM_PREEMPT_DIST` |

## 8 · 防多套仿真叠开（单套仿真原则 + `limo_sim.sh`）

> 这是本机最容易、也最隐蔽的坑。一次实测里同机叠了 **3 套** `demo.launch.py`（自己手开 1 套 + 早先遗留孤儿 1 套 + 另一个 agent 跑长任务 1 套），现象是相机掉到 ~2Hz、Nav2 object-goal 240s 不 SUCCEEDED、底盘乱转。**这些症状极易被误判成策略 bug，实际是仿真叠开。**

### 8.1 为什么叠开会坏事

同机多套仿真有两条独立的破坏路径：

1. **抢硬件**：每套都有 gz sim server + GUI + 相机传感器渲染，挤同一块 GPU/CPU → 相机帧率暴跌（15Hz → 2Hz）。
2. **抢 ROS 话题（更危险）**：默认全在 `ROS_DOMAIN_ID=0`。多套 RTAB-Map 抢同一个 `/map`、多条 Nav2 链抢同一个 `/cmd_vel`、多份 TF 互相污染 → 你的实验读到的是别套机器人的 map/odom/cmd_vel，结果不可复现，Nav2 到达永远收不了尾。

附加坑：**孤儿进程**。`ros2 launch` 的父进程被 `Ctrl-C`/`kill` 后，gz sim 常被 `systemd --user` 收养继续跑，下次 `pgrep` 还在，于是越叠越多。

### 8.2 工具：`scripts/limo_sim.sh`

| 子命令 | 作用 |
| --- | --- |
| `bash scripts/limo_sim.sh status` | 统计在跑的仿真组件（gz/rtabmap/nav2/bridge/rviz）+ 探 6 个 VLM 端口 |
| `bash scripts/limo_sim.sh down` | 拆掉**所有**仿真栈（含孤儿），但**绝不动** `vlfm.vlm.*` 服务（按字样匹配 + 双重跳过 `vlfm.vlm`） |
| `GUI=false bash scripts/limo_sim.sh up` | **带保护启动**：已有 `gz sim server` 就拒绝启动并提示先 `down`，否则起一套干净仿真 |

它从设计上避开了两个手敲会犯的错：
- **不自匹配**：仿真识别字样写在脚本文件里，进程命令行只是 `bash limo_sim.sh down`，所以 `pkill/pgrep` 不会把自己这条命令也匹配上打断（手敲 `pkill -f "gz sim"` 会，因为命令行里就含 `gz sim`）。
- **不误杀 VLM**：`down` 对任何含 `vlfm.vlm` 的进程显式跳过，ITM/YOLO/SAM 服务不受影响（重起一套仿真不必重起这些贵服务）。

### 8.3 三条人肉规矩（和工具配合）

1. **起仿真前先 `status`**：`gz sim server` 必须是 `0`，非 0 先 `down`。
2. **一个 session 只有一个“仿真主”**：当另一个 agent（如 Codex）正在跑长任务用着仿真时，你只**观察**（看它的窗口 / 只读命令 / RViz 旁观），**不要**自己 `up`，否则会串扰它的实验。
3. **必须并行才分域**：万一你和别人都要各跑各的，启动前 `export ROS_DOMAIN_ID=42`（你）/ 让对方留在 `0`，ROS 话题就不串了——但 GPU 仍共享会都慢，**能错开时间就错开，别真并行**。

### 8.4 标准收尾流程

一轮实验结束、要交还干净环境时：

```bash
bash scripts/limo_sim.sh down      # 拆仿真，保 VLM
bash scripts/limo_sim.sh status    # 确认 gz/rtabmap/nav2 全 0，VLM 该 UP 的还 UP
```

期望：

```text
gz sim server : 0
rtabmap       : 0
nav2 ctrl     : 0
:12182 ITM          UP
:12184 YOLO         UP
```
