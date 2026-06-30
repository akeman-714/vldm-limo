# 16 · Limo Gazebo 接入 VLFM 执行计划

## 0. 这份文档的定位

这份文档不是代码方案的最终实现，而是给“是否批准开始改仓库”用的执行计划。

当前状态:

- 已查看 mentor 的远端工作区: `10.89.4.4:/home/xiaowei.du/projects/robot/ROS2/limo_nav_ws`
- 远端主要 ROS2 包: `limo_pro_sim`
- 该包已经包含 Gazebo Harmonic、RGB-D 相机、LiDAR、RTAB-Map、Nav2、RViz、Mission Node
- VLFM 这边还没有 Limo ROS2 适配器

执行原则:

1. 第一轮不重写 VLFM 算法。
2. 第一轮不直接把 mentor 仿真大规模合进 VLFM 仓库。
3. 先验证 ROS2 topic / TF / depth / cmd_vel 链路，再接 VLFM policy。
4. 每一步都有明确验收，验收不过不进入下一步。

---

## 1. 总体结论

Limo Gazebo 接入 VLFM 的核心工作是补一个 ROS2 适配层，而不是重写导航算法。

mentor 的仿真已经提供了 VLFM 需要的最小数据源:

| 数据 | 远端实际 topic / TF | VLFM 用途 |
|---|---|---|
| RGB | `/camera/image` | value map / object detection |
| Depth | `/camera/depth_image` | nav depth / obstacle map / 目标反投影 |
| CameraInfo | `/camera/camera_info` | fx / fy / 图像尺寸 |
| TF | `/tf` | base 和 camera pose |
| Odometry | `/odom` | 位姿 fallback |
| Control | `/cmd_vel` | 控制 Limo 底盘 |

远端 TF 链:

```text
map -> odom -> base_footprint -> base_link -> depth_camera_link -> depth_link
```

其中 `depth_link` 是 optical frame，适配器应优先使用它作为相机 frame。

---

## 2. 当前环境选择: 已确定暂时共用 mentor 工作区

用户已选择:

```text
暂时共用 mentor 工作区，不复制仿真，不把 Gazebo 包合进 VLFM 仓库。
```

当前执行环境:

做法:

```text
Gazebo / ROS2 仿真:
  /home/xiaowei.du/projects/robot/ROS2/limo_nav_ws

VLFM:
  /home/asong/vlfm
```

优点:

- 最快开始试验。
- 不复制大目录，不引入额外 build 问题。
- mentor 原本能跑的东西保持原样，便于对齐。

缺点:

- `/home/xiaowei.du` 普通用户读不了，需要 sudo 或 mentor 授权。
- 如果后续频繁调试 Gazebo/URDF/world，不方便。

适用阶段:

- 第 1 阶段 smoke test
- 第 2 阶段 observation 生成验证
- 第 3-4 阶段 policy dry-run 也可以继续使用

结论:

第一轮和今晚过夜执行就按这个方案做。

### 后续备选: 复制一份到 `/home/asong/limo_nav_ws`

这不是当前方案，只作为后续长期实验备选。

做法:

```bash
mkdir -p /home/asong/limo_nav_ws/src
sudo cp -a /home/xiaowei.du/projects/robot/ROS2/limo_nav_ws/src/limo_pro_sim /home/asong/limo_nav_ws/src/
sudo chown -R asong:asong /home/asong/limo_nav_ws
cd /home/asong/limo_nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

优点:

- 后续完全可控，不依赖 mentor 目录权限。
- 可以自由改 world、URDF、launch、Nav2 参数。
- 适合长期实验记录。

缺点:

- 复制后可能和 mentor 版本分叉。
- 重新 build 可能暴露 ROS2 依赖问题。
- 如果 mentor 继续改仿真，需要同步。

适用阶段:

- VLFM 能稳定读到传感器之后。
- 需要长期改 world / URDF / launch / Nav2 参数时。

结论:

今晚不搬。等 Gate 1-4 通过后，再决定是否复制一份作为长期实验环境。

### 后续备选: 把 Gazebo 仿真合进 VLFM 仓库

不建议现在做。

原因:

- VLFM 是 Python 包，mentor 仿真是 ROS2 workspace，构建体系不同。
- Gazebo models/world/URDF/Nav2 配置会把 VLFM 仓库变重。
- 早期验证阶段不需要统一仓库。

结论:

除非以后要做一个完整 demo release，否则不要把仿真包直接塞进 VLFM 仓库。当前计划不采用这个方案。

---

## 3. 推荐执行路径

推荐路径:

```text
阶段 0: 只读确认远端仿真结构
阶段 1: 在 VLFM 仓库新增 ROS2 smoke test
阶段 2: 新增 LimoROS2Robot，生成 VLFM observation
阶段 3: 跑 VLFM policy 单步推理，但暂不控制机器人
阶段 4: 接 /cmd_vel，低速闭环
阶段 5: 根据需要复制 mentor 仿真到 /home/asong/limo_nav_ws
阶段 6: 稳定实验和记录
```

每个阶段都可以单独验收。

---

## 4. 阶段 0: 远端仿真确认

已完成。

确认内容:

- ROS2 workspace: `limo_nav_ws`
- 主要包: `limo_pro_sim`
- 主启动入口: `ros2 launch limo_pro_sim demo.launch.py`
- 推荐 VLFM 试验入口: `ros2 launch limo_pro_sim demo.launch.py mission:=false`
- Gazebo bridge 配置: `config/bridge.yaml`
- RGB-D 相机:
  - `/camera/image`
  - `/camera/depth_image`
  - `/camera/camera_info`
  - frame: `depth_link`
  - size: `640x480`
  - update rate: `15 Hz`
  - horizontal FOV: `1.2 rad`
- 控制:
  - `/cmd_vel`
  - `geometry_msgs/msg/Twist`

剩余需要现场运行确认:

```bash
ros2 topic list
ros2 topic hz /camera/image
ros2 topic hz /camera/depth_image
ros2 topic echo /camera/camera_info --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo base_footprint depth_link
ros2 run tf2_ros tf2_echo map depth_link
```

验收:

- 能看到上述 topic。
- RGB-D 有稳定频率。
- TF 能查到 `base_footprint -> depth_link`。
- 如果启动了 SLAM，能查到 `map -> depth_link`。

---

## 5. 阶段 1: ROS2 smoke test

目标:

只验证 VLFM 仓库能和 mentor 的 ROS2/Gazebo 仿真通信。

计划新增文件:

```text
vlfm/reality/robots/limo_ros2_robot.py
vlfm/reality/run_limo_smoke.py
```

第一版 `run_limo_smoke.py` 做这些事:

1. 初始化 `rclpy`
2. 订阅:
   - `/camera/image`
   - `/camera/depth_image`
   - `/camera/camera_info`
   - `/odom`
3. 用 `tf2_ros.Buffer` 查询:
   - `base_footprint -> depth_link`
   - 可选 `map -> depth_link`
4. 打印:
   - RGB shape / dtype
   - depth shape / dtype / min / max
   - fx / fy
   - robot x / y / yaw
   - camera transform 4x4
5. 发布一次零速 `/cmd_vel`
6. 可选参数控制是否发布一个很小的前进速度，例如 0.05 m/s 持续 0.5 秒，然后立即停

建议先不开小速度，只发零速。

运行方式:

```bash
# 终端 1: 远端仿真
cd /home/xiaowei.du/projects/robot/ROS2/limo_nav_ws
source install/setup.bash
ros2 launch limo_pro_sim demo.launch.py mission:=false

# 终端 2: VLFM
cd /home/asong/vlfm
source /opt/ros/jazzy/setup.bash
python -m vlfm.reality.run_limo_smoke
```

可能问题:

| 问题 | 原因 | 解决 |
|---|---|---|
| import `rclpy` 失败 | 当前 Python 环境不是 ROS2 环境 | 先 `source /opt/ros/jazzy/setup.bash`，必要时不要用 conda |
| 收不到 image | Gazebo 未启动或 bridge 未启动 | 确认 `sim.launch.py` / `demo.launch.py` 正常 |
| depth dtype 不符合预期 | Gazebo depth 常是 float32 meters | adapter 内统一处理 |
| TF 查询失败 | SLAM/Nav2 未启动或 frame 名称不对 | 先查 `base_footprint -> depth_link`，不要一开始依赖 `map` |

阶段 1 验收:

- smoke script 10 秒内拿到 RGB。
- smoke script 10 秒内拿到 depth。
- 能打印 camera_info 的 `fx/fy`。
- 能打印机器人 `x, y, yaw`。
- 能查到 `base_footprint -> depth_link`。
- 发布零速 `/cmd_vel` 不报错。
- 不要求 VLFM policy 运行。

---

## 6. 阶段 2: LimoROS2Robot 适配器

目标:

让 Limo 仿真看起来像 VLFM 现有的 `BaseRobot`。

计划实现:

```python
class LimoROS2Robot(BaseRobot):
    @property
    def xy_yaw(self):
        ...

    def get_camera_images(self, camera_source):
        ...

    def get_camera_data(self, srcs):
        ...

    def command_base_velocity(self, ang_vel, lin_vel):
        ...

    def get_transform(self, frame="base_footprint"):
        ...
```

内部 topic 映射:

```text
VLFM logical RGB   -> /camera/image
VLFM logical depth -> /camera/depth_image
CameraInfo         -> /camera/camera_info
Base pose          -> TF or /odom
Camera pose        -> TF depth_link
Command            -> /cmd_vel
```

注意:

原 VLFM 的 Spot 版本有多个相机 ID，例如:

```text
FRONTLEFT_DEPTH
FRONTRIGHT_DEPTH
HAND_COLOR
```

Limo 只有一个固定前向 RGB-D 相机，所以不能直接照搬 `ObjectNavEnv` 的多 Spot 相机逻辑。需要单独写一个 Limo 版 env，或者在 adapter 里把同一个相机数据映射成 VLFM 需要的 logical camera。

推荐:

```text
vlfm/reality/limo_objectnav_env.py
```

不要硬改 `vlfm/reality/objectnav_env.py`，避免破坏 Spot 路径。

阶段 2 验收:

- `LimoROS2Robot.xy_yaw` 返回稳定的 `(np.array([x, y]), yaw)`。
- `get_camera_data(["front_rgbd"])` 返回:
  - image
  - depth
  - fx
  - fy
  - `tf_camera_to_global`
- depth 单位明确。
- transform 是 4x4 numpy array。
- 相机朝向经可视化检查没有明显反向。

---

## 7. 阶段 3: 生成 VLFM observation

目标:

不跑 policy，只把 ROS2/Gazebo 数据转成 VLFM `ObjectNav` observation。

目标字段:

```python
{
    "nav_depth": nav_depth,
    "robot_xy": robot_xy,
    "robot_heading": robot_heading,
    "objectgoal": "chair",
    "obstacle_map_depths": obstacle_map_depths,
    "value_map_rgbd": value_map_rgbd,
    "object_map_rgbd": object_map_rgbd,
}
```

Limo 单 RGB-D 相机下的简化:

| VLFM 字段 | 第一版做法 |
|---|---|
| `nav_depth` | 用 `/camera/depth_image` resize / normalize |
| `obstacle_map_depths` | 用同一个 depth + `depth_link` TF |
| `value_map_rgbd` | 用 RGB + depth + `depth_link` TF |
| `object_map_rgbd` | 用 RGB + depth + `depth_link` TF |

这里要承认一个差异:

Spot 真机版把“导航/障碍深度”和“语义腕部 RGB”分开；Limo 第一版只有一个前向 RGB-D，所以会退化成单相机前向探索。能不能达到原论文效果不是第一阶段目标，第一阶段目标是闭环跑起来。

阶段 3 验收:

- 保存一帧 observation 到文件，例如:

```text
data/limo_smoke/obs_000001.npz
```

- 文件里包含:
  - RGB
  - depth
  - camera_info
  - robot pose
  - camera transform
  - normalized nav_depth
- 可视化检查:
  - RGB 正常
  - depth 前方近物体数值更小
  - depth 没有全 0 / 全 NaN
  - robot heading 和 RViz/Gazebo 朝向一致

---

## 8. 阶段 4: VLFM policy 单步推理

目标:

只调用 `policy.get_action(obs, mask)`，先不发 `/cmd_vel`。

原因:

VLFM policy 会加载 VLM/PointNav/检测模型，风险比 ROS2 smoke test 高。先把输出动作打印出来，避免模型输出异常时机器人乱动。

运行前置:

```bash
./scripts/upstream/launch_vlm_servers.sh
```

或使用当前仓库已有的模型服务启动脚本。

测试内容:

- 固定目标: `"chair"`
- 输入一帧或连续几帧 observation
- 打印 policy mode:
  - initialize
  - explore
  - navigate
- 打印 action:
  - linear
  - angular

阶段 4 验收:

- policy 能完成一次 `get_action`。
- 输出 action 是有限数，不是 NaN。
- 不发布 `/cmd_vel`。
- 生成 policy 可视化图或至少保存调试信息。

---

## 9. 阶段 5: 接 `/cmd_vel` 低速闭环

目标:

把 VLFM 输出动作转换成 Limo 底盘速度。

建议映射:

```text
linear.x  = clip(action["linear"], -1, 1)  * max_linear_speed
angular.z = clip(action["angular"], -1, 1) * max_angular_speed
```

保守参数:

```text
max_linear_speed  = 0.10 m/s
max_angular_speed = 0.30 rad/s
command_duration  = 0.5 s
```

每步控制流程:

```text
取 observation
policy.get_action
发布 Twist 0.5s
发布零速
等待新 observation
```

注意:

不要让 Nav2 和 VLFM 同时控制 `/cmd_vel`。

推荐启动方式:

```bash
ros2 launch limo_pro_sim demo.launch.py mission:=false
```

如果发现 Nav2 仍在发速度，改用:

```bash
ros2 launch limo_pro_sim sim.launch.py
ros2 launch limo_pro_sim slam_3d.launch.py
```

不启动 `nav2.launch.py`。

阶段 5 验收:

- 机器人能低速移动。
- 每一步后能停住。
- `/cmd_vel` 没有多个来源抢占。
- RViz/Gazebo 中机器人运动方向与 action 符合预期。
- 连续运行 10-20 步不崩溃。

---

## 10. 阶段 6: 长时间试验

目标:

让接入链路跑一段时间，暴露同步、内存、模型、TF、仿真稳定性问题。

建议先跑短时:

```text
5 分钟
10 分钟
30 分钟
```

再考虑过夜。

过夜前必须满足:

- `/cmd_vel` 限速。
- 每步 action 后强制发布零速。
- 遇到异常强制发布零速。
- log 保存到独立目录。
- 不开启会让机器人无限循环的 mission node。

建议记录:

```text
logs/limo_vlfm/YYYYMMDD_HHMMSS/
  run.log
  obs_samples/
  policy_vis/
  cmd_vel.csv
  robot_pose.csv
  errors.txt
```

过夜运行验收:

- 进程没有异常退出，或者异常退出前安全停住。
- Gazebo 没有明显卡死。
- RGB-D topic 频率没有掉到 0。
- TF 查询失败率可接受。
- 机器人轨迹可解释。
- 保存了足够日志用于复盘。

---

## 11. 主要风险与解决方案

| 风险 | 表现 | 影响 | 解决方案 |
|---|---|---|---|
| 权限问题 | 不能读 mentor 工作区 | 无法启动或查看配置 | 短期 sudo，只读；长期复制到 `/home/asong/limo_nav_ws` |
| ROS2/Python 环境冲突 | `import rclpy` 或 torch 失败 | 节点跑不起来 | 先 smoke test 不加载 torch；必要时 ROS2 adapter 和 VLFM policy 分进程 |
| Nav2 抢控制 | `/cmd_vel` 同时有多个 publisher | 机器人动作不受控 | `mission:=false`；必要时不启动 Nav2 |
| TF 缺失 | 查不到 `map -> depth_link` | 无法建图 | 第一版用 `odom` frame；SLAM 启动后再用 `map` |
| 相机坐标轴错 | 障碍投影到后方/侧方 | map/value map 错 | 保存点云/BEV 可视化，必要时加固定旋转 |
| depth 单位错 | 障碍距离异常 | PointNav 和 obstacle map 错 | 明确 float32 meters / uint16 mm，统一转换 |
| 单相机视野窄 | 探索能力弱 | 效果不如 Spot | 第一版接受退化；后续加原地旋转扫描 |
| VLM server 不稳定 | policy 卡住或超时 | 无法闭环 | 先单步推理；加超时和失败 fallback |
| GPU/headless 渲染问题 | Gazebo 相机没图 | 没有 RGB-D | 使用 mentor 已配置的 NVIDIA EGL；优先 GUI 模式验证 |

---

## 12. 是否需要拆成几个子 MD

当前不需要立刻拆。

原因:

- 现在还在审批和第一轮验证阶段，一份总计划最清晰。
- 真正会变化的是执行记录，不是计划本身。

建议后续在进入实现后拆成 3 份子文档:

```text
docs/notes_zh/limo_vlfm/
  01_环境与启动记录.md
  02_ROS2适配器接口.md
  03_验收与实验日志.md
```

拆分时机:

- smoke test 通过后，拆 `01_环境与启动记录.md`
- 开始写 `LimoROS2Robot` 后，拆 `02_ROS2适配器接口.md`
- 开始闭环实验后，拆 `03_验收与实验日志.md`

现在保留这一份主文档即可。

---

## 13. 过夜阶梯式执行策略

用户已选择:

```text
方案 A: 暂时共用 mentor 工作区
```

也就是:

```text
Gazebo / ROS2 仿真:
  /home/xiaowei.du/projects/robot/ROS2/limo_nav_ws

VLFM:
  /home/asong/vlfm
```

今晚目标不应该只停在 smoke test，而是按“闸门式验收”往下推进。每个阶段通过才进入下一阶段；失败则停在当前阶段，保存日志和原因，明天汇报。

### 今晚建议批准范围

建议批准到 **阶段 4 dry-run**:

```text
允许:
  阶段 1: ROS2 smoke test
  阶段 2: LimoROS2Robot 适配器
  阶段 3: 生成并保存 VLFM observation
  阶段 4: VLFM policy 单步/短序列 dry-run，只打印动作，不控制机器人

不允许:
  不修改 mentor 工作区
  不复制 mentor 仿真
  不启动 Mission Node
  不让 VLFM 直接控制机器人移动
  不做长时间 /cmd_vel 非零速度闭环
```

这样明天能验收的不只是“topic 通了”，而是:

- ROS2/Gazebo 能启动。
- VLFM 仓库能读到 RGB-D / CameraInfo / TF / Odom。
- 能生成 VLFM observation。
- 能保存观测样本。
- 能跑一次或短序列 VLFM policy dry-run。
- 能看到 policy 输出动作是否合理。

### 为什么不默认过夜跑闭环移动

虽然这是 Gazebo 仿真，不是真车，但过夜让 VLFM 直接发非零 `/cmd_vel` 仍然不建议作为默认批准范围。

风险:

- Nav2 / velocity_smoother 可能也在发 `/cmd_vel`，导致控制源不唯一。
- policy 初期 observation 坐标系可能还没验准，机器人可能持续撞墙或卡住。
- VLM server 或 TF 卡住后，如果没有异常处理，可能留下旧速度。
- 过夜日志会变复杂，明天难以判断是 ROS2、Gazebo、policy 还是控制问题。

因此今晚推荐先 dry-run 到 policy 输出，不闭环移动。低速闭环作为明天看到 dry-run 结果后的下一次批准。

如果用户明确批准低速移动，则可以追加阶段 5 的短时闭环，但也应该限制为:

```text
最多 20 步
max_linear_speed  = 0.05 - 0.10 m/s
max_angular_speed = 0.20 - 0.30 rad/s
每步后强制发布零速
异常时强制发布零速
不允许无限循环
```

### 过夜执行闸门

今晚按下面顺序执行。

#### Gate 1: 启动 mentor 仿真

命令:

```bash
cd /home/xiaowei.du/projects/robot/ROS2/limo_nav_ws
source install/setup.bash
ros2 launch limo_pro_sim demo.launch.py mission:=false
```

验收:

- `/camera/image` 存在。
- `/camera/depth_image` 存在。
- `/camera/camera_info` 存在。
- `/tf` 存在。
- `/odom` 存在。
- `/cmd_vel` 存在。

失败处理:

- 不进入 Gate 2。
- 保存 launch 日志。
- 汇报 Gazebo/ROS2 启动失败原因。

#### Gate 2: ROS2 smoke test

验收:

- 10 秒内拿到 RGB。
- 10 秒内拿到 depth。
- 10 秒内拿到 CameraInfo。
- 10 秒内拿到 Odom。
- 能查到 `base_footprint -> depth_link`。
- 发布零速 `/cmd_vel` 成功。

失败处理:

- 不进入 Gate 3。
- 保存 topic、dtype、TF 错误。

#### Gate 3: 生成 observation 样本

验收:

- 保存至少 5 帧 observation 样本。
- RGB shape 正确。
- depth shape 正确。
- depth min/max 合理。
- fx/fy 合理。
- robot pose 连续。
- camera transform 是有限数。

建议输出:

```text
logs/limo_vlfm/YYYYMMDD_HHMMSS/
  obs_000001.npz
  obs_000002.npz
  obs_summary.json
```

失败处理:

- 不进入 Gate 4。
- 保存能拿到的原始 ROS2 数据。

#### Gate 4: VLFM policy dry-run

前提:

- VLM servers 能启动。
- PointNav 权重可加载。
- GPU/torch 环境可用。

验收:

- 至少一次 `policy.get_action(obs, mask)` 成功。
- 如果稳定，跑 10-30 步 dry-run。
- 每步只打印或保存 action，不发非零 `/cmd_vel`。
- action 里 `linear/angular` 是有限数。
- 保存 policy mode / action / timing。

建议输出:

```text
logs/limo_vlfm/YYYYMMDD_HHMMSS/
  policy_dryrun.csv
  policy_timing.csv
  errors.txt
```

失败处理:

- 不进入 Gate 5。
- 汇报是模型服务、torch、observation shape、还是 VLFM 内部接口问题。

#### Gate 5: 低速闭环移动，可选

默认不执行。

只有用户明确批准时执行。

验收:

- 最多 20 步。
- 每步后发布零速。
- 异常时发布零速。
- 机器人运动方向可解释。
- 不撞墙长时间卡死。

---

## 14. 明天验收交付物

如果按今晚建议范围执行，明天应该交付:

1. 一段简短结论:
   - 跑到了哪个 Gate。
   - 哪些通过。
   - 卡在哪里。
2. 日志目录路径:

```text
logs/limo_vlfm/YYYYMMDD_HHMMSS/
```

3. ROS2 topic/TF 检查结果:
   - topic 是否存在。
   - RGB-D 频率。
   - TF 是否可查。
4. observation 样本说明:
   - RGB/depth shape。
   - depth 单位/范围。
   - fx/fy。
   - robot pose。
5. policy dry-run 结果:
   - 是否成功加载。
   - 单步耗时。
   - action 输出范围。
   - 主要错误。
6. 下一步建议:
   - 是否可以进入低速闭环。
   - 是否需要复制仿真到 `/home/asong/limo_nav_ws`。
   - 是否需要拆子 MD。

---

## 15. 批准前需要你确认的点

开始动代码前，需要确认:

1. 第一轮是否采用“共用 mentor 工作区，不复制仿真”的方案。当前用户已选择方案 A。
2. 是否允许在 VLFM 仓库新增 ROS2 适配文件。
3. 是否批准今晚推进到阶段 4 dry-run，而不是只停在 smoke test。
4. 是否允许 smoke test 发布零速 `/cmd_vel`。
5. 是否允许可选地发布极小速度做底盘验证。

建议批准范围:

```text
批准今晚 Gate 1-4:
  新增 ROS2 smoke test
  新增 LimoROS2Robot 适配器
  生成 VLFM observation 样本
  跑 policy dry-run
  不复制 mentor 仿真
  不修改 mentor 工作区
  默认只发零速 cmd_vel
```

如果 Gate 1-4 验收通过，再单独批准 Gate 5 低速闭环移动。
