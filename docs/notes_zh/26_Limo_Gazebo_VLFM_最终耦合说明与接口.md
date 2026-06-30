# 26 · Limo Gazebo + VLFM 最终耦合说明与接口

> 状态：已在线验收通过。
> 最后验收时间：2026-06-30。
> 结论：当前仓库已经完成 “VLFM 大脑 + Limo Gazebo/ROS2/Nav2 司机” 的 G2/G3 耦合。G2 object navigate 和 G3 百炼云端属性验证都已闭环。

这篇是 Limo Gazebo 接入后的**最终事实来源**。
16/18/19/20/21/22/23/24 是执行计划和分模块设计，25 是 DGX/mentor 环境接入手册；如果它们和本文冲突，以本文和当前代码为准。

---

## 1. 一句话结论

当前系统不是把原 Habitat VLFM 原封不动搬到 ROS，也不是重写 VLFM。它做的是：

```text
mentor Limo Gazebo/ROS2 提供 RGB-D、TF、/map、Nav2
VLFM 继续负责语义感知、value map、object map、frontier 选择
Limo 适配层把 ROS observation 转成 VLFM observation cache
LimoITMPolicy 把 VLFM 决策从“离散动作”改成“map 坐标 goal”
Nav2 负责真正规划和控制 /cmd_vel
到达后再由 VLFM 调 AttributeVerifier 做云端二次确认
```

已验证的最终链路：

```text
RGB-D + /map + TF
  -> LimoVLFMNode.build_observation()
  -> LimoITMPolicy.decide_goal()
  -> ITM value map + YOLO + MobileSAM + object point cloud map
  -> mode=navigate, goal=(0.496, -5.334) @ map
  -> Nav2 NavigateToPose
  -> controller_server: Reached the goal
  -> AttributeVerifier(12186, qwen-vl-plus): verify[bailian] match=True
  -> FOUND & VERIFIED
```

---

## 2. 当前能力边界

| 项 | 当前状态 | 说明 |
| --- | --- | --- |
| G0 observation | 已通 | RGB、Depth、CameraInfo、TF、/map 能组成 VLFM observation cache |
| G1 Nav2 goal | 已通 | `BasicNavigator.goToPose()` 能驱动 Limo 到 map-frame goal |
| G2 object navigate | 已通 | YOLO 检出 chair，SAM 分割，object map 定位，policy 切 `mode=navigate` |
| G3 cloud verify | 已通 | 到达后调用 `AttrVerifier :12186`，返回 `source=bailian` |
| `/cmd_vel` 独占 | 已确认 | `/cmd_vel` 只有 `velocity_smoother` 一个发布者，Nav2 是唯一司机 |
| open3d | 未装成功 | `aarch64 + Python 3.12` 无匹配 wheel；当前用 sklearn DBSCAN fallback |
| BLIP2-VQA 12185 | 非主链 | legacy optional detection confirm，不是 G3 主验证 |

关于 open3d：当前 `ObjectPointCloudMap` 优先 import open3d，失败后使用 `sklearn.cluster.DBSCAN`。两者在这里承担同一个任务：对反投影出来的目标点云做 DBSCAN，保留最大非噪声簇。在线 G2/G3 验收是在 sklearn fallback 下通过的。

---

## 3. 架构分层

### 3.1 运行时分层

```text
Gazebo / limo_pro_sim
  - 小房子 world
  - Limo 机器人、RGB-D、LiDAR、IMU、里程计
  - ros_gz_bridge 连接 Gazebo 和 ROS2

ROS2 SLAM / Navigation
  - RTAB-Map 发布 /map 和 map->odom
  - Nav2 接 NavigateToPose action
  - velocity_smoother 发布最终 /cmd_vel

VLFM ROS adapter
  - vlfm/ros/limo_vlfm_node.py
  - 订阅 RGB-D、CameraInfo、/map、TF
  - 输出 VLFM observation cache
  - 发布 /vlfm/frontiers 和 /vlfm/goal 供 RViz 看

VLFM policy adapter
  - vlfm/policy/limo_policy.py
  - 继承 ITMPolicyV2
  - 复用 value map、object map、属性验证、记忆、reject
  - 新增 decide_goal()/on_goal_reached()

VLM services
  - ITM :12182
  - MobileSAM :12183
  - YOLO :12184
  - AttrVerifier :12186
```

### 3.2 为什么这样分

核心原则是让 VLFM 和 Nav2 各做自己擅长的事：

| 部分 | 谁负责 | 原因 |
| --- | --- | --- |
| “哪里可能有目标” | VLFM value map / object map | 这是 VLFM 的语义能力 |
| “下一步去哪个全局点” | `LimoITMPolicy.decide_goal()` | 复用 VLFM 的 explore/navigate 优先级 |
| “怎么避障开过去” | Nav2 | ROS 里已有成熟 planner/controller/costmap |
| “是不是目标实例” | AttrVerifier | 到达后二次确认，减少误检 |
| “谁写 /cmd_vel” | Nav2 only | 避免 Mission Node、VLFM、Nav2 多方抢控制 |

因此 Limo 版本里退役了 Habitat PointNav 的动作输出。VLFM 不再直接输出 turn/forward/stop，而是输出 map 坐标 goal。

---

## 4. 核心代码地图

| 文件 | 角色 | 关键接口 |
| --- | --- | --- |
| `scripts/source_limo_ros_env.sh` | 一键 source ROS Jazzy、mentor workspace、VLFM venv | shell 环境入口 |
| `scripts/limo_sim.sh` | 管理仿真栈 | `status` / `up` / `down` |
| `scripts/limo_sam_server.sh` | 单独起 MobileSAM | `start` / `stop` / `status` |
| `scripts/limo_preflight.py` | 跑前检查 | 输出当前能达到 G2-lite / G3 heuristic / G3 cloud |
| `scripts/limo_mission.py` | G2/G3 在线验收入口 | 构造 node + policy，调用 `run_nav2_mission()` |
| `vlfm/ros/limo_vlfm_node.py` | ROS2 observation adapter + Nav2 loop | `build_observation()` / `to_pose()` / `run_nav2_mission()` |
| `vlfm/policy/limo_policy.py` | VLFM policy goal adapter | `reset_episode()` / `decide_goal()` / `on_goal_reached()` |
| `vlfm/mapping/object_point_cloud_map.py` | 目标点云地图 | open3d 优先，sklearn fallback |
| `vlfm/policy/base_objectnav_policy.py` | 原 VLFM 对象导航共享逻辑 | detection、SAM、attribute verify、reject、memory |
| `vlfm/vlm/attribute_verifier.py` | G3 云端验证服务 | `/verify`，默认 `qwen-vl-plus` |

---

## 5. 关键接口

### 5.1 Observation cache

`LimoVLFMNode.build_observation()` 返回一个 dict，形状与原 VLFM/Habitat policy 期望对齐：

```python
{
    "frontier_sensor": np.ndarray,   # (N, 2), map frame xy
    "robot_xy": np.ndarray,          # (2,), map frame meters
    "robot_heading": float,          # yaw in map
    "nav_depth": np.ndarray,         # normalized depth, float32
    "object_map_rgbd": [
        (rgb, depth, tf_cam2map, min_depth, max_depth, fx, fy)
    ],
    "value_map_rgbd": [
        (rgb, depth, tf_cam2map, min_depth, max_depth, fov)
    ],
}
```

这里最重要的坐标约定：

- 所有 `xy` 都是 ROS `map` 帧下的米。
- 不再做 Habitat 的 y 取反。
- Nav2 goal 直接使用 `xy`。
- `yaw_hint` 只用于 goal orientation，让机器人朝向目标点。

### 5.2 Goal decision

`LimoITMPolicy.decide_goal(obs_cache)` 返回：

```python
{
    "mode": "explore" | "navigate" | "navigate-memory" | "done",
    "xy": np.ndarray | None,
    "yaw_hint": float,
    "value": float,
    "stop_radius": float,
}
```

决策优先级：

```text
1. 更新 object map：YOLO/GDINO -> SAM -> depth 反投影 -> DBSCAN
2. 更新 value map：ITM cosine -> 视锥投影
3. 如果 object map 里有目标：mode=navigate
4. 否则如果有记忆目标：mode=navigate-memory
5. 否则按 value map 选择 best frontier：mode=explore
6. 如果没有 frontier：mode=done
```

### 5.3 Object approach pose

Limo 不直接导航到椅子点云中心，而是退到靠近物体的 approach pose：

```text
object_xy = 目标点云最近点
robot_xy  = 当前机器人位置
approach = VLFM_OBJECT_APPROACH_DIST 或 pointnav_stop_radius
nav_goal = object_xy - unit(object_xy - robot_xy) * approach
```

这样 Nav2 目标落在物体前方可达地面上，而不是椅子实体内部。

### 5.4 到达与验证

`run_nav2_mission()` 收到 Nav2 result：

```text
TaskResult.SUCCEEDED
  -> policy.on_goal_reached(obs, current_goal)
  -> 如果是 explore：继续探索
  -> 如果是 navigate：调用 _attribute_match()
     -> AttrVerifier :12186 /verify
     -> match=True  => FOUND & VERIFIED
     -> match=False => reject 当前区域，继续 explore
```

`AttrVerifier` 的主链是：

```text
crop from last detected target bbox/mask
  -> POST http://localhost:12186/verify
  -> DashScope/Bailian qwen-vl-plus
  -> {"match": true/false, "reason": "...", "source": "bailian"}
```

---

## 6. 一次成功 G3 的关键日志

这次验收的关键日志如下：

```text
[attr] query='a chair' noun='chair' predicate='a chair' parse=env
[basic_navigator]: Navigating to goal: 0.49613151701502584 -5.333715640044384...
[attr] verify[bailian] match=True: The visible object is a chair, identifiable by its structure with a backrest and legs.
[limo_vlfm]: [arrive] navigate accepted=True verify[bailian] match=True...
[limo_vlfm]: FOUND & VERIFIED
```

同时确认：

```text
/cmd_vel Publisher count: 1
Node name: velocity_smoother

MobileSAM log:
POST /mobile_sam 200

AttrVerifier log:
POST /verify 200
```

### 6.1 这个视频证明什么，不证明什么

当前 `outputs/limo_g3_validation_*` 这组视频主要证明的是：

```text
ROS2 observation -> VLFM policy -> YOLO/SAM object map -> Nav2 到达 -> 云端 verify
```

它证明了 Limo/Gazebo/Nav2 耦合和 G2/G3 object-goal 闭环，但它不是一个充分展示 “ITM value map 语义探索” 的视频。原因是 chair 在初始旋转阶段已经被 YOLO/SAM 看到，VLFM 的决策优先级会立刻切到：

```text
检测到目标实例 -> mode=navigate -> Nav2 去 object approach pose
```

这时 ITM/value map 仍然在更新，日志里也能看到 `BLIP2ITMClient.cosine(...)`，但它不是最终 goal 的决定者。最终 goal 来自 object map，而不是 frontier/value map 排序。

因此证据应分层理解：

| 证据 | 证明内容 | 是否等于完整 VLFM 语义探索展示 |
| --- | --- | --- |
| G3 object-goal 视频 | YOLO/SAM 目标实例定位、Nav2 到达、AttrVerifier 云端确认 | 否 |
| `mode=explore` + `/vlfm/frontiers` + `/vlfm/goal` 视频 | value map/frontier 分支能驱动探索 | 部分 |
| 同一地图不同文本 prompt 选择不同 frontier | ITM 语义确实影响探索方向 | 是，更强 |
| 隐藏目标：先 explore，看到目标后切 navigate | VLFM 完整 explore→detect→navigate 行为 | 是，最完整 |

后续如果要做“VLFM 算法语义性”展示，应补录一条专门视频：提高/关闭 COCO 检测阈值或换成初始不可见目标，让 policy 先保持 `mode=explore`，在 RViz/合成视频里显示 `/vlfm/frontiers`、`/vlfm/goal` 和 ITM/value map 选择；再恢复检测或走到可见区域后切 `mode=navigate`。

### 6.2 bed 语义探索补录结果

2026-06-30 追加了一条 `bed` 目标录制：

```text
outputs/limo_bed_validation_20260630_173035_evidence.html
outputs/limo_bed_validation_20260630_173035_h264.mp4
outputs/limo_bed_validation_20260630_173035_contact_sheet.jpg
outputs/limo_bed_validation_20260630_173035.log
```

这条视频和日志证明：

```text
target='bed'
-> ITM/value-map 持续收到 bed 文本 prompt
-> policy 保持 mode=explore
-> Nav2 依次执行多个 frontier goal
-> 到达 frontier 后继续切换到下一个 frontier
```

关键日志形态：

```text
BLIP2ITMClient.cosine: ..., Seems like there is a bed ahead.
[limo] step=... mode=explore xy=...
[arrive] explore accepted=True frontier reached
```

本轮同时修复了一个探索闭环 bug：过去 `on_goal_unreachable()` 会屏蔽不可达 frontier，但 `on_goal_reached()` 不会屏蔽已到达 frontier，导致到达后可能反复选择同一个探索点。现在到达和不可达的 explore goal 都会进入同一个 `_block_frontier()` 路径。

这条 bed 视频不证明完整 `explore -> detect -> navigate -> verify` 闭环。6 分 24 秒内系统没有触发 bed 的 YOLO/SAM 实例命中，也没有出现 `FOUND & VERIFIED`。因此当前证据分层是：

| 目标 | 已证明 | 未证明 |
| --- | --- | --- |
| chair | G2/G3 object navigate + 云端属性验证闭环 | 充分语义探索过程 |
| bed | ITM/value-map/frontier 探索分支可驱动 Nav2 连续探索 | bed 实例发现、切 object navigate、云端验证 |

---

## 7. 怎么运行

### 7.1 查看当前状态

```bash
bash scripts/limo_sim.sh status
```

期望 G3 云端验收前至少：

```text
仿真组件全 0
ITM  :12182 UP
SAM  :12183 UP
YOLO :12184 UP
AttrVerifier :12186 UP
```

### 7.2 启动 SAM

```bash
bash scripts/limo_sam_server.sh start
```

### 7.3 启动 AttrVerifier

不要把 key 写进脚本或文档。只在当前终端设置：

```bash
source scripts/source_limo_ros_env.sh
export DASHSCOPE_API_KEY="你的 key"
export BAILIAN_BASE_URL="https://dashscope.aliyuncs.com/compatible-mode/v1"
export BAILIAN_VERIFY_MODEL="qwen-vl-plus"
export ATTR_VERIFIER_PORT=12186
python -um vlfm.vlm.attribute_verifier --port 12186
```

如果要后台跑，确保日志里有：

```text
[attr] verifier model=qwen-vl-plus api_key=present
```

### 7.4 跑 preflight

```bash
source scripts/source_limo_ros_env.sh
python scripts/limo_preflight.py
```

云端 G3 就绪时应显示：

```text
>>> Achievable now: G3: full object navigate + cloud attribute verify
```

### 7.5 启动单套仿真

```bash
GUI=false RVIZ=false bash scripts/limo_sim.sh up
```

需要人看画面时：

```bash
GUI=true RVIZ=true bash scripts/limo_sim.sh up
```

### 7.6 跑 G3 mission

另开终端：

```bash
source scripts/source_limo_ros_env.sh
VLFM_TARGET_OBJECT=chair \
VLFM_ATTR_PREDICATE="a chair" \
VLFM_ATTR_NOUN=chair \
VLFM_ATTR_VERIFY_TIMEOUT=20 \
python -u scripts/limo_mission.py chair
```

结束后清理仿真：

```bash
bash scripts/limo_sim.sh down
```

---

## 8. 为什么不是大重构

当前代码的耦合边界比较健康：

1. ROS 依赖集中在 `vlfm/ros/limo_vlfm_node.py`。
2. Nav2 依赖集中在 `run_nav2_mission()`。
3. VLFM 原始 policy 基本不需要知道 ROS。
4. Limo 适配 policy 只新增 goal API，不改原 Habitat `act()` 主路径。
5. VLM 服务仍按原 Flask client/server 边界走 HTTP。

因此现在不建议把所有东西合成一个大 ROS node，也不建议把 Nav2 action 写进 `BaseObjectNavPolicy`。那样会让 VLFM 的 Habitat、Reality、Limo 三条路径互相污染。

值得保留的轻量改进方向：

| 方向 | 是否建议现在做 | 原因 |
| --- | --- | --- |
| 给 `GoalDecision` 换成 dataclass | 暂不急 | dict 便于 ROS/日志/测试；字段少且稳定 |
| 把 `run_nav2_mission()` 拆成类 | 暂不急 | 当前函数短、状态少；过早拆类会增加阅读成本 |
| 把 Limo 参数 YAML 化 | 后续可做 | 现在 env + constants 足够；长期实验可整理成 config |
| 把 AttrVerifier 启停做脚本 | 建议后续做 | 现在已有服务代码，缺一个类似 SAM 的安全启动脚本 |
| open3d 源码编译 | 只有硬性要求时做 | sklearn fallback 已通过验收，源码编译成本高 |

本轮已经做的轻量修正：

- `ObjectPointCloudMap.clouds` 改为实例属性，避免多个 map 共享点云状态。
- `scripts/limo_mission.py` preflight 补探 `ATTR :12186`，避免误报云端 verifier down。
- `scripts/limo_sim.sh` 的 Gazebo server 计数改为只数真实 `gz sim` server。

---

## 9. 旧文档怎么看

建议阅读顺序：

1. 当前最终状态：先读本文。
2. 要理解 mentor 仿真：读 17。
3. 要复现 DGX 环境：读 25。
4. 要理解方案怎么推导出来：读 18-24。
5. 要理解原 VLFM 算法：读 00-05 或 `数据链全景README.md`。

旧文档定位：

| 文档 | 当前定位 |
| --- | --- |
| 16 | 历史执行计划。里面“VLFM 还没有 Limo ROS2 适配器”等描述已经过时 |
| 17 | 仍可作为 mentor Gazebo/ROS2 仿真入门手册 |
| 18-24 | 方案设计和模块计划，整体方向仍有效，但验收状态以本文为准 |
| 25 | DGX 同机环境接入和工具固化记录，仍有效 |
| 本文 26 | 最终耦合状态、接口、运行方式、验收结果 |

---

## 10. 最终判断

当前属于：

```text
VLFM 已经耦合到 Limo Gazebo/ROS2/Nav2。
G2 object navigate 已通过。
G3 百炼/DashScope 云端属性验证已通过。
open3d 未成功安装，但 sklearn DBSCAN fallback 已在线验收通过。
```

换句话说：如果验收口径允许 sklearn 替代 open3d，这套系统已经是完整可运行的 Limo Gazebo VLFM 耦合版。
如果验收口径强制要求 `import open3d` 成功，则剩余任务不是 VLFM 耦合，而是环境工程：源码编译 Open3D 或换到支持 wheel 的平台。
