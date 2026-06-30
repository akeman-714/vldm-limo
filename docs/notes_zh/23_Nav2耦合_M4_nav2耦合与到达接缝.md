# 23 · M4 Nav2 耦合 + 到达接缝（verify / remember / reject）

> 目标：用 `nav2_simple_commander.BasicNavigator` 把 `decide_goal` 的目标发给 Nav2 开车；并把「到达事件」从 PointNav 内部上提到 **Nav2 action result**，让记忆/二次确认/reject 三个改进继续工作。
> 依赖：[19](19_Nav2耦合_接口契约.md)、[22](22_Nav2耦合_M3_decide_goal决策包装.md)。代码锚点：`base_objectnav_policy.py` 的 `_verify_on_arrival:361`、`_maybe_remember_object:343`、`_reject_and_continue:447`、`object_point_cloud_map.py reject_region`。

## 1. 规划

两件事：
- **驾驶闭环**：`decide_goal → PoseStamped@map → BasicNavigator.goToPose → Nav2 → /cmd_vel`。
- **到达接缝**：原代码里「写记忆 / 二次确认 / reject」都挂在 `_called_stop`，而 `_called_stop` 是 `_pointnav`/`_navigate_global` 内部「到 stop_radius」时设的。方案 B 不走它们，于是改由 **Nav2 `getResult()==SUCCEEDED`** 触发 `on_goal_reached`，在其中复用这三段逻辑。

> 关键洞察：这三个改进是 **goal 层 / 感知层**，只是触发时机从「PointNav 内部」搬到「Nav2 结果回调」。内核函数复用，不重写。

## 2. 接口

- 输出：`navigate_to_pose`（经 `BasicNavigator`）。
- 新增策略方法：`on_goal_reached(obs_cache, goal) -> dict`、`on_goal_unreachable(goal)`（[19 §D]）。
- 需要的小重构：从 `_verify_on_arrival` 抽出**纯判定** `_attribute_match(obs, robot_xy) -> Optional[bool]`（True=符合/False=拒绝/None=跳过或 fail-open）。

## 3. 详细步骤

**3.1 小重构（安全，保留旧行为）**——把 `_verify_on_arrival` 中「取 crop → `self._verifier.verify` → `heuristic_verify` 兜底 → 得 match bool」这段抽成：
```python
def _attribute_match(self, observations, robot_xy):
    if not self._should_verify_on_arrival(): return None
    if self._verify_calls >= int(os.environ.get("VLFM_ATTR_MAX_VERIFY_CALLS","5")): return None
    crop = self._get_arrival_crop()                       # base:439
    if crop is None:
        return None if os.environ.get("VLFM_ATTR_FAIL_OPEN","1")=="1" else False
    self._verify_calls += 1
    verdict = self._verifier.verify(crop, self._predicate, timeout=...) or \
              heuristic_verify(crop, self._predicate).to_json()
    self._last_verify_result = f"verify[{verdict.get('source')}] match={verdict.get('match')}"
    return bool(verdict.get("match"))
```
原 `_verify_on_arrival` 改为调用它（habitat 路径不变）。**这样 goal-mode 和 habitat-mode 共用同一判定核。**

**3.2 `on_goal_reached`（到达接缝核心）**：
```python
def on_goal_reached(self, obs_cache, goal):
    self._observations_cache = obs_cache
    robot_xy = obs_cache["robot_xy"]
    if goal["mode"] == "explore":
        return {"accepted": True, "reason": "frontier reached", "next": "explore"}  # 选下一个

    # 物体/记忆目标：到点二次确认
    verdict = self._attribute_match(obs_cache, robot_xy)           # True/False/None
    if verdict is False:
        r = float(os.environ.get("VLFM_ATTR_REJECT_RADIUS","0.6"))
        self._object_map.reject_region(self._target_object, np.asarray(goal["xy"]), radius=r)
        self._last_verify_result = "attr mismatch -> reject"
        return {"accepted": False, "reason": self._last_verify_result, "next": "explore"}

    # True 或 None(无谓词/fail-open) → 接受
    self._attribute_verified = True
    self._called_stop = True
    loc = self._get_target_object_location(robot_xy)               # base:321，更准的物体坐标
    self._maybe_remember_object(loc if loc is not None else np.asarray(goal["xy"]))  # base:343 写记忆
    return {"accepted": True, "reason": self._last_verify_result or "accepted", "next": "done"}
```

**3.3 `on_goal_unreachable`（Nav2 FAILED）**：
```python
def on_goal_unreachable(self, goal):
    if goal["mode"] in ("navigate","navigate-memory"):
        self._object_map.reject_region(self._target_object, np.asarray(goal["xy"]),
                                       radius=float(os.environ.get("VLFM_ATTR_REJECT_RADIUS","0.6")))
    else:  # frontier 不可达 → 记入屏蔽集，decide_goal 过滤，防止反复试同一个
        self._blocked_frontiers.append(np.asarray(goal["xy"]))
```
> 在 `decide_goal` 选 frontier 前，过滤掉离 `_blocked_frontiers` 任一点 < `block_radius`（如 0.5m）的 frontier；`reset_episode` 清空该集合。

**3.4 节点主循环（G1 保守版，先发→等结果→再选）**：
```python
nav = BasicNavigator(); nav.waitUntilNav2Active()
policy.reset_episode(target_object, query)
nav.spin(spin_dist=math.pi*2)          # 初始 360° 建 value map（替 _initialize），期间 spin_once 泵 obs

while rclpy.ok():
    obs = node.build_observation()
    g = policy.decide_goal(obs)
    if g["mode"] == "done":
        node.announce("explore exhausted"); break
    nav.goToPose(node.to_pose(g))      # PoseStamped@map（[19 §B]）
    while not nav.isTaskComplete():
        rclpy.spin_once(node, timeout_sec=0.1)   # G2 在此插抢占（见 M5）
    res = nav.getResult()
    if res == TaskResult.SUCCEEDED:
        v = policy.on_goal_reached(node.build_observation(), g)
        node.get_logger().info(f"[arrive] {g['mode']} accepted={v['accepted']} {v['reason']}")
        if v["accepted"] and v["next"] == "done":
            node.announce("FOUND & VERIFIED"); break
    elif res in (TaskResult.FAILED, TaskResult.CANCELED):
        policy.on_goal_unreachable(g)
        node.get_logger().warn(f"[unreachable] {g['mode']} {np.round(g['xy'],2)}")
```

**3.5 `to_pose` / 朝向**：
```python
def to_pose(self, g):
    p = PoseStamped(); p.header.frame_id = "map"; p.header.stamp = self.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(g["xy"][0]), float(g["xy"][1])
    qz, qw = math.sin(g["yaw_hint"]/2), math.cos(g["yaw_hint"]/2)
    p.pose.orientation.z, p.pose.orientation.w = qz, qw
    return p
```

## 4. 注意点

- **`_called_stop` 含义迁移**：方案 B 里它不再由 `_pointnav` 设，只在 `on_goal_reached` 接受时手动置位（给 `_maybe_remember_object` 用）。别再期待它从驾驶里冒出来。
- **不要调 `_reject_and_continue`**：它尾部 `return self._explore(observations)` 会触发 `_pointnav`（驾驶副作用）。`on_goal_reached` 里只调 `reject_region`，不调它。
- **记忆坐标跨运行**：map 帧持久（RTAB-Map 重定位回同图），`VLFM_OBJECT_MEMORY_PATH` 存的 (x,y) 下次能直接 recall。
- **二次确认的 crop 时效**：`_get_arrival_crop` 有 `VLFM_ATTR_CROP_MAX_AGE`（步）限制。方案 B 的「步」= decide_goal 调用次数；保证抵达前最近一次 decide_goal 在物体可见处跑过，否则 crop 过期 → fail-open 接受或跳过（看 `VLFM_ATTR_FAIL_OPEN`）。
- **Nav2 到点容差**：物体目标用较小 `xy_goal_tolerance`（到得够近才能看清验证）；frontier 目标可放松（M5）。

## 5. 可能问题 + 解法

| # | 现象 | 根因 | 解法 |
|---|---|---|---|
| 1 | `waitUntilNav2Active` 卡死 | Nav2 没全起 / lifecycle 没 active | 确认 Nav2 bringup、map_server/amcl(或 RTAB-Map 提供 map→odom)、`ros2 lifecycle get` |
| 2 | `goToPose` 立刻 FAILED | goal 在障碍里/未知区，或 frame≠map | 核对 `frame_id=="map"`；frontier 应在可走已知区边界（M1）；放松 costmap inflation |
| 3 | 到点不触发验证 | crop 过期 / 无谓词 | 抵达前在物体可见处多跑几次 decide_goal；确认 `VLFM_ATTR_PREDICATE` 设了 |
| 4 | reject 后又回到同一物体 | reject 半径太小 / 物体点云大 | 调大 `VLFM_ATTR_REJECT_RADIUS`；检查 `reject_region` 是否作用到正确 target 名 |
| 5 | 反复奔向同一不可达 frontier | 没记屏蔽集 | 实现 §3.3 `_blocked_frontiers` 过滤 |
| 6 | 记忆没写 | `_called_stop` 未置位 / 路径未设 | `on_goal_reached` 接受分支已置 `_called_stop=True`；确认 `VLFM_OBJECT_MEMORY_PATH` |

## 6. 验收标准

**6.1 离线单测**（mock `BasicNavigator` 与 verifier）：
- `on_goal_reached` 对 `mode="explore"` → `{accepted:True,next:"explore"}`，不调 verifier。
- mock verifier 返回 match=False → `on_goal_reached(navigate)` 调了 `reject_region`（断言 object map 该区域被拉黑，`_get_target_object_location` 后续返回 None），`next=="explore"`。
- mock verifier 返回 match=True → `next=="done"`，且（设了 `VLFM_OBJECT_MEMORY_PATH`）记忆文件写入了该 (x,y)。
- `on_goal_unreachable(explore-goal)` → 该 frontier 进 `_blocked_frontiers`，下次 `decide_goal` 不再选它。

**6.2 在线（G1）**：
- 把 G0 打印过的某个 frontier 真正发给 Nav2：机器人开过去，`getResult()==SUCCEEDED`，中途 Nav2 避障不撞（人为放障碍验证）。
- 日志出现 `[arrive] explore accepted=True`，随后自动选下一个 frontier 继续。

**6.3 在线（G2）**：
- 把目标物体放进视野：decide_goal 切 `navigate` 并（M5）抢占；开到物体附近 `SUCCEEDED` → 触发二次确认。
- 故意放一个「像但不对」的物体（属性不符）：验证 `match=False` → 日志 `attr mismatch -> reject`，物体被拉黑，机器人转回探索，**不再奔向该物体**。
- 放正确物体：`match=True` → 日志 `FOUND & VERIFIED`，（设了记忆路径）记忆文件出现该物体 (x,y)；重启再跑同图，能从记忆直接 recall（mode=navigate-memory）。

> 过 6.1+6.2 ⇒ G1；过 6.3 ⇒ G2。
