# 22 · M3 决策包装：`decide_goal`（只出 goal，不出动作）

> 目标：在 VLFM 策略上加一层「输出 (x,y) 目标、不产生任何底层动作」的包装，绕过 PointNav/A\*。决策层的全部改进（记忆/二次确认/reject）原样继承。
> 依赖：[19](19_Nav2耦合_接口契约.md)、[21](21_Nav2耦合_M2_limo观测插线层.md)。代码锚点：`vlfm/policy/itm_policy.py`、`vlfm/policy/base_objectnav_policy.py`。

## 1. 规划

`act()`（`base_objectnav_policy.py:170`）的分支顺序是：初始化 → 多目标 → 调试 → **检测到物体则去物体** → 记忆 → **否则探索选 frontier**。每个分支末尾都调 `_pointnav`/`_navigate_to` 出**动作**。我们做一个**同构但返回 (x,y)** 的 `decide_goal`：把「选点→`_pointnav`」改成「选点→`return goal`」。感知/建图/value map/记忆 recall 全部复用。

新建 `LimoITMPolicy(ITMPolicyV2)`（建议 `vlfm/policy/limo_policy.py`），它继承 `ITMPolicyV2` 的全部大脑，只新增 `reset_episode`/`decide_goal`（+ M4 的 `on_goal_reached`），并提供一个**空的 `_cache_observations`**（因为我们不走 habitat 的 act 流程）。

## 2. 接口（锁定，见 [19 §D]）

`reset_episode(target_object, query="")`、`decide_goal(obs_cache) -> dict`。

## 3. 详细步骤

**3.1 类与构造**：
```python
class LimoITMPolicy(ITMPolicyV2):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)          # 构造同 VLFMConfig：text_prompt/pointnav_policy_path/...
    def _cache_observations(self, observations): pass   # 不用 habitat 流程
```
> 构造参数沿用 `VLFMConfig`（`base_objectnav_policy.py:951`）。`compute_frontiers` 仍可为 True（会建一个内部 ObstacleMap），但**本方案的 frontier 来自 obs_cache["frontier_sensor"]（M1/M2 注入），不依赖内部那张**——见 §4 注意点 2。

**3.2 `reset_episode`（替代 `_pre_step` 首步设置，每局一次）**：
```python
def reset_episode(self, target_object, query=""):
    self._reset()                                  # 复用基类：清状态、加载记忆图
    if query:
        os.environ["VLFM_NAV_QUERY"] = query       # 走属性解析（二次确认用）
    self._target_object = target_object
    self._configure_attribute_query(target_object) # base:265，解析 predicate
    self._maybe_recall_object_memory()             # base:327，recall 记忆
    self._num_steps = 0
    self._did_reset = True
```

**3.3 `decide_goal`（核心）**：
```python
def decide_goal(self, obs_cache):
    self._observations_cache = obs_cache           # M2 已组装好
    self._policy_info = {}

    # ① 感知 + 建 object/value map（复用 act 里的两步）
    for (rgb, depth, tf, mind, maxd, fx, fy) in obs_cache["object_map_rgbd"]:
        self._update_object_map(rgb, depth, tf, mind, maxd, fx, fy)   # base:836
    self._update_value_map()                                          # itm:271（SigLIP2 打分）

    robot_xy = obs_cache["robot_xy"]

    # ② 检测到目标物体 → 去物体（map 帧）
    obj = self._get_target_object_location(robot_xy)                  # base:321
    if obj is not None:
        xy = np.asarray(obj)[:2]
        return {"mode": "navigate", "xy": xy, "value": 1.0,
                "yaw_hint": _yaw_towards(robot_xy, xy),
                "stop_radius": self._pointnav_stop_radius}

    # ③ 记忆命中（未检测到但记得位置）→ 去记忆点
    if self._remembered_goal is not None:
        xy = np.asarray(self._remembered_goal)[:2]
        return {"mode": "navigate-memory", "xy": xy, "value": 0.5,
                "yaw_hint": _yaw_towards(robot_xy, xy),
                "stop_radius": self._pointnav_stop_radius}

    # ④ 否则探索 → value map 排序选 best frontier（map 帧）
    frontiers = obs_cache["frontier_sensor"]
    if frontiers is None or len(frontiers) == 0:
        return {"mode": "done", "xy": None, "value": 0.0, "yaw_hint": 0.0, "stop_radius": 0.0}
    best_frontier, best_value = self._get_best_frontier(obs_cache, frontiers)   # itm:156
    self._num_steps += 1
    return {"mode": "explore", "xy": np.asarray(best_frontier)[:2], "value": float(best_value),
            "yaw_hint": _yaw_towards(robot_xy, best_frontier),   # 朝 frontier，M5 可改“朝未探索”
            "stop_radius": self._pointnav_stop_radius}
```
`_yaw_towards(a,b)=atan2(b[1]-a[1], b[0]-a[0])`。

**3.4 reinspect（可选，保留）**：frontier 耗尽（③返回 done 前）可插入 `_maybe_reinspect` 的「选 value 峰值点」逻辑（`itm_policy.py:92`），把它的 `_pointnav` 改成 `return goal`，作为 done 之前的兜底目标。先不接，G3 再加。

## 4. 注意点

1. **绕过 PointNav/A\***：`decide_goal` 全程不调 `_pointnav`/`_navigate_to`/`_navigate_global`。`VLFM_GLOBAL_NAV` 保持关。
2. **frontier 来源单一**：用 obs_cache 里 M1 注入的 frontier（map 帧）。若同时让内部 ObstacleMap 也建图，会有两套 frontier——**以 obs_cache 为准**，避免歧义。最干净是构造时 `compute_frontiers=False` 并始终外部注入（参考 `habitat_policies.py:216` 的外部 frontier 分支）。
3. **`_acyclic_enforcer` 保留**：`_get_best_frontier` 自带防横跳（`itm_policy.py:210`）继续生效。
4. **value map 与 frontier 同帧**：二者都在 map 帧、ppm=20；`_get_best_frontier`→`_value_map.sort_waypoints` 按 (x,y) 查分，天然对齐（[19 §A]）。
5. **初始扫描**：Habitat 的 `_initialize`（原地转）在此不调；初始 360° 建图改由 M5 用 `BasicNavigator.spin()` 完成。

## 5. 可能问题 + 解法

| # | 现象 | 根因 | 解法 |
|---|---|---|---|
| 1 | 构造即报错/显存爆 | `__init__` 加载 PointNav 权重（`base:90 WrappedPointNavResNetPolicy`） | 保证 `pointnav_policy_path` 指向存在的权重（加载但不用）；或改造成 lazy（仅 `_pointnav` 首次用时加载）——方案 B 永不触发 |
| 2 | `decide_goal` 一直 explore，永不 navigate | 目标物体没进 object map：检测阈值/类别/深度问题 | 查 `_get_object_detections`：COCO vs 非 COCO 阈值（`coco_threshold/non_coco_threshold`）；确认目标名在检测器词表；depth 有效 |
| 3 | 选的 frontier 总在身后/绕圈 | 朝向 yaw_hint 错 或 acyclic 退化 | 核对 `_yaw_towards`；看日志 `Suppressed cyclic frontier`；M5 抢占阈值 |
| 4 | ITM/检测服务超时拖慢 | VLM 服务未起或慢 | 起 [18 §5] 服务；客户端调用加超时与重试 |
| 5 | `best_value` 恒定低 | text_prompt/谓词没传对 | 核对 `text_prompt`（含 `target_object`）与 `reset_episode(query=...)` |

## 6. 验收标准

**6.1 离线单测**（喂 6.1 的 rosbag 或合成 obs_cache）：
- `reset_episode("chair")` 后 `decide_goal(obs)` 返回 dict 含全部键、`mode∈{explore,navigate,navigate-memory,done}`、`xy` 为 map 帧 (2,) 或 None。
- 构造一帧「frontier 非空、无物体」→ 必返回 `explore` 且 `xy` 等于某个 frontier；`value` 与 `_value_map.sort_waypoints` 首位一致。
- 构造一帧「object map 注入了目标点云」→ `_get_target_object_location` 非空 → 必返回 `navigate`，`xy` ≈ 物体坐标。
- 连续两帧同一 frontier 集 → 不无故横跳（acyclic 生效）。

**6.2 在线（G0 收尾，仍 dry-run 不发 Nav2）**：
- 节点每周期打印 `mode/xy/value/yaw_hint`，并发 `/vlfm/goal`（PoseStamped@map）。
- RViz 叠加 `/map`+`/vlfm/frontiers`+`/vlfm/goal`：**goal 箭头落在被选中的 frontier 上、朝向指向未探索方向**；把目标物体放进视野后，goal 应从 frontier **切换到物体位置**（mode→navigate）。
- 打印的 `xy` 与 RViz 量取一致（±0.05 m）。

> 过了 6.1+6.2 ⇒ **G0 帧验证全部通过**，可进 M4 接 Nav2。
