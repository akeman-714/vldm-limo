# 20 · M1 地图适配：`/map` → VLFM frontier

> 目标：让 `ObstacleMap` 改吃 LiDAR 的 `/map`（方案 B），输出 frontier 给决策层。**只换障碍/已探索的来源，不碰 frontier 检测算法。**
> 依赖：[19 接口契约](19_Nav2耦合_接口契约.md) A/D 节。

## 1. 规划

原版 `ObstacleMap.update_map`（`vlfm/mapping/obstacle_map.py:114`）从单目 depth 投影建障碍图——视野窄、探索慢。方案 B 用 RTAB-Map 360° LiDAR 融合的 `/map`（`OccupancyGrid`，-1未知/0空闲/100占据）直接刷新三张内部图：
- `self._map`（bool 障碍）
- `self.explored_area`（bool 已探索 = 已知格）
- `self._navigable_map`（bool 可走 = 障碍膨胀机器人半径后取反）

然后**原样复用** `_get_frontiers()`（`obstacle_map.py:220`，内部 `detect_frontier_waypoints`）算 frontier。frontier 检测一行不改。

## 2. 接口（锁定）

见 [19 §D](19_Nav2耦合_接口契约.md)：`update_from_occupancy_grid(grid, resolution, origin_xy, occ_thresh=65)`。

## 3. 详细步骤

**3.1 新增方法**（`vlfm/mapping/obstacle_map.py`，与 `update_map` 并列）：

```python
def update_from_occupancy_grid(self, grid, resolution, origin_xy, occ_thresh=65):
    H, W = grid.shape
    jj, ii = np.nonzero(grid != -1)                 # 已知格（行 jj、列 ii）
    if len(ii) == 0:
        return
    # OccupancyGrid 行主序，data[0] 在 origin，+x 沿列、+y 沿行（假设 origin.yaw==0）
    wx = origin_xy[0] + (ii.astype(np.float64) + 0.5) * resolution
    wy = origin_xy[1] + (jj.astype(np.float64) + 0.5) * resolution
    px = self._xy_to_px(np.stack([wx, wy], axis=1))  # 复用基类 → VLFM 像素 (col,row)
    col, row = px[:, 0], px[:, 1]

    # 防越界（map 帧落在 VLFM 网格外的格丢弃，不崩）
    ok = (row >= 0) & (row < self.size) & (col >= 0) & (col < self.size)
    col, row, vals = col[ok], row[ok], grid[jj[ok], ii[ok]]

    self.explored_area[row, col] = True              # 已知=已探索
    occ = vals >= occ_thresh
    self._map[row[occ], col[occ]] = True             # 占据=障碍

    self._recompute_navigable()                      # 机器人半径膨胀，原样复用
    self.explored_area[self._navigable_map == 0] = False
    self._frontiers_px = self._get_frontiers()
    self.frontiers = self._px_to_xy(self._frontiers_px) if len(self._frontiers_px) else np.array([])
```

**3.2 单调 vs 全量**：上面是**单调累积**（只置位、不清除），对静态场景稳，且能抗 `/map` 抖动。若 RTAB-Map 会「擦除」误占据，需要镜像它 → 见 §5 问题 4。

**3.3 接线（在 M2 节点里调用，本文件只定义方法）**：每收到 `/map` 回调，把 `msg.data` reshape 成 `(H,W)` 后调用本方法。

## 4. 注意点

- **origin.yaw≠0 的情形**：RTAB-Map 默认 0，公式可省旋转。若非 0，需对 `(wx,wy)` 先按 `origin.yaw` 旋转再加平移；先用 `ros2 topic echo /map --field info.origin` 确认。
- **ppm 必须等于 1/resolution**：否则 `_xy_to_px` 把 LiDAR 格映射到错误 VLFM 格。契约定 ppm=20 ⇔ res=0.05。
- **`occ_thresh`**：RTAB-Map 占据常是 0/100 两极，65 足够；若用了概率灰度（1–99）按需调。
- **不再需要 PointNav 相关**：`_navigable_map` 仍算（frontier 检测要它），但不再喂 A*/PointNav。
- **explored_area 收口**：`self.explored_area[self._navigable_map==0]=False` 保证「已探索」不含障碍格，与原版语义一致，frontier 才落在「可走的已知↔未知」边界。

## 5. 可能问题 + 解法

| # | 现象 | 根因 | 解法 |
|---|---|---|---|
| 1 | frontier 全空 | `/map` 全是 -1 或全已知；或 ppm≠1/res 导致映射错位 | 先 echo `/map` 看是否有 0/100；核对 `resolution==0.05` 且 `pixels_per_meter==20` |
| 2 | frontier 错位、与 RViz 对不上 | origin.yaw≠0 未处理，或 origin_xy 取错字段 | 用 `info.origin.position.{x,y}`；处理 yaw（§4） |
| 3 | `IndexError: edge of map` 或大量格被丢 | map 帧范围 > VLFM 50m 网格 | 调大 `size`（构造 `ObstacleMap(size=1600,...)`）；或给 episodic 原点加偏置 |
| 4 | 旧障碍残留/不消失 | 单调累积，不镜像 RTAB-Map 的擦除 | 改为每次 `self._map.fill(0); self.explored_area.fill(0)` 后全量重建（用全部已知格）；代价是每帧重算，可接受 |
| 5 | frontier 抖动剧烈 | `/map` 每帧小变动 | M5 抢占阈值兜底（frontier 移动 > 阈值才换目标）；或对 `_map` 做轻微闭运算 |

## 6. 验收标准

**6.1 离线单测（不依赖机器人，必须先过）** — 建议 `test/test_occupancy_adapter.py`：
1. 构造一张合成 `OccupancyGrid`：中间一块 0（free）、外围 -1（unknown）、一条 100（wall）。`res=0.05, origin=(-25,-25)`。
2. 调 `update_from_occupancy_grid` 后断言：
   - `obs_map._map` 在墙对应像素为 True；free 区为 False。
   - `obs_map.explored_area` 在 free 区为 True、unknown 区为 False。
   - `len(obs_map.frontiers) > 0`，且每个 frontier 的 (x,y) 反查回去落在「free↔unknown」边界 ±1 格内。
3. 已知格世界坐标 `(wx,wy)` 经 `_xy_to_px`→`_px_to_xy` 往返误差 < 0.05 m。

**6.2 在线验收（G0 的一半）**：
- 真实 `/map` 进来后，发布 `/vlfm/frontiers` Marker，在 RViz 叠加 `/map`：**每个 frontier 圆点都贴在已知白区与未知灰区的交界线上**，没有悬空在墙里或空旷已知区中央。
- `ros2 topic echo /vlfm/frontiers` 的坐标与 RViz「Measure」量取一致（±1 格 = ±0.05 m）。

> 过了 6.1+6.2，M1 完成。
