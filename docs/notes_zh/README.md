# VLFM 代码走读笔记（中文）

> 目的：把你"从论文里读到的直觉"对照本仓库的真实实现，逐条核对、补全、修正，并给出每一步中间数据的具体形态（矩阵大小 / 图像 / JSON 等），方便系统学习。
>
> 阅读顺序建议：
>
> - **想看 Limo Gazebo 最终耦合结果 / 接口 / 怎么验收**：直接读 **26_Limo_Gazebo_VLFM_最终耦合说明与接口.md**。
> - **想深入读代码**：00 → 01 → 02 → 03 → 04 → 05 → 06。
> - **只想做汇报/评审**：直接读 **07_VLFM_全景介绍.md**（多对比表、少代码，含问题定义 / 视频生成 / Habitat→ROS / 与 SLAM 等对比 / 应用展望 5 大块）。

| 文件 | 主题 | 适合 |
| --- | --- | --- |
| [00_总览与直觉核对.md](./00_总览与直觉核对.md) | 你给出的直觉 vs 代码事实；逐条勾正 | 入门 |
| [01_BLIP2_ValueMap_推理与融合.md](./01_BLIP2_ValueMap_推理与融合.md) | BLIP2-ITM 余弦 → 锥形投影 → 置信度通道 → v_new/c_new 融合公式 | 代码细节 |
| [02_YOLO_Grounding_SAM_对象定位.md](./02_YOLO_Grounding_SAM_对象定位.md) | 三个检测/分割模型如何协同得到目标点云与"最近点" | 代码细节 |
| [03_ObstacleMap_与_Frontier.md](./03_ObstacleMap_与_Frontier.md) | 障碍地图、已探索区、可航行区、frontier 的生成 | 代码细节 |
| [04_决策流程_act_explore_navigate.md](./04_决策流程_act_explore_navigate.md) | 一次 `act()` 里 BLIP2 / YOLO / Grounding 的真实先后与覆盖关系 | 代码细节 |
| [05_数据形态速查表.md](./05_数据形态速查表.md) | 每个张量 / 数组 / JSON 的形状、单位与典型数值 | 查表 |
| [06_仿真环境_Habitat与接口.md](./06_仿真环境_Habitat与接口.md) | Habitat 仿真本体、配置/传感器/动作接口、RGBD 是怎么从 sim 流到策略再到 VLM RPC 的 | 代码细节 |
| [07_VLFM_全景介绍.md](./07_VLFM_全景介绍.md) | 问题定义 / 视频生成 / Habitat→ROS 移植 / 与 SLAM 等导航对比 / 应用端展望 | **汇报评审** |
| [08_Reality真机数据格式端到端变化链.md](./08_Reality真机数据格式端到端变化链.md) | 真机(Reality)RGBD / 位姿 / 动作端到端数据格式变化链 | 代码细节 |
| [09_YOLO26_TensorRT替换方案.md](./09_YOLO26_TensorRT替换方案.md) | 用 YOLO26+TensorRT 替换 YOLOv7-e6e:思路 / 坑 / 验收 | 工程方案 |
| [10_YOLO26n升级到l方案.md](./10_YOLO26n升级到l方案.md) | yolo26n → yolo26l 升级:改动面 / n→l 专属坑 / 验收闸门 | 工程方案 |
| [11_SigLIP2_ITM_可回滚替换方案.md](./11_SigLIP2_ITM_可回滚替换方案.md) | SigLIP2-ITM 可回滚替换 BLIP2-ITM:规划 / 坑 / 解决方案 / 验收 | 工程方案 |
| [12_数据流分工交付接口.md](./12_数据流分工交付接口.md) | 真机(Spot)版:把数据流当交付接口,每个数据一句话"长啥样" + 谁产谁用;含动作连续头分叉 | **分工/交付** |
| [13_硬件端侧部署评估.md](./13_硬件端侧部署评估.md) | 基于实测显存补充 Nano / Jetson / Qualcomm / Hailo / CPU 端侧部署判断 | **硬件评估** |
| [14_属性关系导航升级方案.md](./14_属性关系导航升级方案.md) | 属性/关系 instance-nav:找名词→到达→云侧二次判断;VLFM 热循环不动,验证壳拦 STOP + 拉黑实例 | **工程方案** |

### VLFM 高层 + Nav2 驾驶（方案 B）执行计划 — 18~24

| 文件 | 主题 | 适合 |
| --- | --- | --- |
| [18_Nav2耦合_总览与里程碑.md](./18_Nav2耦合_总览与里程碑.md) | 架构/核心决定/模块地图/Gate 里程碑/开什么退什么/依赖服务 | **入口/验收** |
| [19_Nav2耦合_接口契约.md](./19_Nav2耦合_接口契约.md) | 坐标律 + ROS 接口表 + observation cache 结构 + 新增 Python API 签名（单一事实来源） | **接口契约** |
| [20_Nav2耦合_M1_occupancy_grid适配.md](./20_Nav2耦合_M1_occupancy_grid适配.md) | `/map`→`ObstacleMap.update_from_occupancy_grid`→frontier;步骤/坑/单测验收 | 工程方案 |
| [21_Nav2耦合_M2_limo观测插线层.md](./21_Nav2耦合_M2_limo观测插线层.md) | RGBD+TF+/map→obs_cache 的 Limo ROS 插线层（HabitatMixin 的第三个兄弟） | 工程方案 |
| [22_Nav2耦合_M3_decide_goal决策包装.md](./22_Nav2耦合_M3_decide_goal决策包装.md) | 只出 goal 不出动作的 `decide_goal`/`reset_episode`;绕过 PointNav/A* | 工程方案 |
| [23_Nav2耦合_M4_nav2耦合与到达接缝.md](./23_Nav2耦合_M4_nav2耦合与到达接缝.md) | BasicNavigator 闭环 + 到达接缝 `on_goal_reached`（verify/remember/reject 复用） | 工程方案 |
| [24_Nav2耦合_M5_抢占朝向_联调验收_排障.md](./24_Nav2耦合_M5_抢占朝向_联调验收_排障.md) | 抢占节奏 + 到点朝向 + 端到端验收 + 全局排障矩阵 | **联调/排障** |
| [25_DGX同机mentor_Gazebo_ROS2环境接入手册.md](./25_DGX同机mentor_Gazebo_ROS2环境接入手册.md) | 同一台 DGX 上复用 mentor 的 Limo Gazebo/ROS2/Nav2 环境:权限 ACL/source/venv/G0-G1 验收 | **环境接入** |
| [26_Limo_Gazebo_VLFM_最终耦合说明与接口.md](./26_Limo_Gazebo_VLFM_最终耦合说明与接口.md) | 当前最终事实来源:G2/G3 在线验收、代码地图、接口、运行命令、旧文档状态 | **最终入口** |

如果你只读一篇：

- 想知道 Limo Gazebo + VLFM 现在到底做到哪了、怎么跑、接口是什么 → **26_Limo_Gazebo_VLFM_最终耦合说明与接口.md**。
- 做技术汇报、对外讲故事 → **07_VLFM_全景介绍.md**。
- 想从直觉入手核对代码事实 → **00_总览与直觉核对.md**。
- 想搞清楚"环境到底怎么塞 RGBD 给代码"→ **06_仿真环境_Habitat与接口.md** 的 6.4 节。

### 当前文档状态提示

- 16、18-24 是接入过程中的计划和模块设计，方向仍有参考价值，但部分“待完成/未实现”的表述已经过时。
- 25 是 DGX 同机环境接入手册，仍可用于复现环境和排查仿真叠开。
- 26 是最终耦合后的事实来源；若旧文档与 26 冲突，以 26 为准。
