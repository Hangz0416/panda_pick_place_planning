# 完整代码走读

这份文档按一次三工件 RGB-D assisted episode 的实际调用顺序阅读代码。面试时先讲主线，
再根据追问进入 PyBullet、MoveIt 或 Gazebo 分支；`physics` 代码是 experimental / fail-closed，
不在当前主线中。

## 1. 一次任务如何进入状态机

```text
TaskSpec / ManipulationTask
  → task_generator_node
  → /manipulation_task
  → manipulation_node
  → PickPlaceRunner.run(task)
```

1. `core/tasks.py::generate_task()` 使用 `task_seed` 确定性生成工件、目标区和场景参数。
2. `TaskSpec` 保存 reset 所需的真值、后端、规划器、object variant、抓取模式和扰动 profile。
3. `task_generator_node.py` 将 `TaskSpec` 转成类型化 `ManipulationTask`，不再使用 JSON 字符串。
4. `manipulation_node.py::on_task()` 校验消息中的 backend 与当前节点配置一致，再调用唯一的
   `PickPlaceRunner`。
5. `ManipulationBackend` 统一 reset、运动、候选可行性、夹爪、对象选择、attach/release 和
   object state 语义；PyBullet 与 MoveIt 不复制 FSM。

### 数据所有权

| 数据 | 在线控制是否可用 | 用途 |
| --- | --- | --- |
| `TaskSpec.workpieces` 真值 | 仅 ground-truth 模式可作为控制目标 | Gazebo reset、配对任务和离线误差 |
| `SceneEstimate` | RGB-D 模式唯一在线位姿源 | 抓取候选、运动目标和 PlanningScene |
| Gazebo odometry/contact | 是 | 执行状态、释放稳定性、碰撞和最终误差 |
| `EpisodeResult` | 否 | 结果保存、统计和失败分析 |

RGB-D 模式不能在感知失败后退回任务真值继续执行。测试会用不同的估计值和真值验证
运动目标与 PlanningScene 确实来自 `SceneEstimate`。

## 2. RGB-D 场景估计

```text
RGB + depth + CameraInfo
  → ApproximateTimeSynchronizer
  → HSVDepthDetector
  → camera points
  → world points
  → SceneObjectEstimate[]
```

`SceneEstimatorNode` 同步彩色图和深度图。`EstimateScene` 服务收到请求时先记录当前帧代数，
只接受请求之后到达的新同步帧，因此不能返回上一 episode 的缓存结果。CameraInfo 或 TF
缺失分别返回 `camera_info_unavailable` 和 `tf_unavailable`；超时不会继续规划。

### 2.1 分割与深度过滤

`HSVDepthDetector` 对红色工件、绿色目标区和蓝色障碍物分别构造 mask，并执行形态学开闭操作。
三工件模式保留所有面积超过阈值的红色连通域，而不是只取最大区域。连通域数量不等于请求值
时返回 `object_count_mismatch`。

每个连通域只保留位于 `[near_clip, far_clip]` 的有限深度。像素深度反投影后，对三维点使用
中值和 MAD 过滤：

```text
d_i = ||p_i - median(P)||
inlier_i ⇔ d_i ≤ median(d) + 3.5 × MAD(d)
```

有效像素不足返回 `invalid_depth`，置信度低于阈值返回 `low_confidence`。

### 2.2 从像素到 world

CameraInfo 给出 `fx, fy, cx, cy`。深度像素 `(u, v, z)` 在相机光学坐标系中的反投影为：

```text
x_c = (u - cx) z / fx
y_c = (v - cy) z / fy
z_c = z
```

随后使用 TF2 的外参变换：

```text
p_world = R_world_camera · p_camera + t_world_camera
```

配置链路为：

```text
world → overhead_camera_link → overhead_camera_optical_frame
```

已知工件和障碍物尺寸用于把可见顶面换算为物体中心高度；该实现不是未知物体的通用三维重建。

### 2.3 PCA yaw

对矩形工件过滤后的 world-XY 点集计算：

```text
q_i = p_i - mean(P)
C = (1 / (N - 1)) Σ q_i q_iᵀ
C v_k = λ_k v_k
yaw = atan2(v_max,y, v_max,x)
```

主特征向量 `v_max` 给出长轴，但 `v` 与 `-v` 表示同一条无向轴，所以 yaw 归一化到
`[-π/2, π/2)`。方向可观测性使用特征值比：

```text
orientation_valid ⇔ λ_max / λ_min ≥ 1.25
```

低于阈值返回 `orientation_ambiguous`。该工件仍加入 PlanningScene 作为障碍物，只是不生成
抓取候选。全部工件根据估计 world X、再按 Y 排序为 `workpiece_0..2`；在线排序不读取任务
真值，正常噪声下依靠任务生成的最小间距避免实例交换。

## 3. ScenePoseProvider 边界

`core/scene_pose_provider.py` 提供两个实现：

- `GroundTruthPoseProvider`：PyBullet 和 oracle 回归使用；
- `RGBDScenePoseProvider`：调用 `EstimateScene`，返回统一 `SceneEstimate`。

FSM 不直接依赖 ROS 图像消息。RGB-D 估计成功后，`MoveItBackend.apply_scene_estimate()` 将三个
工件及障碍物按估计位置、尺寸和 yaw 写入 PlanningScene。任何对象写入失败都返回
`scene_estimate_sync_failed`，不会使用 reset 真值兜底。

## 4. FSM 状态、输出与失败

| 状态 | 主要输入 | 主要输出/指标 | 典型失败 |
| --- | --- | --- | --- |
| RESET | `TaskSpec` | home、场景与监控状态 | `multi_object_reset_failed`、场景同步失败 |
| PERCEPTION | reset 后新 RGB-D 帧 | 感知耗时、置信度、对象估计 | timeout、缺图、缺 TF、数量不匹配 |
| GRASP_SELECTION | 全部工件估计 | 候选数、可行数、选择分数 | 无有效方向、无可行候选 |
| PRE_GRASP | 选中候选 | 规划/执行时间、路径、跟踪误差 | IK、规划、执行或碰撞失败 |
| GRASP | 手物状态与夹爪宽度 | attach、门控误差 | gate、DetachableJoint 或 scene attach 失败 |
| LIFT | 携物状态 | 运动指标、碰撞状态 | 携物碰撞或执行失败 |
| TRANSFER | 目标区估计 | 顶层规划时间和路径长度 | `invalid_motion_plan` 等 |
| PLACE | 放置位姿 | release、物体速度、放置误差 | detach、同步或稳定性失败 |
| RETREAT | 预放置位姿 | 运动指标 | 规划或执行失败 |

`_move_to_pose()` 将每个状态的规划时间、执行时间、路径长度、轨迹点、MoveIt error code、
碰撞数和最大跟踪误差累加到 `stage_metrics`。顶层 `planning_time_sec` 与 `path_length_rad`
保留关键 TRANSFER 运动的统一口径，不能和所有状态累计值混用。

成功必须同时满足：已确认 release、物体状态可用、XY 误差不超过 40 mm、Z 误差不超过
15 mm、线速度不超过 0.05 m/s、碰撞观测可用且无无效碰撞、未选工件位移不超过 10 mm。

## 5. 多工件抓取候选与联合选择

`GraspPoseGenerator` 对每个方向有效的矩形工件生成两个顶抓候选：`yaw` 和 `yaw + π`。
两者对应夹爪绕接近轴旋转 180° 的对称姿态。三个方向有效工件共生成六个候选，每个候选包含
pre-grasp、grasp、retreat、object ID、yaw 和感知置信度。

`select_feasible_grasp()` 对每个候选分别检查 pre-grasp 与实际 grasp：

```text
backend.check_pose_feasibility(...)
  → MoveIt PlanExecuteMotion.execute = false
  → 只返回轨迹和指标，不执行控制器 Action
```

两个目标都可行后才计算：

```text
selection_score = pre_grasp_path_length_rad + 1.0 × (1 - confidence)
```

按 `selection_score`、`object_id`、候选序号依次稳定排序。这个分数只在当前候选集合中选择
代价较小的工件和姿态：它没有考虑全局任务顺序、后续多个物体，也不能称为全局最优规划。

选中后，`select_manipulation_object(object_id)` 将 MoveItBackend、Gazebo assisted manager、
碰撞监控、AttachedCollisionObject 和结果统计锁定到同一 ID。另两个工件继续作为 world
collision object，不能因未被选中而从 PlanningScene 消失。

## 6. 坐标系与末端语义

机械臂主链为：

```text
world → panda_link0 → ... → panda_hand → panda_grasptarget
```

- 视觉结果和运动目标使用 `world`；
- MoveIt pose goal 的 end-effector link 是无碰撞几何的 `panda_grasptarget`；
- 物体 attachment 使用具有真实手部几何的 `panda_hand`；
- assisted gate 比较手—工件相对位置、相对姿态、距离与实测夹爪宽度；
- 矩形工件的相对姿态误差按夹爪 π 对称归一化。

如果把 motion goal 与 attach link 混为一谈，抓取高度和 AttachedCollisionObject 相对变换会
产生系统性偏差。

## 7. MoveIt 规划与执行

Python `MoveItBackend` 负责状态机语义和 ROS client；C++ bridge 负责 MoveIt 调用：

```text
MoveItBackend.move_to_pose()
  → PlanExecuteMotion Action
  → moveit_bridge_node.cpp
  → MoveGroupInterface.plan()
  → execute=true 时 MoveGroupInterface.execute()
```

Action goal 包含 pose/named target、planner ID、3 秒预算、0.2 速度/加速度缩放、末端 link 和
是否执行。bridge 返回稳定 failure type、MoveIt error code、规划/执行时间、轨迹点数与关节路径
长度。`check_pose_feasibility()` 明确写入 `execute=false`，因此候选筛选不会让机械臂先试着动。

PyBullet 自研 RRT-Connect 与 OMPL RRTConnect 使用不同碰撞引擎、采样实现和计时边界；相同
task seed 用于覆盖审计，绝对耗时不能直接解释为算法优劣。

## 8. Gazebo reset、attach 与 release

### 8.1 reset

`EpisodeResetManager` 的事务顺序是：

1. detach 全部工件；
2. 暂停世界并设置三工件、目标区和障碍物位姿；
3. 恢复世界，使用控制器将机械臂回 home 并打开夹爪；
4. 验证关节位置/速度、实体位置/yaw 和线/角速度；
5. reset 碰撞与 experimental physics monitor；
6. 任一步失败都返回稳定错误码，不报告 reset 成功。

home 控制器最多在同一 reset 事务内有界尝试三次，但最终仍必须通过严格稳态检查；它不会重新
发布或重跑失败 episode。

### 8.2 attach

assisted attach 是跨 Gazebo 与 MoveIt 的事务：

```text
手物位置/姿态/距离/夹爪宽度门控
  → 对应 object ID 的 DetachableJoint 确认 attached
  → MoveIt 移除该 world object
  → 添加同 ID AttachedCollisionObject
```

Gazebo attach 失败时不修改 MoveIt；MoveIt attach 失败时立即请求 Gazebo detach 回滚。
`actual_object_id`、selected ID 和 AttachedCollisionObject ID 不一致时 fail-closed。

### 8.3 release

release 顺序是 Gazebo detach、MoveIt detach、等待新 odometry 证明物体稳定、读取真实位姿、
将该对象重新加入 PlanningScene，最后 retreat。不能在命令打开夹爪后直接把目标位姿当作
最终物体位姿。

## 9. 碰撞与结果保存

碰撞监控同时处理机器人和三个工件：

- 未选工件与桌面的静止支撑接触允许；
- 机器人碰未选工件、携带工件碰未选工件或障碍物均为无效碰撞；
- selected object 身份不明确、接触流缺失或对象位姿缺失时不可判成功；
- 真实执行碰撞与规划失败分别记录，不能用“没有消息”冒充无碰撞。

`evaluator_node` 对 JSONL 执行 append、flush 和 fsync 后发布保存确认。`DemoCompletionGate`
必须同时收到匹配的 EpisodeResult 和保存确认才能请求关闭。`run_status.json` 将 readiness、
episode 和 teardown 分开；Humble `move_group` 在结果落盘后的析构异常不能覆盖成功 episode，
也不能被隐瞒。

## 10. 三种后端分别证明什么

| 模式 | 能证明 | 不能证明 |
| --- | --- | --- |
| PyBullet | 自研 IK、Linear/RRT-Connect、碰撞和状态机语义 | MoveIt/ROS 控制器集成 |
| MoveIt mock | URDF/SRDF、PlanningScene、OMPL、Action 与 controller 接口 | Gazebo 执行、接触和抓取成功 |
| MoveIt + Gazebo assisted | 感知驱动规划、控制执行、场景同步、门控抓放和评测 | 纯摩擦物理抓取、真机迁移 |

## 11. 三类真实失败如何定位

### stale attachment

一次携带 `workpiece_2` 的 transfer 失败后，旧 reset 先把 selected ID 切回
`workpiece_0`，只清理新 ID，导致 MoveIt 中残留 `workpiece_2` AttachedCollisionObject。
后续目标状态连续无效。修复为 reset 同时清理上一批和当前批的全部工件 ID，并对 pre-grasp
和 grasp 都做 feasibility 检查。

### home path tolerance

ground-truth 正式运行中，控制器报告 `panda_joint1` 路径误差约 0.1514 rad，略高于 0.15 rad
容差，两个 reset 返回 `HOME_FAILED`。修复为同一 reset 内有界 home 重试并保留最终严格稳态
验证；同一 task seed 随后通过，失败批次仍原样保存。

### 早期无详情 reset

早期批次出现两条 `multi_object_reset_failed`，但当时后端丢失了 reset manager 的 message，
没有足够证据归因。项目没有猜测根因，而是先保存原始记录，再修改错误传播以保留错误码和
detail。这个案例说明“无法分类”本身也是需要修复的可观测性问题。

## 12. 面试阅读建议

第一遍只读 `tasks → provider → FSM → grasp selection → MoveItBackend`；第二遍再进入 perception
数学和 Gazebo 事务；第三遍结合 `failure_examples.json` 复盘失败。完成后使用
[模拟面试指南](interview_guide.md)回答问题，不要背文件名，重点讲设计原因、证据和边界。
