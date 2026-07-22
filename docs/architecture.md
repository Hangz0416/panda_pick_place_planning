# 系统架构与代码导读

## 主线

项目只保留一套抓取放置状态机。任务来自 `ManipulationTask`，执行结果统一为
`EpisodeResult`，PyBullet 和 MoveIt/Gazebo 只在 backend 层分叉。

当前产品主线固定为 RGB-D 三工件 + MoveIt/Gazebo assisted。physics monitor 虽保留在仓库，
但属于 `experimental / fail-closed`，不参与主线调用图、正式指标或简历能力声明。

```text
task_generator_node
        |
        | /manipulation_task
        v
manipulation_node
        |
        v
ScenePoseProvider
        +--> GroundTruthPoseProvider (oracle regression)
        `--> RGBDScenePoseProvider --> EstimateScene service
                                      --> RGB + depth + CameraInfo + TF2
        |
        v
PickPlaceRunner
        |
        +--> GraspPoseGenerator --> planning-only feasibility --> deterministic selection
        |
        +--> PyBulletBackend --> IK / Linear / RRT-Connect / PyBullet
        |
        +--> MoveItBackend --> PlanExecuteMotion Action --> C++ MoveIt bridge
                                                       |
                                                       +--> mock controllers
                                                       +--> Gazebo controllers
        |
        | /episode_result
        v
evaluator_node --> JSONL
```

## 数据与状态所有权

| 状态 | 唯一权威 | 允许的消费者 |
| --- | --- | --- |
| 任务真值 | `TaskSpec` / `ManipulationTask` | reset、ground-truth provider、离线误差评测 |
| RGB-D 在线位姿 | `SceneEstimate` | 候选生成、运动目标、PlanningScene |
| 关节执行状态 | ros2_control joint state | MoveIt 当前状态、跟踪误差、reset 验证 |
| 工件执行状态 | Gazebo odometry/contact | attach gate、碰撞、释放稳定性、最终结果 |
| 选中对象身份 | `select_manipulation_object` 发布的 selected ID | backend、assisted manager、碰撞监控、评测 |
| episode 结果 | `EpisodeResult` | evaluator、JSONL、summary/report |
| 进程退出状态 | `run_status.json.teardown` | 启动监督与工程诊断 |

RGB-D 模式中，TaskSpec 真值不得作为感知失败后的 fallback。`SceneEstimate` 写入 PlanningScene
失败时 episode 必须以 `scene_estimate_sync_failed` 结束。

## 代码入口

| 需要理解的内容 | 文件 |
| --- | --- |
| 任务生成与固定种子 | `core/tasks.py` |
| 统一数据结构 | `core/schemas.py` |
| 位姿源协议与实现 | `core/scene_pose_provider.py` |
| Backend 协议 | `core/backends/base.py` |
| 抓放状态机 | `core/pick_place_fsm.py` |
| 多工件抓取候选生成与联合排序 | `core/grasp_pose.py` |
| PyBullet 后端 | `core/backends/pybullet.py` |
| MoveIt Python 后端 | `core/backends/moveit.py` |
| Linear / RRT-Connect | `core/planners/linear.py` / `core/planners/rrt_connect.py` |
| PyBullet 碰撞检查 | `core/collision_checker.py` |
| ROS 消息转换 | `nodes/ros_conversions.py` |
| ROS 执行节点 | `nodes/manipulation_node.py` |
| MoveIt C++ 桥接 | `panda_moveit_bridge/src/moveit_bridge_node.cpp` |
| Gazebo reset | `panda_gz_bringup/episode_reset_manager.py` |
| assisted 抓取与回滚 | `panda_gz_bringup/assisted_grasp_manager.py` |
| Gazebo 碰撞观测 | `panda_gz_bringup/execution_collision_monitor.py` |
| RGB-D 检测器 | `panda_perception/detector.py` |
| 感知 ROS 节点 | `panda_perception/scene_estimator_node.py` |

`core/` 不依赖具体 ROS launch 结构；`nodes/` 只负责通信和调度。MoveIt 规划细节集中在
C++ bridge，Gazebo 世界管理集中在 `panda_gz_bringup`。

## 状态机

```text
RESET -> PERCEPTION -> GRASP_SELECTION -> PRE_GRASP -> GRASP -> LIFT
  |        |               |              |          |        |
  +--------+---------------+--------------+----------+--------+--> FAILED

LIFT -> TRANSFER -> PLACE -> RETREAT -> DONE
          |          |         |
          +----------+---------+--> FAILED
```

每次 `move_to_pose` 返回 `BackendMotionReport`。状态机负责失败阶段和任务语义，backend
负责规划、执行、夹爪、物体状态和仿真器特有操作。

## 坐标系与对象语义

```text
world → panda_link0 → ... → panda_hand → panda_grasptarget
world → overhead_camera_link → overhead_camera_optical_frame
```

视觉估计和 pose target 都在 `world` 下表达。MoveIt 运动目标使用 `panda_grasptarget`，避免
把手部碰撞几何的局部偏移混入目标；AttachedCollisionObject 使用实际物理连接 link
`panda_hand`。相机点先由光学坐标系外参变换到 world，再进入 provider 和 FSM。

## PyBullet 分支

1. `PandaIKSolver` 向 PyBullet IK API 传入全部可运动关节的 damping、limits、ranges
   和 rest poses，再映射回七个机械臂关节。
2. Linear 对关节空间直线插值；RRT-Connect 使用双树扩展、连接与 shortcut。
3. 碰撞检查区分刚性组件内部接触、真实自碰撞、环境碰撞和携物碰撞。
4. assisted attach 保存末端到方块的相对变换，候选关节状态会同步预测方块位姿。

## MoveIt / Gazebo 分支

1. `MoveItBackend` 将世界坐标下的目标位姿发送给 `PlanExecuteMotion` Action。
2. C++ bridge 使用 `MoveGroupInterface` 调用 OMPL，计算路径长度、轨迹点和执行时间。
3. Gazebo 中的 `joint_trajectory_controller` 和 `gripper_action_controller` 执行轨迹。
4. attach 顺序是 Gazebo 门控、DetachableJoint、MoveIt AttachedCollisionObject；任一后续步骤
   失败都要回滚前面的外部状态。
5. release 后先等待物体稳定，再读取真实位姿并放回 PlanningScene，最后规划 retreat。

## RGB-D 多工件感知与联合选择

视觉逻辑没有写进运动后端或状态机，而是通过统一 provider 接入：

```text
ScenePoseProvider
|- GroundTruthPoseProvider   # 保留当前基线
`- RGBDScenePoseProvider     # 调用类型化 EstimateScene 服务
```

`panda_perception` 对彩色/深度图做近似同步，HSV 分割全部红色连通域、绿色目标区和蓝色
障碍物，再使用 CameraInfo 反投影、中值/MAD 过滤和 TF2 转到 `world`。矩形工件的有效
world XY 点云通过二维 PCA 得到主轴 yaw，并以特征值比拒绝方向歧义。工件按估计 world X、
再按 Y 排序为 `workpiece_0..2`，整个过程不读取任务真值。服务只使用请求到达后的新帧，
避免跨 episode 缓存泄漏；方向歧义工件仍作为 PlanningScene 障碍物保留。

`GraspPoseGenerator` 对方块保留原固定顶抓，对每个方向有效的矩形工件生成 yaw 与 yaw+π
两个对称候选。Backend 的 `check_pose_feasibility` 对每个候选的 pre-grasp 与实际 grasp 目标
都只规划、不执行；两者均可行后才计算：

```text
selection_score = pre_grasp_path_length_rad + (1 - perception_confidence)
```

按分数、对象 ID、候选序号稳定排序后，`select_manipulation_object` 将 Gazebo DetachableJoint、
碰撞监控、MoveIt AttachedCollisionObject 和结果评测锁定到同一对象。未选对象在完整 episode
中始终保留为世界碰撞物。选中朝向贯穿 pre-grasp、grasp、lift、transfer、place 和 retreat；
视觉模式下真值只用于 reset 与离线误差评测。

## 运行完成与关闭协议

`evaluator_node` 在 JSONL append、flush 和 fsync 完成后发布 episode 保存确认。Demo completion
gate 必须同时收到结果与匹配的保存确认，才允许自动关闭；`run_status.json` 分开记录 readiness、
episode 和 teardown 状态。关闭顺序为应用节点、MoveIt bridge、`move_group`、Gazebo，避免结果
尚未落盘就被 Humble 的析构异常打断。可选 supervisor 最多启动两次，且只对 readiness 失败
重启，不会用重跑掩盖规划或任务失败。
