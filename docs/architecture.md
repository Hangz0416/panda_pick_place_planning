# 系统架构与代码导读

## 主线

项目只保留一套抓取放置状态机。任务来自 `ManipulationTask`，执行结果统一为
`EpisodeResult`，PyBullet 和 MoveIt/Gazebo 只在 backend 层分叉。

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

## 代码入口

| 需要理解的内容 | 文件 |
| --- | --- |
| 任务生成与固定种子 | `core/tasks.py` |
| 统一数据结构 | `core/schemas.py` |
| 位姿源协议与实现 | `core/scene_pose_provider.py` |
| Backend 协议 | `core/backends/base.py` |
| 抓放状态机 | `core/pick_place_fsm.py` |
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
RESET -> PERCEPTION -> PRE_GRASP -> GRASP -> LIFT -> TRANSFER -> PLACE -> RETREAT -> DONE
  |        |          |          |        |          |        |          |
  +--------+----------+----------+--------+----------+--------+----------+--> FAILED
```

每次 `move_to_pose` 返回 `BackendMotionReport`。状态机负责失败阶段和任务语义，backend
负责规划、执行、夹爪、物体状态和仿真器特有操作。

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

## RGB-D 感知分支

视觉逻辑没有写进运动后端或状态机，而是通过统一 provider 接入：

```text
ScenePoseProvider
|- GroundTruthPoseProvider   # 保留当前基线
`- RGBDScenePoseProvider     # 调用类型化 EstimateScene 服务
```

`panda_perception` 对彩色/深度图做近似同步，HSV 分割红色方块、绿色目标区和蓝色障碍物，
再使用 CameraInfo 反投影、中值/MAD 过滤和 TF2 转到 `world`。服务只使用请求到达后的新帧，
避免跨 episode 缓存泄漏。两种 provider 输出相同的世界坐标对象估计，状态机和 backend
不因视觉算法而分叉；视觉模式下真值只用于 reset 和误差评测。
