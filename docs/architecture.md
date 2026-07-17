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

`core/` 不依赖具体 ROS launch 结构；`nodes/` 只负责通信和调度。MoveIt 规划细节集中在
C++ bridge，Gazebo 世界管理集中在 `panda_gz_bringup`。

## 状态机

```text
RESET -> HOME -> PRE_GRASP -> GRASP -> LIFT -> TRANSFER -> PLACE -> RETREAT -> DONE
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

## 视觉扩展位置

后续不应将视觉逻辑写进状态机。计划增加：

```text
ObjectPoseProvider
|- GroundTruthPoseProvider   # 保留当前基线
`- RGBDPoseProvider          # RGB + Depth + CameraInfo + TF2
```

两种 provider 输出相同的世界坐标物体位姿，状态机和 backend 不因视觉算法而分叉。

