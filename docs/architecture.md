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

## 代码入口

| 需要理解的内容 | 文件 |
| --- | --- |
| 任务生成与固定种子 | `core/tasks.py` |
| 统一数据结构 | `core/schemas.py` |
| 位姿源协议与实现 | `core/scene_pose_provider.py` |
| Backend 协议 | `core/backends/base.py` |
| 抓放状态机 | `core/pick_place_fsm.py` |
| 抓取候选生成与排序 | `core/grasp_pose.py` |
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

`panda_perception` 对彩色/深度图做近似同步，HSV 分割红色工件、绿色目标区和蓝色障碍物，
再使用 CameraInfo 反投影、中值/MAD 过滤和 TF2 转到 `world`。矩形工件的有效 world XY
点云通过二维 PCA 得到主轴 yaw，并以特征值比拒绝方向歧义。服务只使用请求到达后的新帧，
避免跨 episode 缓存泄漏。

`GraspPoseGenerator` 对方块保留原固定顶抓，对矩形工件生成 yaw 与 yaw+π 两个对称候选。
Backend 的 `check_pose_feasibility` 只规划、不执行，筛选出可行候选并按 pre-grasp 关节路径
长度、候选序号确定性排序。选中朝向贯穿 pre-grasp、grasp、lift、transfer、place 和 retreat；
视觉模式下真值只用于 reset 与误差评测。
