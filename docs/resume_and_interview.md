# 简历表述与模拟面试提纲

这份文档只使用已经实现且运行验证的事实。当前仓库只有一套源码主线：PyBullet 是
自研规划基线，MoveIt/Gazebo 是工程化规划执行后端。历史目录名只为保持原始数据哈希和
引用稳定，不需要在简历中讲版本迁移。

面试时需要主动说清：Gazebo 使用门控 DetachableJoint 的 assisted grasp，不是 physics
grasp；RGB-D 依赖已知颜色和尺寸，矩形工件只估计平面位置与绕 Z 轴方向，不是通用 6D
位姿估计；结果不外推为真机成功率。

## 当前可直接使用的简历项目经历

**基于 ROS2、MoveIt 2 与 RGB-D 感知的 Panda 抓取放置系统｜个人项目**

技术栈：Python / C++ / ROS2 Humble / OpenCV / RGB-D / PyBullet / MoveIt 2 / Gazebo Fortress / ros2_control / RRT-Connect

- 在 PyBullet 中搭建 Franka Panda 桌面抓取放置环境，实现随机任务生成、逆运动学、关节限位、
  自碰撞与环境碰撞检测、轨迹执行及任务结果判定，形成可复现的机械臂操作闭环。
- 抽象统一 `ManipulationBackend` 接口，使 PyBullet 与 MoveIt 2 后端复用同一抓取放置状态机；
  基于 ROS2 自定义消息和 Action 拆分任务生成、运动执行与评测统计。
- 实现关节空间 Linear 与双向 RRT-Connect。在固定种子任务上，clear 场景两种规划器
  平均规划时间为 1.75 ms 和 1.92 ms；障碍场景中 RRT-Connect 平均规划时间为
  33.16 ms、平均关节路径长度为 1.31 rad，Linear 能识别被障碍物阻断的直线路径。
- 使用 C++ 封装 MoveIt 2 规划执行桥接，支持位姿目标、PlanningScene 障碍物与物体
  attach/detach；接入 Gazebo Fortress 与 `gz_ros2_control`，完成控制器驱动、确定性重置
  和约束辅助抓放，clear/obstacle 平均规划时间为 23.77 ms 和 33.41 ms。
- 增加 Gazebo 顶置 RGB-D 相机与 OpenCV 感知节点，通过近似时间同步、HSV 分割、深度反投影、
  中值/MAD 过滤和 TF2 估计工件、目标区及障碍物位置；对矩形工件 world-XY 点云执行 PCA
  估计 yaw，并依据特征值比拒绝方向不可观测的目标。
- 根据视觉 yaw 生成两个相差 π 的对称顶抓候选，调用 MoveIt planning-only 逐个检查碰撞与
  可规划性，再按预抓取关节路径长度确定性选择；运动目标与 PlanningScene 均来自视觉结果，
  并通过防真值泄漏测试保证任务真值只用于 reset 和离线误差评测。
- 在固定种子 nominal 视觉回归中，矩形工件位置与 yaw 误差 P95 分别不超过 4.16 mm 和
  4.94°，感知耗时 P95 不超过 163.5 ms；在低亮度、RGB 噪声和深度丢失组合扰动下，位置与
  yaw 误差 P95 分别不超过 3.85 mm 和 6.84°。
- 建立 IK、规划、碰撞、抓取、执行、场景同步和放置失败分类；针对 PyBullet IK 可运动关节
  映射、固定关节连杆伪自碰撞、Gazebo 夹爪 `stalled` 及 MoveIt seeded IK 无效目标问题，
  分别完成参数映射、刚性组件内部接触过滤、带门控回滚的抓取事务和预算内 IK 分支回退。

简历可以写“Gazebo assisted 闭环完成”，不能写“完成物理抓取”。实验结果只限于
当前固定仿真任务分布。

## 仍不能使用的 physics 简历句式

physics 模式必须等后续独立实验完成后单列：

> 移除 DetachableJoint 后，仅依靠夹爪碰撞与摩擦完成物理抓取；每个场景运行 [真实 episode
> 数]，记录双指接触、滑落、相对位姿漂移和释放稳定时间。

不能把 assisted 的固定约束结果填进 physics 句式。

## 60 秒项目介绍

这个项目实现了一条 RGB-D 感知、MoveIt 规划、Gazebo 控制执行和量化评测闭环。项目从
PyBullet 中的 Panda 抓取放置与路径规划起步，我先把任务生成、IK、碰撞检测、
Linear/RRT-Connect、状态机和固定种子评测做成闭环，再把状态机与执行引擎分开。
我定义了 ManipulationBackend，PyBullet 和 MoveIt
都实现同一组 reset、move、gripper、attach/detach 与 object_state 接口，因此只保留一套
PickPlaceRunner。ROS2 在线链路也从 JSON 字符串迁移成自定义消息和 Action。MoveIt 部分用
C++ MoveGroupInterface 做规划与可选执行，用 PlanningSceneInterface 管理桌面、方块、障碍物
和 AttachedCollisionObject；Python 继续负责抓放阶段与 EpisodeResult。

我还补上了 PyBullet 携带方块后的碰撞检查：保存 `panda_grasptarget`（末端）到方块的相对
变换，在每个候选关节状态预测方块位姿，分别判断机器人碰撞和携物碰撞。
MoveIt planning-only 全部保持 `not_executed`，不把未执行轨迹写成抓取成功。
Gazebo 中的 grasp 是位姿与实测夹爪宽度门控后的 DetachableJoint 辅助抓取，因此我把结果
限定为 assisted 规划控制闭环，不会混写成 physics grasp。
视觉部分使用顶置 RGB-D：HSV 分割场景目标，利用深度和内参反投影，再经 TF2 转到 world。
`ScenePoseProvider` 保证视觉结果直接驱动运动目标和 PlanningScene；任务真值只负责 reset 与
离线误差统计，因此不会出现视觉模式偷偷读取真值的问题。矩形工件的 world-XY 点云通过 PCA
估计平面 yaw；状态机生成两个 π 对称候选，以 planning-only 检查后选择关节路径较短的候选，
筛选过程不会让机械臂运动。
正式 nominal 回归中，矩形工件位置误差 P95 最差为 4.16 mm，yaw 误差 P95 最差为 4.94°；
组合传感器扰动下两项 P95 最差为 3.85 mm 和 6.84°。这些数字只描述当前已知颜色、尺寸与
顶置相机的 Gazebo assisted 分布，不外推到开放场景或真机。

## 代码流程

### 1. 任务生成与类型化通信

1. packages/panda_pick_place_planning/.../core/tasks.py 根据 task seed 生成物体和目标位置。
2. TaskSpec 记录 planner、planner seed、scene、backend、simulator、grasp_mode、object variant、
   确定性 object yaw 和感知扰动 profile。
3. task_generator_node.py 将 TaskSpec 转成 ManipulationTask，发布 /manipulation_task。
4. manipulation_node.py 校验 task.backend 与节点 backend 一致，再交给共享 PickPlaceRunner。
5. evaluator_node.py 订阅类型化 EpisodeResult，转换为 Python schema 后追加写入 JSONL。

同一 scene 与 task_seed 下，不同规划器共享相同任务几何；planner seed 单独记录。

### 2. 统一 FSM

PickPlaceRunner.run() 的状态顺序是：

~~~text
RESET
  → PERCEPTION：等待 reset 后新 RGB-D 帧并估计 world 坐标对象中心
  → GRASP_SELECTION：生成 π 对称候选，planning-only 筛选并确定抓取朝向
  → PRE_GRASP：到物体上方，再下降到抓取位姿
  → GRASP：闭合夹爪并请求后端确认 attach
  → LIFT：携物抬升
  → TRANSFER：使用任务指定的 planner 跨场景搬运
  → PLACE：下降、打开夹爪、detach、等待稳定
  → RETREAT：末端回到预放置高度
  → DONE / FAILED
~~~

每次 move_to_pose 都返回 BackendMotionReport。FSM 把规划时间、执行时间、路径长度、轨迹点、
跟踪误差和 PlanningScene 同步状态累计到对应 stage_metrics，不在节点里重复统计。

### 3. PyBulletBackend 分支

1. reset() 恢复 Panda home、夹爪、方块、目标与可选障碍物，同时清理上一 episode 的电机目标。
2. PandaIKSolver 向 PyBullet IK 传入全部 9 个可运动关节的 damping、limits、ranges 与 rest
   poses，再映射出 7 个机械臂关节并用 FK 验证位置/姿态误差。
3. LinearPlanner 对关节空间直线插值；RRTConnectPlanner 双树交替扩展、连接并 shortcut。
4. PandaCollisionChecker 检查关节限位、自碰撞、机器人—环境碰撞和安全距离。
5. attach 后记录 `panda_grasptarget（末端）—cube` 相对变换；候选状态下同步预测 cube pose，再检查
   cube—table/obstacle。
6. execute_joint_path 对路径加密并逐仿真步统计真实接触，区分普通执行碰撞与携物执行碰撞。
7. detach 后等待稳定，读取真实方块位置和速度。

### 4. MoveItBackend 分支

1. MoveItBackend.reset() 清理上个 episode 的 attach/world object 状态，向 PlanningScene 加入
   table、cube 和可选 obstacle，然后请求 named home。
2. 统一 TF 链为 `world → panda_link0 → panda_hand → panda_grasptarget`；move_to_pose() 把
   world 坐标系 PoseStamped、`panda_grasptarget` 末端、RRTConnect planner、3 秒预算和 0.2
   速度/加速度缩放写入 PlanExecuteMotion goal。物体 attach 使用 `panda_hand`。
3. C++ bridge 以当前状态为起点，设置 named/pose target，调用 MoveGroupInterface.plan()，
   可按 execute 字段调用 execute()。
4. bridge 将 MoveIt error code 映射成稳定失败类型，并计算轨迹点数和关节路径长度。
5. Upsert/Remove 服务更新世界模型；SetAttachedObject 调用 attachObject/detachObject，并等待
   PlanningScene 中的实际 attached 状态。

Python FSM 使用两线程 MultiThreadedExecutor；长任务订阅和后端 Action/Service/遥测分别放入
两个 MutuallyExclusiveCallbackGroup。同步状态机只等待 Future 事件，另一个线程串行处理后端
回调，既避免再次 spin 导致死锁，也避免 Humble rclpy 原生实体被多个线程并发进入。

### 5. RGB-D 感知与防真值泄漏

1. Gazebo 顶置相机以 640×480、20 Hz 发布 RGB、32FC1 depth 和 CameraInfo，静态 TF 为
   `world → overhead_camera_link → overhead_camera_optical_frame`。
2. `SceneEstimatorNode` 用 ApproximateTimeSynchronizer 配对 RGB/depth；每次 EstimateScene
   请求记住当前帧代数，只接受请求之后到达的新同步帧。
3. `HSVSceneDetector` 取最大连通域，进行面积过滤；有效深度先裁剪范围，再通过中值和 MAD
   剔除异常值。矩形工件对 world-XY 点云做二维 PCA，由主特征向量估计 yaw；特征值比低于
   阈值时返回 `orientation_ambiguous`，不伪造方向。
4. 像素通过 `(u-cx)z/fx, (v-cy)z/fy, z` 反投影到光学坐标系，再使用 TF2 转到 world；
   已知顶面高度用于换算轴对齐物体中心高度。
5. `RGBDScenePoseProvider` 将类型化服务结果转换为统一 `SceneEstimate`。FSM 的抓取候选、
   pre-grasp、transfer 和 MoveIt PlanningScene 使用这些估计；原始 TaskSpec 只传给 Gazebo
   reset，并用于计算 position/yaw error。
6. obstacle 场景缺障碍物立即返回 `obstacle_not_detected`，clear 场景不要求障碍物。任何感知
   失败都停在 PERCEPTION，不继续规划或执行。

### 6. MoveIt mock 分支的含义

mock ros2_control 的 GenericSystem 可以验证：

- URDF/SRDF/KDL/OMPL 是否能加载；
- move_group、bridge 和 Action 是否连通；
- controller 是否 active；
- PlanningScene 增删和 attach/detach 接口是否工作；
- 目标能否规划，以及轨迹字段是否返回。

它不能证明：

- Gazebo 动力学执行正确；
- 真实关节跟踪误差；
- 方块受到重力、摩擦或接触后被抓起；
- 发生了有效碰撞监测；
- 完整抓取放置 task_success。

最终代码先通过 enhanced clear/obstacle smoke 与 5 clear + 5 obstacle，再完成 50+50；两组
均 50/50 规划成功、0 PlanningScene 同步错误。全部记录保持 `task_success=false`、
`execution_success=false`、`failure_type=not_executed`，因此只证明规划链路，不证明抓放执行。

### 7. Gazebo assisted 分支的设计流程

1. `panda_moveit_gz.launch.py` 是已有单入口；它复用共享 Panda Xacro 的 `hardware_type:=gz`，
   只创建一份 robot_state_publisher/controller manager，并组合 move_group、bridge 与应用节点。
2. gz_ros2_control 暴露 7 轴 trajectory controller、gripper controller 与 joint state。
3. readiness gate 等待 `/clock`、关节状态、方块 odometry、三个 active controller 和必要
   service/action 全部就绪，才允许启动共享 FSM。
4. typed ResetEpisode 暂停世界、请求 detach、按任务 seed reset model、用轨迹控制器回 home、
   校验关节位置/速度、再次设置方块/目标/障碍物后恢复。
5. 抓取前保持 76 mm 总开度；方块沿用原固定朝向，60×40 mm 矩形工件则根据视觉 yaw 对齐
   抓取朝向，避免 attach 前手指与工件互穿。
6. AssistedGraspManager 同时检查手—物相对位置、姿态、距离与夹爪宽度；姿态误差按夹爪
   π 对称性归一化。
7. Gazebo 确认 DetachableJoint attached 后，MoveItBackend 才添加
   AttachedCollisionObject；失败则回滚 detach。
8. 类型化碰撞服务只有在机器人接触、方块接触和方块位姿都可用后才允许成功判定。
9. 释放时先 detach Gazebo，再 detach PlanningScene；等待方块稳定后读取实际位姿并重新加入
   MoveIt world，最后规划 retreat。

该序列已经由 5+5 快检和 50+50 正式 episode 完整运行验证。第一次正式尝试的 12 条中有 3 条
在 2 mm 预夹紧时被低增益控制器报告为 stalled；修复只允许 assisted 预夹紧把 stalled 交给
随后独立校验实测指宽和位姿的 attach gate，释放仍严格拒绝 stalled。修复后的 110 条快检与
正式 episode 全部成功，原失败记录仍保留。

### 8. physics 分支为何默认 fail-closed

physics 模式不加载 DetachableJoint，也不允许 fixed-joint attach。双指持续接触、抓取后相对
位姿、平移/旋转漂移、滑落 latch、释放和稳定时间的 monitor 与 typed 结果字段已经实现，
MoveItBackend 也不会伪造 attach 成功。

当前共享 Panda 描述已引入 Franka 上游公开的辨识惯量并通过正定检查，但上游明确说明这些不是
官方 Panda 惯量，因此不能据此宣称可信物理抓取。launch 默认 `physics_model_validated:=false`
并拒绝 physics；后续必须独立标定/审计摩擦、碰撞和动力学，再与 assisted 分组报告。

## 模拟面试问题

建议先口头回答，再看回答要点。回答时始终区分“代码已实现”“单元测试已通过”和“完整运行已验证”。

### 1. 这个项目现在到底完成到什么程度？

回答要点：PyBullet 算法基线、MoveIt planning-only 和 Gazebo assisted 均已验证；RGB-D
感知已经通过 `ScenePoseProvider` 接入同一状态机，矩形工件 PCA yaw 估计和 MoveIt 抓取候选
筛选也进入同一闭环，视觉结果驱动运动目标和 PlanningScene。
当前边界是未完成 physics grasp、真机和通用 6D 位姿，Humble `move_group` 在结果落盘后的
关闭析构存在已知段错误。

### 2. 为什么要保留 PyBullet，还要接 MoveIt 和 Gazebo？

回答要点：PyBullet DIRECT 启动快，适合固定种子回归和展示自研 Linear/RRT-Connect；
MoveIt 提供更完整的 PlanningScene、OMPL 和轨迹接口；Gazebo + ros2_control 用于检验控制器、
时钟、执行同步和接触物理。三者承担不同验证层级，不能互相替代。

### 3. 为什么后端抽象比复制两套状态机更重要？

回答要点：抓取阶段、失败状态和成功判据应该唯一，否则两个后端很快出现语义漂移。后端只回答
“重置、移动、夹爪、attach、物体状态”，FSM 统一决定阶段切换和 EpisodeResult。这样同一
TaskSpec 可以跨后端复现。

### 4. ManipulationBackend 最难统一的是什么？

回答要点：不是函数名，而是语义。PyBullet 一次 move 同时自研规划并执行；MoveIt Action 可
planning-only 或 plan+execute。二者必须统一返回 planning_success、execution_time、路径长度、
轨迹点和失败类型，并明确 simulation_steps 只对 PyBullet 有意义。

### 5. 为什么从 String + JSON 改成自定义 ROS2 接口？

回答要点：编译期字段检查、ros2 interface introspection、Action feedback/cancel、服务请求/
响应语义更清晰，避免节点各自解析字符串。JSON 仍适合离线保存，但不适合作为内部实时协议。
同时保留历史 JSON 向后读取，避免已记录实验失效。

### 6. Panda 是 7 自由度，为什么 IK 参数是 9 维？

回答要点：7 是机械臂关节；PyBullet Panda URDF 还有两个可运动手指关节。IK API 的 damping、
limits、ranges、rest pose 要按全部非固定关节排列，因此是 9 维。求解后再用 joint index 到
solution index 的映射抽取 7 维 arm 结果。

### 7. 为什么 link 6 和 link 8 的接触不算真实自碰撞？

回答要点：二者通过固定关节链属于同一刚性组件，不存在相对运动。代码按 joint type 建固定
关节连通图，过滤同一刚性组件内部 link pair；非相邻、非同刚体的接触仍保留。

### 8. 你如何检查“机械臂没撞，但拿着的方块撞了”？

回答要点：PyBullet attach 时求 `panda_grasptarget`（末端）到 cube 的相对变换。检查候选关节
状态时先做 FK 得到新的 grasptarget pose，再复合相对变换预测 cube pose，对 cube 与
table/obstacle 做 contact 和 closest-point 查询。检查结束恢复真实 pose，避免规划采样污染
仿真。MoveIt AttachedCollisionObject 则挂到有碰撞几何的 `panda_hand`。

### 9. 为什么携物碰撞要单独分类？

回答要点：如果都写 planning_collision，很难判断应修机械臂路径、抓取姿态还是物体几何。
单独的 planning_carried_object_collision 和 execution_carried_object_collision 能支持失败
统计和针对性回归。

### 10. 讲一下你的 RRT-Connect。

回答要点：先验证起终点和直连边；建立起点树与终点树；每轮按 goal bias 或关节限位均匀采样，
第一棵树向样本最多扩展一个 step，整条边按 resolution 离散验碰；第二棵树贪心连接新节点。
连接后按 parent 回溯并按 root 类型拼接，再 shortcut 并最终复验。

### 11. 自研 RRT-Connect 与 MoveIt OMPL RRTConnect 的结果能直接比较吗？

回答要点：不能宣称等价。状态空间、采样器、碰撞几何、离散分辨率、路径后处理、时间参数化和
计时边界都不同。可以共享任务几何与成功定义做工程对照，但绝对耗时不能直接解释成算法优劣。

### 12. C++ MoveIt bridge 为什么不全部写在 Python？

回答要点：Humble 下 MoveGroupInterface 与 PlanningSceneInterface 的官方、稳定接口在 C++
侧成熟。C++ bridge 把规划、执行和场景操作包装成 ROS2 Action/Service；Python 保留更适合快速
迭代的 FSM 和评测，两边职责清楚。

### 13. MoveIt bridge 如何避免 PlanningScene 与规划同时竞争？

回答要点：MoveGroupInterface 和 scene mutation 共用一个 operation mutex；motion 串行执行，
场景服务在 busy 时返回稳定错误，而不是并发改场景。attach/detach 后还轮询实际 scene 状态，
超时则回滚或报告同步失败。

### 14. 为什么 Gazebo attach 要先于 MoveIt attach？

回答要点：Gazebo 是执行世界。只有 Gazebo 门控通过并确认 DetachableJoint 建立，MoveIt 才能
把 cube 作为 AttachedCollisionObject；否则规划场景会假设物体已经被拿起。若 MoveIt 同步失败，
必须立刻 detach Gazebo 回滚。

### 15. 为什么辅助抓取前不直接把夹爪闭到方块上？

回答要点：Fortress DetachableJoint 不能在模型已经互相穿透时可靠建立连接。当前先保持两指
总开度约 76 mm，高于 50 mm cube 在 45° 手爪下约 70.7 mm 的对角宽度；位姿、姿态、距离和开度门控全部通过后 attach，
再执行闭合指令。

### 16. assisted 和 physics 有什么本质区别？

回答要点：assisted 用经过门控的固定约束提供稳定基线；physics 不允许固定约束，只依靠接触、
摩擦和 gripper action。两者必须用 grasp_mode 分组，physics 还要记录双指接触、滑落和漂移。
这些 observation/latch 已实现，但现有辨识惯量不等于官方动力学模型，physics 默认 fail-closed 且未运行；
不能拿 assisted 成功率代替。

### 17. Gazebo reset 为什么不能直接改机器人关节状态？

回答要点：绕过 ros2_control 改关节会让控制器内部目标和仿真状态不一致。当前设计暂停仿真并
重置任务模型，但机器人通过同一个 FollowJointTrajectory controller 回 home，再检查位置误差和
速度，最后恢复场景。

### 18. 为什么“没有检测到碰撞”不一定代表成功？

回答要点：如果 Gazebo collision monitor 没接好，collision_count=0 只是没有数据。后端暴露
collision_state_known；未知时 FSM 返回 collision_state_unavailable，即使移动和放置都完成也
不报 task_success。

### 19. task_success 的完整判据是什么？

回答要点：已经 release；XY 误差 ≤ 40 mm；中心高度误差 ≤ 15 mm；方块线速度 ≤ 0.05 m/s；
无无效机器人或携物碰撞；Gazebo 下碰撞观测可用。瞬时落在目标上但仍运动、未释放或通过穿透
得到的结果都不能算成功。

### 20. MoveIt mock home 成功证明了什么，又没有证明什么？

回答要点：最终 MoveIt smoke、5+5 和 50+50 证明 robot_description/SRDF、KDL/OMPL、
move_group、mock controller、bridge、PlanningScene 与 Action 链路可用；但 100 条都明确
`not_executed`，所以不证明 Gazebo 执行、抓取或 task_success。Gazebo 证据来自另一组完整
episode，不能混用。

### 21. 如何保证批量对照公平？

回答要点：共享相同的 scene 和 task_seed，记录 backend、simulator、planner_id 与 grasp_mode；
先每组 5 次 smoke，再跑固定任务集。统计均值/中位数/P95、路径、放置/跟踪误差和失败类型。
但不同碰撞引擎耗时只做各自回归，不做不严谨的横向快慢结论。

### 22. 当前最大的工程风险是什么？

回答要点：assisted 运行期已经验收，当前风险转为三项：DetachableJoint 结果不能外推为
physics/真机；MoveIt 2.5.9 的批次关闭析构不干净；HSV 方法依赖已知颜色、尺寸和顶置相机，
不能外推为开放场景识别。physics 仍必须 fail-closed，不能因为 assisted 达到较高成功率就
跳过动力学验证。

### 23. 下一步你会按什么顺序推进？

回答要点：先完成 nominal 和合成扰动下的位置/yaw、候选可行率与抓放结果评测，按
`orientation_ambiguous`、检测失败和规划失败分别定位；若岗位需要，再保持
`ScenePoseProvider` 接口不变，用 YOLO 替换颜色分割。仍不在 assisted 动力学边界上直接
叠加强化学习。

### 24. 为什么当前不加入强化学习？

回答要点：当前 Gazebo 依赖 assisted 固定约束，质量/惯量和接触参数也没有达到纯物理抓取
的验证标准。RL 策略可能利用人工约束或仿真误差，得不到可迁移的接触控制证据。岗位更关注
视觉与机械臂，因此优先做 RGB-D 位姿估计闭环。

### 25. 如何从一个深度像素得到 world 坐标？

回答要点：CameraInfo 给出 `fx, fy, cx, cy`，深度为 z 时，光学坐标为
`x=(u-cx)z/fx, y=(v-cy)z/fy, z=z`。先对连通域内有效深度做 median/MAD 过滤，再用
`world <- overhead_camera_optical_frame` 的 TF 变换三维点。已知顶面位置最后换算成物体中心 z。

### 26. 为什么服务必须等待 reset 后的新图像？

回答要点：相机持续发布，若服务直接返回缓存，episode N 可能拿到 episode N-1 的方块位置。
节点记录同步帧 generation，服务到达后只接受 generation 增加的帧；超时返回明确的 RGB、depth
或 perception timeout，不允许用旧数据继续规划。

### 27. 如何证明视觉模式没有偷偷使用真值？

回答要点：TaskSpec 真值只传给 reset 和最终误差计算；pre-grasp、transfer 目标以及 MoveIt
cube/obstacle PlanningScene 更新都读取 `SceneEstimate`。测试用故意偏移的视觉估计，断言 backend
收到偏移目标；再让 provider 失败，断言状态机在任何运动调用前结束。

### 28. 为什么先用 HSV，而不是直接训练 YOLO？

回答要点：当前目标是先打通可解释的视觉—规划—执行接口。HSV 在固定仿真颜色下 CPU 可运行，
误差来源和失败分类清楚；检测器被封装在 `EstimateScene` / `ScenePoseProvider` 后面，后续替换 YOLO
不需要改 FSM、MoveIt 后端或结果 schema。不能把这套 HSV 结果描述成开放场景泛化能力。

### 29. 如何从 RGB-D 点云估计矩形工件 yaw？

回答要点：先把红色连通域的有效深度像素反投影到相机坐标，再通过 TF2 变换到 world；取
world-XY 点云，去中心后计算 2×2 协方差矩阵并做特征分解。最大特征值对应的特征向量是长轴，
`atan2(v_y, v_x)` 得到 yaw，最后归一化到 `[-π/2, π/2)`，因为矩形长轴没有正反方向。

### 30. 为什么用特征值比，而不是任何轮廓都返回 yaw？

回答要点：正方形或近圆目标的两个主方向方差接近，主轴会被像素和深度噪声任意翻转。用
`lambda_max/lambda_min` 衡量方向可观测性，低于 1.25 返回 `orientation_ambiguous`，让 episode
明确失败，而不是给 MoveIt 一个看似合法但没有物理意义的方向。

### 31. 为什么矩形工件生成两个相差 π 的候选？

回答要点：平行夹爪绕接近方向旋转 π 后具有相同几何抓取效果，但 Panda 当前关节状态、障碍物
和关节限位会使两条到达路径不同。两个候选都做 planning-only，保留可规划且无碰撞的候选，
再按预抓取关节路径长度选择；同分按候选序号，保证复现。

### 32. 如何证明候选筛选没有让机械臂先试着动？

回答要点：后端提供独立 `check_pose_feasibility`。MoveIt 复用 `PlanExecuteMotion.action`，但强制
`execute=false`；PyBullet 只做 IK、关节限位和碰撞规划，不调用轨迹执行。测试断言 Action goal
的 execute 字段为 false，并检查筛选前后关节状态不变。

### 33. 合成传感器扰动的意义和边界是什么？

回答要点：low_light、RGB 高斯噪声、深度 dropout 及 combined 由 episode id 和 task seed
确定性生成，目的是检查感知链路的退化趋势和失败分类，不改变任务真值。它们不是相机标定、
真实域随机化或 sim-to-real 证据，因此只能报告为合成扰动鲁棒性实验。

## 面试前自查清单

- 能画出统一 FSM、两个 backend 和 MoveIt/Gazebo 的数据流。
- 能解释 9 个可运动关节与 7 个 arm joint 的区别。
- 能手推 PyBullet 中 `panda_grasptarget-to-cube` 相对变换如何用于预测携物 pose，并说明
  MoveIt attach 使用 `panda_hand`。
- 能解释刚性组件内部接触过滤为什么不是“关闭自碰撞”。
- 能说明 MoveIt Action 每个 goal/result 字段的用途。
- 能区分 planning-only、mock execution、Gazebo execution 和 task_success。
- 能解释 Gazebo/PlanningScene attach 的顺序与回滚。
- 能解释 RGB-D 同步、反投影、TF2 变换，以及视觉模式为何没有真值泄漏。
- 能手推二维 PCA yaw、解释特征值比和 π 对称，并说明候选筛选为何是 planning-only。
- 能主动说出当前未交付的是 physics、真机、通用 6D 感知和干净 teardown。
- 能从冻结 manifest 找到历史数据，并保证新结果写入 results/runs。
- 能根据真实失败类型提出下一项实验，而不是先调成功率口径。
