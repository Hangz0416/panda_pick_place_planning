# 简历表述与 V2 模拟面试提纲

这份文档只使用当前已经实现、已经运行验证的事实。最重要的边界是：

- PyBullet V1 的 200 episode 原始 JSONL/CSV 与汇总文件由 manifest SHA256 和回归测试冻结；
  PyBullet V2 的 200 episode 也是本轮真实运行、可复查的结果；
- seed 42 的 clear/obstacle 单次 PyBullet Demo 均有独立 EpisodeResult 文件；
- V2 已完成多包架构、统一后端、类型化接口、MoveIt bridge/config 和 Gazebo bringup 代码；
- 最终四包 build 成功，colcon 77/77；Gazebo 静态/纯逻辑测试 44/44（仍非 runtime），
  flake8 67 files；
- MoveIt 审计前 enhanced smoke 与 planning-only 5+5 曾 10/10，但审计后因 DDS 无法启动而
  没有最终复验，不能将旧数据当作正式 5+5/50+50；
- Gazebo runtime、完整抓放、批量成功率、physics 抓取和视频均为 NOT YET VERIFIED。

面试时主动说清验证边界，比把“写了配置”说成“系统跑通”更可信。

## 当前可直接使用的简历项目经历

**Panda 双后端抓取放置规划与评测系统｜个人项目**

技术栈：ROS 2 Humble、Python、C++、PyBullet、MoveIt 2、OMPL、ros2_control

- 基于 ROS 2 与 PyBullet 搭建 Franka Panda 抓取放置闭环，实现任务生成、逆运动学、关节限位、
  自碰撞/环境碰撞、Linear 与双向 RRT-Connect、状态机执行及 JSONL/CSV/JSON 评测；将
  PyBullet V1 的 200 episode 原始 JSONL/CSV 与汇总文件冻结为 v0.1-pybullet-baseline，
  通过 manifest SHA256 回归防止基线被静默修改。
- 在固定主 seed 42 的 PyBullet V2 200 episode 实测中，clear 场景 Linear/RRT-Connect 均
  50/50 成功，平均规划耗时 1.750/1.922 ms；obstacle Linear 对 50/50 条受阻直线路径返回
  `planning_collision`，RRT-Connect 50/50 成功，平均规划耗时 33.157 ms、平均关节路径长度
  1.312717 rad；全部 200 episode 的执行碰撞数为 0。
- 对 V1/V2 raw 结果做 `(scene, task_seed)` 配对审计，coverage 为 100/100、missing 均为空，
  成功/失败分类无漂移；耗时与路径只按各自版本报告，不宣称数值相同。
- 抽象 ManipulationBackend 并保留唯一 PickPlaceRunner FSM，使 PyBulletBackend 与
  MoveItBackend 共享 RESET、PRE_GRASP、GRASP、LIFT、TRANSFER、PLACE、RETREAT 流程；
  将 ROS2 在线通信由 String + JSON 迁移为自定义消息、服务与 PlanExecuteMotion Action。
- 完善 PyBullet 携带物体碰撞：根据 `panda_grasptarget（末端）—方块` 相对变换预测候选
  状态下的方块位姿，
  检查其与桌面/障碍物的接触及 10 mm 安全距离，并区分规划阶段与执行阶段的携物碰撞失败。
- 使用 C++ MoveGroupInterface 与 PlanningSceneInterface 实现 MoveIt 规划/执行 bridge，
  支持 named/pose target、世界碰撞物、ACM 与 attach/detach 同步；实现共享 Xacro、Gazebo
  combined launch、readiness gate、typed reset/碰撞观测。MoveIt 最终批量回归和 Gazebo
  runtime 均待环境恢复后运行，不在简历中声称闭环成功。

这版简历没有写“Gazebo 闭环完成”“完成物理抓取”“Gazebo 成功率达到 95%”或“生成 Demo
视频”，因为目前没有对应运行证据。

## Gazebo 跑通后才能替换的简历句式

下面是占位模板，不是当前成果。只有逐 episode 文件、汇总文件和失败案例都保存后，才能把
方括号替换为真实值：

> 接入 Gazebo Fortress 与 gz_ros2_control，实现确定性 episode reset、约束门控辅助抓取和
> PlanningScene/Gazebo 双向同步；在固定任务集上完成 50 clear + 50 obstacle episode，
> 成功率分别为 [真实 clear 结果] 与 [真实 obstacle 结果]，主要失败为 [真实失败分布]。

physics 模式也必须单列：

> 移除 DetachableJoint 后，仅依靠夹爪碰撞与摩擦完成物理抓取；每个场景运行 [真实 episode
> 数]，记录双指接触、滑落、相对位姿漂移和释放稳定时间。

不能把 assisted 的固定约束结果填进 physics 句式。

## 60 秒项目介绍

这个项目最初是一个 PyBullet 中的 Panda 抓取放置与路径规划系统。我没有停留在“机械臂能动”
的 Demo，而是先把任务生成、IK、碰撞检测、Linear/RRT-Connect、状态机和固定种子评测做成
闭环，并冻结了 200 episode 的 V1 基线。

V2 的核心改动是把状态机与执行引擎分开。我定义了 ManipulationBackend，PyBullet 和 MoveIt
都实现同一组 reset、move、gripper、attach/detach 与 object_state 接口，因此只保留一套
PickPlaceRunner。ROS2 在线链路也从 JSON 字符串迁移成自定义消息和 Action。MoveIt 部分用
C++ MoveGroupInterface 做规划与可选执行，用 PlanningSceneInterface 管理桌面、方块、障碍物
和 AttachedCollisionObject；Python 继续负责抓放阶段与 EpisodeResult。

我还补上了 PyBullet 携带方块后的碰撞检查：保存 `panda_grasptarget`（末端）到方块的相对
变换，在每个候选关节状态预测方块位姿，分别判断机器人碰撞和携物碰撞。最终四包 build 与
77 项 colcon 测试通过，Gazebo
代码的 44/44 静态/纯逻辑测试通过，但这仍不是 Gazebo runtime 证据。MoveIt 在审计前跑过
enhanced smoke 和 5+5 planning-only，
但最终审计后没有重新启动 DDS，因此我不会把旧 10/10 当作正式结果。Gazebo 还缺
`ignition-msgs8`、`ros_gz`、`gz_ros2_control`，所以 runtime、50+50、physics 与视频都不写成
已完成成果。

## 代码流程

### 1. 任务生成与类型化通信

1. packages/panda_pick_place_planning/.../core/tasks.py 根据 task seed 生成物体和目标位置。
2. TaskSpec 记录 planner、planner seed、scene、backend、simulator 和 grasp_mode。
3. task_generator_node.py 将 TaskSpec 转成 ManipulationTask，发布 /manipulation_task。
4. manipulation_node.py 校验 task.backend 与节点 backend 一致，再交给共享 PickPlaceRunner。
5. evaluator_node.py 订阅类型化 EpisodeResult，转换为 Python schema 后追加写入 JSONL。

同一 scene 与 task_seed 下，不同规划器共享相同任务几何；planner seed 单独记录。

### 2. 统一 FSM

PickPlaceRunner.run() 的状态顺序是：

~~~text
RESET
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

Python FSM 使用 MultiThreadedExecutor 和 ReentrantCallbackGroup。同步状态机等待 Action/Service
Future 时只等待事件，由其他 executor 线程处理回调，避免在已经加入 executor 的 Node 上再次
spin 导致死锁。

### 5. MoveIt mock 分支的含义

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

审计修复前，enhanced clear/obstacle smoke 与 5 clear + 5 obstacle planning-only 曾 10/10。
最终审计又收紧了 reset/readback、物理状态 latch 和 typed metrics；随后本轮 DDS 无法启动，
所以旧结果只能作为历史链路证据，不能写成最终代码的正式 5+5 或 50+50。

### 6. Gazebo assisted 分支的设计流程

1. `panda_moveit_gz.launch.py` 是已有单入口；它复用共享 Panda Xacro 的 `hardware_type:=gz`，
   只创建一份 robot_state_publisher/controller manager，并组合 move_group、bridge 与应用节点。
2. gz_ros2_control 暴露 7 轴 trajectory controller、gripper controller 与 joint state。
3. readiness gate 等待 `/clock`、关节状态、方块 odometry、三个 active controller 和必要
   service/action 全部就绪，才允许启动共享 FSM。
4. typed ResetEpisode 暂停世界、请求 detach、按任务 seed reset model、用轨迹控制器回 home、
   校验关节位置/速度、再次设置方块/目标/障碍物后恢复。
5. 抓取前先保持 60 mm 总开度，避免 50 mm 方块与手指互穿。
6. AssistedGraspManager 同时检查手—物相对位置、姿态、距离与夹爪宽度。
7. Gazebo 确认 DetachableJoint attached 后，MoveItBackend 才添加
   AttachedCollisionObject；失败则回滚 detach。
8. 类型化碰撞服务只有在机器人接触、方块接触和方块位姿都可用后才允许成功判定。
9. 释放时先 detach Gazebo，再 detach PlanningScene；等待方块稳定后读取实际位姿并重新加入
   MoveIt world，最后规划 retreat。

这是代码设计，不是当前已经跑过的完整序列。

### 7. physics 分支为何默认 fail-closed

physics 模式不加载 DetachableJoint，也不允许 fixed-joint attach。双指持续接触、抓取后相对
位姿、平移/旋转漂移、滑落 latch、释放和稳定时间的 monitor 与 typed 结果字段已经实现，
MoveItBackend 也不会伪造 attach 成功。

当前共享 Panda 描述没有经过审计、可用于可信动力学的质量/惯量模型。因此 launch 默认
`physics_model_validated:=false` 并拒绝 physics，monitor 返回
`physics_model_unvalidated`。下一步不是补一段接触逻辑，而是补充/审计惯量，完成 Gazebo
runtime 验证后才允许开启 physics，并与 assisted 分组报告。

## 模拟面试问题

建议先口头回答，再看回答要点。回答时始终区分“代码已实现”“单元测试已通过”和“完整运行已验证”。

### 1. 这个项目现在到底完成到什么程度？

回答要点：PyBullet V1 基线冻结，PyBullet V2 的 200 episode 和 seed 42 双场景 Demo 已真实
验证；最终四包 build、colcon 77/77、Gazebo 静态测试 44/44（仍非 runtime）、flake8 67
files。MoveIt 审计前
smoke/5+5 曾通过，但审计后 DDS 未恢复，不能当最终批量结果。Gazebo runtime、50+50、physics
与视频尚未验证。

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
同时保留 V1 JSON 向后读取，避免历史结果失效。

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
总开度约 60 mm，围绕 50 mm cube 留出间隙；位姿、姿态、距离和开度门控全部通过后 attach，
再执行闭合指令。

### 16. assisted 和 physics 有什么本质区别？

回答要点：assisted 用经过门控的固定约束提供稳定基线；physics 不允许固定约束，只依靠接触、
摩擦和 gripper action。两者必须用 grasp_mode 分组，physics 还要记录双指接触、滑落和漂移。
这些 observation/latch 已实现，但共享 Panda 惯量未审计，physics 默认 fail-closed 且未运行；
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

回答要点：审计前的 MoveIt smoke/5+5 证明当时 robot_description/SRDF、KDL/OMPL、
move_group、mock controller、bridge、PlanningScene 与 Action 链路可用。最终审计后未能重启
DDS，所以它不证明最终代码的正式回归，更不证明 Gazebo 执行、抓取或 task_success。

### 21. 如何保证批量对照公平？

回答要点：共享相同的 scene 和 task_seed，记录 backend、simulator、planner_id 与 grasp_mode；
先每组 5 次 smoke，再跑固定任务集。统计均值/中位数/P95、路径、放置/跟踪误差和失败类型。
但不同碰撞引擎耗时只做各自回归，不做不严谨的横向快慢结论。

### 22. 当前最大的工程风险是什么？

回答要点：combined launch、readiness、typed reset/collision 和 physics monitor 已实现，但
Gazebo 缺 `ignition-msgs8`、`ros_gz`、`gz_ros2_control`，runtime 尚未验收；共享 Panda 惯量
也不可信，physics 必须 fail-closed。先恢复依赖和运行证据，再谈成功率和视频。

### 23. 下一步你会按什么顺序推进？

回答要点：恢复 DDS 并重跑最终代码 MoveIt smoke/5+5 → 安装缺失 Gazebo 依赖并 build → 用已有
combined launch 验证 readiness/controller/10 次 reset → seed 42 assisted 双场景 → 每组 5 次
再 50+50 → 补充并审计 Panda 惯量 → 最后开启并单列 physics。

### 24. 为什么当前不加入强化学习？

回答要点：全局规划、控制执行、reset、观测和成功判据尚未在 Gazebo 形成可信闭环，此时加入
RL 会把基础设施错误混入 reward 和训练不稳定。未来接入点限定在 PRE_GRASP → GRASP 的局部
阶段，不改动 MoveIt 全局规划与统一评测接口。

## 面试前自查清单

- 能画出统一 FSM、两个 backend 和 MoveIt/Gazebo 的数据流。
- 能解释 9 个可运动关节与 7 个 arm joint 的区别。
- 能手推 PyBullet 中 `panda_grasptarget-to-cube` 相对变换如何用于预测携物 pose，并说明
  MoveIt attach 使用 `panda_hand`。
- 能解释刚性组件内部接触过滤为什么不是“关闭自碰撞”。
- 能说明 MoveIt Action 每个 goal/result 字段的用途。
- 能区分 planning-only、mock execution、Gazebo execution 和 task_success。
- 能解释 Gazebo/PlanningScene attach 的顺序与回滚。
- 能主动说出当前 NOT YET VERIFIED 的项目，而不是等面试官追问。
- 能从冻结 manifest 找到 V1 数据，并保证新结果写入新目录。
- 能根据真实失败类型提出下一项实验，而不是先调成功率口径。
