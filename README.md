# Panda 抓取放置规划与评测系统 V2

这是一个面向机械臂运动规划与具身智能工程实践的 Franka Panda 仿真项目。V2 在已经跑通的
PyBullet V1 基线上，引入统一状态机、双后端接口、MoveIt 2 规划桥接和现代 Gazebo
Fortress bringup。

当前目标是把“任务生成 → 规划 → 执行 → 抓取/释放 → 成功判定 → 评测”做成可复现的工程闭环，
而不是堆叠尚未验证的功能。本阶段不包含强化学习、相机感知、真机、MoveIt Task Constructor
或自定义 MoveIt planner 插件。

## 先看结论：证据边界

- **PyBullet V1 基线仍保持冻结。** 标签为 `v0.1-pybullet-baseline`；200 episode 的原始
  JSONL、CSV 和汇总 JSON 已复制到基线目录，并由 manifest 中的 SHA256 与回归测试防止
  静默改写。V2 输出不会覆盖这些文件。
- **最终 PyBullet V2 已真实运行。** 固定主 seed 42、每组 50 个任务，共 200 episode；
  clear 的 Linear/RRT-Connect 与 obstacle 的 RRT-Connect 均 50/50 成功，obstacle Linear
  对 50/50 条受阻直线路径返回 `planning_collision`，全部 200 episode 的执行碰撞数为 0。
  seed 42 的 clear 和 obstacle 单次 Demo 也均成功。
- **全部 5 个 ROS2 包已 clean build 并通过测试。** `rosdep check` 无未满足依赖；独立
  `/tmp` build/install/log 目录中的 `colcon test-result` 为 165 tests、0 errors、0 failures、
  0 skipped，包含真实 Xacro→SDF 检查和 PyBullet 冒烟测试。
- **MoveIt mock 最终规划回归已完成。** 最终代码的 clear/obstacle bridge smoke、5+5 快检
  均通过；固定 seed 42 的 50 clear + 50 obstacle 为 100/100 规划成功、0 PlanningScene
  同步错误。该组严格是 planning-only：`task_success=false`、`failure_type=not_executed`，
  不把未执行的轨迹描述成抓取成功。
- **Gazebo runtime 仍为 NOT VERIFIED。** Fortress、ros_gz 与 gz_ros2_control 依赖现已安装，
  Gazebo 包也已进入完整 build/test；但控制器 runtime、10 次 reset、assisted 完整 episode、
  50+50 和视频尚未验收，因此当前没有可引用的 Gazebo 成功率。

## 系统架构

~~~text
ManipulationTask（ROS2 类型化消息）
             │
             ▼
      PickPlaceRunner（唯一 FSM）
             │
      ManipulationBackend
       ├──────────────────────────────┐
       ▼                              ▼
PyBulletBackend                 MoveItBackend
Linear / RRT-Connect            PlanExecuteMotion Action
PyBullet 规划与执行                    │
       │                       C++ MoveIt bridge
       │                       MoveGroupInterface
       │                       PlanningSceneInterface
       │                              │
       │                       ┌──────┴────────┐
       │                       ▼               ▼
       │                  mock ros2_control   Gazebo Fortress
       │                                      gz_ros2_control
       └──────────── EpisodeResult ─────────────┘
                            │
                      JSONL / CSV / JSON
~~~

两个后端共用任务结构、阶段状态机、结果结构和成功判据。Python 负责 FSM、任务生成与评测；
C++ 只负责 MoveIt 规划、可选执行和 PlanningScene 修改。PyBullet 中的 Linear/RRT-Connect
仍是教学与算法基线，不宣称与 OMPL 的 RRTConnect 完全等价。

统一状态机为：

~~~text
RESET → PRE_GRASP → GRASP → LIFT → TRANSFER → PLACE → RETREAT → DONE
                                                             └→ FAILED
~~~

ManipulationBackend 合约包括 reset、move_to_pose、move_home、set_gripper、
attach_object、detach_object、object_state、wait_for_object_settle 和 shutdown。
因此切换后端不会复制一套抓取放置流程。

## 多包工作区

仓库是一个包含 5 个 ROS2 package 的源码仓库：

| Package | 职责 |
| --- | --- |
| panda_pick_place_interfaces | ManipulationTask、EpisodeResult、PlanExecuteMotion，以及场景、attach 和 reset 服务 |
| panda_pick_place_planning | 统一 FSM、PyBullet/MoveIt Python 后端、Linear/RRT-Connect、任务和评测节点 |
| panda_moveit_bridge | 基于 MoveGroupInterface 与 PlanningSceneInterface 的 C++ Action/Service bridge |
| panda_moveit_config | Panda 统一 Xacro、SRDF、KDL、OMPL、关节限位、mock ros2_control 与 RViz 配置 |
| panda_gz_bringup | Fortress world、gz_ros2_control、topic/service bridge、确定性 reset 与辅助抓取门控 |

目录结构：

~~~text
panda_pick_place_planning/
├── baselines/v0.1-pybullet-baseline/
├── docs/
├── packages/
│   ├── panda_pick_place_interfaces/
│   ├── panda_pick_place_planning/
│   ├── panda_moveit_bridge/
│   ├── panda_moveit_config/
│   └── panda_gz_bringup/
├── results/                         # 本地运行结果，不覆盖冻结基线
└── README.md
~~~

## V2 已实现内容

### 1. 类型化 ROS2 接口

在线通信已经从 String + JSON 迁移为 ROS2 类型：

- ManipulationTask：episode/task seed、backend、simulator、planner ID、scene、grasp mode、
  物体位置和目标位置。
- EpisodeResult 与 StageMetric：规划/执行/任务成功、attach/release、碰撞、逐阶段耗时与路径、
  跟踪/放置误差，以及双指接触、滑移、漂移和释放稳定时间。未执行的物体指标写为 JSON
  `null`，ROS 浮点边界使用 NaN，不用 0 冒充观测值。
- PlanExecuteMotion.action：named/pose target、planner、规划预算、速度/加速度缩放和是否执行；
  返回 MoveIt error code、失败类型、规划/执行时间、路径长度和轨迹点数。
- UpsertWorldObject、RemoveWorldObject、SetAttachedObject 与 ResetEpisode 服务。

JSONL、CSV 和汇总 JSON 只作为离线实验文件，旧 V1 JSON 仍可由 V2 schema 向后读取。

### 2. PyBullet 携带物体碰撞

辅助抓取建立约束时，PyBullet 后端保存 `panda_grasptarget`（末端）到方块的相对变换。
规划器检查候选关节状态时：

1. 临时设置 7 个机械臂关节；
2. 用末端正运动学与保存的相对变换预测方块位姿；
3. 检查方块与桌面、障碍物的接触和安全距离；
4. 恢复机械臂与方块的真实仿真状态。

规划阶段与执行阶段分别报告 planning_carried_object_collision 和
execution_carried_object_collision。桌面只允许抓起瞬间配置的轻微接触容差，不能把深度穿透
过滤掉。

### 3. MoveIt 2 规划后端

MoveIt 配置与 Gazebo 共用 `packages/panda_moveit_config/config/panda.urdf.xacro`，
坐标链统一为 `world → panda_link0 → panda_hand → panda_grasptarget`。位姿规划目标使用
`panda_grasptarget`，物体 attach 仍使用有碰撞几何的 `panda_hand`。当前默认配置为：

- panda_arm、hand 和末端执行器 SRDF；
- KDL 运动学；
- OMPL RRTConnectkConfigDefault；
- 3 秒规划预算；
- 速度和加速度缩放均为 0.2；
- 10 mm robot padding；
- mock 与 gz 两种 ros2_control hardware macro。

C++ bridge 串行化 MoveGroupInterface 与 PlanningScene 修改，支持 named/pose target、可选
execute、世界碰撞物增删、Allowed Collision Matrix 更新及物体 attach/detach。attach 后会等待
PlanningScene 同步；若同步失败会请求 detach 回滚，避免 Gazebo 与 MoveIt 各自认为不同状态。
OMPL 的随机 `PLANNING_FAILED` 或 MoveIt 最终路径校验返回的 `INVALID_MOTION_PLAN` 最多重规划
3 次，所有尝试共享原 3 秒预算并累计规划耗时；目标碰撞、IK 失败和场景错误不会被重试掩盖。

### 4. Gazebo Fortress bringup

Gazebo 代码只使用现代 Gazebo 的 ros_gz、gz_ros2_control/GazeboSimSystem 和 Fortress
DetachableJoint ABI，不使用 Gazebo Classic 或 gazebo_ros2_control。

已提供：

- 1 ms 物理步进和 1000 Hz real-time update 配置；
- joint_state_broadcaster、7 轴 joint trajectory controller 和 hand gripper controller，
  ros2_control 更新率 100 Hz；
- 与 PyBullet 对应的桌面、50 mm 方块、目标标记和障碍物；
- /clock、方块位姿、接触、世界控制和实体 set-pose bridge；
- ResetEpisode：暂停、detach、按 seed reset、控制器回 home、校验关节位置/速度、再次设置场景后恢复；
- assisted attach 门控：末端相对位姿、姿态、手物距离和夹爪开度全部通过后，才允许
  DetachableJoint attach；
- 类型化执行碰撞 monitor：只有机器人接触、方块接触和方块位姿均到达时才报告 available；
- `panda_moveit_gz.launch.py` 单入口，以及等待 `/clock`、关节/方块状态、三个控制器、必要
  service/action 全部就绪后才启动应用的 readiness gate。

这些是已经实现并通过静态/纯逻辑测试的代码，不等于 Gazebo 完整运行已经验收。

### 5. assisted 与 physics 严格分开

- assisted：先保持不会与 50 mm 方块互穿的预 attach 开度；Gazebo 确认 DetachableJoint
  attach 后，MoveIt 才添加 AttachedCollisionObject。PlanningScene 同步失败时回滚 Gazebo
  detach。
- physics：不使用 DetachableJoint；物理 monitor 已实现左右指持续接触、抓取后相对位姿、
  平移/旋转漂移、滑落、释放与稳定时间的类型化观测，MoveItBackend 也不会伪造 attach。

共享 Panda 描述现已包含 Franka 上游公开的辨识惯量，并通过质量为正、惯性矩阵正定和
Xacro→SDF frame/joint 检查；来源 commit 与许可证记录在 `THIRD_PARTY_NOTICES.md`。上游明确
说明这些不是官方 Panda 惯量，因此它们只用于 assisted 控制器运行，
`physics_model_validated:=false` 仍保持 fail-closed，不能据此宣称物理抓取可信。

## 真实的 PyBullet V2 固定种子结果

运行环境：2026-07-17，Ubuntu 22.04，Python 3.10.12，Intel Core i5-14600KF，
PyBullet DIRECT；每组 50 个任务，主 seed 为 42。

| 场景 / 规划器 | 任务/规划成功率 | 平均规划耗时 | 平均路径长度 | 平均执行耗时 | 失败类型 |
| --- | ---: | ---: | ---: | ---: | --- |
| clear / Linear | 50/50 (100%) | 1.750 ms | 0.778358 rad | 0.085929 s | 无 |
| clear / RRT-Connect | 50/50 (100%) | 1.922 ms | 0.778358 rad | 0.079812 s | 无 |
| obstacle / Linear | 0/50 (0%) | 0.560 ms | N/A（无成功路径） | 0.052244 s | `planning_collision`: 50 |
| obstacle / RRT-Connect | 50/50 (100%) | 33.157 ms | 1.312717 rad | 0.093104 s | 无 |

obstacle / Linear 的 0% 只表示这组任务刻意阻断了关节空间直线路径，不代表 Linear 在所有
障碍场景都失败。clear 中 RRT-Connect 会先检查直连边，因此得到相同路径长度。四组共
200 episode，执行碰撞 episode 为 0；seed 42 的 clear/obstacle 单次 Demo 均成功。
两条独立 Demo 结果分别保存在
`results/v2/demos/pybullet_clear_linear_seed42.jsonl` 与
`results/v2/demos/pybullet_obstacle_rrt_connect_seed42.jsonl`，对应 SHA256 记录在同目录
`manifest.json`，不再只依赖批量结果佐证。

V1 的 200 episode 原始 `benchmark_results.jsonl`、`benchmark_results.csv`、汇总 JSON 与
SHA256 manifest 位于 `baselines/v0.1-pybullet-baseline/`；冻结回归会逐文件复算哈希。V2
数据与 V1 分开保存，不能用新结果覆盖冻结基线，也不能把两版耗时混成同一组实验。

V1/V2 原始结果还做了 `(scene, task_seed)` 配对校验：两侧 coverage 都是 100，
`paired=true`，双方 missing 集合均为空；成功/失败分类完全不漂移。报告位于
`results/v2/reports/v1_v2_pybullet_compare/`。这只说明任务覆盖和分类一致，不表示两版规划
耗时或路径长度数值相同。最终 V2 原始结果/汇总与单版本报告分别位于
`results/v2/pybullet_shared_geometry_200_final/` 和 `results/v2/reports/pybullet_final/`。

## MoveIt mock 最终 planning-only 结果

运行环境与包版本记录在 `results/v2/moveit_mock_final/environment.json`。固定 seed 42，
RRTConnectkConfigDefault、3 秒总规划预算、速度/加速度缩放 0.2；每个 episode 从 mock 当前
home 状态独立规划 home、pre-grasp、transfer 和 place 请求，不执行轨迹、夹爪或物体运动。

| 场景 | 规划成功率 | 规划耗时均值 / 中位数 / P95 | 平均关节路径长度 | 平均轨迹点 |
| --- | ---: | ---: | ---: | ---: |
| clear | 50/50 (100%) | 120.088 / 120.919 / 131.707 ms | 7.868903 rad | 152.04 |
| obstacle | 50/50 (100%) | 121.211 / 120.670 / 133.843 ms | 8.149984 rad | 164.70 |

两组共 100/100 规划成功、0 PlanningScene 同步错误。表中规划耗时采用 Action 墙钟时间；
若有限重规划触发，会把同一 3 秒预算内的所有尝试计入，而不是只统计最终成功路径。第一次
未加入该机制的正式运行真实得到 99/100，失败 episode 和汇总保存在
`results/v2/moveit_mock_failed_attempt_20260717_99_of_100/`，没有被覆盖或删除。

这不是抓取成功率：全部 100 条都保持 `task_success=false`、`execution_success=false`、
`failure_type=not_executed`，跟踪误差、XY/Z 放置误差和物体速度均为 `null`。原始 JSONL、
summary、环境快照、SVG 与 SHA256 manifest 位于 `results/v2/moveit_mock_final/`。

## V2 验证矩阵

| 验证项 | 当前状态 | 可引用证据 |
| --- | --- | --- |
| V1 冻结基线 | VERIFIED | 200 条 raw JSONL/CSV、汇总 JSON、SHA256 manifest 与冻结回归 |
| PyBullet V2 seed 42 Demo | VERIFIED | clear、obstacle 均真实成功 |
| PyBullet V2 200 episode | VERIFIED | 上表四组真实结果；0 collision episode |
| V1/V2 raw paired audit | VERIFIED | coverage 100/100、paired=true、missing 为空、分类无漂移 |
| 全部 5 包 clean build | VERIFIED | 独立 build/install/log 目录，`rosdep check` 通过 |
| 最终 colcon 测试 | VERIFIED | 165 tests；0 errors、0 failures、0 skipped |
| Xacro→SDF 模型审计 | VERIFIED | 7 arm、2 finger、hand/grasptarget；质量/惯量检查通过 |
| 最终 MoveIt enhanced smoke | VERIFIED | clear/obstacle、控制器、ACM、attach/detach 均通过 |
| 最终 MoveIt planning-only 快检 | VERIFIED | 5 clear + 5 obstacle，10/10 |
| 最终 MoveIt 50+50 | VERIFIED | 100/100 planning；0 scene sync error；不宣称 task success |
| Gazebo 包 build | VERIFIED | Fortress/ros_gz/gz_ros2_control 依赖已安装并进入 5 包 build |
| Gazebo runtime | NOT VERIFIED | 控制器、reset 与完整 episode 尚无 runtime 证据 |
| Gazebo reset、assisted 50+50 | NOT VERIFIED | 无 runtime EpisodeResult，不提供成功率 |
| physics 与视频 | OUT OF SCOPE | 辨识惯量只验收到 assisted；physics 保持 fail-closed；未录制视频 |

## 环境与依赖

锁定环境：

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- MoveIt 2 Humble
- Gazebo Fortress
- PyBullet 3.2.6 或更高的 3.x

安装基础工具和本阶段系统依赖：

~~~bash
sudo apt update
sudo apt install -y \
  python3-pip python3-colcon-common-extensions python3-rosdep \
  ros-humble-moveit \
  ros-humble-moveit-resources-panda-description \
  ros-humble-moveit-resources-panda-moveit-config \
  ros-humble-ros-gz ros-humble-ros-gz-interfaces \
  ros-humble-ros-gz-bridge ros-humble-ros-gz-sim \
  ros-humble-gz-ros2-control \
  libignition-msgs8-dev libignition-transport11-dev
~~~

当前工作机已经安装 `libignition-msgs8-dev`、`libignition-transport11-dev`、
`ros-gz-interfaces/bridge/sim` 和 `gz-ros2-control`；具体 dpkg 版本保存在最终
`environment.json`。`rosdep check --from-paths packages --ignore-src --rosdistro humble` 报告
全部系统依赖满足。

也可以让 rosdep 从 5 个 package 的 package.xml 补齐其余依赖：

~~~bash
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
cd ~/ros2_ws/src/panda_pick_place_planning
rosdep install --from-paths packages --ignore-src -r -y
python3 -m pip install --user -r packages/panda_pick_place_planning/requirements.txt
~~~

## 获取与构建

仓库必须位于 ROS2 工作区的 src 下。package 实际位于仓库的 packages 子目录：

~~~bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Hangz0416/panda_pick_place_planning.git

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/panda_pick_place_planning/packages
source install/setup.bash
~~~

复现本轮 5 包 clean build：

~~~bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/panda_pick_place_planning/packages
source install/setup.bash
~~~

每个新终端都要执行：

~~~bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
~~~

类型化 Python 测试会导入 ROS2 自动生成的 panda_pick_place_interfaces；因此不能在尚未构建
并 source 工作区时直接运行完整测试。

## 测试

V2 Python 测试：

~~~bash
cd ~/ros2_ws/src/panda_pick_place_planning/packages/panda_pick_place_planning
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 -m unittest discover -s test -v
~~~

Gazebo 纯逻辑与资产测试：

~~~bash
cd ~/ros2_ws/src/panda_pick_place_planning/packages/panda_gz_bringup
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 -m pytest test -q
~~~

全部 5 包的 colcon 测试：

~~~bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --return-code-on-test-failure
colcon test-result --verbose
~~~

最终 clean build 结果为 165 tests（0 errors、0 failures、0 skipped），5 个包均进入 build/test。
这仍不等于 Gazebo runtime 已验收；runtime 必须继续通过 readiness、controller、reset 和
完整 episode 门槛。

进入批量评测前，必须先保证对应依赖、build 和 test 全部通过，再为每组跑 5 次快速检查。

## 运行 PyBullet

单次 DIRECT 冒烟：

~~~bash
ros2 run panda_pick_place_planning run_demo --planner linear --scene clear --seed 42 --no-gui
~~~

需要保留独立证据时可追加
`--output results/v2/demos/pybullet_clear_linear_seed42.jsonl`；已有文件默认不会被覆盖，只有显式
传入 `--force` 才会重写。

障碍绕行：

~~~bash
ros2 run panda_pick_place_planning run_demo --planner rrt_connect --scene obstacle --seed 42
~~~

类型化 ROS2 三节点链路：

~~~bash
ros2 launch panda_pick_place_planning demo.launch.py backend:=pybullet planner:=rrt_connect scene:=obstacle seed:=42 grasp_mode:=assisted use_gui:=true
~~~

快速评测，每个 scene/planner 5 次，共 20 episode：

~~~bash
ros2 run panda_pick_place_planning run_benchmark --episodes 5 --seed 42 --scenes clear obstacle --planners linear rrt_connect --output-dir results/v2_pybullet_smoke
~~~

若要复跑 200 episode，必须使用新的输出目录，不能覆盖 V1：

~~~bash
ros2 run panda_pick_place_planning run_benchmark --episodes 50 --seed 42 --scenes clear obstacle --planners linear rrt_connect --output-dir results/v2_pybullet_recheck
~~~

## 运行 MoveIt mock

终端 A：

~~~bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch panda_moveit_config mock_demo.launch.py use_rviz:=true
~~~

终端 B 检查控制器：

~~~bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
~~~

位姿 Action 的末端链接必须使用 `panda_grasptarget`，例如 named home planning-only 请求：

~~~bash
ros2 action send_goal --feedback /plan_execute_motion panda_pick_place_interfaces/action/PlanExecuteMotion "{target_type: 1, named_target: home, end_effector_link: panda_grasptarget, planner_id: RRTConnectkConfigDefault, allowed_planning_time_sec: 3.0, max_velocity_scaling_factor: 0.2, max_acceleration_scaling_factor: 0.2, execute: false}"
~~~

最终 clear/obstacle bridge smoke、5+5 与 50+50 已通过。单个 planning-only 请求可用：

~~~bash
ros2 launch panda_pick_place_planning demo.launch.py backend:=moveit planner:=rrt_connect scene:=clear seed:=42 use_gui:=false use_gazebo:=false moveit_execute:=false moveit_command_gripper:=false
~~~

## 运行 Gazebo Fortress

依赖和 5 包 build/test 已通过后，可先运行独立 bringup：

~~~bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch panda_gz_bringup panda_gz.launch.py scene:=clear grasp_mode:=assisted headless:=false auto_reset:=true
~~~

仓库已经提供单一的 MoveIt + Gazebo 集成入口；`start_application:=true` 时，readiness gate
只有在 `/clock`、关节/方块状态、三个控制器以及必要 service/action 全部就绪后才启动 FSM：

~~~bash
ros2 launch panda_gz_bringup panda_moveit_gz.launch.py \
  scene:=clear seed:=42 grasp_mode:=assisted \
  headless:=false use_rviz:=true start_application:=true
~~~

检查基础运行状态：

~~~bash
ros2 topic echo /clock --once
ros2 control list_controllers
ros2 service list | grep panda_gz
~~~

**截至当前版本，这些 Gazebo runtime 命令尚未完成验收。** 单入口、typed reset、typed
collision query、120 秒 readiness timeout 与 batch wrapper 已实现并通过代码测试，但控制器、
连续 reset 和完整 episode 仍需真实运行。不能把 build/launch 文件存在写成 Gazebo 已跑通。

## 成功判据与指标

统一 task_success 要求同时满足：

- 方块已经释放；
- 最终 XY 误差不超过 0.04 m；
- 方块中心高度误差不超过 0.015 m；
- 方块线速度不超过 0.05 m/s；
- 执行中没有无效机器人或携带物体碰撞；
- Gazebo 模式必须有可用的执行碰撞观测，不能把“没有碰撞数据”当成“没有碰撞”。

EpisodeResult V2 记录：

- backend、simulator、planner_id、grasp_mode 和 seed；
- 每个 FSM 状态的规划/执行时间；
- 规划时间、关节路径长度、航点/轨迹点；
- 最大关节跟踪误差；
- 最终 XY/Z 放置误差和物体速度；
- attach、release、规划碰撞、执行碰撞；
- failure_state、failure_type 和 PlanningScene/Gazebo 同步状态；
- 左/右指与双指接触、滑移、平移/旋转漂移、释放稳定时间和最后碰撞对。

汇总输出包含成功率、失败计数、规划时间均值/中位数/P95、路径长度、放置误差和跟踪误差。
不同后端共享相同的 scene 与 task_seed，但不能把不同碰撞引擎、控制器和计时边界下的绝对耗时
直接解释为算法优劣。

主要失败类型：

| 类别 | 例子 |
| --- | --- |
| IK/目标 | ik_failed、no_ik_solution、target_rejected、invalid_goal_constraints |
| 规划 | planning_collision、planning_carried_object_collision、planning_timeout、start_state_collision、goal_collision |
| 执行 | execution_collision、execution_carried_object_collision、execution_control_failed、execution_timeout |
| 抓取/释放 | grasp_failed、release_failed、gazebo_attach_failed、gazebo_detach_failed |
| 放置 | place_failed |
| 同步 | planning_scene_upsert_failed、planning_scene_detach_failed、gazebo_reset_pose_mismatch、environment_changed |
| 观测 | collision_state_unavailable |
| 边界异常 | moveit_action_unavailable、moveit_client_error、bridge_exception、runtime_error |

目标门槛不是当前实验结果：后续 Gazebo assisted 评测希望 clear ≥ 95%、obstacle ≥ 90%、
碰撞 episode ≤ 2%，且无未分类 runtime_error。若未达到，应保留真实结果并按失败状态修复，
不能缩小任务范围来掩盖失败。

## 下一步验收顺序

1. 启动 combined launch，验证 `/clock`、readiness gate、三个 active controller 和全部
   reset/attach/detach/碰撞服务。
2. 对同一 `(scene, seed=42)` 连续 reset 10 次，保存关节、夹爪、实体和速度误差。
3. 跑 clear + seed 42、obstacle + seed 42 的 assisted 完整闭环，检查 scene/entity/joint 同步。
4. 每组先跑 5 次；无 runtime_error、未知碰撞或 reset 失败后，再跑 50+50 正式评测。
5. 保存逐 episode、汇总、失败案例、环境、SHA256/SVG 审计和两段 assisted 视频。

本阶段不把 `physics_model_validated` 改为 true，也不评测 physics 抓取。

只有上述运行证据完成后，才更新 V2 成功率、图表、视频和简历中的 Gazebo 表述。
