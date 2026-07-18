# Panda RGB-D 视觉抓取放置规划与评测系统

这是一个面向机械臂运动规划与仿真工程的 Franka Panda 抓取放置项目。
项目只有一套当前代码：Python 负责任务、状态机和评测，C++ 负责 MoveIt 2
规划执行桥接。PyBullet 和 MoveIt/Gazebo 是同一状态机的两种后端，不是两个互相
竞争的项目版本。

## 项目能做什么

- 在 PyBullet 中运行 Panda 桌面抓放，使用自研 Linear 或双向 RRT-Connect。
- 通过 MoveIt 2 / OMPL 进行规划，管理 PlanningScene 和 AttachedCollisionObject。
- 通过顶置 RGB-D 相机、OpenCV HSV 分割、深度反投影和 TF2 估计场景目标位置；
  对 `60×40×50 mm` 矩形工件的 world XY 点云做 PCA，估计平面 yaw。
- 根据视觉 yaw 生成两个相差 π 的对称顶抓候选，使用 MoveIt planning-only 筛选后再执行。
- 通过 `ros2_control` 在 Gazebo Fortress 执行视觉驱动轨迹，完成可重置的约束辅助抓放。
- 记录规划、执行、放置误差、跟踪误差、碰撞与失败阶段。
- 在 clear 与 obstacle 场景中进行单次演示或固定种子批量评测。

Gazebo 抓取使用经过位姿、距离和夹爪开度门限的 DetachableJoint，属于
`assisted` 抓取，不是纯摩擦接触的物理抓取。

## 架构

```text
Gazebo RGB-D -> EstimateScene -> ScenePoseProvider
                                  |
ManipulationTask -----------------+
       |
       v
PickPlaceRunner  (RESET -> PERCEPTION -> GRASP_SELECTION -> PRE_GRASP -> GRASP
                  -> LIFT -> TRANSFER -> PLACE -> RETREAT)
       |
       +---------------------------+
       |                           |
       v                           v
PyBulletBackend               MoveItBackend
IK + Linear/RRT-Connect       PlanExecuteMotion Action
PyBullet collision            MoveIt PlanningScene
       |                           |
       v                           v
PyBullet execution            mock ros2_control / Gazebo Fortress
                                   |
                                   v
                            controller + assisted grasp
       \___________________________/
                    |
                    v
             EpisodeResult + report
```

状态机只依赖 `ManipulationBackend`，不会为不同仿真器复制任务流程。详细调用关系见
[docs/architecture.md](docs/architecture.md)。

## ROS 2 包

| 包 | 职责 |
| --- | --- |
| `panda_pick_place_interfaces` | 任务、结果、运动 Action 和 Gazebo 管理 Service |
| `panda_pick_place_planning` | 状态机、后端、规划器、ROS 2 节点和评测 |
| `panda_perception` | RGB-D 同步、HSV 分割、深度反投影、TF 变换和场景估计服务 |
| `panda_moveit_bridge` | 基于 MoveGroupInterface 的 C++ 规划与执行桥接 |
| `panda_moveit_config` | Panda Xacro、SRDF、OMPL、关节限位和控制器配置 |
| `panda_gz_bringup` | Gazebo Fortress、`gz_ros2_control`、reset、抓取和碰撞观测 |

## 环境与构建

验证环境为 Ubuntu 22.04、ROS 2 Humble、MoveIt 2 Humble、Gazebo Fortress、
Python 3.10 和 PyBullet 3.x。仓库需要放在 ROS 2 工作空间的 `src` 下。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Hangz0416/panda_pick_place_planning.git

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/panda_pick_place_planning/packages --ignore-src -r -y
python3 -m pip install --user -r \
  src/panda_pick_place_planning/packages/panda_pick_place_planning/requirements.txt
colcon build --symlink-install --base-paths src/panda_pick_place_planning/packages
source install/setup.bash
```

每个新终端先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 三种运行方式

### 1. PyBullet 算法演示

```bash
# clear 场景：关节空间线性规划
ros2 run panda_pick_place_planning run_demo \
  --planner linear --scene clear --seed 42

# obstacle 场景：RRT-Connect 绕行
ros2 run panda_pick_place_planning run_demo \
  --planner rrt_connect --scene obstacle --seed 42
```

ROS 2 三节点链路：

```bash
ros2 launch panda_pick_place_planning demo.launch.py \
  backend:=pybullet planner:=rrt_connect scene:=obstacle \
  seed:=42 grasp_mode:=assisted use_gui:=true
```

### 2. MoveIt 2 planning-only

终端 A：

```bash
ros2 launch panda_moveit_config mock_demo.launch.py use_rviz:=true
```

终端 B：

```bash
ros2 launch panda_pick_place_planning demo.launch.py \
  backend:=moveit planner:=rrt_connect scene:=clear seed:=42 \
  use_gui:=false use_gazebo:=false moveit_execute:=false \
  moveit_command_gripper:=false
```

planning-only 只验证规划和 PlanningScene，结果保持 `not_executed`，不计为抓放成功。

### 3. MoveIt 2 + Gazebo 完整抓放

```bash
ros2 launch panda_gz_bringup panda_visual_pick_place.launch.py \
  scene:=clear seed:=42 object_variant:=rect_prism \
  pose_source:=rgbd perception_profile:=nominal \
  headless:=false use_rviz:=true
```

将 `scene:=clear` 替换为 `scene:=obstacle` 即可观察障碍绕行。readiness gate 会在
`/clock`、相机图像、CameraInfo、TF、关节/方块状态、控制器、Service 和 Action 都就绪后
才启动任务。`pose_source:=ground_truth` 保留为 oracle 回归；视觉模式的运动目标与
PlanningScene 均来自 `EstimateScene`，任务真值只用于 reset 和误差评测。将
`object_variant:=rect_prism` 改为 `cube` 可回归原方块；矩形工件 yaw 由 task seed
确定性生成。

相机发布 `/panda_perception/color/image_raw`、`depth/image_raw` 和 `camera_info`；调试叠加图
发布到 `/panda_perception/debug_overlay`。当前方法估计已知尺寸物体的位置，并只对矩形工件
估计绕 world Z 的平面方向；不宣称通用六自由度位姿识别。

## 批量评测

PyBullet：

```bash
ros2 run panda_pick_place_planning run_benchmark \
  --episodes 5 --seed 42 \
  --scenes clear obstacle --planners linear rrt_connect \
  --output-dir results/runs/pybullet_quick
```

MoveIt mock 需先启动 `mock_demo.launch.py`，再在另一终端执行：

```bash
ros2 launch panda_pick_place_planning batch_benchmark.launch.py \
  episodes_per_scene:=5 backend:=moveit simulator:=moveit_mock \
  use_gazebo:=false moveit_execute:=false \
  output_path:=results/runs/moveit_mock/episodes.jsonl \
  summary_path:=results/runs/moveit_mock/summary.json
```

Gazebo RGB-D assisted：

```bash
ros2 launch panda_gz_bringup gazebo_batch.launch.py \
  episodes_per_scene:=5 seed:=42 scenes_csv:=clear,obstacle \
  object_variant:=rect_prism pose_source:=rgbd \
  perception_profile:=nominal headless:=true use_rviz:=false \
  output_path:=results/runs/gazebo_rgbd_grasp_pose/episodes.jsonl \
  summary_path:=results/runs/gazebo_rgbd_grasp_pose/summary.json
```

`results/runs/` 为新实验的统一输出位置，默认不提交 Git，避免覆盖已审计数据。

## 已验证结果

### PyBullet 规划器对比

固定主种子 42，clear/obstacle 各 50 个任务，Linear 与 RRT-Connect 共形成
200 个 episode。

| 场景 / 规划器 | 任务结果 | 平均规划时间 | 平均关节路径长度 |
| --- | --- | ---: | ---: |
| clear / Linear | 50/50 成功 | 1.750 ms | 0.778358 rad |
| clear / RRT-Connect | 50/50 成功 | 1.922 ms | 0.778358 rad |
| obstacle / Linear | 受阻直线路径均被拒绝 | 0.560 ms | N/A |
| obstacle / RRT-Connect | 50/50 成功 | 33.157 ms | 1.312717 rad |

obstacle / Linear 是专门构造的直连受阻任务，不表示 Linear 在所有障碍场景中
都失败。

### MoveIt mock planning-only

固定任务集中，clear 与 obstacle 均完成 50/50 次规划。clear 平均规划时间
0.125806 s，obstacle 为 0.129607 s。该结果是独立阶段规划回归，不是执行或抓取
成功率。

### Gazebo assisted 完整闭环

| 场景 | 完整任务 | 平均规划时间 | 平均关节路径长度 | 平均执行时间 |
| --- | --- | ---: | ---: | ---: |
| clear | 50/50 成功 | 23.768 ms | 0.797926 rad | 13.296 s |
| obstacle | 50/50 成功 | 33.413 ms | 2.669548 rad | 17.230 s |

这些结果表示 assisted 规划、控制、attach/release 和评测链路的稳定性，不外推为
物理抓取或真机成功率。实验口径、原始数据路径和失败修复见
[docs/experiments.md](docs/experiments.md)。

### Gazebo RGB-D assisted 视觉闭环

固定主种子 42，顶置 640×480、20 Hz RGB-D，相同 clear/obstacle 任务各 50 个 episode。

| 场景 | 任务结果 | 感知耗时均值 / P95 | 方块位置误差 P95 | 目标位置误差 P95 | 障碍物误差 P95 |
| --- | --- | ---: | ---: | ---: | ---: |
| clear | 50/50 成功 | 72.0 / 165.5 ms | 4.07 mm | 3.35 mm | N/A |
| obstacle | 50/50 成功 | 54.3 / 125.9 ms | 2.75 mm | 3.19 mm | 2.31 mm |

两场景规划与执行成功率均为 100%，无无效碰撞 episode 或未分类 `runtime_error`；平均最终
XY 放置误差分别为 2.10 mm 和 2.12 mm。该结果只说明固定颜色、已知尺寸、轴对齐物体的
Gazebo assisted 视觉闭环，不代表开放场景感知、物理夹持或真机表现。

### 矩形工件 yaw 与抓取候选正式回归

固定主种子 42，`rect_prism + RGB-D + nominal` 在 clear/obstacle 各运行 50 个完整 assisted
episode。两组必需目标检测、候选筛选、规划和任务均为 50/50 成功，未出现无效碰撞、
PlanningScene 同步错误或未分类 `runtime_error`。

| 场景 | 工件位置误差 P95 | yaw 误差 P95 | 感知耗时 P95 | 候选路径代价 P95 | 最终 XY 误差均值 |
| --- | ---: | ---: | ---: | ---: | ---: |
| clear | 4.16 mm | 3.28° | 130.63 ms | 3.218 rad | 2.29 mm |
| obstacle | 2.50 mm | 4.94° | 163.45 ms | 3.764 rad | 2.94 mm |

`combined` 合成扰动另运行 20 clear + 20 obstacle：两组任务均完成，工件位置误差 P95
最大为 3.85 mm，yaw 误差 P95 最大为 6.84°。这里的 `combined` 是确定性低亮度、RGB
高斯噪声和深度丢失组合，不代表真实相机域随机化。

原始 JSONL、summary、SVG 报告、环境记录与 SHA256 manifest 分别位于
`results/runs/gazebo_rgbd_grasp_pose/nominal_rgbd/` 和 `combined_rgbd/`。同任务
ground-truth 对照与 RGB-D 数据已通过 `(scene, task_seed)` 配对覆盖审计。

## 成功判据与指标

`task_success` 同时要求：方块已释放，XY 误差不超过 40 mm，中心高度误差不超过
15 mm，线速度不超过 0.05 m/s，执行中无无效机器人或携物碰撞，并且 Gazebo
碰撞观测可用。

`EpisodeResult` 记录 backend、simulator、planner、seed、pose source、感知耗时与位置误差、各状态规划/执行时间、
物体 yaw 误差、候选数量与筛选代价、关节路径长度、轨迹点、跟踪误差、放置误差、
物体速度、attach/release、碰撞和失败分类。
缺失观测使用 `null`/NaN，不会被冒充为安全结果。

## 测试

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --base-paths src/panda_pick_place_planning/packages \
  --return-code-on-test-failure
colcon test-result --verbose
```

测试覆盖颜色分割、深度反投影、TF 变换、新帧约束、防真值泄漏、IK/坐标变换、规划器、
后端合约、携物碰撞、消息序列化、MoveIt bridge、Gazebo 资产、reset、抓取门控、碰撞分类和结果审计。

## 数据和版本说明

当前源码只有一条主线。仓库中仍有 `baselines/v0.1-pybullet-baseline/` 和
`results/v2/` 路径，是为了保持已发布原始数据的 SHA256 和历史引用稳定，不表示需要
选择两套代码。新运行一律写入 `results/runs/`。历史对应关系见
[docs/development_history.md](docs/development_history.md)。

## 已知边界

- 抓取使用 assisted 约束，不是纯接触物理抓取。
- RGB-D 方法依赖已知颜色和尺寸；矩形工件只估计平面位置与 yaw，方块和圆形目标区不声明
  可观测方向；未做相机标定、YOLO 或通用 6D 位姿。
- 没有真机部署，不将仿真结果外推到真实硬件。
- `physics_model_validated` 保持为 false，物理抓取模式 fail-closed。
- ROS 2 Humble 中的 MoveIt 2.5.9 在任务和结果写入后，整套系统退出时可能在
  `move_group` 析构路径段错误。

详细说明见 [docs/known_issues.md](docs/known_issues.md)。检测器通过 `ScenePoseProvider` 与
状态机解耦，后续可替换 YOLO；当前不在 assisted 动力学边界上叠加强化学习。
