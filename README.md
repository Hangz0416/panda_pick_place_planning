# Panda RGB-D 多工件视觉抓取与联合选择系统

面向视觉与机械臂方向的 Franka Panda 仿真项目，当前主线是：使用 Gazebo 顶置 RGB-D
相机检测三个矩形工件并估计平面位置与 yaw，为全部工件生成抓取候选，经 MoveIt 2
planning-only 联合筛选后，在 Gazebo Fortress 中通过 `ros2_control` 完成 assisted 抓取放置，
最后输出可复现的逐 episode 结果和统计报告。

项目只有一套当前代码。PyBullet 是自研 Linear/RRT-Connect 的教学与快速回归后端；
MoveIt 2 + Gazebo 是当前视觉抓取主线，不是另一套状态机。

> 能力边界：视觉依赖已知颜色和尺寸，只估计平面位置与 yaw；Gazebo 抓取使用门控后的
> DetachableJoint，属于 assisted grasp，不是纯接触物理抓取，也没有真机部署证据。

## 当前主线

```text
Gazebo RGB-D
  → HSV 多实例分割、深度反投影、TF2、PCA yaw
  → ScenePoseProvider
  → PickPlaceRunner
  → 三工件 × 两个 π 对称候选
  → MoveIt planning-only 联合筛选
  → ros2_control 执行与 assisted attach/release
  → EpisodeResult、JSONL、summary 和 SVG report
```

统一状态机为：

```text
RESET → PERCEPTION → GRASP_SELECTION → PRE_GRASP → GRASP
      → LIFT → TRANSFER → PLACE → RETREAT → DONE / FAILED
```

运动目标和 PlanningScene 在 RGB-D 模式中来自视觉估计；任务真值只用于确定性 reset 和
离线误差评测。详细的数据所有权、坐标系和事务顺序见
[系统架构](docs/architecture.md)与[代码走读](docs/code_walkthrough.md)。

## 环境与构建

验证环境：Ubuntu 22.04、ROS 2 Humble、MoveIt 2 Humble、Gazebo Fortress、Python 3.10。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/panda_pick_place_planning/packages --ignore-src -r -y
python3 -m pip install --user -r \
  src/panda_pick_place_planning/packages/panda_pick_place_planning/requirements.txt
colcon build --symlink-install \
  --base-paths src/panda_pick_place_planning/packages
source install/setup.bash
```

每个新终端都需要重新 source ROS 2 和工作空间：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 唯一推荐演示入口

```bash
ros2 launch panda_gz_bringup panda_visual_pick_place.launch.py \
  scene:=obstacle seed:=42 \
  object_variant:=rect_prism workpiece_count:=3 \
  pose_source:=rgbd perception_profile:=nominal \
  headless:=false use_rviz:=true
```

该命令启动 RGB-D 相机、感知节点、MoveIt、Gazebo、控制器、共享 FSM 和评测器。
readiness gate 会等待图像、CameraInfo、TF、关节状态、三个工件状态、控制器、Service 和
Action 就绪后才发布任务。图形演示完成后默认保持 Gazebo/RViz 打开；结果默认写入
`~/.ros/panda_pick_place/runs/<timestamp>/episodes.jsonl`。

RViz 可观察：

- `/panda_perception/object_markers`：工件中心、PCA 主轴、目标区和障碍物；
- `/panda/grasp_candidate_markers`：全部候选、不可行候选和最终选中候选；
- `/panda_perception/debug_overlay`：实例编号、位置、yaw、置信度和选择状态。

## 其他入口的定位

| 入口 | 用途 | 是否为当前演示主线 |
| --- | --- | --- |
| `panda_visual_pick_place.launch.py` | RGB-D 三工件完整抓放 | 是 |
| `gazebo_batch.launch.py` | 固定种子自动评测 | 否，评测入口 |
| `run_demo` / `run_benchmark` | PyBullet 自研规划教学与快速回归 | 否，算法基线 |
| `mock_demo.launch.py` | MoveIt planning-only 接口调试 | 否，调试入口 |
| `panda_moveit_gz.launch.py` / `panda_gz.launch.py` | 底层集成诊断 | 否，内部入口 |

批量评测示例：

```bash
ros2 launch panda_gz_bringup gazebo_batch.launch.py \
  episodes_per_scene:=5 seed:=42 scenes_csv:=clear,obstacle \
  object_variant:=rect_prism workpiece_count:=3 pose_source:=rgbd \
  perception_profile:=nominal headless:=true use_rviz:=false \
  output_path:=results/runs/gazebo_rgbd_multi_object/quick/episodes.jsonl \
  summary_path:=results/runs/gazebo_rgbd_multi_object/quick/summary.json
```

PyBullet 基线与 MoveIt mock 的完整命令分别见
[`panda_pick_place_planning` 包说明](packages/panda_pick_place_planning/README.md)和
[`mock_demo.launch.py`](packages/panda_moveit_config/launch/mock_demo.launch.py)。

## ROS 2 包职责

| 包 | 职责 |
| --- | --- |
| `panda_pick_place_interfaces` | 类型化任务、感知、结果、运动 Action 和 Gazebo Service |
| `panda_pick_place_planning` | 状态机、后端、任务生成、PyBullet 规划器和评测 |
| `panda_perception` | RGB-D 同步、分割、反投影、PCA yaw、TF 和 Marker |
| `panda_moveit_bridge` | C++ `MoveGroupInterface` 规划/执行桥接与 PlanningScene 服务 |
| `panda_moveit_config` | Panda Xacro、SRDF、OMPL、关节限位和控制器配置 |
| `panda_gz_bringup` | Gazebo、`gz_ros2_control`、reset、assisted grasp 和碰撞观测 |

## 当前三工件正式结果

固定主种子 42，工件尺寸为 `60×40×50 mm`。nominal RGB-D 与 ground-truth 使用相同
`(scene, task_seed)`，通过严格配对覆盖审计。

| 场景 | 完整检测 / 联合选择 / 任务成功率 | 位置误差 P95 | yaw 误差 P95 | 平均规划时间 | 平均最终 XY 误差 |
| --- | ---: | ---: | ---: | ---: | ---: |
| clear | 100% / 100% / 100% | 5.15 mm | 3.95° | 28.70 ms | 2.42 mm |
| obstacle | 100% / 100% / 100% | 3.25 mm | 4.76° | 40.91 ms | 2.95 mm |

`combined` 合成低亮度、RGB 高斯噪声和深度丢失扰动下，完整检测、联合选择和任务成功率
仍为 100%；位置误差 P95 最大 4.34 mm，yaw 误差 P95 最大 7.56°。正式组没有无效碰撞、
`selected_attachment_mismatch` 或未分类 `runtime_error`，未选工件无可测位移。

这些数字只适用于当前已知颜色、尺寸、顶置相机和 assisted 仿真任务分布。原始 JSONL、
summary、环境记录、SVG 和 SHA256 manifest 位于
[`results/runs/gazebo_rgbd_multi_object`](results/runs/gazebo_rgbd_multi_object)。完整历史实验、
统计口径和失败批次见[实验记录](docs/experiments.md)。

## 推荐代码阅读顺序

1. `core/tasks.py` 与 `core/schemas.py`：任务和结果的数据契约；
2. `nodes/task_generator_node.py` 与 `nodes/manipulation_node.py`：ROS 2 入口；
3. `core/scene_pose_provider.py` 与 `panda_perception`：视觉位姿来源；
4. `core/pick_place_fsm.py`：唯一抓放状态机；
5. `core/grasp_pose.py`：候选生成、π 对称和稳定排序；
6. `core/backends/moveit.py` 与 C++ bridge：规划、执行和 PlanningScene；
7. `panda_gz_bringup`：reset、attach/release 和碰撞监控；
8. `nodes/evaluator_node.py` 与 `core/reporting.py`：落盘、统计和审计。

完整函数级调用链、公式和真实故障定位见[代码走读](docs/code_walkthrough.md)。

## 验证

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --base-paths src/panda_pick_place_planning/packages \
  --return-code-on-test-failure
colcon test-result --verbose
```

结果审计会重算 summary、检查 JSONL 语义、解析 SVG，并验证输入文件 SHA256。episode 成功与
Humble `move_group` 的关闭阶段异常分别记录，不能互相替代。

## 项目边界

- HSV 依赖已知颜色，当前不是开放类别检测器；
- 矩形工件只估计 world XY 与绕 Z 轴 yaw，不是通用 6D pose；
- assisted grasp 验证规划、控制和场景同步，不证明纯接触夹持；
- 自动选择只在当前候选集合中代价最优，不是全局任务规划最优；
- physics 相关实现保持 `experimental / fail-closed`，不属于当前主线；
- 没有相机标定、YOLO、强化学习或真机部署。

## 文档导航

- [系统架构与状态所有权](docs/architecture.md)
- [完整代码走读](docs/code_walkthrough.md)
- [实验数据与失败修复](docs/experiments.md)
- [简历与项目介绍](docs/resume_and_interview.md)
- [模拟面试指南](docs/interview_guide.md)
- [已知问题与能力边界](docs/known_issues.md)
- [开发历史](docs/development_history.md)
