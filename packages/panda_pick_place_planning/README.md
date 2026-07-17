# panda_pick_place_planning

该包实现共享 Python 抓取放置状态机、PyBullet/MoveIt backend、ROS 2 任务/结果节点、
批量调度和报告工具。

## 模块

```text
panda_pick_place_planning/
|- core/
|  |- backends/       # ManipulationBackend, PyBulletBackend, MoveItBackend
|  |- planners/       # Linear, RRT-Connect
|  |- pick_place_fsm.py
|  |- collision_checker.py
|  |- tasks.py
|  |- schemas.py
|  `- reporting.py
`- nodes/
   |- task_generator_node.py
   |- manipulation_node.py
   |- evaluator_node.py
   `- batch_task_generator_node.py
```

## 单次演示

PyBullet：

```bash
ros2 run panda_pick_place_planning run_demo \
  --planner rrt_connect --scene obstacle --seed 42
```

ROS 2 节点链路：

```bash
ros2 launch panda_pick_place_planning demo.launch.py \
  backend:=pybullet planner:=rrt_connect scene:=obstacle seed:=42
```

MoveIt planning-only 需要另外启动 `panda_moveit_config/mock_demo.launch.py`：

```bash
ros2 launch panda_pick_place_planning demo.launch.py \
  backend:=moveit planner:=rrt_connect scene:=clear seed:=42 \
  use_gazebo:=false moveit_execute:=false moveit_command_gripper:=false
```

MoveIt/Gazebo 完整执行请使用 `panda_gz_bringup/panda_moveit_gz.launch.py`，不要在本包内
复制 `move_group` 或 Gazebo 节点。

## 批量运行

PyBullet：

```bash
ros2 run panda_pick_place_planning run_benchmark \
  --episodes 5 --seed 42 \
  --scenes clear obstacle --planners linear rrt_connect \
  --output-dir results/runs/pybullet_quick
```

MoveIt planning-only：

```bash
ros2 launch panda_pick_place_planning batch_benchmark.launch.py \
  episodes_per_scene:=5 backend:=moveit simulator:=moveit_mock \
  use_gazebo:=false moveit_execute:=false \
  output_path:=results/runs/moveit_mock/episodes.jsonl \
  summary_path:=results/runs/moveit_mock/summary.json
```

快速检查通过后，使用独立输出运行正式 planning-only 回归：

```bash
ros2 launch panda_pick_place_planning batch_benchmark.launch.py \
  episodes_per_scene:=50 backend:=moveit simulator:=moveit_mock \
  use_gazebo:=false moveit_execute:=false \
  output_path:=results/runs/moveit_mock_formal/episodes.jsonl \
  summary_path:=results/runs/moveit_mock_formal/summary.json
```

This is not Gazebo physics execution. planning-only 结果不证明控制、抓取或物体运动成功。

批量生成器等待与当前任务匹配的 `EpisodeResult` 后才发布下一任务。超时时 summary
标记不完整，不会伪造失败 episode。

## 指标口径

完整 PyBullet 和 MoveIt/Gazebo episode 的顶层 `planning_time_sec`、`path_length_rad`、
`path_waypoints` 和 `trajectory_points` 保留关键 TRANSFER 阶段口径；`stage_metrics` 保留
全部 FSM 阶段。

MoveIt mock planning-only 不执行轨迹，顶层运动指标是显式独立规划请求的汇总，
并保持 `failure_type=not_executed`。跟踪误差、放置误差和物体速度在 planning-only
中为 `null`，不会被补成安全或成功数据。

## 详细文档

- 系统调用链：`../../docs/architecture.md`
- 实验口径：`../../docs/experiments.md`
- 能力边界：`../../docs/known_issues.md`
