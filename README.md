# 基于 ROS2 与 PyBullet 的 Panda 机械臂抓取放置与路径规划仿真系统

本项目在 PyBullet 中控制 Franka Panda 完成单物体抓取与放置，
并对比关节空间线性插值和 RRT-Connect 两种规划方法。
ROS2 负责任务发布、仿真执行和结果记录，核心规划算法保持为普通 Python 模块，便于单独测试。

项目只使用仿真真值位姿，不包含视觉感知、真机部署、模仿学习、强化学习或 VLA。抓取采用经过距离校验的约束辅助方式，重点验证运动学、路径规划、碰撞检测、状态机和评测闭环。

## 当前验证状态

- 已通过 18 项测试，包括数据结构、Linear、RRT-Connect、任务公平配对、指标汇总、
  PyBullet Panda 冒烟测试、连续 episode 环境重置和障碍物安全间距测试。
- 已在 Ubuntu 22.04、ROS2 Humble、Python 3.10 和 PyBullet DIRECT 模式验证完整抓取放置闭环。
- 已验证 ROS2 三节点链路：任务发布、仿真执行、结果发布和 JSONL 保存均成功。
- 已使用固定种子完成 200 个 episode 的最终对照评测，结果见下表和
  `results/final_benchmark/`。实验数字来自实际运行，不是预编数据。

## 系统流程

```text
随机任务生成
  -> 读取 PyBullet 物体真值位姿
  -> 生成预抓取、抓取、抬升和放置位姿
  -> IK 求解并验证末端位姿误差
  -> Linear 或 RRT-Connect 规划搬运路径
  -> 逐点碰撞检测
  -> Panda 轨迹与夹爪状态机执行
  -> 成功判定和失败分类
  -> JSONL / CSV / JSON 评测结果
```

ROS2 节点：

- `task_generator_node`：生成固定随机种子的任务，发布 `/manipulation_task`。
- `manipulation_node`：持有唯一 PyBullet 仿真连接，完成规划和执行，发布 `/episode_result`。
- `evaluator_node`：保存 episode 结果，不参与成功判定。

第一版使用 `std_msgs/String` 传递 UTF-8 JSON。任务与结果字段由 `schemas.py` 统一解析，避免节点各自拼接数据。

## 规划器

### Linear

在起止关节角之间均匀插值，并检查每个离散状态。计算开销低，但路径与障碍物相交时直接返回 `planning_collision`，不具备绕障能力。

### RRT-Connect

在 Panda 七维关节空间中从起点和终点建立两棵搜索树，交替执行采样、最近节点查询、扩展和连接。每段扩展均进行离散碰撞检测，找到路径后执行 shortcut 简化并再次验证。

两种规划器使用相同任务、关节限位和碰撞定义。RRT-Connect 单独记录随机种子，保证同一规划可复现。

障碍场景固定从 Panda home 构型同侧抓取，再跨越障碍物放到另一侧，避免把初始接近路径混入
搬运段对比。规划碰撞检测对障碍物保留 10 mm 安全间距，执行碰撞仍按 PyBullet 真实接触统计。

## 固定种子评测结果

2026-07-17 在 Intel Core i5-14600KF、PyBullet DIRECT 模式运行；每组 50 个任务，主种子为 42。
两种规划器在每个场景中使用完全相同的任务几何。墙钟规划耗时会受硬件和系统负载影响。

| 场景 / 规划器 | 任务成功率 | 规划成功率 | 平均规划耗时 | P95 规划耗时 | 平均路径长度 | 失败类型 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| clear / Linear | 50/50 (100%) | 50/50 (100%) | 1.679 ms | 1.980 ms | 0.778358 rad | 无 |
| clear / RRT-Connect | 50/50 (100%) | 50/50 (100%) | 1.757 ms | 2.029 ms | 0.778358 rad | 无 |
| obstacle / Linear | 0/50 (0%) | 0/50 (0%) | 0.505 ms | 0.598 ms | - | `planning_collision`: 50 |
| obstacle / RRT-Connect | 50/50 (100%) | 50/50 (100%) | 26.197 ms | 42.716 ms | 1.199255 rad | 无 |

这里 obstacle / Linear 的 0% 表示测试场景刻意阻断了关节空间直线插值，不能推广为 Linear
在所有障碍场景中都失败。clear 场景下 RRT-Connect 先检查直连边，因此与 Linear 得到相同路径长度。

加入 10 mm 规划安全间距前的同种子基线保存在
`results/final_benchmark_baseline_no_margin/`：obstacle / RRT-Connect 规划成功 50/50，
但有 2 次执行阶段擦碰，任务成功率为 48/50。该失败复现并定位为障碍物与 hand/finger link 的短暂接触。

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10
- PyBullet 3.2.6 或更高的 3.x 版本

安装基础工具：

```bash
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions
source /opt/ros/humble/setup.bash
```

## 获取和构建

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Hangz0416/panda_pick_place_planning.git
cd panda_pick_place_planning
python3 -m pip install --user -r requirements.txt

cd ~/ros2_ws
colcon build --symlink-install --packages-select panda_pick_place_planning
source install/setup.bash
```

每次新开终端都需要加载：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 测试

在仓库目录运行：

```bash
python3 -m unittest discover -s test -v
```

安装 PyBullet 后，`test_pybullet_smoke.py` 会自动执行 Panda 场景、环境重置和预抓取 IK 冒烟测试；未安装时该测试会明确显示 `skipped`。

从 ROS2 工作区根目录也可以通过 colcon 运行同一组测试：

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select panda_pick_place_planning --return-code-on-test-failure
colcon test-result --verbose
```

## 单次可视化演示

不经过 ROS2，直接运行核心模块：

```bash
python3 -m panda_pick_place_planning.core.demo \
  --planner rrt_connect \
  --scene obstacle \
  --seed 42
```

建议先运行最简单组合：

```bash
python3 -m panda_pick_place_planning.core.demo \
  --planner linear \
  --scene clear \
  --seed 42
```

服务器或 CI 没有显示环境时，可以运行同一完整状态机的 DIRECT 模式：

```bash
python3 -m panda_pick_place_planning.core.demo \
  --planner rrt_connect \
  --scene obstacle \
  --seed 42 \
  --no-gui
```

## ROS2 完整链路

```bash
ros2 launch panda_pick_place_planning demo.launch.py \
  planner:=rrt_connect \
  scene:=obstacle \
  seed:=42 \
  use_gui:=true
```

运行结果默认写入：

```text
results/ros2_episode_results.jsonl
```

可以通过 launch 参数修改输出位置：

```bash
ros2 launch panda_pick_place_planning demo.launch.py \
  output_path:=/tmp/panda_episode_results.jsonl
```

## 批量对照评测

快速检查，每个场景、每种规划器运行 5 个任务，总计 20 个 episode：

```bash
ros2 run panda_pick_place_planning run_benchmark \
  --episodes 5 \
  --seed 42
```

最终评测，每组 50 个任务，总计 200 个 episode：

```bash
ros2 run panda_pick_place_planning run_benchmark \
  --episodes 50 \
  --seed 42 \
  --scenes clear obstacle \
  --planners linear rrt_connect \
  --output-dir results/final_benchmark
```

输出文件：

- `benchmark_results.jsonl`：逐 episode 完整记录。
- `benchmark_results.csv`：便于表格统计。
- `benchmark_summary.json`：按场景和规划器汇总。

每个 episode 记录：

- 任务种子与规划器种子
- 规划是否成功、规划时间和搜索节点数
- 路径航点数和关节空间路径长度
- 执行仿真步数
- 是否完成放置
- 检测到无效碰撞的仿真步数
- 物体初始位置、最终位置和目标位置
- 失败类型

DIRECT 模式下的 CPU 墙钟时间不会被当作机械臂执行时间。规划阶段单独记录计算时间，执行阶段记录仿真步数。

## 成功与失败定义

任务成功需要同时满足：

- 物体已经释放。
- 等待固定仿真步数后，物体中心位于目标区域内。
- 物体线速度低于稳定阈值。
- 执行过程中没有检测到无效机器人碰撞。

主要失败类型：

- `ik_failed`：IK 结果超过关节限位或末端误差阈值。
- `planning_collision`：Linear 路径与场景碰撞，或规划扩展无效。
- `planning_timeout`：RRT-Connect 在预算内未找到路径。
- `execution_collision`：执行轨迹时发生无效碰撞。
- `grasp_failed`：闭合夹爪时末端与物体距离不满足约束建立条件。
- `place_failed`：释放后物体未稳定落入目标区域。
- `runtime_error`：ROS2 节点捕获到未预期运行时错误。

## Ubuntu 首次运行顺序

不要一上来就跑 200 组评测，先按顺序确认：

1. 执行全部单元测试，确认 PyBullet 冒烟测试通过。
2. 运行 `clear + linear + seed 42`，观察固定场景抓取放置。
3. 连续更换 3 个种子，确认随机位置仍可达。
4. 运行 `obstacle + linear`，确认碰撞路径会被拒绝，而不是穿过障碍物。
5. 运行 `obstacle + rrt_connect`，确认搜索路径能够绕障。
6. 运行每组 5 个任务的快速评测，检查失败日志。
7. 修正场景或控制参数后，再运行每组 50 个任务的最终评测。

如果第 2 步都没有通过，先检查末端姿态、抓取高度和 Panda 关节状态，别急着调 RRT 参数。基础抓取没通时扫规划参数，属于给轮胎打气想修发动机。

## 项目结构

```text
panda_pick_place_planning/
├── launch/demo.launch.py
├── panda_pick_place_planning/
│   ├── core/
│   │   ├── benchmark.py
│   │   ├── collision_checker.py
│   │   ├── demo.py
│   │   ├── kinematics.py
│   │   ├── pick_place_fsm.py
│   │   ├── pybullet_env.py
│   │   ├── schemas.py
│   │   ├── tasks.py
│   │   └── planners/
│   │       ├── base.py
│   │       ├── linear.py
│   │       └── rrt_connect.py
│   └── nodes/
│       ├── evaluator_node.py
│       ├── manipulation_node.py
│       └── task_generator_node.py
├── test/
├── package.xml
├── requirements.txt
└── setup.py
```

## 已知边界

- 物体位姿来自 PyBullet 真值，不包含相机感知误差。
- 约束辅助抓取不是完整接触抓取控制，只在距离校验通过后建立固定约束。
- 当前碰撞规划只考虑机器人与桌面、障碍物及机器人自碰撞，没有建立携带物体的完整几何膨胀模型。
- 当前只比较抓取放置中的搬运段，不包含推移、堆叠或多物体任务。
- MoveIt 2、Gazebo 和真机适配属于后续阶段。

上述边界是项目的工程范围，不应在简历或面试中扩大描述。
