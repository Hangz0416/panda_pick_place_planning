# 简历与项目介绍

本页只使用当前代码和已审计实验数据。详细代码流程见[代码走读](code_walkthrough.md)，互动训练
与答案卡见[模拟面试指南](interview_guide.md)。

## 一页简历精简版

**项目名称：基于 RGB-D 与 MoveIt 2 的 Panda 多工件视觉抓取系统**

**Python / C++ / ROS2 Humble / OpenCV / MoveIt 2 / Gazebo Fortress / PyBullet / RRT-Connect**

- 在 PyBullet 中搭建 Franka Panda 桌面抓取放置环境，实现任务生成、逆运动学、关节限位、
  自碰撞/环境碰撞检测、轨迹执行及结果判定，并实现关节空间 Linear 与双向 RRT-Connect；
  障碍场景下 RRT-Connect 平均规划时间 `33.16 ms`、平均关节路径长度 `1.31 rad`。
- 抽象统一 `ManipulationBackend`，使 PyBullet 与 MoveIt 2 后端复用同一抓取放置状态机；使用
  ROS2 自定义消息、Service 和 Action 拆分任务、感知、执行与评测，基于 C++
  `MoveGroupInterface` 管理 OMPL 规划、PlanningScene、AttachedCollisionObject，并通过
  `ros2_control` 在 Gazebo Fortress 完成可重置的 assisted 抓放闭环。
- 基于顶置 RGB-D、HSV 多实例分割、深度反投影、MAD 过滤和 TF2 估计三个矩形工件位置，
  对 world-XY 点云执行 PCA 估计 yaw；为三工件生成六个 π 对称抓取候选，以 MoveIt
  planning-only 联合筛选工件与姿态。nominal 回归中完整检测、联合选择和任务成功率均为
  `100%`，位置/yaw 误差 P95 不超过 `5.15 mm/4.76°`，clear/obstacle 平均规划时间为
  `28.70/40.91 ms`。
- 建立感知、IK、规划、碰撞、抓取、场景同步、放置和 reset 失败分类；结合原始日志定位并修复
  Panda 九个可运动关节的 IK 参数映射、固定关节刚性组件伪自碰撞、跨 episode 残留
  AttachedCollisionObject 及 reset home 路径容差问题，通过全工件状态清理、有界内部重试和
  Gazebo/MoveIt attach 事务回滚避免状态污染。

## 为什么这样写

- 第一条证明机械臂基础与自研规划，不把项目描述成只会调用 MoveIt。
- 第二条体现 ROS2、MoveIt、Gazebo 和系统抽象能力。
- 第三条直接匹配视觉机械臂岗位，并给出可核对的数据。
- 第四条用真实故障证明调试、碰撞语义和跨系统一致性能力。

简历正文不写单元测试数量、历史版本迁移、失败批次条数或关闭阶段段错误；这些内容适合面试
追问，不适合挤占项目经历篇幅。

## 60 秒项目介绍

这个项目实现了一条 RGB-D 感知、抓取候选选择、MoveIt 规划、Gazebo 控制执行和量化评测
闭环。任务层使用统一的 PickPlaceRunner，PyBullet 和 MoveItBackend 只在后端分叉，所以
任务阶段、失败分类和成功判据只有一套。视觉端使用顶置 RGB-D，通过 HSV 分割、深度反投影、
MAD 过滤和 TF2 得到三个矩形工件的 world 坐标，再用 PCA 估计平面 yaw。系统为每个工件生成
两个相差 π 的顶抓候选，通过 MoveIt planning-only 检查 pre-grasp 和 grasp 的可达性与碰撞，
按路径长度和置信度自动选择工件。选中后由 ros2_control 在 Gazebo 中执行，并同步
DetachableJoint 与 MoveIt AttachedCollisionObject。nominal 回归的完整检测、联合选择和任务
成功率均为 100%，位置和 yaw 误差 P95 最差为 5.15 mm 和 4.76°。当前抓取是 assisted，视觉
依赖已知颜色和尺寸，只估计平面位置与 yaw，不外推到物理抓取或真机。

## 3 分钟技术展开版

项目从 PyBullet 中的 Panda 抓放和路径规划开始。我实现了任务生成、Panda IK、关节限位、
碰撞检测、Linear 和双向 RRT-Connect，并用固定种子评测规划时间、路径长度和失败类型。
Panda 名义上是七自由度机械臂，但 PyBullet IK 参数必须覆盖两个手指在内的九个可运动关节，
求解后再映射回七个 arm joint；自碰撞检查还要根据固定关节过滤同一刚性组件内部接触。

为了接入 MoveIt 和 Gazebo，我定义了 ManipulationBackend，让 PyBullet 与 MoveIt 复用同一个
PickPlaceRunner。Python 负责任务、状态机和结果统计，C++ bridge 使用 MoveGroupInterface
完成 pose/named target 规划与可选执行，并通过 PlanningScene 服务管理全部工件、障碍物和
AttachedCollisionObject。Gazebo 使用 gz_ros2_control 的七轴 trajectory controller 和 gripper
controller；reset 会 detach 全部工件、恢复场景、通过控制器回 home，再验证关节和实体状态。

视觉端使用 RGB/depth 近似同步。连通域内的深度像素通过相机内参反投影，再由 TF2 变换到
world；中值和 MAD 用于过滤异常深度。矩形工件对 world-XY 点云做二维 PCA，最大特征向量
给出主轴 yaw，特征值比低于 1.25 时认为方向不可观测。三个工件按估计 world X/Y 排序，不读
任务真值。每个方向有效工件生成 yaw 和 yaw+π 两个夹爪对称候选，MoveIt 只规划、不执行，
再按 `path length + 1 - confidence` 稳定选择。

执行阶段先锁定 selected object。assisted attach 必须依次通过手物位姿和夹爪宽度门控、对应
Gazebo DetachableJoint 确认，再更新 MoveIt attachment；后一步失败会回滚前一步。释放时先
detach，再等待 Gazebo 物体稳定并读取真实位姿，重新加入 PlanningScene 后才 retreat。碰撞
监控对未知状态 fail-closed，机器人碰未选工件、携带工件碰障碍物都明确分类。

我保留了真实失败记录。例如一次 transfer 失败后残留 `workpiece_2` AttachedCollisionObject，
导致后续目标连续无效；根因是 reset 只清理了新 selected ID，修复为清理上一批和当前批全部
工件并验证 readback。另一次控制器 home 误差略高于 path tolerance，修复为同一 reset 内有界
重试，同时保留最终严格稳态判据。项目当前能证明感知驱动的规划控制和 assisted 抓放闭环，
不能证明纯接触物理夹持、通用 6D pose 或真机性能。

## 数据证据映射

| 简历数据 | 证据文件 | summary 字段 |
| --- | --- | --- |
| PyBullet obstacle 规划 `33.16 ms` | `results/v2/pybullet_shared_geometry_200_final/benchmark_summary.json` | `obstacle/rrt_connect.avg_planning_time_sec` |
| PyBullet obstacle 路径 `1.31 rad` | 同上 | `obstacle/rrt_connect.avg_path_length_rad` |
| nominal 完整检测 `100%` | `results/runs/gazebo_rgbd_multi_object/rgbd_nominal/summary.json` | `*.complete_workpiece_detection_rate` |
| nominal 联合选择 `100%` | 同上 | `*.target_selection_success_rate` |
| nominal 任务成功 `100%` | 同上 | `*.task_success_rate` |
| 位置误差 P95 `≤5.15 mm` | 同上 | `*.selected_object_position_error_m.p95` |
| yaw 误差 P95 `≤4.76°` | 同上 | `*.selected_object_yaw_error_rad.p95`，由 rad 转 degree |
| clear/obstacle `28.70/40.91 ms` | 同上 | `clear/rrt_connect`、`obstacle/rrt_connect.avg_planning_time_sec` |
| combined `4.34 mm/7.56°` | `results/runs/gazebo_rgbd_multi_object/rgbd_combined/summary.json` | selected position/yaw error P95 |

正式结果的 JSONL、环境记录、SVG 与 SHA256 manifest 和 summary 位于同一结果目录；
[实验记录](experiments.md)说明统计口径和失败批次为何不并入正式汇总。

## 不能写进简历的表述

禁止写：

- “完成真机部署”或“可直接迁移真机”；
- “实现稳定物理抓取”或“仅靠摩擦夹持”；
- “通用 6D 位姿估计”；
- “适应任意光照、材质、颜色或遮挡”；
- “视觉方法优于 ground truth”；
- “候选选择达到全局最优”；
- “MoveIt mock 证明抓放执行成功”；
- “combined 扰动代表真实相机域随机化”。

可以准确写：

- “Gazebo assisted 抓放闭环”；
- “已知颜色和尺寸条件下的 RGB-D 多实例位置与平面 yaw 估计”；
- “当前抓取候选集合内的确定性代价选择”；
- “相同任务种子的 ground-truth/RGB-D 配对回归”。

## 面试材料索引

- [系统架构](architecture.md)：高层组件、FSM、坐标系与状态所有权；
- [代码走读](code_walkthrough.md)：函数级调用链、公式、事务和真实故障；
- [模拟面试指南](interview_guide.md)：两轮题目、答案卡和评分规则；
- [实验记录](experiments.md)：全部历史数据与正式口径；
- [已知边界](known_issues.md)：Humble teardown、assisted 与视觉限制。
