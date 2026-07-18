# 实验与结果口径

## 数据分组

| 数据集 | 运行语义 | 保存位置 |
| --- | --- | --- |
| PyBullet 当前基线 | 自研规划和 PyBullet 执行 | `results/v2/pybullet_shared_geometry_200_final/` |
| MoveIt mock | 独立阶段 planning-only | `results/v2/moveit_mock_final/` |
| Gazebo assisted | MoveIt 规划、控制器执行和约束辅助抓放 | `results/v2/gazebo_assisted_final/` |
| Gazebo RGB-D assisted | RGB-D 位置估计驱动 MoveIt/Gazebo 闭环 | `results/runs/gazebo_rgbd/` |
| Gazebo RGB-D grasp pose | 矩形工件 yaw 估计、候选筛选与 assisted 闭环 | `results/runs/gazebo_rgbd_grasp_pose/` |
| Gazebo RGB-D multi-object | 三矩形工件检测、联合候选筛选与 assisted 闭环 | `results/runs/gazebo_rgbd_multi_object/` |
| 历史 PyBullet 冻结数据 | 保留原始哈希的早期基线 | `baselines/v0.1-pybullet-baseline/` |
| 失败证据 | 用于定位问题，不混入最终汇总 | `results/v2/*failed_attempt*/` |

表中的 `v0.1` 和 `v2` 只是已发布数据的历史路径。为避免破坏 SHA256 审计和旧引用，
这些目录不重命名。它们不对应两份当前源码。

新实验统一写入 `results/runs/<backend_or_purpose>/`。普通临时运行默认由 `.gitignore`
排除；完成哈希审计的 `gazebo_rgbd_multi_object/` 是当前保留的证据集。

## PyBullet

固定主种子 42，clear 和 obstacle 各 50 个任务，分别使用 Linear 和 RRT-Connect，
共形成 200 个 episode。

| 场景 / 规划器 | 任务结果 | 平均规划时间 | 平均关节路径 |
| --- | --- | ---: | ---: |
| clear / Linear | 50/50 成功 | 1.750 ms | 0.778358 rad |
| clear / RRT-Connect | 50/50 成功 | 1.922 ms | 0.778358 rad |
| obstacle / Linear | 50 条受阻直线路径返回 `planning_collision` | 0.560 ms | N/A |
| obstacle / RRT-Connect | 50/50 成功 | 33.157 ms | 1.312717 rad |

clear 中 RRT-Connect 先检查直连边，因此与 Linear 得到相同的路径长度。obstacle / Linear
是特意构造的直连受阻对照，不表示该规划器在任意障碍场景中都失败。

## MoveIt mock planning-only

固定主种子 42，clear 和 obstacle 各 50 个 episode。

| 场景 | 规划结果 | 均值 / 中位数 / P95 | 平均路径 | 平均轨迹点 |
| --- | --- | ---: | ---: | ---: |
| clear | 50/50 | 0.125806 / 0.125351 / 0.139475 s | 7.401611 rad | 144.76 |
| obstacle | 50/50 | 0.129607 / 0.130023 / 0.145545 s | 7.704005 rad | 155.16 |

这里的顶层路径是 home、pre-grasp、transfer 和 place 四个独立规划请求的汇总。所有记录
保持 `task_success=false`、`execution_success=false` 和 `failure_type=not_executed`，
因此不能当作抓放成功数据。

## Gazebo assisted

固定主种子 42，MoveIt `RRTConnectkConfigDefault`，规划预算 3 s，速度和加速度
缩放 0.2。clear 和 obstacle 各 50 个完整 episode。

| 场景 | 任务结果 | 平均规划时间 | 平均路径 | 平均执行时间 | 平均 XY 误差 |
| --- | --- | ---: | ---: | ---: | ---: |
| clear | 50/50 成功 | 23.768 ms | 0.797926 rad | 13.296161 s | 0.142 mm |
| obstacle | 50/50 成功 | 33.413 ms | 2.669548 rad | 17.230351 s | 0.145 mm |

这些顶层规划时间与路径长度保留关键 `TRANSFER` 阶段口径；每个 FSM 阶段的详细值
在 `stage_metrics` 中。assisted 的放置误差反映约束辅助链路，不是对纯接触物理抓取的
证明。

## 真实失败与修复

- PyBullet IK API 需要覆盖两个手指在内的全部可运动关节参数，然后才映射回机械臂七关节。
- 固定关节连接的连杆属于同一刚性组件；只过滤刚性体内部接触，保留真实自碰撞与环境碰撞。
- Gazebo 夹爪在 assisted 预夹紧的小位移中曾被低增益控制器报告 `stalled`。修复只允许
  assisted 预夹紧将该结果交给独立位姿/开度门控，release 仍 fail-closed。
- 三工件 nominal 首轮在一次携带 `workpiece_2` 的 transfer 失败后连续出现目标状态无效。根因是
  下一 episode 的 MoveIt reset 先切回默认 `workpiece_0`，只清理了新 ID，旧的
  `workpiece_2` AttachedCollisionObject 残留。修复后 reset 对上一批和当前批全部工件逐一
  detach；候选筛选也同时验证 pre-grasp 与实际 grasp 目标。失败批次原样保留。

失败批次位于
`results/runs/gazebo_rgbd_multi_object/rgbd_nominal_failed_attempt_stale_attachment/`：clear 为
50/50，obstacle 为 37/50；首先出现 1 条 `invalid_motion_plan`，随后 12 条
`goal_state_invalid`。该批次不并入修复后的正式汇总。

- ground-truth 首次正式运行在两个连续 reset 中返回 `HOME_FAILED`。控制器日志显示
  `panda_joint1` 的路径误差约为 0.1514 rad，略高于 0.15 rad 容差；下一次 reset 因机械臂
  已经部分回 home 而自然恢复。修复在同一个 reset 事务内部增加最多三次 home 尝试，并继续
  使用严格的最终关节位置/速度稳态检查，不重跑失败 episode。原失败批次和两个 task seed
  保存在 `ground_truth_nominal_failed_attempt_home_tolerance/`，修复后同一组种子及完整正式
  批次通过。
- 更早的一次 RGB-D 批次出现两条无详情的 `multi_object_reset_failed`。该批次发生在服务错误
  详情透传实现之前，因此只保留原始数据，不推测根因；后端现在保留 reset manager 的错误码
  与 message，后续同类问题可以定位。结构化说明见 `failure_examples/failure_examples.json`。

## Gazebo RGB-D assisted

正式视觉评测沿用相同主种子、任务生成规则、MoveIt 参数和 assisted 成功判据。RGB-D 相机为
640×480、20 Hz，处理端加入固定 2 mm 标准差的轻量深度噪声。运动目标和 PlanningScene
使用视觉估计，任务真值只用于 reset 与误差评测。

| 场景 | 任务结果 | 感知耗时均值 / 中位数 / P95 | 方块误差均值 / P95 | 目标误差均值 / P95 | 障碍物误差均值 / P95 |
| --- | --- | ---: | ---: | ---: | ---: |
| clear | 50/50 | 72.0 / 50.6 / 165.5 ms | 2.38 / 4.07 mm | 2.18 / 3.35 mm | N/A |
| obstacle | 50/50 | 54.3 / 56.7 / 125.9 ms | 1.43 / 2.75 mm | 2.00 / 3.19 mm | 2.30 / 2.31 mm |

clear/obstacle 的关键 transfer 平均规划时间分别为 25.71 ms 和 33.93 ms，平均关节路径为
0.792 rad 和 2.142 rad；平均最终 XY 放置误差为 2.10 mm 和 2.12 mm。全部记录均完成
release，碰撞率为 0%，没有 PlanningScene 同步错误或未分类 `runtime_error`。结果目录中的
summary、环境信息、SVG 报告和 artifact manifest 均由原始 JSONL 重算或哈希生成。

## Gazebo RGB-D grasp pose

新增工件保持内部实体名 `cube/cube_link`，外形为 60×40×50 mm。yaw 由 task seed 在
`[-π/2, π/2)` 内确定性生成；视觉节点对矩形工件的 world-XY 点云做二维 PCA，低特征值比
返回 `orientation_ambiguous`。状态机根据估计 yaw 生成两个相差 π 的顶抓候选，以 MoveIt
planning-only 检查可规划性与碰撞，按 pre-grasp 关节路径长度选择，筛选期间不执行轨迹。

nominal 正式回归的原始记录位于
`results/runs/gazebo_rgbd_grasp_pose/nominal_rgbd/`，clear 和 obstacle 各 50 个 episode。
必需目标检测、抓取候选生成/筛选、规划、执行与任务在两组均为 50/50 成功；全部记录的
感知、候选、跟踪、放置与碰撞字段完整。

| 场景 | 任务结果 | 工件位置误差均值 / P95 | yaw 误差均值 / P95 | 感知耗时均值 / P95 | 候选路径 P95 |
| --- | --- | ---: | ---: | ---: | ---: |
| clear | 50/50 | 2.55 / 4.16 mm | 1.44° / 3.28° | 54.81 / 130.63 ms | 3.218 rad |
| obstacle | 50/50 | 1.74 / 2.50 mm | 2.20° / 4.94° | 65.57 / 163.45 ms | 3.764 rad |

clear/obstacle 的关键 transfer 平均规划时间为 26.94 ms 和 35.22 ms，平均关节路径长度为
0.890 rad 和 2.303 rad，平均最终 XY 放置误差为 2.29 mm 和 2.94 mm。两组无无效碰撞、
PlanningScene 同步错误或未分类 `runtime_error`。

同一 `(scene, task_seed)` 的 ground-truth 对照位于 `nominal_ground_truth/`：clear 为 50/50，
obstacle 为 47/50，3 条均明确分类为 `invalid_motion_plan`。两组覆盖经
`paired_nominal_report/` 严格配对审计；该差异受 OMPL 采样和目标位姿细微变化影响，不能解释为
“视觉比真值更准确”。

第一次 nominal RGB-D 正式尝试中，单一 seeded IK 在 obstacle 的 PLACE 阶段产生 11 条
`GOAL_STATE_INVALID`，原始记录保存在 `nominal_rgbd_failed_attempt_single_ik/`。修复将该
MoveIt error code 映射为稳定失败类型，并只在此错误出现时、原 3 秒预算的剩余时间内切换一次
pose-sampling IK；不执行试探轨迹，也不对其他规划错误扩大重试。固定任务重跑后 obstacle 从
39/50 恢复到 50/50。

`combined` 扰动结果位于 `combined_rgbd/`，clear/obstacle 各 20 个 episode，均完整成功。
两组工件位置误差 P95 分别为 3.85 mm、2.83 mm，yaw 误差 P95 分别为 6.84°、6.59°，
感知耗时 P95 分别为 132.25 ms、144.94 ms。`low_light`、`rgb_noise`、`depth_dropout` 与
`combined` 均由 `(episode_id, task_seed)` 确定，不代表真实相机域随机化或真机鲁棒性。

旧方块兼容回归分别位于 `cube_ground_truth_quick/` 和 `cube_rgbd_quick/`。两种 pose source
均完成 5 clear + 5 obstacle，全部任务成功；方块继续使用单个旧版固定朝向候选，不声明 yaw
可观测性。两组也通过 `(scene, task_seed)` 配对覆盖检查，说明新增矩形姿态链路没有复制状态机
或破坏原方块语义。

## 三工件 RGB-D 与联合抓取选择

当前多工件正式数据位于 `results/runs/gazebo_rgbd_multi_object/`。任务生成器为三个矩形工件
独立生成位置和 yaw，并按真实 X、Y 固定实例 ID；在线感知只按估计 world X、Y 排序，不读取
任务真值。每个工件生成两个 π 对称顶抓候选，MoveIt planning-only 筛选后按路径长度与置信度
联合代价确定性选择。

nominal RGB-D 与 ground-truth 使用相同 `(scene, task_seed)`，各运行 50 clear + 50 obstacle。
两组均为 100/100 任务成功，完整三工件检测和联合选择均为 100%，所有 episode 的六个候选均
可行；三个 object ID 都曾被选择，未选工件最大位移为 0 mm。

| pose source / 场景 | 任务结果 | 平均规划时间 / P95 | 平均路径 | 选中工件位置误差 P95 | yaw 误差 P95 | 最终 XY 误差均值 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| ground truth / clear | 50/50 | 27.99 / 37.09 ms | 1.039 rad | 0 mm | 0° | 0.12 mm |
| ground truth / obstacle | 50/50 | 39.85 / 74.74 ms | 3.183 rad | 0 mm | 0° | 0.15 mm |
| RGB-D / clear | 50/50 | 28.70 / 37.20 ms | 1.088 rad | 5.15 mm | 3.95° | 2.42 mm |
| RGB-D / obstacle | 50/50 | 40.91 / 71.90 ms | 3.224 rad | 3.25 mm | 4.76° | 2.95 mm |

`combined` 是由 episode ID 和 task seed 确定的低亮度、RGB 高斯噪声与深度丢失组合扰动，
clear/obstacle 各运行 20 个 episode，均为 20/20 完整检测、联合选择和任务成功。位置误差
P95 分别为 4.34 mm 和 3.43 mm，yaw 误差 P95 分别为 7.56° 和 7.43°。三组碰撞率均为 0，
没有 `selected_attachment_mismatch`、PlanningScene 同步失败或未分类 `runtime_error`。

确定性 reset 连续验证 10 次均通过，三个工件最大位置误差为 `6.15e-7 m`、最大 yaw 误差为
`5.45e-7 rad`。每个正式目录包含 JSONL、summary、environment、run status、SVG report 和
SHA256 manifest；配对覆盖见 `paired_nominal_report/report_manifest.json`。

## 跨后端比较边界

PyBullet 自研 RRT-Connect 和 MoveIt OMPL RRTConnect 的算法实现、碰撞引擎、规划请求和计时
边界不同。相同 `(scene, task_seed)` 用于检查任务覆盖，不能将两者的绝对耗时
直接解释为算法优劣。
