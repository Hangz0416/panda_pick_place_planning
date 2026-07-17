# 实验与结果口径

## 数据分组

| 数据集 | 运行语义 | 保存位置 |
| --- | --- | --- |
| PyBullet 当前基线 | 自研规划和 PyBullet 执行 | `results/v2/pybullet_shared_geometry_200_final/` |
| MoveIt mock | 独立阶段 planning-only | `results/v2/moveit_mock_final/` |
| Gazebo assisted | MoveIt 规划、控制器执行和约束辅助抓放 | `results/v2/gazebo_assisted_final/` |
| 历史 PyBullet 冻结数据 | 保留原始哈希的早期基线 | `baselines/v0.1-pybullet-baseline/` |
| 失败证据 | 用于定位问题，不混入最终汇总 | `results/v2/*failed_attempt*/` |

表中的 `v0.1` 和 `v2` 只是已发布数据的历史路径。为避免破坏 SHA256 审计和旧引用，
这些目录不重命名。它们不对应两份当前源码。

新实验统一写入 `results/runs/<backend_or_purpose>/`，默认由 `.gitignore` 排除。

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

## 跨后端比较边界

PyBullet 自研 RRT-Connect 和 MoveIt OMPL RRTConnect 的算法实现、碰撞引擎、规划请求和计时
边界不同。相同 `(scene, task_seed)` 用于检查任务覆盖，不能将两者的绝对耗时
直接解释为算法优劣。

