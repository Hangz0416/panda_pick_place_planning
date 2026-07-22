# 模拟面试指南与答案卡

使用方式：模拟面试时只读“问题”，先在 60–90 秒内作答，再由面试官依据答案卡追问和评分。
不要逐字背诵，统一使用“结论—证据—边界—改进”结构。

## 评分规则

每轮从五个维度分别打 0–4 分，总分 20 分：

| 维度 | 4 分标准 |
| --- | --- |
| 系统流程 | 能从任务、感知、FSM、后端一直讲到评测，组件职责不混淆 |
| 视觉数学 | 能写出反投影、TF、MAD、PCA 和 π 对称，并说明假设 |
| 规划控制 | 能区分自研规划、OMPL、planning-only、控制器与 attachment |
| 故障分析 | 能以日志和状态证据定位问题，解释事务、回滚和 fail-closed |
| 实验边界 | 不把 mock 写成执行、不把 assisted 写成 physics、不外推真机 |

评分含义：

- 17–20：能够独立主导该项目面试；
- 13–16：主线清楚，但少数数学或工程追问不稳；
- 9–12：知道功能，不足以证明理解实现；
- 0–8：主要依赖背诵，需要重新走读代码。

## 第一轮：基础流程

### 问题 1：用一分钟介绍这个项目

追问：你个人完成了哪些部分？当前最有价值的贡献是什么？

答案卡：

- 结论：这是 RGB-D 感知、抓取候选选择、MoveIt 规划、Gazebo 控制执行和评测闭环。
- 证据：三工件完整检测、联合选择和 assisted 任务成功率均为 100%；位置/yaw P95 最差
  5.15 mm/4.76°。
- 边界：HSV、已知尺寸、平面 yaw、assisted、无真机。
- 改进：优先做实例分割或相机外参误差实验，不立即加强化学习。

### 问题 2：为什么保留 PyBullet、MoveIt mock 和 Gazebo 三层？

追问：它们的耗时能直接比较吗？

答案卡：PyBullet 快速验证自研 IK/Linear/RRT-Connect；mock 验证 URDF/SRDF、PlanningScene、
OMPL 和接口；Gazebo 验证控制器、仿真时钟、执行同步、接触和对象状态。三者碰撞引擎和计时
边界不同，绝对耗时不能直接解释为算法优劣。

### 问题 3：为什么要抽象 ManipulationBackend？

追问：最难统一的是函数名还是语义？

答案卡：FSM 的阶段、失败分类和成功判据应只有一份；后端只实现 reset、move、feasibility、
gripper、object selection、attach/release 和 state。最难的是统一 planning-only、执行指标、
碰撞可用性和 attach 语义，而不是统一函数名。

### 问题 4：完整 FSM 如何运行？

追问：PERCEPTION 与 GRASP_SELECTION 为什么必须独立？

答案卡：依次说明 RESET、PERCEPTION、GRASP_SELECTION、PRE_GRASP、GRASP、LIFT、TRANSFER、
PLACE、RETREAT。感知失败应停止而不是规划；候选筛选可能因方向歧义、碰撞或不可达失败，
必须和目标检测失败分开统计。

### 问题 5：如何从深度像素得到 world 坐标？

追问：深度异常值如何处理？

答案卡：写出 `(u-cx)z/fx, (v-cy)z/fy, z`，再写 `Rp+t`。先裁剪深度范围，再对三维点使用
median/MAD 过滤；有效像素不足返回 `invalid_depth`，不能返回默认坐标。

### 问题 6：如何用 PCA 估计矩形工件 yaw？

追问：为什么正方形不适合？

答案卡：world-XY 去均值后计算 2×2 协方差，最大特征值对应的特征向量是主轴；
`atan2(v_y,v_x)` 得 yaw。矩形轴是无向的，因此 yaw 与 yaw+π 等价。特征值比低于 1.25
说明长短轴不可区分，返回 `orientation_ambiguous`。

### 问题 7：三个相同工件如何编号？

追问：遮挡或交叉运动后还可靠吗？

答案卡：在线估计按 world X、再按 Y 排序，任务生成保证工件中心间距；过程不读真值。
它适用于当前静态、分离工件，不是通用多目标跟踪；遮挡和交叉需要时序关联或 learned
instance embedding。

### 问题 8：为什么有六个候选，如何选择？

追问：选择分数是不是全局最优？

答案卡：三个方向有效矩形工件各有 yaw/yaw+π 两个夹爪对称候选。pre-grasp 与 grasp 都先
planning-only；可行候选按 `path_length + 1 - confidence`、object ID、候选号稳定排序。
它只在当前候选集合内确定性最优，不处理多步任务顺序。

### 问题 9：如何证明候选筛选没有让机械臂运动？

追问：只检查 pre-grasp 够吗？

答案卡：`MoveItBackend.check_pose_feasibility()` 构造同一个 `PlanExecuteMotion` Action，但明确
设置 `goal.execute=false`；集成测试比较筛选前后关节状态。只检查 pre-grasp 不够，实际 grasp
可能碰撞或 IK 不可达，因此两个位姿都必须通过。

### 问题 10：PlanningScene 如何使用视觉结果？

追问：任务真值在哪些地方仍然允许使用？

答案卡：RGB-D 成功后 `apply_scene_estimate()` 把全部工件、目标相关运动目标和障碍物按估计
位置/yaw写入规划链路；未选工件始终作为 world object。真值只用于 reset、ground-truth
oracle 对照和离线误差计算。

### 问题 11：Gazebo attach 为什么要先于 MoveIt attach？

追问：第二步失败怎么办？

答案卡：先通过手物位置/姿态/距离/夹爪宽度 gate，再确认目标 ID 的 DetachableJoint，最后
移除 MoveIt world object 并添加 AttachedCollisionObject。MoveIt 同步失败必须回滚 Gazebo
detach，否则物理世界和规划世界会分叉。

### 问题 12：任务成功的完整判据是什么？

追问：“没有碰撞消息”为什么不等于安全？

答案卡：release 已确认、对象状态可用、XY≤40 mm、Z≤15 mm、速度≤0.05 m/s、碰撞观测
可用且无无效碰撞、未选工件位移≤10 mm。接触流或 selected identity 未知时必须 fail-closed，
不能把未知当作 false。

## 第二轮：压力追问

### 问题 1：HSV 这么简单，为什么还能写进视觉项目？

答案卡：HSV 是可解释、CPU 可运行的闭环基线，项目重点是验证 RGB-D 三维估计如何真实驱动
PlanningScene 和机械臂。它的价值是建立接口、误差和失败基线，不是证明开放场景识别能力。
后续实例分割应替换 detector，而不修改 provider、FSM 和 MoveItBackend。

### 问题 2：为什么不能称为通用 6D pose？

答案卡：系统只估计轴对齐桌面工件的 world XY、已知中心高度与绕 Z yaw；roll/pitch 来自场景
假设，尺寸已知，也没有遮挡下完整模型配准。因此准确表述是平面位置与 yaw 估计。

### 问题 3：为什么 assisted 成功率不能写成物理抓取成功率？

答案卡：attach 在门控通过后建立固定约束，后续保持不依赖指尖摩擦、接触刚度和夹持力。
它能验证目标选择、轨迹、控制、场景同步和 release，但不能证明纯接触抓取或真机动力学。

### 问题 4：你如何证明视觉模式没有偷用真值？

答案卡：数据接口把 TaskSpec 与 SceneEstimate 分开；RGB-D 模式的候选和 PlanningScene 使用
EstimateScene 返回值。防泄漏测试故意让估计与真值不同，检查 motion goal/world object 跟随
估计；感知失败必须停在 PERCEPTION，不允许 fallback。

### 问题 5：视觉组比 ground-truth 某次规划更快，能否说明视觉更优？

答案卡：不能。OMPL 有采样波动，估计位姿会改变几何和 IK 分支；task seed 配对只保证任务
覆盖一致。应报告配对分布与误差影响，不能把不同目标状态下的绝对耗时解释为感知优越性。

### 问题 6：stale attachment 为什么会污染后续 episode？

答案卡：旧实现 reset 时先切 selected ID，只 detach 新 ID，上一 episode 的
`workpiece_2` 仍作为 AttachedCollisionObject 绑定在手上，后续起点/目标碰撞检查因此连续
失败。修复清理 previous∪current 全部 ID，并通过 PlanningScene readback 证明 detach 完成。

### 问题 7：为什么 home 失败采用内部重试，而不是重跑 episode？

答案卡：控制器仅比 path tolerance 超出约 0.0014 rad，且第一次动作已部分回 home；这是
reset 内部可恢复的执行瞬态。最多三次有界尝试后仍执行严格位置/速度验证，保持相同 task，
不会用重新生成或重跑 episode 美化成功率。

### 问题 8：早期 reset 失败为什么没有写一个猜测的根因？

答案卡：当时 service message 被扁平化，只剩顶层 failure type，没有充分证据。正确处理是保留
失败数据、把 root cause 标为未知，再增强错误码/detail 传播；可观测性修复比事后猜测可信。

### 问题 9：为什么 episode 成功与 teardown 异常要分开？

答案卡：EpisodeResult 表示任务语义，teardown 表示进程退出质量。评测器必须先 append、flush、
fsync 并发布保存确认，completion gate 才关闭。Humble `move_group` 的析构段错误发生在结果
落盘后，应在 `run_status.json` 单独记录，不能覆盖成功任务，也不能隐藏。

### 问题 10：迁移真机时优先补什么？

答案卡：先补真实相机内外参标定与时间同步、机器人/相机 TF 校验、速度/力矩/工作空间限制、
急停和 watchdog、夹爪力与接触反馈、轨迹执行监控、PlanningScene 实时更新和安全 reset。
在这些基础上验证单目标低速抓取，再扩大任务分布；不直接把 assisted 参数搬到真机。

## 60 秒回答结构

```text
10 秒：问题与主线
20 秒：自己实现的关键模块
20 秒：真实数据或真实故障证据
10 秒：明确能力边界
```

避免从技术栈开始逐个报工具，也不要把所有测试数量塞进项目介绍。

## 回答纠正规则

每题回答后按以下顺序反馈：

1. 先指出事实错误或缺失的关键条件；
2. 给出仓库中的证据位置；
3. 补充不能外推的边界；
4. 将回答压缩成 30–60 秒版本；
5. 再追问一个比原题更深的问题。

## 两轮结束后的输出

- 五维评分与总分；
- 三个最薄弱知识点；
- 需要重新阅读的文件和函数；
- 错题的最终答案卡；
- 一版经过口语化修正的 60 秒和 3 分钟项目介绍。
