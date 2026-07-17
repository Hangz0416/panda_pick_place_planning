# 已知问题与能力边界

## MoveIt / Gazebo 关闭路径

ROS 2 Humble 的 MoveIt 2.5.9 在任务完成、JSONL 和 summary 已写入后，整套 launch 关闭时
可能在 `move_group` 的 callback group 析构路径中段错误。Gazebo GUI 在 Qt/QML 析构时也可能
输出类似堆栈。

这是 teardown-only 问题，不能用来证明 episode 成功，也不能一概否定已完成的运行。
批量结果必须同时满足：

- 任务生成器和评测写入器完成；
- summary 的 `batch_status` 完整；
- JSONL 行数与预期一致；
- 结果重算和 artifact audit 通过。

如果后续迁移到 ROS 2 Jazzy / Gazebo Harmonic，需要重新检查关闭行为，不默认该问题仍存在。

## assisted 不等于 physics

assisted 抓取只在末端位姿、姿态、手物距离和夹爪开度全部通过时创建
DetachableJoint。它是用来验证全局规划、控制、场景同步和评测的稳定基线。

`physics_model_validated=false` 时 physics 模式拒绝运行。在质量/惯量、摩擦、碰撞几何和接触参数
未经独立标定前，不宣称物理抓取能力。

## 视觉与真机

当前任务使用仿真中的已知方块位姿，未从 RGB-D 图像估计位姿。没有手眼标定、相机噪声
建模或真机部署证据。

下一阶段应该先加入 `GroundTruthPoseProvider` / `RGBDPoseProvider` 对照，再评估视觉误差
对抓放的影响。

## 强化学习

当前不适合将强化学习接在 assisted 抓取上。策略可能利用固定约束或未验证动力学，
无法证明学会了可迁移的接触操作。只有在物理模型审计、无约束抓取基线和明确的
局部控制接口完成后，才重新评估强化学习。

