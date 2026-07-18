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

## Gazebo 控制插件启动抖动

连续重启整套 RGB-D 栈时，曾出现一次 Gazebo 已启动但 `gz_ros2_control` 服务在 60 秒内未
出现。`control_system_loader` 返回非零状态后，readiness gate 在任务生成器启动前关闭整套
launch，因此没有 episode 或不完整结果写入正式数据；残留进程清理后重启成功。

这表明门禁能够 fail-closed，但 Fortress 启动仍可能受本机渲染/资源释放时序影响。自动化运行
应为每个批次启动唯一栈、等待整套进程退出后再进入下一批，并以 summary、JSONL 行数和
artifact audit 判定完成，不能仅凭 Gazebo 窗口出现判定 ready。

## assisted 不等于 physics

assisted 抓取只在末端位姿、姿态、手物距离和夹爪开度全部通过时创建
DetachableJoint。它是用来验证全局规划、控制、场景同步和评测的稳定基线。

`physics_model_validated=false` 时 physics 模式拒绝运行。在质量/惯量、摩擦、碰撞几何和接触参数
未经独立标定前，不宣称物理抓取能力。

## 视觉与真机

当前已接入 Gazebo 顶置 RGB-D 相机和 `GroundTruthPoseProvider` / `RGBDScenePoseProvider`
对照。RGB-D 模式使用 HSV 颜色分割和已知尺寸；对矩形工件只估计 world XY 位置与绕 Z
方向，并利用 π 对称语义生成顶抓候选。方块和圆形目标区不声明可观测 yaw；它不是通用目标
检测或六自由度姿态估计。扰动 profile 是固定种子的合成传感器扰动，没有真实相机标定、
真实域随机化或真机部署证据。

运动目标和 PlanningScene 在 RGB-D 模式中来自视觉估计，任务真值只用于 reset 和误差评测。
若后续加入 YOLO，应替换检测器/provider 实现，不修改 FSM 或 MoveIt 后端。

## 强化学习

当前不适合将强化学习接在 assisted 抓取上。策略可能利用固定约束或未验证动力学，
无法证明学会了可迁移的接触操作。只有在物理模型审计、无约束抓取基线和明确的
局部控制接口完成后，才重新评估强化学习。
