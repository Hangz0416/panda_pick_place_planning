# 开发历史与数据兼容

这份文档只解释历史路径，不是当前使用入口。新用户应直接阅读顶层 README 和
[architecture.md](architecture.md)。

## 早期 PyBullet 基线

项目最初完成了 PyBullet 中的 IK、碰撞检查、Linear/RRT-Connect、抓放状态机和批量评测。
该阶段的原始 JSONL、CSV、summary 和 SHA256 manifest 冻结在：

```text
baselines/v0.1-pybullet-baseline/
```

这些文件是历史实验证据，不是另一套需要运行的代码。

## 统一后端与 MoveIt/Gazebo

后续工作将状态机与执行引擎分离，增加 `ManipulationBackend`、类型化 ROS 接口、
C++ MoveIt bridge、共享 Panda 描述、Gazebo Fortress 和 `gz_ros2_control`。同时对 PyBullet 当前几何
与算法进行了固定任务回归。

当时输出保存在 `results/v2/`。该目录名称已进入 manifest、报告和 README 历史提交，
因此不进行物理重命名，避免破坏数据审计。

## 当前规则

- 当前代码就是 `packages/` 下的六个 ROS 2 包，不再用 V1/V2 区分功能。
- `baselines/` 和已提交的 `results/v2/` 只读保留。
- 新演示、快检和批量运行写入 `results/runs/`。
- 新结果如需成为策展证据，需先重算 summary、生成 environment 和 artifact manifest，
  再单独决定是否提交。
- 简历和顶层 README 只介绍当前能力，不以版本迁移作为主叙事。
