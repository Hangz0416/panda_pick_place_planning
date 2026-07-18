# panda_perception

ROS 2 Humble 的轻量 RGB-D 场景估计包。它为 Panda Gazebo 抓放系统估计红色工件、绿色目标区
和蓝色障碍物在 `world` 坐标系中的中心位置；对 `rect_prism` 工件额外使用 world XY 点云 PCA
估计平面 yaw。

处理链路：

```text
RGB + depth --ApproximateTimeSynchronizer--> HSV 最大连通域
                                             |
CameraInfo --> 深度范围 / median / MAD ------+--> 反投影 --> TF2 --> 位置
                                                          `--> XY PCA --> yaw
```

`EstimateScene` 每次只接受请求到达后的新同步帧，避免 reset 前的缓存图像跨 episode 复用。
当前实现假设已知颜色和尺寸。矩形工件输出位置及绕 world Z 的 quaternion；若 PCA 特征值比
低于门限，则返回 `orientation_ambiguous`。方块、圆形目标区不声明可观测 yaw，这不是通用
六自由度姿态识别。

```bash
ros2 launch panda_perception perception.launch.py \
  use_sim_time:=true use_debug_overlay:=true perception_profile:=nominal
```

主要话题与服务：

- `/panda_perception/color/image_raw`
- `/panda_perception/depth/image_raw`
- `/panda_perception/camera_info`
- `/panda_perception/debug_overlay`
- `/panda_perception/estimate_scene`

相机与处理参数集中在 `config/perception.yaml`，Gazebo 相机模型和静态 TF 从同一配置读取。
`nominal`、`low_light`、`rgb_noise`、`depth_dropout`、`combined` 是由 episode 和 task seed
确定的合成传感器扰动配置，不代表真实相机数据。
