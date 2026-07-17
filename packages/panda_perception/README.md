# panda_perception

ROS 2 Humble 的轻量 RGB-D 场景定位包。它为 Panda Gazebo 抓放系统估计红色方块、绿色目标区
和蓝色障碍物在 `world` 坐标系中的中心位置。

处理链路：

```text
RGB + depth --ApproximateTimeSynchronizer--> HSV 最大连通域
                                             |
CameraInfo --> 深度范围 / median / MAD ------+--> 反投影 --> TF2 --> EstimateScene
```

`EstimateScene` 每次只接受请求到达后的新同步帧，避免 reset 前的缓存图像跨 episode 复用。
当前实现假设已知颜色、已知尺寸和轴对齐物体，只输出位置与单位四元数，不是通用六自由度
姿态识别。

```bash
ros2 launch panda_perception perception.launch.py \
  use_sim_time:=true use_debug_overlay:=true
```

主要话题与服务：

- `/panda_perception/color/image_raw`
- `/panda_perception/depth/image_raw`
- `/panda_perception/camera_info`
- `/panda_perception/debug/overlay`
- `/panda_perception/estimate_scene`

相机与处理参数集中在 `config/perception.yaml`，Gazebo 相机模型和静态 TF 从同一配置读取。
