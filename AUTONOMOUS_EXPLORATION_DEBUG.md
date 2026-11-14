# Go2 自主建图与探索：诊断记录（2026-09-10）

## 当前结论

自主探索链路已启动并验证可运动：Gazebo、FAST-LIO、`fastlio_tf_bridge`、OctoMap、Nav2 和 `frontier_explorer` 都在运行。

一次 0.4 m 的 Nav2 正前方探针已成功：

- 起点：`map -> base_footprint = (0.388, 0.028, -0.6 deg)`
- 目标：`(0.788, 0.024)`
- 动作状态：`4`（`SUCCEEDED`）
- Gazebo ground truth：`(0.403, 0.021)` 移至 `(0.612, 0.016)`

这证明当前 `+linear.x` 是朝 Go2 头部前进，且规划、控制、TF 与仿真实际运动方向一致。

## 根因与处理

### 1. Nav2 使用的旧里程计不可用

旧链路是：`/odom/raw -> footprint_to_odom_ekf -> /odom`。诊断时 `/odom/raw` 的位置已经异常到约 `1e15`，而 twist 为零；因此机器人在 Gazebo 中移动时，Nav2 认为自己几乎没有移动，导致局部路径与实际位置严重脱节。

处理：

- 在 `src/unitree-go2-ros2/champ/champ_bringup/launch/bringup.launch.py` 的节点列表中保持 `footprint_to_odom_ekf` 禁用。
- `fastlio_nav_bridge/fastlio_tf_bridge.py` 订阅 `/odom/ground_truth`，发布 Nav2 使用的 `/odom` 和 `odom -> base_footprint`。
- 该桥同时根据 FAST-LIO `/Odometry` 发布 `map -> odom`。

验证：`ros2 topic info /odom -v` 的唯一发布者应为 `fastlio_tf_bridge`；`map -> odom -> base_footprint` 两段 TF 都能查询到。

### 2. OctoMap 一开始订阅了不连通坐标系的点云

`/cloud_registered` 的 frame 是 `camera_init`，而 TF 树中没有 `map -> camera_init`。OctoMap 因而不断丢弃点云，`/map` 为空，前沿点与 Nav2 代价地图都不可靠。

处理：OctoMap 改订阅原始 `/livox/lidar`。其 frame 为 `livox_frame`，已验证可通过机器人 TF 转换到 `map`。

当前正确启动参数包含：

```bash
-r cloud_in:=/livox/lidar -r projected_map:=/map
```

验证：`ros2 topic echo /map --once` 应显示非零 `width`、`height`，日志不应再持续出现 `frame 'camera_init'` 的丢弃消息。

### 3. 控制器正方向

文件 `src/unitree-go2-ros2/champ/champ_base/src/quadruped_controller.cpp` 必须保持：

```cpp
req_vel_.linear.x = msg->linear.x;
```

不要加负号。实测 `+linear.x` 会让 Go2 向头部方向走。

## 当前后台进程与日志

所有本次启动日志在：

```text
~/scan/.codex_logs/
  gazebo.log
  fastlio.log
  tf_bridge.log
  octomap.log
  nav2.log
  frontier.log
```

当前探索器启用了 autostart；它已向 `/navigate_to_pose` 发送前沿目标。Nav2 的 `planner_server`、`controller_server`、`bt_navigator` 均为 `active`。

## 推荐启动顺序

每个命令运行前均执行：

```bash
source /opt/ros/humble/setup.bash
source ~/scan/install/setup.bash
```

1. `ros2 launch go2_config gazebo.launch.py`
2. `ros2 launch fast_lio mapping.launch.py use_sim_time:=true rviz:=false config_file:=mid360.yaml`
3. `ros2 run fastlio_nav_bridge fastlio_tf_bridge --ros-args -p use_sim_time:=true`
4. `ros2 run octomap_server octomap_server_node --ros-args -p use_sim_time:=true -p frame_id:=map -p base_frame_id:=base_link -p resolution:=0.10 -p sensor_model.max_range:=8.0 -p point_cloud_min_z:=0.35 -p point_cloud_max_z:=1.20 -p occupancy_min_z:=0.35 -p occupancy_max_z:=1.00 -r cloud_in:=/livox/lidar -r projected_map:=/map`
5. `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true params_file:=/home/dobby/scan/config/nav2_go2_fastlio.yaml`
6. 先发一个短距离 Nav2 目标确认运动，再启动：
   `ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py use_sim_time:=true params_file:=/home/dobby/scan/src/frontier_exploration_ros2/config/params.yaml log_level:=info`

## 仍需注意

- 当前 Nav2 日志显示 `inflation_radius: 0.010`，小于 Go2 footprint 的内接半径约 `0.304 m`。这不是本次“规划与实际运动不一致”的根因，但会降低避障安全裕量；完成功能验证后，应至少恢复到接近机器人半径的量级并重新测试窄通道。
- Gazebo 当前偶尔报告控制循环达不到 20 Hz。这会让运动较慢，但当前不阻止路径执行。
