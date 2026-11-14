# Go2 探索目标连通性修正（2026-09-13）

## 已确认的证据与之前解释的纠正

用户日志显示 GridBased 全局规划失败；仅凭日志不能确认是目标、机器人起点还是地图通道造成。
原 build_dispatch_goal_pose 是最小距离调整函数，不是完整的安全规划器。
删除距离提前返回不等于增加障碍净空。搜索失败仍返回 fallback_goal_pose。
本修正不再让实际 send_frontier_goal 使用该旧函数；旧接口保留以避免破坏其他代码/旧单元测试。

## 修改行为

1. 每次发送前，在当前全局代价地图上建立机器人所在的四邻接可达区域。
2. 每一个可通行中心都检查半径 0.35 米圆形范围与网格的相交关系。全局代价图和原始 /map 中的未知、越界或大于等于 occ_threshold 的格子会阻断。四邻接避免从对角障碍缝中穿越。
3. 只在原候选目标附近 1.5 米内寻找可达目标，优先距离原目标最近的点。不强迫目标离前沿至少 0.8 米。
4. 找不到就尝试序列中的下一候选；全部失败就不发送，不回退到原目标，也不把这次发送失败宣布为探索完成。
5. 机器人起点不满足净空时不跳到邻近空格，输出明确日志，等待地图/位姿变化。不会自动将机器人从墙角移出。
6. bypass_min_distance_dispatch 只允许忽略最小机器人距离，不能绕过净空与连通性。
7. /explore/selected_frontier 发布实际目标，去掉发布函数内部的 debug 限制。该 topic 仍是 volatile：RViz 必须在发送目标之前订阅，后加入需等待下一个目标。

## 坐标系问题

上传配置中全局 costmap 是 map，局部 costmap 是 odom。现有 OccupancyGrid2d 查询不做 TF 转换。
本次探索器忽略与 global_frame 不同的局部 costmap，防止拿 map 坐标直接查询 odom 格子。
Nav2 的局部避障保持正常工作。如果局部图本来与全局同 frame，探索器仍检查它覆盖范围内的格子。
这不是将局部图转换到 map；完整跨坐标系融合是后续工作。

## 参数（探索器 params.yaml）

- dispatch_clearance_m: 0.35：上传的 Nav2 robot_radius=0.30 加 0.05 米余量；圆形近似，不是经实测确认的 Go2 完整腿部包络。
- dispatch_search_radius_m: 1.5：目标最大挪动范围。
- frontier_candidate_min_goal_distance_m: 0.8：保留为距离机器人下限。

全局规划允许未知，但本筛选保守地不穿未知；窄门或未充分观测区域可能被跳过。
净空无法满足时应查看地图而不是盲目减小机器人半径。未修改 Nav2 膨胀层、控制器和里程计。
候选点可达是网格检查结果，不保证 Nav2 必然成功：实时地图变化、TF、不同碰撞模型仍可能造成失败。
每次发送重建区域，计算开销随地图尺寸增加；没有放进每个速度控制周期。

## 安装与验证（Ubuntu）

先停止探索器。停止 action client 不保证已取消 Nav2 的活动目标；若机器人还在走，先在 Gazebo 暂停仿真。
将 frontier_connectivity_fix.tar.gz 放到 /home/dobby/scan。

```bash
cd /home/dobby/scan
tar -czf frontier_before_connectivity_fix.tar.gz src/frontier_exploration_ros2
tar -xzf frontier_connectivity_fix.tar.gz -C /home/dobby/scan
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash
colcon build --packages-select frontier_exploration_ros2 --symlink-install --cmake-args -DBUILD_TESTING=ON
```

仅编译成功后运行回归测试：

```bash
colcon test --packages-select frontier_exploration_ros2 --ctest-args -R '^test_connected_dispatch$' --output-on-failure
colcon test-result --verbose
```

测试包括空房间、实墙/未知带断开、宽门/窄门、起点占据、坐标系不一致、避开目标障碍。
当前交付环境为 Windows，没有 ROS2/C++ 编译器，未执行这些 C++ 测试或 Gazebo 测试；需以上命令验证。

只在编译和测试成功后重新启动探索器：

```bash
source /home/dobby/scan/install/setup.bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py use_sim_time:=true params_file:=/home/dobby/scan/src/frontier_exploration_ros2/config/params.yaml log_level:=info
```

如果其他五个进程还在运行，不必全部重启。新目标由 Nav2 接受后再恢复暂停的仿真。
RViz Fixed Frame=map；添加 Pose，Topic=/explore/selected_frontier；添加 Map，Topic=/global_costmap/costmap。

## 新日志解释

- Connected dispatch checked：已通过此次网格净空和连通性检查的目标坐标。
- Skipping frontier: no connected clear goal：该候选附近没有符合条件的点。
- Dispatch withheld: robot cell/clearance blocked or unknown：起点或周围范围不通，需查看机器人周围地图。
- Dispatch withheld: invalid grid geometry/frame or clearance：地图结构/坐标系或参数不符合要求。

如果通过检查仍 Goal failed，保留前后 planner_server 日志和同一时刻的全局代价地图，不再凭最终失败字样推断原因。

## 回退

停止探索器，将自己刚生成的 frontier_before_connectivity_fix.tar.gz 解压回 /home/dobby/scan，然后重新编译探索器。
备份不包含新增的 connected_dispatch.hpp/test_connected_dispatch.cpp，但旧 CMake 和发送函数不会使用它们，无需删除即可回退。
