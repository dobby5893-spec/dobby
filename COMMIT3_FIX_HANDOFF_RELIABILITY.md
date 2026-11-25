# Go2 多楼层探索 Commit 3 Fix 实施说明

本次基于 Ubuntu 虚拟机 `/home/dobby/scan` 的 `1514e17` 开发。开工时工作区干净，确认该提交依次继承 `6cf0f10`、`94fba98`、`b8f2cfb` 和 `ba34df1`，且既有回滚标签仍在。修复分支为 `feature/floor-session-protocol-fix`，基线标签为 `baseline/commit3-fix-start-20261125`。本说明随修复一同提交，可在仓库执行 `git log -1 --format=%H -- COMMIT3_FIX_HANDOFF_RELIABILITY.md` 读取包含本说明的提交哈希。

原来的三个问题均可由源码调用链确认。地图变化时，Manager 从 `tick()` 调用 `invalidateHandoff()`，清掉待处理对象，但保留 `handled_reports_` 与 `report_sequence_`。此后同 ID 的续报因缺少可刷新的对象而被去重。TF 失效、地图落后等情况下，旧异步结果虽然已有地图和请求校验，但暂时失效没有明确的重新验证状态。其次，Explorer 的 `measuredClearanceRestored(true)` 只在探索运行期提供发布侧站姿证据。Manager 的 `handoffReady()` 在 STOP 之后仍只比较 TF、位置、高度与地图关联，运动前缺少对实际站姿的支撑和净空再检。最后，`followPath()` 在调用 `async_send_goal()` 前运行 `consumeHandoff()`，明确拒绝也会留下已关闭会话和已消费授权，使后续路径重试可能沿用交接后的分支。

`multifloor_manager.cpp` 中，`acceptFloorCompletion()`、`suspendHandoff()` 与 `invalidateHandoff()` 通过一个待处理对象和一个暂缓对象实现报告生命周期。PENDING 可以提供当前请求号。TEMPORARILY_INVALID 不具备授权，先废弃旧局部请求号、候选路线与 STOP 关联，再保留原报告以便检查有更新生成时间的续报。重新激活时，范围和生产者必须匹配当前 ACTIVE 会话，而且要重新验证身份、单调时间、v5 状态、当前配对地图、完整未知清单、区域几何依赖、实测站姿以及当前运动所有权。新鲜且一致的续报重建局部请求号，失效前的楼梯搜索即使延迟返回也不能生效。已生效的同 ID 续报可以更新时间和证据版本，需保留分类、面积、未知清单、区域依赖范围与摘要、探索边界以及审计起点。相同 ID 的内容被改写或者时间倒退时，续报被拒绝。

撤销、替代和消费都退休旧授权。显式撤销即使到达时报告处于暂缓状态，也会清除该报告。更高序列且通过验证的新报告才可以替换当前候选。已接收及收到撤销的 UUID 保存在每会话最多 4096 项的集合内，配合序列水位，阻止旧报告以原序列或另一个序列重入。容量用满时继续拒绝额外的新报告，现有报告的身份匹配条件仍然有效。换楼时按既有规则切换会话 UUID，再初始化该会话自己的水位和集合。

`frontier_explorer_node.cpp` 的 `considerResidualCompletion()` 继续使用原有联合审计、安全起点和稳定性条件。续报仍沿用原 UUID 和序列，先确认原始地图证据在当前地图上仍有效，审计起点仍相容，再保留原始区域和边界证据，只刷新生成时间与稳定性。相关证据实质变化时仍通过 `revokeFloorCompletion()` 撤销旧报告，并由符合原条件的新审计发出新报告。任何历史 String 或 Empty 消息仍没有授权作用。

`robot()` 使用同一时间点的 `base_link` 与 `base_footprint` TF，核对新鲜度、有限的坐标和朝向，以及机身位于足底之上的基本关系。`safeStandingPose()` 通过既有 `config()` 拟合局部支撑面，通过当前三维地图上的 `SurfaceGraph::diagnosePose()` 同时检查实测足底位置是否处于有效表面、是否获得多点支撑以及机身双圆柱模型的净空。净空沿用 Manager 配置，并保留 Explorer 已有的 1 厘米硬净空下限。这个步骤直接检查当前足底位姿，不把站姿吸附到另一个格点来通过检查。收到匹配 STOP 命令的真实 IDLE 时，`stopFrontier()` 将确认绑定到该报告的局部请求。随后 `stoppedHandoffReady()` 在首次规划、提交 FollowPath 和收到首个接受响应时核对真实停止确认及当前站姿。最后一层也经过同一检查后才发布完成。已有效的普通续报和周期性等待只校验必要的定位、身份与地图证据，三维站姿复核安排在授权重新激活和实际控制关口。

首次交接在 `requestPath()` 中捕获当前会话代次、授权请求号与三维路径地图摘要。有效路径返回后，`followPath()` 完成地图、站姿、IDLE 和路径来源检查，设置唯一 `follow_owner_`，保留 `first_follow_` 的提交上下文，再向 FollowPath 发目标。此时楼层会话还处于 CLOSING，授权仍待消费。只有 `followGoalResponse()` 收到当前独占目标的实际接受响应，并复核请求、时间、地图与站姿后，才运行 `consumeHandoff()`，创建唯一交接身份并关闭源楼层。最后一层没有楼梯运动目标，消费点是收到真实 IDLE 且站姿有效之后。

明确拒绝时，`releaseFollow()` 释放相符的运动所有权，原授权仍须通过复核才能开始新的规划。第一目标响应超时、提交期间授权被撤销、路径地图变化或迟到接受时，`fail()` 与 `abandonUnconsumedHandoff()` 将流程置于安全等待，收到迟到的接受响应后发出取消请求。发送取消本身不释放所有权。明确拒绝或收到同一目标的 Action 终态后，才能关闭失效尝试并在确认站姿、地图及 Explorer 空闲后恢复。同一回调无法释放其他目标的所有权。首个目标一经合法接受并正式消费，后续按原有楼梯接近、步态切换、分段执行、落脚和高度确认流程运行。Action Server 彻底失联且无法确认旧动作终止时，Manager 保持等待，需确认控制器的真实状态后才能恢复任务。

生产源码改动集中在 `/home/dobby/scan/src/go2_multifloor_exploration/src/multifloor_manager.cpp` 与 `/home/dobby/scan/src/frontier_exploration_ros2/src/frontier_explorer_node.cpp`。`go2_multifloor_exploration/test/test_handoff_consistency.cpp` 覆盖报告续报、暂缓后重新验证、不同关键证据、永久失效、请求隔离、集合容量、当前支撑和当前净空、过期 TF 及旧目标通知。`test_stair_protocol.py` 和 `make_stair_fixture.cpp` 用真实 ROS Manager 和替身服务、Action 验证明确拒绝、延迟响应、超时后迟到接受、提交期间撤销与地图改变、取消等待终态、最后一层重新确认站姿，以及原来的楼梯回归。`CMakeLists.txt` 调整这组测试的总运行上限，供扩展用例使用。`frontier_exploration_ros2/test/test_active_residual_node.cpp` 在原有主动观察回归上核对 v5 同 ID 续报确实保留关键证据。

ROS 2 Humble 下，`go2_surface_3d_planner`、`frontier_exploration_ros2`、`go2_multifloor_exploration`、`fastlio_nav_bridge` 和 `multifloor_exploration` 均编译成功，完整 demo 的 launch 参数解析成功。C++ 与 ROS 自动化共 613 项通过，包括 SurfaceGraph 47 项、Explorer 483 项、多楼层 Manager 和切片 83 项。其中楼梯协议的 11 个替身场景全部通过，真实 Explorer 与 FloorSlice 的会话握手测试通过。追加 UNKNOWN 结果保护后重新编译了多楼层包，并重跑该包全部九组测试，包含确认 UNKNOWN 不释放所有权的新增单元测试。历史 Python Coordinator 的兼容测试独立执行并通过一项。

`fastlio_nav_bridge` 的 Python 回归仍是 30 项通过、1 项跳过、5 项失败，与 `1514e17` 的交付记录一致。五项失败分别为连续地面 launch 的三个事件处理器数量断言、flake8 和 pep257。本阶段保留了原测试及断言，这些相关源码也与基线一致。

Gazebo 和实机运动本阶段尚未运行。实际动作拒绝与控制器断连、雷达连续更新下的检查耗时、楼梯步态与落脚平台仍需在完整仿真或受控实机上验证。本次未改变 v5 消息、地图同步协议、残余区域分类、Frontier、MRTSP、Nav2 配置、既有运动超时或楼梯运动算法。

可在修复分支用 `git revert` 撤销本说明对应的提交，保留历史。需要单独复核修复前的基线，可用 `git worktree add --detach /home/dobby/scan-commit3-fix-baseline baseline/commit3-fix-start-20261125`。本次交付保存在虚拟机本地。
