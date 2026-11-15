模型：GPT-6 系列 Codex。本文记录 Commit 2-B 的实际交付，实施目录为虚拟机 /home/dobby/scan，日期为 2026-11-15（Asia/Shanghai）。本阶段从 b8f2cfb 开始，使用 feature/active-residual-exploration-commit2b 独立分支，新增 baseline/commit2b-start-20261115 回滚标签，Commit 1、Commit 2-A 和此前标签全部保留。没有修改 main，没有推送远端，没有启动或操作 Gazebo，也没有实施 Commit 3。

这次实现的是残余区域的主动观察闭环。之前常规 Frontier 找不到目标时，三维审计主要反复计算静态观察点，RegionObservationTask 虽然已经有到达后的观察窗口，但没有与残余区域的三维增益和逐区域调度完整连接。现在常规选择没有合格目标、且常规三维候选验证没有待处理工作时，会先请求主动残余观察，再考虑随机脱困。正常 Frontier 的聚类、评分、排序和 MRTSP 不变；有新的常规可用目标时仍能恢复常规探索。

/home/dobby/scan/src/frontier_exploration_ros2/include/frontier_exploration_ros2/active_residual.hpp 是新增的区域证据和调度结构。SpatialRegion 保存区域成员格、边界、候选枚举游标、地图依赖指纹、尝试结果、规划起点上下文、预算轮次和冷却状态。SpatialAttempt 保存空间位置、唯一二维格编号、合理朝向、预期可见覆盖、导航检查、实际到达、新鲜点云标记、三维增益及验证时间。ActiveResidualBook 的 sync、next、apply、audit、recoverBudgets 和 changedOrigin 分别负责区域更新、轮转选择、异步结果应用、统一审计输入、有限预算恢复和新实测起点下的重新评估。没有在区域历史中保存整棵 OctoMap 或原始点云。

/home/dobby/scan/src/frontier_exploration_ros2/src/active_residual.cpp 是新增的空间候选与三维验证实现。planSpatialRegion 沿用原 v4 的当前地图和四米可见性模型，在区域周围已知自由栅格上动态枚举位置，不固定生成前、左、右三个点。候选位置有实际 XY 差异，knownFreeBodyFootprint 按现有双圆柱机身模型、原碰撞半径和安全余量检查完整二维占地，拒绝身体越出地图或覆盖未知格的目标。每个位置再通过 SurfaceGraph 的实际支撑、完整机身和路径检查以及 auditCellVisible 的激光雷达视场、遮挡检测。朝向分箱只用于在同一 XY 选择可站立、可到达的导航姿态，不创建独立的原地旋转任务。路径验证保留 v4 既有的粗网格到细网格回退及原有有界搜索期限，不改变 SurfaceGraph 算法本体。新的观察任务限制在当前楼层，必要的小高度差使用现有 max_step_height 约束。

同一文件中的 captureUnknownVoxels 在实际到达之后建立未知体素基线，measureVoxelGain 只统计这些体素后来真实变为 Free 或 Occupied 的变化。自由体素数、占用体素数、体积以及去重后的二维投影面积分开记录，同一 XY 柱的多个高度体素不会被重复计作总投影面积。移动距离、导航成功、朝向变化和重复点云都不直接产生收益。基线存储沿用 geometry_max_points 作为内存上限；截断时保留不完整标记，不能把不完整基线的无收益结果当成充分验证。已有二维 RegionObservationTask 的到达、位置稳定、新地图时间和观察窗口逻辑继续复用。

同一文件中的 observedClosedFence 提供明确而保守的允许保留策略。二维未知连通区域必须具有已观测占用边界，随后在真实 OctoMap 体素格上验证三维阻断。未知柱按可连通处理，缺少墙体体素会使证明失败；验证高度覆盖当前候选站立范围和传感器射线所需范围。只有获得这类独立充分证明，才使用 verified_enclosed 作为允许保留的不可达残余分类。开放但证据不足的区域、单纯遮挡、找不到已知通路或所有导航请求失败，都不能自动升级成这个分类。完全搜索后无路径与搜索范围内三维信息不足也分开处理；后者保留 map_insufficient，而不是宣称物理不可达。

/home/dobby/scan/src/frontier_exploration_ros2/src/active_residual_node.cpp 是新增的 ROS 调度适配层。serviceActiveResidual 管理 CheckingPath、Navigating、AwaitBaseline、Observing 与 Idle 阶段，按区域轮转并接收有界异步三维规划结果。beginResidualNav2Check 调用现有 /compute_path_to_pose，再经现有 Core::send_pose_goal、dispatchGoalRequest 和 /navigate_to_pose 执行导航。实际配置只有 Floor3D 和 Stair3D，因此明确选用与现有平面导航行为树相同的 Floor3D，并保留已有二维 ConnectedDispatch 验证；没有假设存在 GridBased，也没有新增或更换规划插件。activeResidualNavigationResult 验证实际机器人位置和已有机身安全检查，不能只相信动作返回成功。

该节点文件在实际到达后先保留原 post_goal_min_settle 稳定等待，再等待到达后的新地图建立基线，随后等待更新鲜的激光雷达点云及其后的来源地图 generation。信息增益必须来自这之后的真实体素变化。没有收益时仍继续其他合理空间位置；位置重合、已执行观察和有限次数失败会被抑制，相关地图实质变化或新的未评估规划起点可以重新评估。finishResidualAttempt 记录结果，并释放已经执行的候选覆盖数组，只保留覆盖指纹、数量及结果等紧凑历史。楼层地图 schema 或 OctoMap 分辨率变化会拒绝旧观察窗口，避免把不可比较的地图算成收益或无收益。

区域预算继续使用原 floor_cleanup_budget_s、floor_cleanup_min_attempts 和 RegionObservationTask 的观察/冷却机制，没有新增参数或修改默认值。本轮预算耗尽后让出区域，保存游标和历史，经过原冷却时间再继续剩余的有限候选工作。有意义的游标推进或实际尝试才能支持下一轮恢复；相同地图下没有任何进展的昂贵证明不会无限靠清零预算重跑。搜索超时的同一候选有有限重试次数，已经实际观察过但因数据不足结束的同一位置不会被原地无限重复。地图或定位不新鲜时保持不确定，超时判断在数据新鲜度提前返回之前执行，避免永久占有任务。一个区域受阻不会直接阻止其他区域获得轮转。

/home/dobby/scan/src/frontier_exploration_ros2/include/frontier_exploration_ros2/residual_region_audit.hpp 提取了共享的 extractResidualLayout，保留原未知连通分组和可达队列顺序，供既有审计与主动观察共同使用。ResidualAudit::status 现在明确区分纯遮挡、支撑未确认、地图不足、传感器视场不适用、导航失败、无收益和验证暂缓，这些状态不能正式完成；no_route 和 no_known_access 仍是受阻结果。verified_enclosed 与楼梯 transition 等充分证据才可能形成 VERIFIED_WITH_RESIDUALS，零残余仍可 VERIFIED_COMPLETE。没有把原始未知格写成已探索，也没有删除既有三维验证。

/home/dobby/scan/src/frontier_exploration_ros2/include/frontier_exploration_ros2/floor_completion.hpp 收紧正式报告的允许残余理由，纯 occluded 不再直接通过，使用具有更强证据含义的 verified_enclosed。仍保留 VERIFIED_COMPLETE、VERIFIED_WITH_RESIDUALS、INCOMPLETE、BLOCKED、ERROR 五类结果。/home/dobby/scan/src/frontier_exploration_ros2/src/frontier_explorer_node.cpp 将正式发布集中到 considerResidualCompletion，继续执行 Commit 2-A 的地图证据、来源配对、稳定性和当前机身安全条件，仍使用 /explore/floor_completion。v1～v4 和 Empty 不恢复直接授权。主动观察的 v4 诊断不会再虚报已获得三处实际观察或六十秒运行证据。

/home/dobby/scan/src/frontier_exploration_ros2/include/frontier_exploration_ros2/frontier_explorer_core.hpp 增加主动残余任务的请求及忙碌回调；/home/dobby/scan/src/frontier_exploration_ros2/src/frontier_explorer_core_dispatch.cpp 在常规目标耗尽时接入该请求，并在主动规划、导航或观察拥有调度权时暂停普通重选。residual_observation 的终结不立即跳过观察窗口去发送下一个普通目标。正常 Frontier 和 MRTSP 的算法、评分与参数没有修改。

/home/dobby/scan/src/frontier_exploration_ros2/include/frontier_exploration_ros2/frontier_explorer_node.hpp 增加主动任务的有限状态、单个异步 worker、单个 Nav2 路径检查以及当前观察基线等成员。对应节点实现让点云新鲜度监听独立于可选的几何评分开关；只保留经过布局、有限坐标和 TF 检查的消息时间，不缓存原始点云历史。停止时取消待处理路径请求，仍由原 Core 管理导航取消，并通过已有停止轮询异步回收已经结束的 worker，避免完成任务缓冲一直保留到下次启动。

/home/dobby/scan/src/frontier_exploration_ros2/include/frontier_exploration_ros2/map_consistency.hpp 允许同来源摘要的 FloorMapSnapshot 复用已经解码的不可变树；/home/dobby/scan/src/go2_multifloor_exploration/src/multifloor_manager.cpp 只接入同样的解码复用，Commit 2-A 的待交接报告校验、进程内请求关联、实际消费前检查和楼梯状态机不变。主动模块不保存历史地图列表，同一模块至多运行一个三维规划 worker、一个当前导航流程及一份有界观察基线。原 FloorMapSnapshot 的原子配对仍保留，没有通过拆开传输或混用版本来减少内存。

/home/dobby/scan/src/frontier_exploration_ros2/CMakeLists.txt 注册新增实现与测试。/home/dobby/scan/src/frontier_exploration_ros2/test/test_active_residual.cpp 新增 24 项几何、收益、缓存、有限预算、空间去重、封闭证明和资源生命周期测试。/home/dobby/scan/src/frontier_exploration_ros2/test/test_active_residual_node.cpp 新增 16 项实际 ROS 任务调用链测试，覆盖 Nav2 请求、实际到达验证、点云和地图等待、三维收益、无收益后真实发送下一位置目标、统一完成发布、地图变化、停止和数据中断。Nav2 action server 在测试中是可控替身，Node、Core、SurfaceGraph、TF、点云接收、状态处理和消息通道使用实际实现；不能据此宣称已经通过实际 Nav2 插件或 Gazebo 运动验证。

/home/dobby/scan/src/frontier_exploration_ros2/test/test_connected_dispatch.cpp 调整纯遮挡不再授权的断言，保留实际三维调用和原始未知格不变的检查；/home/dobby/scan/src/frontier_exploration_ros2/test/test_map_consistency.cpp 让稳定性测试使用更强的合成已验证分类，原地图失效检查保留。/home/dobby/scan/src/go2_multifloor_exploration/test/test_floor_handoff.cpp 扩充对弱证据的拒绝；/home/dobby/scan/src/go2_multifloor_exploration/test/test_stair_protocol.py 仅更新正式报告夹具的强证据类别，原历史报告拒绝、停止到 IDLE、步态确认、实测高度及落脚检查均保留。没有通过降低测试标准使测试通过。

完整调用链为 Core::try_send_next_goal 或 dispatch_pending_frontier_goal 的常规目标耗尽分支请求主动观察，serviceActiveResidual 使用 FloorMapSnapshot 和共享未知区域提取更新 ActiveResidualBook，由 planSpatialRegion 检查实际空间位置、完整机身、雷达可见性和 SurfaceGraph 路径，结果经过区域地图依赖与 epoch/位姿验证后进入 beginResidualNav2Check，先请求现有 Nav2 规划再通过 Core::send_pose_goal 和 dispatchGoalRequest 执行导航。回调进入 activeResidualNavigationResult，验证实际到达与安全，等待稳定、新鲜点云和来源地图后，由 captureUnknownVoxels、measureVoxelGain 和 RegionObservationTask::poll 更新逐区域证据。其他位置或其他区域继续轮转，只有所有区域形成允许的结论，才通过 ResidualAudit、considerResidualCompletion 及 Commit 2-A 的一致性和稳定性条件发布正式完成，再由原 Multifloor Manager 独立检查交接与楼梯条件。

最终三个相关包 frontier_exploration_ros2、go2_multifloor_exploration、multifloor_exploration 均编译通过。实际重新执行的 Frontier 测试为 468 项，go2_multifloor_exploration 测试为 63 项，共 531 项 C++／ROS 用例，失败、错误和跳过均为零；旧 Python 完成通知兼容测试另有一项通过。新增测试共 40 项。全局 colcon test-result 还包含其他包以前留下的结果，不把它的汇总数冒充为本阶段新执行数。构建与测试日志为 /tmp/commit2b-build-delivery.log、/tmp/commit2b-test-delivery.log、/tmp/commit2b-frontier-test-verified.log、/tmp/commit2b-test-results-verified.log 和 /tmp/commit2b-legacy-test.log。

开发过程中修正了测试订阅回调类型及序列化测试缺失头文件导致的中间编译失败。几何测试发现二维浮点格和体素边界不齐，三维封闭证明因此改为实际体素连通性检查。另一个测试误将 OctoMap deleteNode 的父节点删除返回值当作单个体素删除是否成功，现改为直接断言体素删除前存在、删除后不存在，并确认缺口确实撤销封闭证明。最终测试均通过，未放宽占用、可见性、支撑或稳定性要求。

资源检查验证区域历史不会保留整棵地图、异步结果消费后不会保持旧树、停止会回收完成任务、观察基线达到上限时保留不确定性。同一来源快照能复用树实例。测试夹具序列化的 OctoMap 为 14603 字节，配对快照为 16228 字节，多出的 1625 字节主要是二维图与元数据；原 OctoMap 话题仍在，因此仍存在随发布频率变化的额外传输开销，未擅自调整频率或 QoS。独立的连续更新与无收益换位置 ROS 测试，实测峰值 RSS 为 36736 KiB，耗时 6.30 秒；这是小型可控夹具数据，不是整张真实地图的长期内存结论。资源日志为 /tmp/commit2b-memory-profile.txt 和 /tmp/commit2b-resource-profile.log。

仍待验证的项目包括实际 Nav2 Floor3D 配置联调、Gazebo 中真实运动和到达误差、真实点云与 OctoMap 时间链路、门口和遮挡场景的信息增益、多房间长时间运行，以及大 OctoMap 下的传输、指纹计算和 CPU 开销。区域指纹和三维封闭证明有保守计算成本，预算退出不等于完成。开放但证据不足、持续定位异常、导航失败或传感器中断仍会保留未完成／受阻状态，不承诺任何地图都能在固定时间内结束。没有执行 Gazebo 或实机验证。

本阶段未修改 Frontier 评分、MRTSP、Nav2 配置、SurfaceGraph 核心、楼梯控制参数、原碰撞余量或原观察与导航超时参数。未引入 floor_session_id、v5 或跨楼层会话隔离。交付提交完成后立即停止，未经明确确认不得进入 Commit 3。本文件连同上述十八个源码、构建和测试文件组成独立 Commit 2-B，可通过撤销该提交回滚，或查看 baseline/commit2b-start-20261115。
