# 双机循环巡航近邻崩溃与树显示连续性修复

日期：2026-09-29。范围仅 `/data/robotics/air_ground_smart3d`。保留640×360相机、SAM3总2 Hz、原地图/传感器/速度/修树参数；没有实施性能计划中的频率调整或地图算法优化。

## 根因与实际证据

用户报告Go2树突然消失时，运行epoch为 `1ddaeeef8d1848909e9161e65dbba5ad`。Go2完成124个巡航点，目标版本125，`planner={}`，状态waiting_path。原生stderr为Kiddo 5.0.3 `construction.rs:223`：

```
Too many items with the same position on one axis.
```

SMART改目标根时保留旧根节点。长期四角巡航不断产生同坐标节点；原近邻索引的固定32项叶和循环切分轴无法容纳这类退化分布。错误在一次update追加索引时触发，不是每次set_goal必然立即触发。

原生层捕获panic后丢弃内部session并返回 `native_panic_session_invalidated`。Python层把它当不可恢复合同错误，清空planner并结束该机器人的规划线程。共享地图、雷达、UE仍在运行，问题不是GPU耗尽或RViz漏画。

## 修复内容

1. `planning/vendor/smart3d_observed/src/rrt/neighbors.rs`：精确自适应轴Kd索引。只在坐标有跨度的维度、不同值之间分裂；完全重合的坐标存入动态叶，保留全部节点ID。不随机抖动坐标、不丢重复节点，也不是把32改大后等待下一次溢出。最近k与半径查询和朴素精确距离对照；半径边界包含相等点。
2. `sim_io/runtime.py`：仅对明确的 `native_panic_session_invalidated` 重新初始化该机器人规划会话。立即撤销旧路径，使用最新地图快照、当前位置和当前目标，独立session generation，有限退避；不clear/load/restart共享mapper。其他合同错误仍报告，不用不断重试掩盖坏数据。执行前两处身份检查都使用当前活动会话，防止恢复后被旧会话名拒绝。
3. 目标切换只撤销旧路径，保留仍有效的真实树。目标暂时受阻时，原生update会继续用最新地图校验旧树，再显示这一份树；旧路径为null，目标版本门禁不允许执行旧目标。panic、非法结果仍清空无效树，不能用缓存假装规划器正常。
4. `sim_io/restart_state.py`与启动入口：支持显式 `--resume-map ... --resume-state ...`。只接受同场景、同profiles、同机器人身份、已停止的state；位置/朝向/计数合法，目标必须唯一匹配原巡航路线。保留两机位置、方向、目标和累计进度，重新生成有效树与路径。旧自由空间仍需新传感器证据，不恢复旧运动指令。

原LINKS来源另有记录，本次更改只在双机项目vendored副本，`planning/CORE_PROVENANCE.json`已补充近邻修复来源。没有修改Town10、Factory或旧单Go2空场源码。

## 验证

- Rust工作区：104项通过。其中2D和3D各160次重复四角目标切换，始终同tree instance、原节点不丢失、路径有效、无非法树边；另含512重复/共线/共面点、混合1024点、空索引与半径边界。
- 最终Python：330项通过。覆盖显式panic恢复、受阻目标恢复中的panic、恢复后有效非空路径实际可执行、旧会话被拒绝、换目标树保留/旧路径撤销、快照恢复及既有地图/人物/合作避碰合同。
- 部署使用停止快照保存地图和两机位姿/巡航进度。期间有多次分阶段runtime部署，UE和RViz进程保持运行；部署时间点和恢复位置见各checkpoint JSON。地图第一次保存包含970511个已观测体素，恢复时完整保留；后续仍继续观测积累。
- 树显示补丁部署后约44.9秒、124次状态抽样：Go2前进35.25 m、完成2个目标；UAV前进18.35 m、完成1个目标。两机空树样本均为0、规划错误均为0。遇人/几何障碍仍可能按原检查暂时等待。
- 同阶段RViz真实ROS树标记约35秒收到70条消息：两机主树各70次ADD、0次DELETE；最大间隔1.05 s，小于3秒Marker寿命。这验证显示连续性，不把画出来的树当作额外执行许可。
- 最终会话门禁补丁部署后又独立验证实际位移、树和感知，结果见 `final-deployment-live.json`。上述短时在线验证和长期重复目标回归不能替代数小时压力测试，也不表示所有规划器异常已消除。

## 提速工作的边界

后续方案见 `docs/plans/2026-09-29-performance-followup.md`。优先分离高频状态与整树快照，再只计算增量键的diff，随后评估地图复制和清障队列。640×480已按用户要求撤销；相机保持640×360，采集频率与SAM3总2→3 Hz均未实施。

一次早期恢复后的35秒样本双雷达各约3.86 Hz入图，但重建后的树规模、人物位置与修复前不同，不能把这个数字当作性能优化收益。本轮主要修复正确性和恢复行为。

## 明确恢复方式

从停止的兼容运行快照恢复地图与两机巡航进度：

```bash
cd /data/robotics/air_ground_smart3d
./lab start --rviz \
  --resume-map /absolute/stopped/run/shared/map.ot \
  --resume-state /absolute/stopped/run/state.json
```

`--resume-state`必须同时配合`--resume-map`；相邻`resolved_profiles.json`必须存在。默认`./lab start --rviz`行为不变。本次源码身份、错误日志、运行检查与ROS标记证据在本目录。
