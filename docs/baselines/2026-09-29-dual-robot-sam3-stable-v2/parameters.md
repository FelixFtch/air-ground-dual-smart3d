# 双机共享 OctoMap＋SAM3 稳定基线 v2

记录日期：2026-09-29。基线 ID：`2026-09-29-dual-robot-sam3-stable-v2`。对应用户认可雷达时序修复后当前效果、要求更新保存的版本。本次归档不改变运行参数、不重启服务、不清空地图。旧 `2026-09-29-dual-robot-sam3-stable` 基线保留。

**当前整个 `profiles/` 与旧基线副本逐字节相同，当前运行 `resolved_profiles.json` 也相同；改变的是代码、独立 UE 模块与有效观测行为。** 不能仅恢复旧 YAML 就认为恢复了当前版本。

这是**配置、源码/原生程序/UE补丁身份及性能证据基线**。配套归档的确切文件以最终 manifest 为准；不包含完整 UE 引擎、全部商业资产、模型权重和依赖环境，也不将活动运行中的地图冒充一致性快照。本配置归档 **不保存活动地图**。恢复已停止运行的 `.ot`/元数据及 `state.json` 是独立操作。冷启动不能保证随机树、瞬时障碍和性能数字完全相同。

## 1. 项目与运行身份

| 项目 | 当前值 |
|---|---|
| 程序 | `/data/robotics/air_ground_smart3d` |
| UE 工程 | `/data/sim/AirGroundLab_UE58/CarlaUnreal/CarlaUnreal.uproject` |
| 关卡 | `/Game/AirGroundLab/Maps/AirGroundLab_Main` |
| 场景身份 | `air_ground_lab_v1`；内容版本 `2026-09-29-v2-calibrated` |
| 当前运行 epoch | `d83ccef432d3410e8e573a93122096cb` |
| Profile hash | `c7bb2aef1b98c82e3db2b82fcffd1bb9af0f62e6779adb74af0462f34e38be59` |
| CARLA / ROS | 端口 2400 / domain 45 |
| UE 运行 | UE5.8，`-game -RenderOffScreen`，无音频，帧率上限 30，1280×800 |
| 渲染池 | Texture streaming 1800 MiB；Nanite streaming 512 MiB |
| 仿真时钟 | 异步世界；传感器保留采集帧和仿真时间 |
| 人流 | 6 人；配置速度 0.5/0.7/0.9/1.0/1.2/1.4 m/s |
| 视觉推理 | SAM3 开；本双机实验没有另开 YOLO、Qwen 或新网页控制台 |

旧 Town10、Factory、单 Go2 空场工程保持独立。启动/停止顺序见项目根目录 `README.md`；`./lab start --rviz` 会拉起本实验，`./lab stop` 只管理本实验自有服务。默认冷启动重新观测地图；恢复指定 `.ot` 要显式使用 `--resume-map`，旧自由空间需重新观测。

## 2. 共享地图、几何与执行

| 参数 | 值 |
|---|---|
| 体素 | 真实 OctoMap，0.25 m |
| 地图范围 | X/Y 各 −30～30 m，Z −0.5～8 m；60×60×8.5 m |
| UAV 规划范围 | X/Y 各 −20～20 m；高度 0.8～6 m |
| 坐标转换 | map x=CARLA x，map y=−CARLA y，map z=CARLA z−0.22 m |
| 测得地面 | CARLA Z≈0.20125 m；map Z≈−0.01875 m |
| 占用概率 | hit=0.9，miss=0.1，clamp=0.12～0.88 |
| 障碍首次记录/清除 | LiDAR 9 m 内首次标记；有效实测射线最远清除 40 m；物理量程 45 m |
| 已知表面远距复测 | 曾由真实近距命中建立的体素，在 9～40 m 再次真实命中可刷新或恢复占用；远处未知体素不因此首次建占用 |
| 射线保留 | `all_distinct_measured_rays`；仅完全相同端点去重，不按端点体素丢弃不同实际射线 |
| 同帧证据 | 全部近/远真实命中体素优先于同帧 free；每体素每帧最多一次 hit/miss，不因保留更多射线放大概率次数 |
| 时间处理 | 严格按采集时间应用，旧帧不能覆盖新证据；同一时刻命中优先 |
| 空闲保留 | 不按 TTL 自动清墙；已知空闲保留，直到新占用证据出现 |
| 未知区域 | 可有候选树，实际近端运动必须通过已知空闲与碰撞检查 |
| 执行检查 | 近端 0.4 m；本机已融合 LiDAR 最大年龄 1.2 s；规划结果最大年龄 3 s |
| 合作机器人 | 本体/伙伴点从共享地图过滤，双方真实位置及机身包络作为临时避碰约束 |

这里 hit/miss 的快速翻转针对当前零噪声仿真，尚不是经过真机验证的参数。远处实际命中表面不会冒充本帧自由空间；裁剪到量程/边界的虚拟端点不当真实表面。无回波不会制造清理射线。没有被有效射线重新穿过的遮挡区域不会凭空消失。远距已知表面不是永久锁定：以后没有该命中、而有更新的真实穿越射线时仍可清空。25 cm 体素中细表面与空隙的离散化歧义仍存在，不能宣称所有细梁误清和动态残影都已根治。

## 3. 两机运动与巡航路线

| 参数 | Go2 | Fighter UAV |
|---|---|---|
| 身份 | `go2_01` / `go2_ground_v1` | `uav_01` / `fighter_uav_v1` |
| 移动/规划维度 | XY＋yaw；二维 SMART | XYZ＋yaw；真正三维 SMART |
| 外观缩放 | 1 | 0.35 |
| 规划机身半尺寸 | (0.43,0.43,0.28) m | (0.70,0.70,0.36) m |
| 最高平移速度 | 2 m/s | 1.5 m/s |
| 加速度 | 2 m/s² | 1.2 m/s² |
| 最高转速 | 1 rad/s（约57.3°/s） | 0.8 rad/s（约45.8°/s） |
| 到达容差 | 0.55 m | 0.60 m |
| 高度 | 固定 map Z=0.33 m，平地支撑检查关 | 0.8～6 m |
| 起点 | (−8,−8,0.33) | (−8,−3,2) |

规划半尺寸是保守净空包络，不应当作网格模型实测长宽。UAV UE 实体碰撞半尺寸为 (0.48,0.48,0.34) m，规划 XY 额外覆盖转向扫过范围。Go2 不提供楼梯步态，UAV 为运动学飞行。

Go2 循环巡航：`(−8,−8,0.33) → (8,−8,0.33) → (8,8,0.33) → (−8,8,0.33)`。

UAV 循环巡航：`(8,−3,2) → (8,7,3.2) → (−8,7,2) → (−8,−3,2)`。

路径跟随前视 0.7 m；朝向误差≥0.65 rad（约37°）时原地转向，其余按 cos(误差) 降速。控制随 UE tick 执行，dt 限制 1～75 ms。上述速度是上限，绕障/转弯/等待时会更慢。

## 4. 传感器与融合

| 参数 | Go2 | UAV |
|---|---|---|
| LiDAR | 96 线，192000 点/秒，10 Hz，45 m，噪声0 | 相同 |
| 垂直视场 | −75～45° | −85～75° |
| LiDAR 安装平移 | (0,0,0.7) m | (0,0,0.4) m |
| LiDAR 入图目标 | 各5 Hz；保留全部不同实测射线，仅完全相同端点去重 | 相同 |
| 前向 RGB-D | 640×360，FOV100°，每 UE 帧采集 | 相同 |
| 前向相机安装平移 | (0.5,0,0.4) m | (0.5,0,0) m |
| 前向深度入图目标 | 2 Hz，6 m，stride20 | 相同 |
| 辅助深度 | 无 | 六向96×96，FOV150°，采集5 Hz |
| 辅助深度入图目标 | — | 每向约0.667 Hz，3 m，stride3 |

具体六向安装姿态见同目录 `profiles/sensors/uav_rgbd_lidar.yaml`。融合采用最新帧与公平调度、有界流水线，不排队处理大量过时帧；目标频率不保证在联合负载下达到。

**UE 雷达时序已修复：** 独立 AirGroundLab 的 `RayCastSemanticLidar.h/.cpp` 在 `PrePhysTick` 保存传感器真实 elapsed time，`PostPhysTick` 消费一次；普通 `RayCastLidar.cpp` 使用同一间隔，并同步普通点云 `horizontal_angle` 头。原来 `sensor_tick=0.1` 却用了约 1/30 s 世界帧间隔计算扫描，典型每包只约120°，发射量约64320条/s；现在在该稳定间隔下恢复配置的192000条/s/雷达。首包或间隔调度波动仍可产生部分扫描包，**不是强制每包360°**。观测诊断 `observed_xy_azimuth_span_deg` 测的是实际回波覆盖，不是发射扫描角。

`mapping_sampling: representative_of_every_hit_voxel` 仍留在两份 YAML 中，但只是代码未读取的旧描述；实际策略以 helper、原生实现和运行统计 `ray_geometry_policy` 为准。为保持已验证 profile 身份，本轮没有为改文字而修改 profile/hash。

## 5. SAM3、人物速度与 OHZ

| 参数 | 当前值 |
|---|---|
| 模型 | `facebook/sam3`，revision `3c879f39826c281e95690f02c7821c4de09afae7` |
| 推理 | Transformers Sam3VideoModel；CUDA BF16，非量化，SDPA |
| 实例 | 一个共享权重模型，两个独立视频 session；文本查询 `person` |
| 输入发布 | 每机最多3 Hz，完整640×360同帧配准 RGB-D，照片包最多保留8份 |
| 服务处理上限 | **总计2 Hz**；两相机轮流，约各1 Hz |
| 视频记忆 | 每session最多16对象；每16帧重检测；非条件历史128帧；每对象最多4个条件帧；丢失轨迹清退策略30 s |
| 人物位置 | 每mask最多2048深度点，0.3～15 m有效深度；用拍摄位姿转换至map |
| 估速 | 同源至少4次有效观测；最多6点、4.5 s窗口；连续间隔≤1.5 s |
| 算法 | XY位置成对斜率的中位数；最大速度3 m/s、残差阈值0.6 m；无卡尔曼滤波 |
| 未估计内容 | Z速度不可靠/不估；不读取 UE 行人的方向或速度作为输入 |
| 几何有效期 | 2 s；最多外推1 s，与视频跟踪丢失保留30 s不同 |
| OHZ | 预测时域1 s；Go2二维椭圆，UAV三维椭球；含各自机身包络 |
| 误差先验 | 位置余量0.15 m；可见人物范围上下各补0.85 m，**尚未实验校准** |
| 合作本体过滤 | 同帧两机器人 OBB＋0.08 m；过滤后平面残余垂直跨度<0.05 m时拒绝 |

没有可靠速度时使用无方向扩张，不能将其解释成已经测出人物移动。合作机器人避碰仍保留。

**当前人物 OHZ 显示已与慢规划回执解耦：** runtime 从同一份最新有效人物文档、共同当前仿真时刻，为两台机器人按各自包络生成当前 OHZ；执行检查与 RViz 使用这一时刻的数据。Rust 规划器在各次请求时消费同源构造的形状，其 `planner.people_hazards` 保留当次规划证据，不能当当前显示的唯一来源。该改变去掉显示等待规划的额外延迟，并未提高 SAM3 推理频率、延长2秒TTL或保证每帧存在可靠速度。

共享身份只进行时空兼容的无歧义关联，不是外观 ReID：跨相机关联距离0.65 m、时间差0.75 s，允许可见高度中心差0.8 m；双源有效速度差超过0.75 m/s不合并估速。可见表面三维框、规划OHZ和速度箭头含义不同。

注意：team YAML 中 `shared_model_inference:false` 是当前运行代码未读取的陈旧字段，实际状态明确为 `shared_model_instances=1`、两个 session。本轮保留原文件并记录差异，没有静默修改。

## 6. SMART3D 与展示

| 参数 | 当前值 |
|---|---|
| 核心 | LINKS Rust `SMART<f32,2>` / `SMART<f32,3>`；来源见 `planning/CORE_PROVENANCE.json` |
| 初始化 | Go2最多5000迭代，UAV最多7000迭代；各1500 ms预算，先到者止；迭代数不是节点数 |
| 更新目标 | 每机0.4 s一轮 |
| 局部修树预算 | 3000迭代或50 ms；裁树、改根、收尾检查及导出不算在50 ms内 |
| LRZ | 两机均4 m |
| 清障恢复 | 每轮64区域 / 512节点 / 64连接 / 10 ms限额 |
| 空地补树 | 每轮最多64尝试、8新节点、10 ms |
| 状态快照 | 5 Hz目标；快速状态/路径与不可变整树文件分离 |
| 树文件复用 | 同一规划对象跳过重复编码；相同几何复用文件；SHA256及epoch/机器人/session/tree-instance校验 |
| 树文件保留 | 6 s读取宽限＋每机最新4版；显示缓存有界，不改变控制器持有的完整路径/规划对象 |
| RViz | 位姿/路径10 Hz轮询，地图5 Hz，树2 Hz，相机5 Hz，点云10 Hz |
| 树显示 | 连通边宽0.025 m、alpha0.35；断开边0.02 m、alpha0.20 |
| Marker寿命 | 3 s；首次DELETEALL，后续仅删除消失ID/更新现存项 |
| 显示限额 | 点云每帧最多5000点；体素100000；树边30000；不截断底层地图输入 |

本实验直接对共享体素做机身净空检查，没有额外启动 Nav2；RViz 的二维投影不等于另一套 Nav2 costmap。树可进入未知区域，执行端独立检查近端可通行性。

已部署 Rust `Map::delta` 改为只对本轮 changed keys 分类占用新增/清除/撤回unknown；保留完整clone、验证顺序、失败不提交与时间refresh语义。整树和最终路径碰撞检查没有省略。循环巡航重复坐标引发的近邻索引崩溃已通过本项目vendor内的精确自适应轴KD索引修复，保留全部ID/坐标；来源与偏离上游的范围见 `planning/CORE_PROVENANCE.json`。不能将该fork称为逐字未改官方源码。


## 7. 当前版本测量与限制

此处只将**雷达时序修复后的 epoch `d83ccef432d3410e8e573a93122096cb`**作为当前测量。旧基线各约3.07 Hz、较早性能补丁后3.66/3.68 Hz均发生在旧扫描时序下，不能与当前约整圈/包的工作量混称同一个性能基准。

证据：[雷达时序验收](../../verification/2026-09-29-lidar-sensor-interval/README.md)。较后44.86秒窗口：

| 指标 | Go2 | UAV |
|---|---:|---:|
| LiDAR接收 | 9.99 Hz | 10.01 Hz |
| LiDAR实际地图融合 | **1.96 Hz** | **1.98 Hz** |
| 采集至地图应用平均 / P95 | 0.230 / 0.457 s | 0.340 / 0.544 s |
| 地图请求平均 / P95 | 0.103 / 0.150 s | 0.150 / 0.225 s |
| 窗口内移动距离 | 31.46 m | 14.44 m |
| 结束树节点数 | 4997 | 4439 |

60秒角覆盖窗口中，单包实际回波方位跨度中位数均358.20°；与融合匹配的包超过340°分别57/75、58/77。较早的60秒窗口包含启动期，融合约1.35/1.37 Hz；另一个启动后约110～145秒的35.1秒窗口，两机各1.74 Hz。后者后20包处理总耗时中位数为Go2 228 ms/UAV 298 ms，P95约374/437 ms。`cache_apply_s`包含等待地图锁，`transport.await_s`包含原生计算及IPC/调度等待，不能全部当作单一函数CPU耗时。

早期窗口出现过短时`sensor_stale`，35秒窗口仍出现约2秒UAV此状态，较后45秒没有。说明启动成本会改善但持续吞吐限制仍在；不能把这一版宣传为10 Hz地图或长时间不会再暂停。1.2秒执行新鲜度限制保持不变。

当前模型/采样配置仍是640×360、发布最多各3 Hz、SAM3共享模型总上限2 Hz。归档核对时SAM3状态为一个权重实例/两个session，运行约630秒累计处理627/621帧；这是包含启动时间的累计量级核对，不代替独立计时窗口或OHZ持续性评分。本轮未重新量化完整OHZ持续比例或GPU峰值，不复用旧硬件读数冒充当前实测。

当前45秒样本没有fatal、map、people、planner错误。10个抽样人物位置ROI的初始占用在窗口内均清零；其首次清零距**采样起点**0.66～7.68秒，包含人在ROI中继续行走的时间，**不是确认人离开后的清除延迟**。细梁ROI占用仍有波动。这些是现场短时证据，不是全场真值或长期压力认证。

当前已部署内容包括射线完整保留、真实远距命中证据、UE采样间隔修复、树传输和增量分类。尚未部署Wavemap、TTL自动消融、全图事务存储重写或无条件跳过碰撞检查。`observed`索引改为unordered_map的/tmp候选仅约0.4～0.8%收益，没有部署。

## 8. 运行依赖与启动/恢复

| 项目 | 当前来源/值 |
|---|---|
| 独立UE工程 | `/data/sim/AirGroundLab_UE58/CarlaUnreal/CarlaUnreal.uproject` |
| 共用只读引擎 | `/data/sim/CARLA_UE58_Research/UnrealEngine5_carla58` |
| 本次独立Carla模块SHA256 | `33d291317d125f548d4dfcd8ba7497d70480d84b8f775aed3336c63678803bf2` |
| CARLA Python环境 | `/data/robotics/ue58_ohz_test_go2/.venv/`，匹配UE5.8的CARLA 0.10.0接口 |
| SAM3 Python环境 | `/data/robotics/sam3_video_people/.venv/`；离线加载已安装权重 |
| ROS | `/opt/ros/humble`；domain45；RViz系统Python |
| 私有JSON依赖 | `runtime/python_deps/ujson.cpython-310-x86_64-linux-gnu.so`，5.12.1，SHA256 `2967cb08d9454085e9ed947e4d4598a59d50c817d383b8dc0c2d5fa4737eae3b` |
| 数值库线程 | 自有服务OPENBLAS/OMP/MKL各1线程，避免小矩阵变换启动过量线程 |
| 服务内存上限 | UE30G；runtime/ROS view默认8G；SAM3 12G；RGB-D capture2G；LiDAR preview1G；RViz3G（上限不是实际占用） |

启动器 `simulation/launch_sim.py` 还固定关闭音频与目录监视，使用本工程私有DDC/XDG目录；UE `carla.Go2.RequireSupport 0`、30 FPS上限及1800/512 MiB渲染池参数见源码。默认正常游戏模式不启动MCP；仅场景准备模式启用18183。

从停止状态冷启动：

```bash
cd /data/robotics/air_ground_smart3d
./lab start --rviz
./lab status
```

停止时只管理本实验的`air-ground-lab-*`服务，并让runtime保存一致的退出地图：

```bash
./lab stop
```

恢复地图、机器人位姿和巡航进度须选择**兼容且已正常停止**的一次运行：

```bash
./lab start --rviz \
  --resume-map /absolute/stopped-run/shared/map.ot \
  --resume-state /absolute/stopped-run/state.json
```

保留`.ot`旁的元数据文件；不能仅复制`.ot`忽略身份、概率校验和近距命中历史。恢复会创建新epoch，保留地图概率/已知表面历史，但清空旧观测时间；旧自由空间需在新epoch重新观测才允许执行。树按新worker重新初始化，不是载入原随机树。`--resume-state`必须同时有`--resume-map`；仅地图恢复不会恢复原机器人位姿/进度。新旧场景/profile不兼容时不能绕过校验。

`./lab start --rviz --visible`在从停止状态启动时打开UE窗口；`--no-perception`是纯几何对照的启动选项，不会在线关闭已运行的SAM3。平常不要同时启动同端口第二实例。

## 9. 与旧基线的差异及陈旧说明

1. **配置未变，代码与UE模块已变。** profiles和resolved配置完全相同；旧source哈希不能代表新版。新包必须包含当前`sim_io/lidar_rays.py`、runtime、mapper、Rust源码/原生程序、树快照模块，以及UE雷达三份源码/对应模块身份。
2. **射线策略已变。** 旧“每个命中体素代表点”会丢掉穿过不同中间格的free证据；现在所有不同实际射线保留。YAML中的原描述仍是未读取的历史字段，当前README已更新，实际统计明确`all_distinct_measured_rays`。
3. **远距更新更完整。** 同帧真实far命中保护已加入；此前曾经近距命中的表面可由9～40m真实回波恢复，未知远格仍不首次占用。`known_near_hit_keys`随地图元数据持久化，显式forget同时清该历史。
4. **真正修复sensor_tick时间。** YAML仍写10Hz/192kpps，但只有新UE模块按真实采样间隔计算；旧普通点云角度头也已同步修正。不能仅替换Python代码而漏掉UE模块。
5. **OHZ当前显示与规划历史分离。** 当前有效OHZ用同一人物文档在当前时刻构造；旧planner历史形状不是当前显示。TTL、估速窗口和形状参数未变。
6. **已完成树传输/地图diff优化和重复坐标崩溃修复。** 旧基线第8节把这些列为待做事项，已过时；全图clone及每次最终碰撞检查仍保留，不能声称已经实现完整增量存储。
7. **残留文档不得当运行事实。** `shared_model_inference:false`没有被当前代码读取，实际是一份权重、两会话；`PersonRegistry`/service类默认TTL1.5秒被启动器显式`--ttl-seconds 2.0`覆盖。`contracts`中的初期“人物OHZ待迁入”文字本次已更新；运行YAML和代码保持原状，历史参数字段的差异在这里明确。
8. **不是Git版本历史承诺。** 当前该项目本地Git仍显示未跟踪目录，旧manifest也无有效HEAD；归档依靠实际文件及SHA256标识，不能称已经完成Git提交、推送或认证。

## 10. 应一并保留的证据与归档边界

- 本目录的`profiles/`、`resolved_profiles.json`、源码/原生程序/UE文件哈希及最终manifest；归档为`source-config.tar.gz`、`native-linux.tar.gz`、`ue-integration-delta.tar.gz`，精确校验见`manifest.json`。
- 当前导航源码、tests、规划vendor来源文件、原生mapper/worker；`.venv`、Cargo target缓存、一般日志不混入源码包。
- 独立UE修改源码镜像、`simulation/uav_source_changes.json`和对应启动/配置；模型资产、引擎和整套Content需要原有外部备份/安装，不随小型参数包自动齐全。
- 私有ujson运行依赖及其provenance/许可证包含在`native-linux.tar.gz`中，未因排除runtime缓存而遗漏。
- 当前验收：`docs/verification/2026-09-29-lidar-sensor-interval/`；此前已部署树/OHZ/diff证明：`docs/verification/2026-09-29-performance-ohz-farhit/`；循环巡航修复：`docs/verification/2026-09-29-patrol-index-fix/`。
- 本次归档不停止仿真、不保存活动地图、不变更Town10、Factory、原单Go2空场的配置。若以后需要更新可恢复地图，须另行取一致性快照，不能把正在写入的state与旧map随意配对。

不同版本证据分别标明epoch、时间窗和工作量。新的基线用于后续重复实验与恢复配置，不是宣称全部动态/遮挡问题已根治。
