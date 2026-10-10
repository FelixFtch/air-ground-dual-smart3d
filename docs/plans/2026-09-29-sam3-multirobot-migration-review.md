# SAM3 多机人物跟踪与 OHZ 迁移：只读源码核对

2026-09-29。来源 `/data/robotics/ue58_ohz_test_go2`；目标 `/data/robotics/air_ground_smart3d`。本次只检查本地源码、安装版本及非认证回执，未修改旧项目、加载模型或启动 GPU 服务。目标仍使用现有 Rust SMART；没有使用 CARLA 行人 actor ID、位置或速度作为感知。

## 结论

旧跟踪代码和已下载模型仍在；新双机实例尚未接入这条感知链。可以复用旧 SAM3 Video 推理、受限历史、同帧 RGB-D 投影和速度质量门，但需要新建多源调度、持久采集包、跨相机关联与三维风险适配。共享模型权重不会自动共享人物身份。

旧链路：同帧 RGB + organized XYZ + capture-time TF → SAM3 Video 本地 track ID/mask → PeopleProjector → 采集时刻 map 位置与 XY 速度 → 临时人物约束/OHZ → 旧 Go2 规划消费。

最关键的当前缺口是目标 `sim_io/runtime.py:187` 仅保存匹配帧的 `latest_rgb.jpg/json`，未保存完整像素深度/XYZ，也没有可供异步模型固定读取的不可变采集包。`sim_io/geometry.py:25` 是稀疏建图点解码，不能拿这些点对完整 SAM3 mask 做逐像素投影。

## 可独立复制的边界

源文件均在 `/data/robotics/ue58_ohz_test_go2/scripts/`。目标目录名称为建议，不代表已经建立。完整源路径、SHA-256 与测试文件见 [复制清单](/tmp/sam3_multirobot_copy_candidates.json)。

| 源文件 / 关键入口 | 建议用途 | 必须改动或保留的边界 |
|---|---|---|
| `car1_sam3_video_people.py:71` VideoModel；`:114` reset；`:126` infer；`:230` FrameSequence | `perception/sam3_backend.py` + service | 把单个 `self.session/frame_index/session_id` 拆为按相机保存；权重和 processor 只加载一次。提取 overlay、帧顺序检查；不搬旧 main、HTTP fetch、lock/output。 |
| `car1_sam3_video_memory.py:167/192/199/314` | 独立 vendored 内存策略 | 保留模型源码哈希检查、每路 session 限额、先 lost-track 清理再裁剪历史的顺序。配置模型一次，配置每个新 session 一次。 |
| `car1_sam3_people_projection.py:87/123/191` encode/decode/PeopleProjector | `registered_rgbd.py` + `people_projection.py` | 保留严格 shape/dtype/ZIP/哈希/采集时间检查；参数化 camera、尺寸、frame、epoch。每路一个 projector/history。 |
| `car1_person_observations.py:29/105/137` | 新观测校验和临时人物层 | 复用证据门，重写多源身份、时间与共享 OctoMap 适配；不要照搬旧 LocalCostmap 包装。 |
| `car1_ohz_geometry.py:23/42/52` | Go2 二维 OHZ 几何与对照测试 | 明确只处理 XY；半径已经包含机器人包络，不能让 UAV 再次按 Go2 半径解释。 |
| `car1_prediction_error.py:14/37` | 固定标定预算读取 | 新环境先为 uncalibrated；不能默认沿用旧镜头/分辨率/场景的裕量。 |
| `car1_sam3_rviz_image.py:41` Sam3OverlayReader | 各相机 RViz overlay 读取 | 保留 image/status 哈希不一致时拒绝显示；改新 schema、路径与相机身份。 |
| `car1_dynamic_policy.py:122` CrossingWaitPolicy | 可选第二阶段 | 在有效观测、OHZ 几何和 planner/guard 一致后再迁移；不是新的规划器。 |

不宜整体复制：

- `car1_vlm_evidence.py`：`:212` 精确帧配对、`:235` 同 capture stamp 的 map→optical/body TF 值得借鉴；但 ROS loop、旧 costmap 状态、car1 控制所有权等耦合不适合新项目。
- `car1_rgbd_app.py`：导入即执行旧 prep guard 并实例化旧 VlmEvidence；旧端口/HTTP 路由不必保留。
- `car1_native_planner.py`：`:678` 以后可用来理解旧 fusion/OHZ 接线，不移植整个旧规划器或控制器。
- `car1_sam3_node.py`：这是 road/sidewalk/grass/cone 静态语义节点，并非人物 Video tracker；不要因此多加载第二份模型。其 ExactFrameBuffer 仅可作配对测试参考。

## 单权重、多视频 session

旧 `VideoModel` 已按 `model(inference_session=..., frame=..., frame_idx=..., reverse=False)` 推理（`:134`）。安装的 Transformers 源码 `Sam3VideoInferenceSession` 将 object IDs、track history、prompt 和 cache 放在 session 内；processor 的 `init_video_session` 创建独立对象。最小实现：

1. 一个进程、一份 Sam3VideoModel、一份 processor、同一已校验本地权重。启动时离线加载一次，模型配置只做一次。
2. `sessions[(scene_epoch, robot_id, camera_id)]` 各含 session UUID、连续的已处理帧序号、FrameSequence、PeopleProjector 和 lost-track 状态。更换 calibration 或时钟回退也需隔离重置。
3. 每源最多一个待处理的最新完整 bundle，公平轮询/按截止时间调度；GPU 推理串行，一次只处理一个 session。没有验证线程并发模型调用安全性，先不要并行调用同一模型。
4. 可以跳过旧采集帧，但 SAM3 的 session frame index 对该源保持从 0 连续；速度使用原 capture time，而非推理完成时间。某一路重置不能清掉另一路的历史。
5. 旧 4 Hz 是单流限制，不保证两路各 4 Hz。先测单帧延迟和显存，再定每路频率；把队列等待、采集年龄、推理延迟和丢帧原因分别记录。

旧内存策略为每路 history 128、最大对象 16、每 16 帧重新 conditioning、conditioning context 4、lost TTL 30 秒。两路会增加 session 状态，不能把“一个权重”宣传为“显存完全不变”。策略明确限制了上游上下文，不能声称与无界历史数值等价。只因推理慢不应触发“断流重置”；已有 FrameSequence 测试覆盖此区别。

本地依赖已通过文件元数据和源码哈希核对：Python 3.10 环境 `/data/robotics/sam3_video_people/.venv`；torch `2.14.0+cu126`、transformers `5.17.0`、kernels `0.16.0`、numpy `1.26.4`、Pillow `11.1.0`、safetensors `0.8.0`。模型 revision `3c879f39826c281e95690f02c7821c4de09afae7`，本地 model/receipt 可复用，权重无需复制到新项目或重新下载。保留 `car1_sam3_video_memory.py:19` 的两份 Transformers 源码 SHA 校验及内核回执校验；新服务使用自己的输出与锁，避免操控旧服务目录。

## 需要改掉的单机假设

- `video_people.py:17–21` 旧 ROOT/OUTPUT、HTTP 20280/20287、`/api/fubao/v1/…`；`:45` `X-Go2-*`、`d435i_rgb`、848×480。
- `people_projection.py:22/43` 固定 848×480、camera ID、两元素旧 map_epoch，body TF child `car1/base_link`；这些都应来自当前实例/传感器 profile。新前视相机是 640×360，不能只缩放 RGB 而不同时变换深度像素与内参。
- `video_people.py:188/267/272` 一个输出目录、worker.lock、Publisher/FrameSequence/session；每相机输出独立，服务锁归新项目。
- `person_observations.py:79` 连合后的列表中禁止重复裸整数 track ID。两路各出现 native ID 1 是正常情况；必须先保留完整本地身份，不能直接拼旧列表。
- `person_observations.py:87` map_stamp 指旧 costmap 发布时间。新共享地图 watermark 不等于统一“当前预测时刻”；改为明确的 `prediction_reference_sim_time`，另记录所消费 map revision 和 capture time。

## 持久 registered RGB-D bundle 输入契约

建议以版本化 `air_ground.registered_rgbd/v1` 代替旧 factory schema，复用旧 NPZ 校验思想。每包必须同时包含：

- 身份：`scene_id, scene_epoch, content_version, frame_id, robot_id, camera_id, calibration_id`。
- 采集：`frame_seq, capture_sim_time, capture_stamp`；RGB、depth、camera transform、body pose 必须同 capture frame/time。`receive_wall_time/receive_monotonic_time` 是收包时间，不能替代 capture time。
- 像素：RGB 图像及 `width,height,K`；同像素网格的完整 float32 `[H,W,3]` optical XYZ，无效像素 NaN；记录有效深度范围与颜色编码。建图 stride/downsample 不得作用于这份输入。
- 位姿：采集时刻的 `camera_to_map` 和 `body_to_map`，声明坐标轴和单位；禁止查询 latest pose 补齐旧图像。
- 完整性：`frame_token` 覆盖源身份、帧与标定；RGB/XYZ 哈希与 payload 一起封装并校验。输出观测继续保留 token、capture time 和原 bundle 路径/哈希。

使用独立文件名 `<frame_seq>-<token>.npz`，先写临时文件再原子改名，最后更新小型 latest manifest；每源保留有界环（例如 8–16 包），正在推理的包不得被回收。需要复核的采样包和结果可按配置另行留存，避免无界视频归档。这样可以异步重放、追查速度和 overlay，而不从新帧补深度。旧 `encode_rgbd_capture/decode_rgbd_capture` 有 ZIP_STORED、成员数/体积、Numpy dtype/header、JPEG hash 等边界检查，值得保留并按 640×360 重新测试。

坐标转换是实际移植风险：当前 `geometry.depth_points` 输出 CARLA sensor `+X forward,+Y right,+Z up`；旧 projector 按 optical `+X right,+Y down,+Z forward` 读取 z 深度。应使用 `xyz_optical=[localY,-localZ,localX]`。若 `S=diag(1,-1,1)` 为 CARLA world→map，`C=[[0,0,1],[1,0,0],[0,-1,0]]` 为 optical→CARLA sensor，则 `R_map_optical=S @ R_carla_sensor @ C`，`t_map=S @ (t_world-world_origin)`。同时改变坐标基，不能只把 Y 反号的反射矩阵直接当 quaternion。用已知像素、相机旋转/UAV pitch、静止人物世界点做数值测试。

SAM3 前视 bundle 的生产应独立于 mapper 的深度降频/抽样成功与否。当前 runtime 是融合完成后才保存 RGB；若照搬，会把感知可用性错误绑定到 mapper 队列。UAV 96×96 的六向辅助 depth 没有 RGB 配对，不应作为人物视频流。

## 共享人物 ID 与速度

本地身份至少为 `(scene_epoch,robot_id,camera_id,tracking_session_uuid,native_track_id)`。旧 `PeopleProjector:259` 只有 `sessionUUID:nativeID`，`normalize_people` 的现有约束不能直接成为多机 registry。

新建共享 person registry：用同步到同一参考时刻的 map 空间位置、位置质量、时间差和速度一致性关联不同来源，保存 global ID→各来源 local key 的证据关系。同一相机内必须一对一；不同相机可支持同一全局人物。两人接近/遮挡/不同时间观测产生歧义时保留待定或新 ID，不强行“最近点就是同一人”。不引入未经实现的外观 ReID；跨长时间失联后的身份延续不能保证。

旧投影是可见表面点的中位数与 10–90% 分位范围，不是完整人体中心/包围盒。不同视角的表面中心会偏移，因此不能把来自两机的中心交替串成一条速度序列。先各源按 capture time 估速，再融合相容的有效速度；关联变化/异常跳变时清理对应速度证据。旧速度函数 `people_projection.py:197` 要求至少 4 次观测、足够时间跨度、鲁棒斜率和速度/残差门；保留这些有效性标志。

旧链路只测得 XY 速度，Z 速度未估计。新输出应明确 `velocity_xy_valid` 与 `velocity_z_valid=false`，不能填零后宣称测得完整 XYZ 速度。共享 registry 发布原始观测/不确定度，机器人各自按 profile 加包络，避免共享一个已按 Go2 膨胀的风险给 UAV 再次膨胀。

## OHZ 如何接回现有 SMART

人物层是有时间与来源的临时感知/预测约束；场景 OcTree 仍保留实际深度/LiDAR 障碍。不要把人物预测路径写成永久墙，不要为“人物移动”批量删除真实地图点；旧 `person_observations.py:182–208` 也明确不把历史占用格伪装为移动人物。

当前新 worker 已 vendored 的 `planning/vendor/smart3d_observed/src/smart/obstacle.rs:11/30/38` 有 `SphericalObstacleWithOhz<F,N>`、sweep、ellipse；`hazard_zone.rs:7/40/87` 有 generic N 的 sphere/capsule/ellipsoid 判断。但目前调用 core update 时人物 hazard 列表为空；现有 `dynamic_boxes` 是伙伴机体占用，不等于 SAM3/OHZ 已接入。

最小后续接线是新 `dynamic_people` 契约：global ID、source contributors、capture/reference time、measured map geometry、velocity 有效分量、horizon、uncertainty/calibration 状态、expiry；分别由 Go2/UAV profile 转为可验证风险几何，并让 planner、近端执行 guard、RViz 使用同一份几何。保留真实 SMART<f32,3> XYZ 树。旧二维 ellipse 可直接作 Go2 对照；UAV 必须显式处理观测 z 范围、可见表面不完整与垂直速度未知。若采用保守人体高度先验，需标注其为 profile 先验，不能声称 SAM3 测得全人体。不能直接无限拉伸 XY 椭圆当三维感知。

旧预测误差校准是固定 held-out 预算，不是在线真值输入。新镜头、分辨率、UAV 视角变化后先 uncalibrated，再根据新感知轨迹评估；不得悄悄复用旧标定裕量。

## 最小实施顺序与验收

1. 新建独立 `perception/`，复制带哈希的有限源码/测试；先实现 bundle writer/reader 和完整坐标边界，CPU 测试通过后再启用模型。
2. 一份权重、两路 session 以 shadow mode 跟踪，不影响当前运动；输出各源 overlay、mask+depth 质量、map 位置、速度有效性与帧证据。实测延迟、公平性、显存和丢帧。
3. 接共享 registry 和 RViz 全局 ID/来源显示；验证同时 native ID 1 不碰撞、单路重置不伤另一流、两机看到同一人可合并、交叉人物不被强合并。
4. 先离线/shadow 验证同一轨迹下 OHZ/native/guard/display 几何一致，再启用人物风险对执行与 SMART 树修复的影响。丢失/过期要触发临时层清除，不能清除真实占用。

可继承 10 份 CPU 测试，列表见复制清单。必须补充：错帧/错标定/错 epoch 拒绝；640×360 可变形状；bundle 发布/读取无混帧；UAV pitch/机体转动面对静止人物不产生假速度；双路 frame index/session reset 独立；跨视角可见表面偏差不制造速度；身份歧义不强合并；公平调度无饥饿；未知垂直运动不冒充有效 3D 速度；人物过期几何清除后同树恢复；两种机器人各自包络膨胀且不重复。

此次依据是源码与本地依赖检查。没有运行双视频模型或性能测试，因此不承诺每路帧率、显存预算、跨相机身份准确率或 Factory 实时运动效果。
