# Go2 + Fighter Drone / shared-map SMART3D

空场景双机：Go2 走二维树，Fighter Drone 走三维树，共用一张 OctoMap。人员现场用 YOLO26 + MapTracker；SAM3 仍可用 `./lab people sam3` 换上。Bee 对成团障碍另有 0.35 米三维膨胀，单独一格不加。

| 谁来用 | 看哪里 |
|---|---|
| 做新的 UE 场景 | `profiles/scenes`、`profiles/teams`、`profiles/robots`、`profiles/sensors`。UE 工程不在这个仓库里，在本机 `/data/sim/AirGroundLab_UE58` |
| 改人员 OHZ | [`ohz/README.md`](ohz/README.md)。现行椭圆在 `perception/hazards.py`，剪树在 `planning/` |
| 规划、RRT*、LRZ | `planning/`，不要拆开 |


当前用户认可的完整配置、源码归档和运行证据：[双机＋SAM3 稳定配置 v2](docs/baselines/2026-09-29-dual-robot-sam3-stable-v2/parameters.md)。当前版本索引为 `docs/baselines/CURRENT.json`；[早期基线](docs/baselines/2026-09-29-dual-robot-sam3-stable/parameters.md)保留用于对照。

最新雷达修复与实测见[传感器采样时序验证](docs/verification/2026-09-29-lidar-sensor-interval/README.md)，地图射线与远处真实表面更新见[射线证据恢复](docs/verification/2026-09-29-ray-evidence-recovery/README.md)。之前的[循环巡航崩溃与树连续性修复](docs/verification/2026-09-29-patrol-index-fix/README.md)。支持停止后显式同时传入 `--resume-map` 和 `--resume-state`，恢复原位姿与巡航进度。性能后续方案见[分步优化计划](docs/plans/2026-09-29-performance-followup.md)。

## 本机启动、查看、停止

```bash
cd /data/robotics/air_ground_smart3d
./lab start --rviz
./lab status
./lab stop
# 显式恢复同一场景已有地图（保留占用，旧自由空间需重新观测）
./lab start --rviz --resume-map /absolute/previous/run/shared/map.ot
```

默认 UE 离屏运行，RViz 在桌面显示。希望看到 UE 游戏窗口时，先停止本实验，再用 `./lab start --rviz --visible`。不启用新网页控制台。仅管理 `air-ground-lab-*` 自有服务，不关旧项目或其他应用。

启动器按顺序拉起：独立CARLA 2400 → 两机器人与传感器 → 一个共享OcTree → 每机独立SMART/控制器 → 注册RGB-D采样 → 一个SAM3模型/两套独立视频会话 → ROS domain45/RViz。ROS图中青色为Go2二维树，橙色为UAV真实三维树；灰色为实际断开的分支。真实树边和路径直接来自Rust父子关系，不填补不存在的边。RViz透明机身盒是明确标注的碰撞包络；实际Go2/Fighter外观在UE里。

## 文件结构

| 目录 | 用途 |
|---|---|
| `profiles/robots` | 移动维度、机身净空、速度、执行限制 |
| `profiles/appearances` | Go2、Fighter、Bee、Cute3、小Slime、AirSim备选外观 |
| `profiles/sensors` | 外参、标定、配对和融合采样参数 |
| `profiles/scenes`、`profiles/teams` | 场景身份、端口、地图范围、实例与各自巡航点 |
| `sim_io` | 捕获时位姿桥、真实传感器、共享融合及独立任务生命周期 |
| `mapping` | C++ OctoMap服务、版本增量、完整`.ot`快照 |
| `planning` | 固定来源的LINKS Rust核心、N=2/N=3适配、真实树输出 |
| `execution` | 连续机身扫掠、近端已知自由空间检查、路径跟随 |
| `perception` | 配准RGB-D、人物掩码/ID（SAM3 或 YOLO26，二选一）、深度投影、共享人物记录与临时OHZ |
| `observability` | 只读ROS/RViz树、路径、体素、TF与相机 |
| `simulation` | 独立UE启动、地图构建、UAV C++源码镜像与接口说明 |
| `docs/plans`、`docs/verification` | 完整计划、实际测试证据、截图与限制 |
| `runtime/runs/<epoch>` | 每次运行的配置快照、地图、日志、最新图像与状态；不纳入Git |

仿真工程：`/data/sim/AirGroundLab_UE58/CarlaUnreal/CarlaUnreal.uproject`。
只读共用引擎：`/data/sim/CARLA_UE58_Research/UnrealEngine5_carla58`。
只读共用CARLA Python环境：`/data/robotics/ue58_ohz_test_go2/.venv/`，使用已匹配UE5.8的0.10.0接口；不要换成旧CARLA wheel。

## 原单机与新双机的隔离

| 项目 | 原单Go2多人空场 | 新Go2＋无人机共享OctoMap |
|---|---|---|
| 导航工程 | `/data/robotics/ue58_ohz_test_go2` | `/data/robotics/air_ground_smart3d` |
| UE工程 | `/data/sim/CARLA_UE58_Research/Carla-ue58/Unreal/CarlaUnreal` | `/data/sim/AirGroundLab_UE58/CarlaUnreal` |
| 关卡 | `/Game/OHZTest/Maps/OHZTest_Main` | `/Game/AirGroundLab/Maps/AirGroundLab_Main` |
| CARLA / ROS域 | `2200 / 43` | `2400 / 45` |
| 控制台 | `20280` | 本阶段没有新控制台，使用RViz |
| 人数、传感器、路线 | 原工程`configs` | 新工程`profiles`与每次运行快照 |
| 地图状态 | 原工程自己的运行状态 | 新工程`runtime/runs/<epoch>/shared` |

共同使用引擎二进制和现有Python依赖环境，实验配置与地图数据不共用。原版SAM3/OHZ保留在原工程。新双机版已单独迁入SAM3视频人物跟踪，复用一个权重实例；两台相机有独立视频会话，再按时空兼容性关联共享人物ID。两个真实LiDAR分别在`/air_ground/go2_01/lidar/points`和`/air_ground/uav_01/lidar/points`显示；新启动器统一管理雷达预览服务。

## 运行合同与当前边界

原Town10、Factory、单Go2空场与SSD备份没有被替换。新地图有独立身份和运行epoch；退出保存本轮`.ot`及元数据，下次默认从新观测开始，不自动导入旧场景地图。`runtime/latest_run.json`指向当前/最近一轮，状态有时间戳和running标志。

无人机是运动学飞行，使用真正XYZ/yaw、UE实体扫掠和实际位姿反馈，不是旋翼动力学/PX4。Go2是固定高度平地导航，不支持楼梯/叠层。两机的伙伴位置来自合作位姿通信，为临时碰撞约束；人的障碍只来自LiDAR/深度，不拿行人真值喂规划。

本阶段六人同时作为真实传感器几何障碍和SAM3人物实验。人物的部分可见表面三维框来自掩码＋配准深度；XY速度需要同源连续至少四次可靠观测，不估计Z速度，也没有外观ReID保证。人物OHZ为临时约束，Go2使用二维椭圆，UAV使用三维椭球；RViz显示与实际Rust规划器同一份OHZ。不得把显示出来的框当成已完整测得人体体积。这里的规划核心来自先前修复过的native fork，并包含已记录的多维清障/重定根适配；不是声称逐字未修改上游。原始RRT*与来源哈希见`planning/CORE_PROVENANCE.json`。

当前无人机除前视RGB-D与LiDAR之外，还用六个低分辨率近距深度视图覆盖机身四周的自由空间，防止仅有前视画面时把侧后未知区域擅自放行。它们的原始数据与前视都保持有界队列，未加载额外视觉大模型。前向深度融合6m、全方向补盲3m；两机LiDAR按原单Go2 costmap配置，9m内记录障碍、真实测量射线最远清理40m（传感器量程45m）。远处实际命中表面不当作自由空间；无回波不产生清理射线。来源与更新期限分别管理。已知空闲保留到新占用证据出现；实际执行仍检查本机近期已融合雷达、当前占用、伙伴位置和UE实体碰撞，未知区域不直接执行。

雷达保留所有不同的真实测量射线，只合并完全相同的端点；不按末端体素只选一条代表射线。远处真实表面保护单独按体素去重。UE旋转雷达按实际传感器采样间隔计算点数与扫描角度；首帧和调度波动仍可能产生部分扫描包。YAML内 `mapping_sampling` 是未被代码读取的历史描述，以此实际实现为准。LiDAR目标融合频率为每机5Hz，前向深度每机2Hz，六路补盲各约0.67Hz；这些是调度目标，实测应查看当前状态的`sensor_stats`及`fusion_metrics`。地图显示5Hz、树2Hz、位姿/路径10Hz分开发布；不会用显示频率冒充融合频率。同步UE运动回执由自有`motion_rpc`子进程等待，避免CARLA的Python GIL阻塞传感器处理；碰撞回执仍返回控制器，共享CARLA安装未修改。

融合还采用有界流水线：下一帧原生 OctoMap 请求可与上一帧 Python 缓存应用并行，但最多只有一个缓存应用任务在途；提交下一个应用前必须完成前一个，不排队积压旧图像。每帧绑定自己的采集时间、位姿和身份，缓存按版本顺序应用后才更新执行新鲜度。若规划器版本暂时领先执行缓存，控制器继续等待对应版本。停止或异常时会处理完已提交的应用任务。`apply_queue_s` 区分应用等待，`cache_apply_s` 记录实际缓存应用，`arrival_to_apply_wall_s` 保留传感器到达至应用完成的整体时延。

当前实时性与离线／在线验收统一见[OctoMap 实时性验证](docs/verification/2026-09-29-octomap-realtime.md)；调度目标、显示频率和实际传感器融合频率分别报告。

## 重建与验证

```bash
cmake -S mapping -B mapping/build -DCMAKE_BUILD_TYPE=Release
cmake --build mapping/build -j4
cargo build --release --offline --locked --manifest-path planning/Cargo.toml
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  /data/robotics/ue58_ohz_test_go2/.venv/bin/python -m pytest -q tests -p no:cacheprovider
```

UE的9份修改源码、原/新哈希与UAV运行测试见`simulation/UAV_SIMULATION.md`。商业资产只留在本机UE工程，不放进源码Git。源码测试、实体测试与持续联合运行证据分别记录；不要用离线通过数代替实际运行效果。

## 人物、实时地图与修复记录

人员感知后端二选一，同一时刻只运行一个：

```bash
./lab people            # 查看当前后端
./lab people yolo26     # 运行中切换：先停 SAM3，再起 YOLO26
./lab people sam3       # 切回 SAM3
./lab start --rviz --people yolo26   # 启动时指定
```

选择记在 `runtime/people_backend.json`，下次 `./lab start` 沿用，缺省为 SAM3。两个后端只负责找人和给每台相机的跟踪 ID；包围框、速度、方向和 OHZ 椭圆都由同一套深度投影、测速和 `shared/people.json` 产生，运行时和 RViz 不区分来源。YOLO26 用 `yolo26m-seg`（只开 person 类，权重哈希见 `provenance/yolo26_people.json`），每一帧 RGB-D 都处理，实测每相机约 2.7 Hz，上限是 RGB-D 采集的 3 Hz；ID 在地图坐标里按位置关联，离开视野 20 秒内回来、衣着颜色相近就认回原 ID。2026-10-04 在本场景 100 秒录制上对照 CARLA 行人真值：YOLO26 检出率 95.3%、精确率 98.4%，SAM3 为 97.8%、99.5%；每帧约 11 ms 对约 300 ms；速度误差中位数都是 0.02 m/s。细节见 `perception/PEOPLE_SERVICE.md`。

默认启动RGB-D采样和所选人员后端；`--no-perception`用于从停止状态启动纯几何基线，不会关闭已经运行的感知服务。SAM3总处理上限2帧/秒，双相机公平调度，实测每相机约1帧/秒；最新帧替换待处理帧，不积压图像。当前人物几何TTL为2秒，预测最多外推1秒。估速最多保留6帧/4.5秒历史，需要连续至少4次可靠测量；这不延长人物占用有效期。没有可靠速度时不画速度箭头，不伪造运动方向。

RViz两机LiDAR独立显示；两块人员画面显示当前后端（SAM3 或 YOLO26）的掩码和ID，话题为 `/air_ground/<robot>/people/image`。人物线框是深度可见表面的三维范围，绿色/橙色OHZ分别来自两机规划器。LRZ直接取自native规划器，本实验半径4米（不是原单Go2版的9米）。

本次修复：恢复地图后的空观测时间不再导致native解析失败/全图无限重读；地图融合拒绝旧帧覆盖新证据，同一时刻命中优先；当前模拟器profile采用单次命中/自由射线即可翻转占用的概率参数。没有被自由射线重新穿过的区域不会凭空清空。此概率设置用于当前仿真，并非已验证的真机传感器参数。

较早的恢复修复阶段曾做40秒运行核对：地图显示快照相对传感器时钟平均落后0.288秒，最大0.625秒，两台机器人均完成两个新巡航目标。这是[历史恢复验证](docs/verification/2026-09-29-map-people-recovery.md)，不是当前端到端人体清除延迟或最新配置的性能结果；后续实时性改动和实测以[OctoMap 实时性验证](docs/verification/2026-09-29-octomap-realtime.md)为准。
