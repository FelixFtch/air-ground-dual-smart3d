# 使用本次配置备份

这是双机独立实验的配置、源码和兼容本机的原生程序归档，不是完整 UE 引擎、商业模型或 SAM3 权重备份。Town10、Factory、原单机空场配置不在本包中。

当前本机运行无需恢复或重启。本机正常启动：

```bash
cd /data/robotics/air_ground_smart3d
./lab start --rviz
./lab status
# 需要停止时才执行：
./lab stop
```

启动顺序由 lab 负责：独立 UE/CARLA 2400 → 共享地图、双机规划与控制 → 雷达展示、ROS domain45/RViz → RGB-D 与 SAM3。默认从新观测开始；恢复已有地图需要显式 --resume-map；恢复位姿和巡航进度还需同一次停止保存的 --resume-state。不能将本包 live-status.json 作为停止快照。上次已验证的快照位置与哈希见 saved-map-reference.json。

源码恢复时先停止本实验，保留原目录，再将 source-config.tar.gz 解包到 /data/robotics/air_ground_smart3d，将 native-linux.tar.gz 解包到相同位置。native包包括两份原生程序及私有ujson依赖、来源和许可证。将本配置目录保留于项目 docs/baselines/ 下，供README链接与核对使用。原生文件依赖当前 Linux 环境，其他机器应按 README 重编。UE delta 仅能应用于当前兼容 AirGroundLab 工程，需停止该实例后解包到 /data/sim/AirGroundLab_UE58/CarlaUnreal；它包含九份集成源码、当前 Carla 模块、场景主地图与工程配置，依赖现有引擎、插件库与引用资产。不得解包覆盖旧 Factory 或 Town10。

dependencies.json 记录外部环境；未包括虚拟环境、Cargo 缓存、SAM3 权重、完整 UE Content 或引擎。归档内均为相对路径。manifest.json 校验整个配置包，source-sha256.json、native-sha256.json、ue-delta-sha256.json 校验各归档条目。旧参数基线保留，新版通过 CURRENT.json 指向。
