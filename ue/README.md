# 空场景 UE 关卡（给 Windows）

这里是能打开的手工关卡，大约 400 KB。57 GB 的 CARLA 城镇、贴图和缓存没有放进来。关卡用引擎自带方块 `/Engine/BasicShapes/Cube`，颜色来自 `Content/OHZTest/Materials`，父材质是引擎的 `BasicShapeMaterial`。

| 文件 | 作用 |
|---|---|
| `Content/AirGroundLab/Maps/AirGroundLab_Main.umap` | 现场在跑的空场景。启动参数是 `/Game/AirGroundLab/Maps/AirGroundLab_Main` |
| `Content/OHZTest/Maps/OHZTest_Main.umap` | `Config/DefaultEngine.ini` 里的编辑器默认关卡 |
| `Content/*/Maps/OpenDrive/*.xodr` | CARLA 路网，和关卡同名 |
| `CarlaUnreal.uproject`、`Config/`、`Source/` | 工程入口、地图与渲染配置、模块名 |

场景原点、地面高度、范围和行人数量在 `profiles/scenes/air_ground_lab.yaml`。地图坐标是 `x=carla.x, y=-carla.y, z=carla.z-0.22`。

## 在已有的 Windows CARLA 工程里打开

需要本机已经能打开 CARLA 的 UE 5.8 工程（`CarlaUnreal.uproject`）。把下面两个目录原样放进那个工程的 `Content`：

```
Content/AirGroundLab
Content/OHZTest
```

然后在编辑器里打开 `AirGroundLab/Maps/AirGroundLab_Main`。缺的只有 CARLA 自带的 `CarlaGameMode`，那份蓝图在 CARLA 工程里，不在这个包里。

`DefaultEngine.ini` 里现在写的启动图是 `OHZTest_Main`。现场双机启动时会改成 `AirGroundLab_Main`。在 Windows 上做新场景时，直接打开 `AirGroundLab_Main`，不要整份覆盖自己的 `Config`。这份 ini 里有 Linux 平台段。

`CarlaUnreal.uproject` 的 `EngineAssociation` 指向上面这台 Linux 的 CARLA 引擎。Windows 上第一次打开时，把它关联到自己的 CARLA UE 5.8。
