# Editing the 3D scenes on Windows

This folder holds the handmade levels (`.umap` files), about 400 KB total. The 57 GB of CARLA towns, textures, and caches is not included. CARLA is the simulator these scenes run in. Each level is built from the engine cube `/Engine/BasicShapes/Cube` and colored by the materials in `Content/OHZTest/Materials`. Their parent material is the engine's `BasicShapeMaterial`.

| File | Purpose |
|---|---|
| `Content/AirGroundLab/Maps/AirGroundLab_Main.umap` | The empty scene that runs live. Launch argument is `/Game/AirGroundLab/Maps/AirGroundLab_Main` |
| `Content/OHZTest/Maps/OHZTest_Main.umap` | Editor default level referenced by `Config/DefaultEngine.ini` |
| `Content/*/Maps/OpenDrive/*.xodr` | CARLA road networks, one per level |
| `CarlaUnreal.uproject`, `Config/`, `Source/` | Project entry, map and rendering config, module names |

Scene origin, ground height, bounds, and pedestrian count live in `profiles/scenes/air_ground_lab.yaml`. Map coordinates are `x=carla.x, y=-carla.y, z=carla.z-0.22`.

Two robots appear in the rules below. Go2 is the robot dog. Bee is the drone.

## Option 1: plain UE 5.8 editor, no CARLA needed

This route is enough for scene editing. The handmade levels use only engine cubes plus the materials in this repo. So Epic's vanilla UE 5.8 editor (UE as Epic ships it, no CARLA plugins), on Windows or Linux, can view and edit them.

1. Install UE 5.8 (exact 5.8.x). Create a new Blank project. No starter content needed.
2. Copy this repo's `Content/AirGroundLab` and `Content/OHZTest` into that project's `Content/`.
3. Open `AirGroundLab/Maps/AirGroundLab_Main`. You will see a warning about the missing `CarlaGameMode`. It is expected. The level opens with the default GameMode (the class that sets a level's rules), and all geometry and materials stay intact.
4. Edit freely. Keep the ground flat and walkable where Go2 patrols. Keep the Bee corridor flyable between 1.2 m and 3.2 m.
5. Copy the edited `.umap` back, plus any new materials. After it lands in a CARLA project, check that World Settings → GameMode Override still points at `CarlaGameMode`. A plain-editor save can clear it. Reselect it if so.

Do **not** open `CarlaUnreal.uproject` itself in vanilla UE 5.8. It references CARLA plugins that vanilla UE does not have. Always go through a new blank project and copy `Content` instead.

## Option 2: inside an existing CARLA UE 5.8 project

You need a CARLA UE 5.8 project (its own `CarlaUnreal.uproject`) that already opens on your machine. Copy these two directories as-is into that project's `Content`:

```
Content/AirGroundLab
Content/OHZTest
```

Then open `AirGroundLab/Maps/AirGroundLab_Main` in the editor. The only missing piece is CARLA's own `CarlaGameMode` blueprint. It lives in the CARLA project, not in this package.

The startup map written in `DefaultEngine.ini` is `OHZTest_Main`. The live dual-robot launcher overrides it with `AirGroundLab_Main`. So when you build a new scene on Windows, just open `AirGroundLab_Main` directly. Also, do not overwrite your own `Config` directory with this one. This ini contains Linux platform sections.

The `EngineAssociation` in `CarlaUnreal.uproject` points at the CARLA engine on the Linux machine this repo was exported from. On first open on Windows, re-associate it with your own CARLA UE 5.8.

## Making a new scene for this stack

1. Build your level on top of engine basic shapes, or your own assets, inside your Windows CARLA project.
2. Keep the ground flat and walkable where Go2 patrols. Keep Bee's corridor flyable between 1.2 m and 3.2 m.
3. Export or copy the `.umap`, plus any custom materials, back into `Content/AirGroundLab/Maps/` in the Linux machine's UE project at `/data/sim/AirGroundLab_UE58`.
4. Add or edit a scene profile under `profiles/scenes/` (map name, origin, ground height, bounds, flight band, `human_count`) and a team profile under `profiles/teams/`.
5. Start it with `./lab start --rviz --team <team>`.
