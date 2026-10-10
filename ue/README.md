# Empty-scene UE levels (for Windows)

These are the handmade levels, about 400 KB total. The 57 GB of CARLA towns, textures, and caches is not included. The levels are built from the engine cube `/Engine/BasicShapes/Cube`, colored by the materials in `Content/OHZTest/Materials`, whose parent material is the engine's `BasicShapeMaterial`.

| File | Purpose |
|---|---|
| `Content/AirGroundLab/Maps/AirGroundLab_Main.umap` | The empty scene that runs live. Launch argument is `/Game/AirGroundLab/Maps/AirGroundLab_Main` |
| `Content/OHZTest/Maps/OHZTest_Main.umap` | Editor default level referenced by `Config/DefaultEngine.ini` |
| `Content/*/Maps/OpenDrive/*.xodr` | CARLA road networks, one per level |
| `CarlaUnreal.uproject`, `Config/`, `Source/` | Project entry, map and rendering config, module names |

Scene origin, ground height, bounds, and pedestrian count live in `profiles/scenes/air_ground_lab.yaml`. Map coordinates are `x=carla.x, y=-carla.y, z=carla.z-0.22`.

## Option 1 — plain UE 5.8 editor, no CARLA needed (enough for scene editing)

The handmade levels use only engine cubes plus the materials in this repo, so Epic's vanilla UE 5.8 editor (Windows or Linux) is enough to view and edit them:

1. Install UE 5.8 (exact 5.8.x) and create a new Blank project (no starter content needed).
2. Copy this repo's `Content/AirGroundLab` and `Content/OHZTest` into that project's `Content/`.
3. Open `AirGroundLab/Maps/AirGroundLab_Main`. A warning about the missing `CarlaGameMode` is expected — the level opens with the default GameMode and all geometry and materials intact.
4. Edit freely. Keep the ground flat and walkable where Go2 patrols; keep the Bee corridor flyable between 1.2 m and 3.2 m.
5. Copy the edited `.umap` (plus any new materials) back. After it lands in a CARLA project, check World Settings → GameMode Override still points at `CarlaGameMode` — a plain-editor save can clear it; reselect it if so.

Do **not** try to open `CarlaUnreal.uproject` itself in vanilla UE 5.8: it references CARLA plugins vanilla UE does not have. Always go through a new blank project and copy `Content` instead.

## Option 2 — opening inside an existing CARLA UE 5.8 project

You need a CARLA UE 5.8 project (`CarlaUnreal.uproject`) that already opens on your machine. Copy these two directories as-is into that project's `Content`:

```
Content/AirGroundLab
Content/OHZTest
```

Then open `AirGroundLab/Maps/AirGroundLab_Main` in the editor. The only missing piece is CARLA's own `CarlaGameMode` blueprint, which lives in the CARLA project, not in this package.

The startup map currently written in `DefaultEngine.ini` is `OHZTest_Main`; the live dual-robot launcher overrides it with `AirGroundLab_Main`. When you build a new scene on Windows, just open `AirGroundLab_Main` directly, and do not overwrite your own `Config` directory with this one — this ini contains Linux platform sections.

The `EngineAssociation` in `CarlaUnreal.uproject` points at the CARLA engine on the Linux machine this repo was exported from. On first open on Windows, re-associate it with your own CARLA UE 5.8.

## Making a new scene for this stack

1. Build your level on top of engine basic shapes (or your own assets) inside your Windows CARLA project.
2. Keep the ground flat and walkable where Go2 patrols; keep Bee's corridor between 1.2 m and 3.2 m flyable.
3. Export/copy the `.umap` (plus any custom materials) back into `Content/AirGroundLab/Maps/` on the Linux machine's UE project at `/data/sim/AirGroundLab_UE58`.
4. Add or edit a scene profile under `profiles/scenes/` (map name, origin, ground height, bounds, flight band, `human_count`) and a team profile under `profiles/teams/`, then start with `./lab start --rviz --team <team>`.
