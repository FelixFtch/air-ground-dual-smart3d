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

## The two everyday edits

**Adding wall cubes.** Open `AirGroundLab_Main` (Option 1 above), drag Engine basic shape cubes from the Place Actors panel, scale and place them, and give them one of the materials in `Content/OHZTest/Materials`. Save and send the `.umap` back. That is the whole edit: the robots' lidar sees any solid geometry you add, so new walls become obstacles on the next restart, no other file needs changing.

**Editing pedestrian walking routes.** UE already draws the curve. Open `AirGroundLab_Main`, look at the floor in the 3D viewport, and click the path onto that floor. One curve is one person.

1. Switch the mode dropdown at the top left from **Selection** to **Modeling** (Shift+5). In the left tool panel choose **Create**, then **Draw Spline**.
2. Click on the lab floor where the walk starts. Each following click drops the next point on the ground, and the curve appears between them. Press Enter to accept. The new actor shows up in the World Outliner.
3. Rename that actor to `PED1`, `PED2`, ... One spline is one person. Set `human_count` in the scene profile to that same count, or extra people pile onto the last curves.
3. Copy `tools/ue_export_ped_routes.py` into the project's `Content/Python/`. In the Output Log, switch the console to Python and run `import ue_export_ped_routes; ue_export_ped_routes.export()`.
4. Replace the `pedestrian_routes:` block in `profiles/scenes/air_ground_lab.yaml` with the generated fragment, and restart the simulation.

The person walks the curve from the first point to the next, about one metre at a time, then from the last point back to the first and repeats. PED1, PED3, PED5 follow the direction you drew. PED2, PED4, PED6 start at the last point and walk back toward the first. Map XY equals UE world XY divided by 100.
