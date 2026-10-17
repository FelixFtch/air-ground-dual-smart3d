# Go2 + Bee / shared-map SMART3D

Two robots share one empty scene. Go2, the robot dog, drives on the floor and plans in 2D. The Bee, the drone, flies and plans in 3D. Both plan with SMART3D, the method from LINKS Lab: it grows a tree of candidate routes, in the style of RRT* (a sampling-based planner), and repairs that tree when obstacles change instead of replanning from zero.

Both robots fill one shared OctoMap, a 3D occupancy grid built from lidar scans and RGB-D images (a color image plus a per-pixel depth image). People detection runs YOLO26, the faster of our two detectors, together with MapTracker, our cross-frame tracker that matches people by their position on the map. The alternative detector is SAM3, which has video memory; swap it in with `./lab people sam3`. Run one detector at a time.

Every person the robots see gets a hazard zone (OHZ): an ellipse on the ground for Go2, an ellipsoid for the Bee. Once the person's speed is known, the zone stretches along their walking direction. It expires after 2 s. The Bee also adds an extra 0.35 m of 3D inflation to clustered obstacles; isolated single voxels are not inflated.

The scene runs in CARLA, the simulator built on Unreal Engine 5.8 (UE 5.8). You watch everything in RViz, the 3D visualization window. One run session is one epoch, stored under `runtime/runs/`.

> **Terminology.** These docs call the ground robot Go2 and the drone the Bee. A few legacy identifiers carry older names: the team profile `go2_fighter.yaml`, the robot profile `fighter_uav.yaml`, and the UE actor class `robot.uav.fighter_drone`. All three still refer to the Bee.

## Quick start for teammates

Two independent tasks live in this repo. Pick your lane:

| You are building… | Start here |
|---|---|
| New UE scenes | [`ue/README.md`](ue/README.md): levels and project files, plus `profiles/scenes`, `profiles/teams`, `profiles/robots`, `profiles/sensors`. The CARLA towns themselves stay on the Linux box at `/data/sim/AirGroundLab_UE58` |
| A new people-OHZ shape | [`ohz/README.md`](ohz/README.md). The current ellipse lives in `perception/hazards.py`; tree pruning lives in `planning/` |
| Planning, RRT*, LRZ | `planning/` (kept whole; do not split) |

The local repair zone (LRZ) is the area around a robot where the planner reacts to map changes. This repo uses a 4 m radius.

**UE scene authoring (Windows).** You edit levels inside your own CARLA UE 5.8 project. This repo carries only the handmade levels, materials, and project config, roughly 400 KB in total. [`ue/README.md`](ue/README.md) has the import steps and the coordinate convention `x=carla.x, y=-carla.y, z=carla.z-0.22`. To see your scene drive robots, copy the `.umap` back to the Linux machine's UE project, describe it in `profiles/scenes/` and `profiles/teams/`, then run `./lab start --rviz --team <team>`.

**OHZ shape work.** The agreed replacement is a velocity-shaped ground shadow. A standing person becomes a disc for Go2 or a vertical cylinder for the Bee. A walking person becomes a ground rectangle that starts at the feet and runs along the velocity direction, with length scaled by speed. It must land as a second shape kind next to the existing ellipse, not as an edit to the ellipse. Conventions are in [`ohz/README.md`](ohz/README.md).

## Reproducing the stack

Everything first-party is in this repository: code, the Rust planner, the C++ map service, YOLO26 weights, profiles, and the handmade UE levels. Three heavy dependencies cannot travel through Git: the CARLA UE 5.8 project plus engine, a Python 3.10 environment, and ROS 2 Humble (the robot middleware). [`docs/SETUP_LINUX.md`](docs/SETUP_LINUX.md) walks through all three and ends at `./lab start --rviz`, with both robots moving on your own copy.

The fastest route is to restore the reference environment snapshot listed there. The full-from-upstream route is to build CARLA UE 5.8 yourself and import the levels from `ue/`.

| Task | On Windows | On the Linux host |
|---|---|---|
| Author/edit UE scenes | Yes: your own CARLA UE 5.8 project plus the `ue/Content` import (see [`ue/README.md`](ue/README.md)); hand the `.umap` back | Yes, plus running it with robots |
| Read/modify OHZ and planning code | Yes, editing only | Yes, plus `cargo build` / `cmake` / pytest |
| Run `./lab start --rviz` (full dual-robot stack) | Not natively. See [`docs/WINDOWS_TEAMMATE.md`](docs/WINDOWS_TEAMMATE.md): remote into a Linux host, or use the proven hybrid (CARLA on Windows plus the Python stack in WSL2, Windows Subsystem for Linux 2) | Yes, after [`docs/SETUP_LINUX.md`](docs/SETUP_LINUX.md) |
| See robots running | Through a Linux host: remote desktop, SSH, or the hybrid setup above | Yes: RViz on ROS domain 45 |

After cloning, build the two native components once on the Linux machine:

```bash
cmake -S mapping -B mapping/build -DCMAKE_BUILD_TYPE=Release && cmake --build mapping/build -j4
cargo build --release --offline --locked --manifest-path planning/Cargo.toml
```

Baselines and evidence:

- Archived, user-approved full configuration, source archive, and run evidence: [dual-robot + SAM3 stable config v2](docs/baselines/2026-09-29-dual-robot-sam3-stable-v2/parameters.md).
- Current version index: `docs/baselines/CURRENT.json`. The [early baseline](docs/baselines/2026-09-29-dual-robot-sam3-stable/parameters.md) is kept for comparison.
- Latest lidar repair and measurements: [sensor sampling-timing verification](docs/verification/2026-09-29-lidar-sensor-interval/README.md).
- Map rays and distant real-surface updates: [ray-evidence recovery](docs/verification/2026-09-29-ray-evidence-recovery/README.md).
- Earlier: [patrol-loop crash and tree-continuity fix](docs/verification/2026-09-29-patrol-index-fix/README.md).
- Performance follow-ups: [stepwise optimization plan](docs/plans/2026-09-29-performance-followup.md).

Stopped runs support explicit `--resume-map` together with `--resume-state`; the combination restores poses and patrol progress.

## Start, watch, stop (Linux host)

```bash
cd /data/robotics/air_ground_smart3d
./lab start --rviz
./lab status
./lab stop
# Explicitly resume an existing map from the same scene
# (occupancy is kept; old free space must be re-observed)
./lab start --rviz --resume-map /absolute/previous/run/shared/map.ot
```

This is the launch that actually runs here. The desktop shows RViz only. `./lab start --rviz` starts CARLA off-screen (`-game -RenderOffScreen`) on port 2400, ROS domain 45, team `go2_fighter`, and the people backend saved in `runtime/people_backend.json` (YOLO26). Do not add `--visible`. That opens the UE 5.8 editor window and spends extra GPU memory. Do not copy `ue/Content/AirGroundLab/Maps/AirGroundLab_Main.umap` over the live level as part of startup: that git copy contains an extra `Cube2` and blocks both spawn points. The running level is the one already in `/data/sim/AirGroundLab_UE58`. No web console is started. The launcher manages only the `air-ground-lab-*` services.

Startup order:

1. a standalone CARLA on port 2400
2. both robots and their sensors
3. one shared OcTree
4. the SMART/controller for each robot
5. registered RGB-D sampling
6. one SAM3 model with two independent video sessions (or YOLO26)
7. ROS domain 45 and RViz

In the ROS graph, the cyan tree is Go2's 2D tree and the orange tree is the Bee's true 3D tree. Gray edges are branches that are actually disconnected. Tree edges and paths come straight from the Rust parent-child relations; no edge is invented. The translucent body boxes in RViz are labeled collision envelopes. The actual Go2 and Bee bodies live in UE.

## Repository layout

| Directory | Purpose |
|---|---|
| `profiles/robots` | Motion dimension, body clearance, speeds, execution limits |
| `profiles/appearances` | Meshes for Go2 and the Bee (`robot_bee`), plus `fighter_drone` / `cute_robot_3` / slime / AirSim fallback looks |
| `profiles/sensors` | Extrinsics, calibration, pairing and fusion sampling parameters |
| `profiles/scenes`, `profiles/teams` | Scene identity, ports, map bounds, instances, per-robot patrol points |
| `sim_io` | Pose-at-capture bridge, real sensors, shared fusion, per-robot task lifecycle |
| `mapping` | C++ OctoMap service, revision deltas, full `.ot` snapshots |
| `planning` | The fixed-provenance LINKS Rust core, N=2/N=3 (2D/3D) adaptation, true tree output |
| `execution` | Continuous body sweep, near-field known-free check, path following |
| `perception` | Registered RGB-D, people masks/IDs (SAM3 or YOLO26, one at a time), depth projection, shared people records, transient OHZ |
| `observability` | Read-only ROS/RViz trees, paths, voxels, TF (pose transforms) and cameras |
| `simulation` | Standalone UE launch, map building, the Bee's C++ source mirror, interface notes |
| `docs/plans`, `docs/verification` | Full plans, actual test evidence, screenshots, limits |
| `runtime/runs/<epoch>` | Per-run config snapshot, map, logs, latest images, state; not in Git |

Paths outside the repo:

- Simulation project: `/data/sim/AirGroundLab_UE58/CarlaUnreal/CarlaUnreal.uproject`.
- Read-only shared engine: `/data/sim/CARLA_UE58_Research/UnrealEngine5_carla58`.
- Read-only shared CARLA Python env: `/data/robotics/ue58_ohz_test_go2/.venv/`, pinned to the 0.10.0 API that matches UE 5.8. Do not swap in the old CARLA wheel.

## Isolation from the older single-robot project

| Item | Old project: single Go2, multi-person empty field | New project: Go2 + Bee, shared OctoMap |
|---|---|---|
| Navigation project | `/data/robotics/ue58_ohz_test_go2` | `/data/robotics/air_ground_smart3d` |
| UE project | `/data/sim/CARLA_UE58_Research/Carla-ue58/Unreal/CarlaUnreal` | `/data/sim/AirGroundLab_UE58/CarlaUnreal` |
| Level | `/Game/OHZTest/Maps/OHZTest_Main` | `/Game/AirGroundLab/Maps/AirGroundLab_Main` |
| CARLA / ROS domain | `2200 / 43` | `2400 / 45` |
| Console | `20280` | No new console this phase; use RViz |
| People, sensors, routes | Old project `configs` | New project `profiles` plus per-run snapshots |
| Map state | Old project's own run state | New project `runtime/runs/<epoch>/shared` |

The engine binaries and the Python dependency env are shared between the two projects; experiment configs and map data are not. The original SAM3/OHZ stack stays in the old project. This dual-robot version has its own migrated SAM3 video people tracker: it reuses one weight instance, gives each camera an independent video session, and then associates shared person IDs by space-time compatibility.

The two real lidars publish at `/air_ground/go2_01/lidar/points` and `/air_ground/uav_01/lidar/points`. The new launcher manages the lidar preview service uniformly.

## Runtime contract and current boundaries

### Runs and maps

The original Town10, Factory, and single-Go2 empty-field projects, plus the SSD (solid-state drive) backups, have not been replaced. The new map has its own identity and run epochs. On exit, the current `.ot` and metadata are saved; the next start begins from fresh observations, and old scene maps are never auto-imported. `runtime/latest_run.json` points at the current or most recent run, and the state carries timestamps plus a running flag.

### How the robots move

The Bee flies kinematically: true XYZ and yaw, UE entity sweeps, and actual pose feedback. It does not simulate rotor dynamics or PX4 (open-source flight-control firmware). Go2 navigates at a fixed height on flat ground; stairs and stacked floors are unsupported.

### Where obstacles come from

Partner positions arrive by cooperative pose messaging and act as a transient collision constraint. People obstacles come only from lidar and depth; pedestrian ground truth is never fed to planning.

### People in the scene

The six pedestrians play two roles at once: real sensor-geometry obstacles and SAM3 people subjects. A person's 3D box covers only the partially visible surface and comes from the mask plus registered depth (depth aligned to the color image). Estimating XY speed takes at least four consecutive reliable observations of the same source. Z speed is not estimated, and no appearance-based re-identification (ReID) is guaranteed.

People OHZ are transient constraints: a 2D ellipse for Go2, a 3D ellipsoid for the Bee. RViz displays the same OHZ the Rust planner consumes. Do not read the displayed box as a fully measured body volume.

The planning core comes from a previously repaired native fork and includes documented multi-dimensional clearing and re-rooting adaptations; it is not claimed to be a verbatim unmodified upstream. The original RRT* sources and hashes are in `planning/CORE_PROVENANCE.json`.

### Sensing and map updates

Besides the front RGB-D camera and lidar, the Bee covers the free space around its body with six low-resolution near-range depth views, so unknown side and rear regions are not optimistically released from a front view alone. That raw data stays in bounded queues, like the front view, and no extra large vision model is loaded.

Front depth fuses out to 6 m, and the all-direction blind-fill fuses to 3 m. Both robots' lidar follows the original single-Go2 costmap config: obstacles are recorded within 9 m, and real measurement rays clear up to 40 m (sensor range 45 m). Distant surfaces that are actually hit are not treated as free space; no echo produces no clearing ray. Sources and expiry deadlines are managed separately.

Known-free space is kept until new occupancy evidence appears. Before moving, execution checks this robot's recently fused lidar, current occupancy, partner positions, and UE entity collisions. Unknown regions are never executed into blindly.

The lidar keeps every distinct real measurement ray and merges only identical endpoints; it does not pick one representative ray per end voxel. Distant real-surface protection is deduplicated per voxel, separately. The UE rotating lidar computes point counts and sweep angles from the actual sensor sampling interval; the first frame and scheduler jitter can still produce partial scan packets. `mapping_sampling` in YAML is a historical description that the code does not read; the implementation above is authoritative.

### Rates: targets versus reality

Scheduling targets: lidar 5 Hz per robot, front depth 2 Hz per robot, and about 0.67 Hz for each of the six blind-fill views. Measure the real rates through `sensor_stats` and `fusion_metrics` in the current state. Display publishes separately: map at 5 Hz, trees at 2 Hz, poses and paths at 10 Hz. Display rate is never passed off as fusion rate.

Synchronous UE motion receipts are awaited by a dedicated `motion_rpc` subprocess, which keeps CARLA's Python GIL (global interpreter lock) from blocking sensor handling. Collision receipts still return to the controller, and the shared CARLA install is unmodified.

### The bounded fusion pipeline

The next frame's native OctoMap request may run in parallel with the previous frame's Python cache application, but at most one cache-application task is in flight. The next apply waits for the previous one, so stale images never queue up. Each frame is bound to its own capture time, pose, and identity, and caches apply in version order before execution freshness updates. If the planner version temporarily leads the execution cache, the controller keeps waiting for the matching version. On stop or failure, already-submitted apply tasks are completed.

Three metrics track this. `apply_queue_s` isolates apply waiting, `cache_apply_s` records the actual cache application, and `arrival_to_apply_wall_s` preserves the end-to-end sensor-arrival-to-applied latency.

Current real-time and offline/online acceptance are unified in the [OctoMap real-time verification](docs/verification/2026-09-29-octomap-realtime.md), which reports scheduling targets, display rates, and actual sensor fusion rates separately.

## Rebuild and verification

```bash
cmake -S mapping -B mapping/build -DCMAKE_BUILD_TYPE=Release
cmake --build mapping/build -j4
cargo build --release --offline --locked --manifest-path planning/Cargo.toml
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  /data/robotics/ue58_ohz_test_go2/.venv/bin/python -m pytest -q tests -p no:cacheprovider
```

The nine modified UE sources, their original and new hashes, and the Bee run tests are documented in `simulation/UAV_SIMULATION.md`. Commercial assets stay in the local UE project and never enter source Git. Source tests, entity tests, and continuous joint-run evidence are recorded separately. Offline pass counts must not stand in for actual run behavior.

## People backend, live map, and repair history

Exactly one people-perception backend runs at a time:

```bash
./lab people            # show the current backend
./lab people yolo26     # switch while running: stops SAM3, starts YOLO26
./lab people sam3       # switch back to SAM3
./lab start --rviz --people yolo26   # choose at startup
```

The choice is stored in `runtime/people_backend.json`, reused by the next `./lab start`, and defaults to SAM3. Both backends only find people and give per-camera track IDs. Bounding boxes, speed, heading, and the OHZ ellipses all come from the same depth projection, speed fitting, and `shared/people.json`, so the runtime and RViz are source-agnostic.

YOLO26 runs `yolo26m-seg` (person class only; the weight hash is in `provenance/yolo26_people.json`) and processes every RGB-D frame. It measures about 2.7 Hz per camera, capped by the 3 Hz RGB-D capture. MapTracker associates IDs by position in map coordinates: a person who returns within 20 s of leaving view, with similar clothing color, regains the old ID.

Benchmark from 2026-10-04, against CARLA pedestrian ground truth on a 100 s recording of this scene:

| Backend | Detection | Precision | Time per frame | Median speed error |
|---|---|---|---|---|
| YOLO26 | 95.3% | 98.4% | ~11 ms | 0.02 m/s |
| SAM3 | 97.8% | 99.5% | ~300 ms | 0.02 m/s |

Details are in `perception/PEOPLE_SERVICE.md`.

Startup brings up RGB-D capture and the chosen people backend by default. `--no-perception` starts a pure-geometry baseline from stop and does not shut down perception services that are already running. SAM3 is capped at 2 frames/s total, fairly scheduled across both cameras, which measures about 1 frame/s per camera. The newest frame replaces pending ones, so images never pile up.

Person geometry has a time-to-live (TTL) of 2 s, with at most 1 s of extrapolation. Speed fitting keeps at most 6 frames / 4.5 s of history and needs at least four consecutive reliable measurements; it does not extend a person's occupancy validity. Without a reliable speed there is no velocity arrow; no fake direction is ever drawn.

RViz shows both lidars independently. The two people panels show the current backend's masks and IDs on `/air_ground/<robot>/people/image`. Person wireframes are the 3D extent of the depth-visible surface. The green and orange OHZ come from the two planners. The LRZ is taken directly from the native planner; this experiment uses a 4 m radius, not the 9 m of the original single-Go2 version.

Fixes in this round:

- Empty observation windows after a map resume no longer break native parsing or trigger infinite full-map re-reads.
- Map fusion rejects old frames overwriting newer evidence; hits win at the same instant.
- The current simulator profile uses single-hit / free-ray probability parameters that flip occupancy.
- Regions not re-crossed by a free ray are never cleared into free space.

The probability setting is for the current simulation only; it is not a validated real-sensor parameter.

An earlier resume-repair phase ran a 40 s check: map display snapshots lagged the sensor clock by 0.288 s on average and 0.625 s at most, and both robots completed two new patrol goals. That is the [historical resume verification](docs/verification/2026-09-29-map-people-recovery.md), not current end-to-end people-clearing latency or latest-config performance. Later real-time changes and measurements are governed by the [OctoMap real-time verification](docs/verification/2026-09-29-octomap-realtime.md).
