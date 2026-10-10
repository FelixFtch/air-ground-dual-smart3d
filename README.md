# Go2 + Fighter Drone / shared-map SMART3D

Dual-robot empty scene: Go2 runs a 2D tree, the Fighter Drone runs a 3D tree, both share one OctoMap. People perception is YOLO26 + MapTracker; SAM3 can still be swapped in with `./lab people sam3`. Bee applies an extra 0.35 m 3D inflation to clustered obstacles only — isolated single voxels are not inflated.

## Quick start for teammates

Two independent tasks live in this repo. Pick your lane:

| You are building… | Start here |
|---|---|
| New UE scenes | [`ue/README.md`](ue/README.md) — levels and project files, plus `profiles/scenes`, `profiles/teams`, `profiles/robots`, `profiles/sensors`. The CARLA towns themselves stay on the Linux box at `/data/sim/AirGroundLab_UE58` |
| A new people-OHZ shape | [`ohz/README.md`](ohz/README.md) — the current ellipse lives in `perception/hazards.py`, tree pruning in `planning/` |
| Planning, RRT*, LRZ | `planning/` — kept whole, do not split |

**UE scene authoring (Windows):** you edit levels inside your own CARLA UE 5.8 project; this repo carries only the ~400 KB handmade levels, materials, and project config (see `ue/README.md` for import steps and the coordinate convention `x=carla.x, y=-carla.y, z=carla.z-0.22`). To see your scene drive robots, copy the `.umap` back to the Linux machine's UE project and describe it in `profiles/scenes/` + `profiles/teams/`, then `./lab start --rviz --team <team>`.

**OHZ shape work:** the agreed replacement is the velocity-shaped ground shadow — standing person = disc (Go2) / vertical cylinder (Bee); walking person = ground rectangle from the feet along the velocity direction, length scaled by speed. It must land as a second shape kind next to the existing ellipse, not as an edit to it. Conventions in [`ohz/README.md`](ohz/README.md).

## Reproducing the stack

Everything first-party is in this repository: code, Rust planner, C++ map service, YOLO26 weights, profiles, and the handmade UE levels. Three heavy dependencies cannot travel through Git — the CARLA UE 5.8 project + engine, a Python 3.10 env, and ROS 2 Humble — and [`docs/SETUP_LINUX.md`](docs/SETUP_LINUX.md) walks through all of them, ending at `./lab start --rviz` with both robots moving on your own copy. Fastest route: restore the reference environment snapshot listed there; fully-from-upstream route: build CARLA UE 5.8 yourself and import the levels from `ue/`.

| Task | On Windows | On the Linux host |
|---|---|---|
| Author/edit UE scenes | Yes — your own CARLA UE 5.8 project + `ue/Content` import (see [`ue/README.md`](ue/README.md)); hand the `.umap` back | Yes, plus run it with robots |
| Read/modify OHZ and planning code | Yes (editing only) | Yes, plus `cargo build` / `cmake` / pytest |
| Run `./lab start --rviz` (full dual-robot stack) | No — needs systemd user units, ROS 2 Humble, and the Linux CARLA/UE install | Yes, after [`docs/SETUP_LINUX.md`](docs/SETUP_LINUX.md) |
| See robots running | Via the Linux host (SSH / remote desktop / a machine set up with the guide) | Yes — RViz on ROS domain 45 |

After cloning, build the two native components once on the Linux machine:

```bash
cmake -S mapping -B mapping/build -DCMAKE_BUILD_TYPE=Release && cmake --build mapping/build -j4
cargo build --release --offline --locked --manifest-path planning/Cargo.toml
```

The archived user-approved full configuration, source archive, and run evidence: [dual-robot + SAM3 stable config v2](docs/baselines/2026-09-29-dual-robot-sam3-stable-v2/parameters.md). The current version index is `docs/baselines/CURRENT.json`; the [early baseline](docs/baselines/2026-09-29-dual-robot-sam3-stable/parameters.md) is kept for comparison.

Latest lidar repair and measurements: [sensor sampling-timing verification](docs/verification/2026-09-29-lidar-sensor-interval/README.md); map rays and distant real-surface updates: [ray-evidence recovery](docs/verification/2026-09-29-ray-evidence-recovery/README.md). Earlier: [patrol-loop crash and tree-continuity fix](docs/verification/2026-09-29-patrol-index-fix/README.md). Stopped runs support explicit `--resume-map` together with `--resume-state` to restore poses and patrol progress. Performance follow-ups: [stepwise optimization plan](docs/plans/2026-09-29-performance-followup.md).

## Start, watch, stop (Linux host)

```bash
cd /data/robotics/air_ground_smart3d
./lab start --rviz
./lab status
./lab stop
# Explicitly resume an existing map from the same scene (occupancy kept; old free space must be re-observed)
./lab start --rviz --resume-map /absolute/previous/run/shared/map.ot
```

UE runs off-screen by default and RViz shows on the desktop. To get a UE game window, stop this experiment first and use `./lab start --rviz --visible`. No new web console is started. Only the `air-ground-lab-*` services are managed; other projects and applications are never touched.

The launcher brings up, in order: a standalone CARLA on port 2400 → both robots and sensors → one shared OcTree → per-robot SMART/controller → registered RGB-D sampling → one SAM3 model with two independent video sessions (or YOLO26) → ROS domain 45 / RViz. In the ROS graph the cyan tree is Go2's 2D tree and the orange one is the UAV's true 3D tree; gray edges are branches actually disconnected. Tree edges and paths come straight from the Rust parent-child relations — no edges are invented. The translucent body boxes in RViz are labeled collision envelopes; the actual Go2/Fighter looks live in UE.

## Layout

| Directory | Purpose |
|---|---|
| `profiles/robots` | Motion dimension, body clearance, speeds, execution limits |
| `profiles/appearances` | Go2, Fighter, Bee, Cute3, small Slime, AirSim fallback looks |
| `profiles/sensors` | Extrinsics, calibration, pairing and fusion sampling parameters |
| `profiles/scenes`, `profiles/teams` | Scene identity, ports, map bounds, instances and per-robot patrol points |
| `sim_io` | Pose-at-capture bridge, real sensors, shared fusion, per-robot task lifecycle |
| `mapping` | C++ OctoMap service, revision deltas, full `.ot` snapshots |
| `planning` | The fixed-provenance LINKS Rust core, N=2/N=3 adaptation, true tree output |
| `execution` | Continuous body sweep, near-field known-free check, path following |
| `perception` | Registered RGB-D, people masks/IDs (SAM3 or YOLO26, one at a time), depth projection, shared people records and transient OHZ |
| `observability` | Read-only ROS/RViz trees, paths, voxels, TF and cameras |
| `simulation` | Standalone UE launch, map building, UAV C++ source mirror and interface notes |
| `docs/plans`, `docs/verification` | Full plans, actual test evidence, screenshots and limits |
| `runtime/runs/<epoch>` | Per-run config snapshot, map, logs, latest images and state; not in Git |

Simulation project: `/data/sim/AirGroundLab_UE58/CarlaUnreal/CarlaUnreal.uproject`.
Read-only shared engine: `/data/sim/CARLA_UE58_Research/UnrealEngine5_carla58`.
Read-only shared CARLA Python env: `/data/robotics/ue58_ohz_test_go2/.venv/`, pinned to the 0.10.0 API matching UE 5.8; do not swap in the old CARLA wheel.

## Isolation between the old single-robot project and this dual-robot one

| Item | Old single Go2, multi-person empty field | New Go2 + UAV shared OctoMap |
|---|---|---|
| Navigation project | `/data/robotics/ue58_ohz_test_go2` | `/data/robotics/air_ground_smart3d` |
| UE project | `/data/sim/CARLA_UE58_Research/Carla-ue58/Unreal/CarlaUnreal` | `/data/sim/AirGroundLab_UE58/CarlaUnreal` |
| Level | `/Game/OHZTest/Maps/OHZTest_Main` | `/Game/AirGroundLab/Maps/AirGroundLab_Main` |
| CARLA / ROS domain | `2200 / 43` | `2400 / 45` |
| Console | `20280` | No new console this phase; use RViz |
| People, sensors, routes | Old project `configs` | New project `profiles` plus per-run snapshots |
| Map state | Old project's own run state | New project `runtime/runs/<epoch>/shared` |

The engine binaries and the existing Python dependency env are shared; experiment configs and map data are not. The original SAM3/OHZ stays in the old project. The dual-robot version has its own migrated SAM3 video people tracker reusing one weight instance; the two cameras hold independent video sessions, then associate shared person IDs by space-time compatibility. The two real LiDARs publish at `/air_ground/go2_01/lidar/points` and `/air_ground/uav_01/lidar/points`; the new launcher manages the lidar preview service uniformly.

## Runtime contract and current boundaries

The original Town10, Factory, single-Go2 empty-field projects and the SSD backups have not been replaced. The new map has its own identity and run epochs; on exit the current `.ot` and metadata are saved, and the next start begins from fresh observations — old scene maps are never auto-imported. `runtime/latest_run.json` points at the current/most recent run; state carries timestamps and a running flag.

The UAV flies kinematically with true XYZ/yaw, UE entity sweeps, and actual pose feedback — not rotor dynamics/PX4. Go2 navigates at fixed height on flat ground; stairs and stacked floors are unsupported. Partner positions come from cooperative pose messaging as a transient collision constraint; people obstacles come only from LiDAR/depth — pedestrian ground truth is never fed to planning.

In this phase the six pedestrians act simultaneously as real sensor-geometry obstacles and SAM3 people subjects. A person's partially-visible-surface 3D box comes from mask + registered depth; XY speed needs at least four consecutive reliable observations of the same source, Z speed is not estimated, and no appearance-ReID is guaranteed. People OHZ are transient constraints: Go2 uses a 2D ellipse, the UAV a 3D ellipsoid; RViz displays the same OHZ the Rust planner consumes. The displayed box must not be read as a fully measured body volume. The planning core comes from a previously repaired native fork and includes documented multi-dimensional clearing/re-rooting adaptations; it is not claimed to be a verbatim unmodified upstream. Original RRT* and source hashes: `planning/CORE_PROVENANCE.json`.

Beyond the front RGB-D and LiDAR, the UAV now covers free space around the body with six low-resolution near-range depth views, so unknown side/rear regions are not optimistically released from a front view alone. Their raw data, like the front view, stays in bounded queues; no extra large vision model is loaded. Front depth fuses to 6 m, all-direction blind-fill to 3 m; both robots' LiDAR follows the original single-Go2 costmap config — obstacles recorded within 9 m, real measurement rays clearing up to 40 m (sensor range 45 m). Distant actually-hit surfaces are not treated as free space; no echo produces no clearing ray. Sources and expiry deadlines are managed separately. Known-free is kept until new occupancy evidence appears; execution still checks this robot's recently fused lidar, current occupancy, partner positions, and UE entity collisions — unknown regions are never executed into blindly.

The lidar keeps every distinct real measurement ray and only merges identical endpoints; it does not pick a single representative ray per end voxel. Distant real-surface protection is deduplicated per voxel separately. The UE rotating lidar computes point counts and sweep angles from the actual sensor sampling interval; the first frame and scheduler jitter can still produce partial scan packets. `mapping_sampling` in YAML is a historical description the code does not read — the implementation above is authoritative. Target fusion rates: LiDAR 5 Hz per robot, front depth 2 Hz per robot, six blind-fill views about 0.67 Hz each; these are scheduling targets — measure reality via `sensor_stats` and `fusion_metrics` in the current state. Map display publishes at 5 Hz, trees at 2 Hz, poses/paths at 10 Hz, all separately; display rate is never passed off as fusion rate. Synchronous UE motion receipts are awaited by a dedicated `motion_rpc` subprocess to keep CARLA's Python GIL from blocking sensor handling; collision receipts still return to the controller, and the shared CARLA install is unmodified.

Fusion also runs a bounded pipeline: the next frame's native OctoMap request may proceed in parallel with the previous frame's Python cache application, but at most one cache-application task is in flight; the next apply must wait for the previous one — stale images never queue up. Each frame is bound to its own capture time, pose, and identity; caches apply in version order before execution freshness updates. If the planner version temporarily leads the execution cache, the controller keeps waiting for the matching version. On stop or failure, already-submitted apply tasks are completed. `apply_queue_s` isolates apply waiting, `cache_apply_s` records the actual cache application, and `arrival_to_apply_wall_s` preserves the end-to-end sensor-arrival-to-applied latency.

Current real-time and offline/online acceptance are unified in [OctoMap real-time verification](docs/verification/2026-09-29-octomap-realtime.md); scheduling targets, display rates, and actual sensor fusion rates are reported separately.

## Rebuild and verification

```bash
cmake -S mapping -B mapping/build -DCMAKE_BUILD_TYPE=Release
cmake --build mapping/build -j4
cargo build --release --offline --locked --manifest-path planning/Cargo.toml
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  /data/robotics/ue58_ohz_test_go2/.venv/bin/python -m pytest -q tests -p no:cacheprovider
```

The nine modified UE sources, original/new hashes, and UAV run tests are in `simulation/UAV_SIMULATION.md`. Commercial assets stay only in the local UE project and never enter source Git. Source tests, entity tests, and continuous joint-run evidence are recorded separately; offline pass counts must not stand in for actual run behavior.

## People, live map, and repair history

Exactly one people-perception backend runs at a time:

```bash
./lab people            # show the current backend
./lab people yolo26     # switch while running: stops SAM3, starts YOLO26
./lab people sam3       # switch back to SAM3
./lab start --rviz --people yolo26   # choose at startup
```

The choice is stored in `runtime/people_backend.json`, reused by the next `./lab start`, defaulting to SAM3. Both backends only find people and give per-camera track IDs; bounding boxes, speed, heading, and the OHZ ellipses all come from the same depth projection, speed fitting, and `shared/people.json` — the runtime and RViz are source-agnostic. YOLO26 uses `yolo26m-seg` (person class only; weight hash in `provenance/yolo26_people.json`) and processes every RGB-D frame, measured ~2.7 Hz per camera, capped by the 3 Hz RGB-D capture. IDs are associated by position in map coordinates; a person returning within 20 s of leaving view with similar clothing color regains the old ID. On 2026-10-04, against CARLA pedestrian ground truth on a 100 s recording of this scene: YOLO26 detection 95.3%, precision 98.4%; SAM3 97.8%, 99.5%; ~11 ms vs ~300 ms per frame; median speed error 0.02 m/s for both. Details in `perception/PEOPLE_SERVICE.md`.

Startup brings up RGB-D capture and the chosen people backend by default; `--no-perception` starts a pure-geometry baseline from stop without shutting down already-running perception services. SAM3 is capped at 2 frames/s total with fair scheduling across both cameras, measured ~1 frame/s per camera; the newest frame replaces pending ones, so images never pile up. Person geometry TTL is 2 s with at most 1 s of extrapolation. Speed fitting keeps at most 6 frames / 4.5 s of history and needs at least four consecutive reliable measurements; this does not extend a person's occupancy validity. Without a reliable speed there is no velocity arrow — no fake direction is ever drawn.

RViz shows both LiDARs independently; the two people panels show the current backend's masks and IDs on `/air_ground/<robot>/people/image`. Person wireframes are the depth-visible surface's 3D extent; green/orange OHZ come from the two planners. LRZ is taken directly from the native planner; this experiment uses a 4 m radius (not the 9 m of the original single-Go2 version).

Fixes in this round: empty observation windows after map resume no longer break native parsing / trigger infinite full-map re-reads; map fusion rejects old frames overwriting newer evidence, hits win at the same instant; the current simulator profile uses single-hit / free-ray probability parameters that flip occupancy. Regions not re-crossed by a free ray are never cleared into free space. This probability setting is for the current simulation and is not a validated real-sensor parameter.

An earlier resume-repair phase ran a 40 s check: map display snapshots lagged the sensor clock by 0.288 s on average, 0.625 s max, and both robots completed two new patrol goals. That is the [historical resume verification](docs/verification/2026-09-29-map-people-recovery.md), not current end-to-end people-clearing latency or latest-config performance; later real-time changes and measurements are governed by [OctoMap real-time verification](docs/verification/2026-09-29-octomap-realtime.md).
