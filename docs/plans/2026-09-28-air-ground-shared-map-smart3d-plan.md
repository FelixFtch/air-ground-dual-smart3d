# Go2 + UAV: Shared 3D Map and Multi-Robot SMART3D Implementation Plan

Updated: 2026-09-29. Status: **Group 1 (Go2 + Fighter Drone + 6 pedestrians) is running verification in a standalone empty scene; 2D/true-3D SMART and the shared OctoMap are in place. Later phases are still accepted item by item as described below — Group 1's success must not be treated as completion of the whole plan.**

The new project root is `/data/robotics/air_ground_smart3d/`; the simulation project is kept separately at `/data/sim/AirGroundLab_UE58/CarlaUnreal/`. Source code, profiles, launchers, and run verification already exist; Town10, Factory, and the original single-Go2 empty scene are preserved. Only the compiled engine and the read-only CARLA Python environment are shared.

## 0. September 29 Implementation Update

| Item | Current state and explicit boundaries |
|---|---|
| Appearance choice | Go2 + Fighter Drone is the primary choice; Robot Bee, Cute Robot 3, and Slime Creator moved into the appearance catalog. Slime target height 35 cm. AirSim QuadRotor1 kept as a model candidate; the AirSim planner is not installed |
| Identity isolation | robot_id, role, TF, sensors, planning processes, and logs are independent; appearance and mobility are separated. In Group 1, go2_01 is XY/yaw, uav_01 is XYZ/yaw |
| UE carrier | Fighter genuinely has XYZ motion, full-body sweep, velocity/acceleration/bounds checks, and actual pose feedback; 11 independent run tests passed. Kinematic flight — this does not represent PX4 physical flight control |
| Native SMART | Fixed-origin LINKS Lab Rust RRT*/SMART. Go2 is SMART<f32,2>, the UAV is SMART<f32,3>; the map-increment, 3D clearing-recovery, and retarget-reroot adaptations are recorded in planning/CORE_PROVENANCE.json and the README. Not a completely unmodified upstream |
| Shared map | Real C++ OcTree probabilistic occupancy; per-robot capture sensor poses, multi-source rays, occupied/free/unknown, incremental revision, .ot snapshots; transient peer-position constraints kept separate from the long-term map |
| Sensor calibration | Real floor worldZ=.20124996, map origin Z=.22; Go2 mapZ=.33 keeps worldZ=.55. Depth uses pixel-center u+.5/v+.5 and forward-X depth; ground mislabeling is not worked around by ignoring z=0 obstacles |
| UAV blind spots | Beyond forward RGB-D, six 96×96 near-field wide-angle depth views cover all directions, fused only within 3 m; forward RGB and depth are paired per sim frame, and the fuser processes only bounded latest queues |
| Actual navigation | Go2 multi-goal patrol and a UAV actually climbing over a wall and reaching its first goal have occurred; later long runs/cold starts are per the docs/verification reports |
| RViz | Cyan ground 2D tree, orange aerial 3D tree; directly uses the Rust-exported XYZ and parent edges; shared voxels, paths, both robots' TF, RGB preview. The envelope box is explicitly labeled a collision body, not disguised as a UE model |
| People | 6 real collidable pedestrians on long routes at different speeds; this milestone updates the map with real sensor geometry and does not read pedestrian ground truth for perception. The SAM3 multi-person/multi-camera OHZ port is not yet implemented and is not claimed as done |
| Still pending | Switchable Actor registration for the remaining roles, SAM3 shared-model scheduling, predictive OHZ, multi-robot spatiotemporal collision-avoidance strategy, 3/4-robot stress tests, Factory multi-robot port, ground floors/stairs |

The minimal startup sequence is packaged: `./lab start --rviz` → CARLA 2400 → both robots' sensors and shared map → independent 2D/3D workers and execution → ROS45/RViz. Stop with `./lab stop`; do not batch-kill by process name. By default no new web console is created.

## 1. Final Deliverable and Phase 1 Scope

The final goal is multiple Go2 robots and multiple UAVs running in the same CARLA UE5.8 world: each carrying LiDAR and optional RGB-D, contributing observations to the same 3D map; each maintaining its own SMART tree, tasks, control authority, and run state. Go2 uses a 2D traversability map generated from the ground and body clearance; the UAV uses true-3D SMART with XYZ state. Pedestrian prediction and other robots' motion constraints stay independent of the long-term environment map.

Full acceptance for Group 1 is **1 Go2 + 1 UAV**; then extend to **2 Go2 + 1 UAV**; **2 Go2 + 2 UAV** is the capacity stress tier. This is a test ladder, not a confirmed maximum load for the 4090.

Phase 1 uses a standalone empty scene with ambient light, featuring ground, low walls, beams, narrow passages, and movable boxes. The initial geometry-navigation validation now includes 6 collidable pedestrians; SAM3 shared inference is a separate later acceptance. The factory serves as the second scene configuration, reusing the already-accepted multi-robot chain.

The UAV first uses smooth kinematic motion with full XYZ/yaw pose feedback, velocity/acceleration limits, and solid sweep collision. Phase 1 does not cover PX4, real rotor dynamics, real flight stabilizers, or physical-vehicle localization verification. Only the AirSim model is reused; the full AirSim simulation plugin is not installed and the CARLA GameMode is not replaced.

Go2 in Phase 1 runs on the same continuous ground level; multi-level overlapping floors, stairs, and ground-level switching are listed separately as follow-up tasks and must not be implied as capabilities an ordinary 2D costmap already supports.

Initial geometry experiments use an independent copy of the existing 80×80 m empty scene; Go2 keeps a 60×60 m navigation range at first, and the UAV first validates in a controlled area of 40×40 m at center heights 0.8–6 m before expanding to larger airspace in the scene. Heights are all relative to the measured ground datum; flight ceilings and permitted altitudes are given explicitly by the scene profile. The 3D initial sample count is determined by connectivity and runtime — the 2D Factory's 210,000 initializations are not copied over directly.

## 2. Pre-Implementation Baseline as of September 28 (historical cross-check; current progress is per Section 0 and the acceptance reports)

| Item | Confirmed current state | What is still missing |
|---|---|---|
| Original Go2 Factory / empty scene | Prior run acceptance, independent directories, sensors, SMART, executor, and SAM3 people OHZ exist | Keep the versions and build isolated multi-robot copies; not relaunched this round |
| Standalone source repo | `/data/robotics/sam3-rgbd-smart3d-navigation`, HEAD `815eb64befb080c0b3d7a8aaffed2c4b472787a1`, working tree clean this round | The extracted standalone launcher still lacks one full simulation cold-start acceptance |
| Offline verification | Python 436 + 180 subtests passed, 2 skipped; Rust 94 + 81 passed | Not equivalent to a new multi-robot run acceptance or third-party certification |
| Go2 current planning dimension | The worker is explicitly `SMART<f32,2>` | Keep its 2D use; add a 3D worker separately |
| True-3D SMART | The local machine has `SMART<f32,3>` and a real XYZ example | The example's LiDAR is synthetic rays against ground-truth boxes and is not yet connected to real PointCloud2 maps |
| USB UAV | Has Town10 kinematic sensors, ROS2, LiDAR, tri-state voxels, executor, and historical run evidence | The old bridge's timestamps and clear-ray origins need fixing; the old carrier has no independent visible body |
| New UAV branch | Previously verified that GitHub `e99f32a` is 13 commits ahead of the USB `3fab808` | Select only fixes related to isolation, pose, and lifecycle; independent chains like EGO must not mix into the SMART main chain |
| UAV model | Both FBX files of the AirSim QuadRotor1, 5 textures, and the UE original assets are saved; UE5.8 import-export succeeded, 31 files verified | Rigging, material appearance, scale, collision body, and CARLA Actor registration pending acceptance |
| OctoMap | The local `/opt/ros/humble/include/octomap` has the library headers; upstream supports occupied/free/unknown and multi-source updates | The new shared service, version protocol, bridging, and throughput are not implemented; build dependencies still need P0 confirmation |
| SAM3 multi-camera | Each service currently loads one BF16 CUDA model | Shared weights, independent video sessions, and fair scheduling are not implemented |
| Multi-robot profile | A reusable USB contract and a unified project blueprint exist | The current executor still has the hardcoded `car1` identity, topics, and paths; copying and renaming cannot be called multi-robot |

Existing data reference: a three-minute window on 2026-09-27 in the empty scene with three pedestrians and the Go2 stationary — SAM3 actual 3.41 Hz, VRAM 9,319–9,346 MiB, prediction-time availability about 97.25%–99.89%. This is one specified single-robot window, not a conclusion about the complex factory, multiple robots, or long runs.

2026-09-28 single-robot baseline configuration reference: RGB and registered depth 848×480, configured 10 Hz; ordinary LiDAR 128 channels, 240k points/s, 10 Hz, 80 m range. The original Lab costmap was 60×60 m at 0.25 m; Factory 300×300 m at 0.25 m. Configured rates do not represent actual achieved rates, and LiDAR range does not equal the actual ingest/clearing range.

One hardware query before this round: 4090 24 GiB, the whole card at about 7.6 GiB used, another model process active, three compute samples at 100%/56%/76%; about 42 GiB system memory available. This is not a navigation baseline — background load must be re-recorded before capacity testing begins, and that process must not be stopped automatically.

## 3. File and Version Boundaries to Avoid Contaminating Old Projects

The following stable roots are preserved and are not write targets for the new multi-robot programs:

- Town10: `/home/felix/Documents/ChatGPT/ue5robot/`
- Factory: `/data/robotics/ue58_factory_go2/`
- Empty scene: `/data/robotics/ue58_ohz_test_go2/`
- Published extracted repo: `/data/robotics/sam3-rgbd-smart3d-navigation/`
- USB and SSD raw backups: source and recovery material only.

The new project uses the following layout (the README is authoritative for concrete files), retaining the unified system's layered concept without importing its not-yet-implemented framework wholesale:

```text
/data/robotics/air_ground_smart3d/
  README.md
  docs/plans/                   this plan, change decisions, and acceptance notes
  provenance/                   source commits, file checksums, dependency locks, and derivation notes
  contracts/                    observations, map versions, candidate paths, and motion receipts
  profiles/
    robots/go2.yaml             ground motion, body, executor capabilities
    robots/quadrotor.yaml       XYZ motion, body, flight bounds
    sensors/                    camera/LiDAR parameters and extrinsics
    scenes/                     map identity and bounds for the empty scene and Factory
    teams/                      1+1, 2+1, 2+2 instance combinations
  sim_io/                       CARLA clock, spawn, sensors, and capture-frame binding
  mapping/                      single OctoMap authority service, Go2 ground projection
  planning/                     native SMART 2D/3D workers and collision adaptation
  perception/                   SAM3 multi-session, RGB-D people state, and OHZ
  execution/                    Go2/UAV execution adapters and per-instance exclusive control authority
  team/                         peer state, short-term intent, and narrow-passage coordination
  observability/                RViz, metrics, and bounded logs; no web console
  simulation/                   source/import scripts for the standalone map and UAV Actors
  third_party/                  pinned versions and necessary patches; no full model weights
  tests/                        contracts, offline replay, simulation integration tests
  experiments/                  scene seeds, combinations, sharing and OHZ controls
  runtime/<scene>/<run>/
    shared/                     environment map, map metadata, dynamic constraints
    robots/go2_01/              tree/route/lease/memory/logs
    robots/uav_01/              tree/route/lease/memory/logs
```

P0 sets up a local Git in the new directory using the `codex/air-ground-shared-map` branch, recording source versions. This document does not run Git init, commit, or push this round. Later, source code, configs, tests, and docs go into Git; model weights, UE build artifacts, virtual environments, large map snapshots, run logs, and secrets do not.

The UE engine can be reused read-only as the already-compiled build. New UAV C++ features need an independent CARLA project/worktree and module outputs; splitting source into separate directories must not still end up overwriting plugin binaries that Factory is using. Large assets may use filesystem-supported copy-on-write; hardlinks on assets that will be modified are forbidden. Maps, project settings, and build outputs each have their own home.

The AirSim model is pinned at `/data/sim/assets/airsim/quadrotor1/1ca93f6f77e4/`. Original assets are not rewritten; rigging and collision modifications produce new derived versions.

New run instances tentatively use CARLA 2400 (with auxiliary ports checked), ROS domain 45, and an optional read-only RGB-D service 22287; check occupancy before implementation. These are kept distinct from the old 2100/2200, the extracted repo's 2300, and the old ROS domains 42/43/44. Within one multi-robot world the ROS domain, `/clock`, and `map` are shared; robots do not each start their own UE world.

## 4. Profiles, Instances, and Control Authority

| Config layer | Content | Must not mix in |
|---|---|---|
| Robot model | Type, size, body envelope, velocity/acceleration limits, motion dimensions, executor capabilities, ground rules | Goals, maps, or leases of any particular run |
| Sensor rig | Resolution, rates, LiDAR parameters, extrinsics, self-filter volumes | Other robots' Actor IDs or latest poses |
| Scene | map frame, content version, legal bounds, ground level, map-save identity | Hardcoded motion state of any one robot |
| Robot instance | `robot_id`, model, rig, initial position, namespace, tasks, and runtime paths | Overriding another instance's state via shared files |
| Team composition | Instance set, sharing switches, capability assignment, coordination policy | Copying a whole service set via string replacement |

Instances are `go2_01`, `go2_02`, `uav_01`, `uav_02`. TF uses unique names such as `go2_01/base_link`, `uav_01/base_link`; sensor topics are isolated per instance; the global `map` is consistent. Each robot has exactly one executor that actually writes motion, independently holding control authority and action sequence numbers.

Startup preflight prints the fully resolved configuration and hashes. Duplicate IDs, duplicate TF, paths pointing at old runtimes, two executors contending for the same Actor, or a ground profile bound to a UAV executor are reported directly as configuration conflicts.

The existing Go2 profile's footprint, camera extrinsics, and speeds are kept first as a sourced baseline; the multi-robot migration will not incidentally lower obstacle thresholds, forward emergency stop, SAM3 road rules, or old patrol points. The new empty scene uses geometry and people perception and does not enable Town10's permanent road-ban logic.

## 5. Shared 3D Environment Map

The main approach is fixed as **a single C++ map service + OctoMap OcTree**. The official library provides occupied, free, and unknown representations; we own multi-sensor synchronization, source deduplication, update versioning, per-robot traversability views, and the execution interface. [Official OctoMap documentation](https://octomap.github.io/)

We borrow the old Exyn mapper's input, batching, and hit/miss experience, but do not maintain Exyn and OctoMap as two parallel authoritative maps. JIE serves for now as a reference for old-map import/viewing, and its ground planner is not put in front of SMART. The existing JIE semantic bridge accumulates semantic voxels and ignores dynamic categories but lacks complete ray clearing, so it cannot directly serve as the new map service.

### 5.1 Observation Contract

Each frame carries at least:

```text
scene_id, scene_epoch, robot_id, sensor_id, frame_seq
capture_sim_time, receive_wall_time
points_in_sensor_frame, T_map_sensor_at_capture, sensor_origin
range/validity metadata, calibration_id
```

The unique key is `scene_epoch + robot_id + sensor_id + frame_seq`. A resent observation must not be double-weighted; observations genuinely captured by different robots are distinct evidence. The shared map must not be re-fed as point-cloud input for repeated accumulation.

Fix the old-bridge gaps: ROS stamps use capture time, not callback `now()`; ray origins and endpoints use the same-frame sensor transform, not `latest_pose`. The CARLA-to-ROS axis conversion happens at one explicit entry point. If a real spinning LiDAR later carries per-point timestamps it will need motion-distortion correction; this round does not claim physical-vehicle localization drift is solved.

### 5.2 Update Rules

- Valid hit endpoints update occupied; voxels the ray passes through update free; unobserved parts stay unknown.
- Use voxel ray traversal; do not copy the old large-step equidistant sampling, to avoid missing thin obstacles.
- After hitting a near object, the occluded space behind must not be cleared; no-return/no-depth does not automatically mean free. If CARLA's explicit far-plane depth is used for cleanup, it must keep a simulation-only marker.
- Deduplicate voxels per frame, bound log-odds saturation, and make hit/free conflict handling and the allowed out-of-order window explicit. Cross-sensor conflicts must not be resolved by last-message-wins.
- Start with 0.25 m voxels to stay comparable with the original Go2 resolution; compare 0.20 m or finer after thin-obstacle acceptance. The old UAV's 0.8 m setting is not reused directly.
- Initially reuse the verified conservative near-range observation range as a baseline, then separately widen the UAV's effective marking range; an 80 m sensor range is not automatically 80 m of reliable clearing.

The new map distinguishes currently observed occupancy from man-made permanent forbidden zones; dynamic pedestrians and peers must not become un-clearable permanent walls just because they were seen repeatedly. Predicted OHZ is not written into the long-term occupancy map. Tracking expiry can only cancel predictions or mark state uncertain — it must not clear real geometry on that basis.

### 5.3 Increments and Versions

The map publishes `scene_epoch + base_revision + revision + changed_cells/regions`, with changes covering the three outcomes occupied, free, unknown. Any traversal-affecting operation produces a new version, including cleanup, import, pruning, or later permitted decay; the old `scan_seq` alone must not stand in for the map version.

The planner keeps a local read-only snapshot so one planning pass reads a consistent version; it does not make a ROS remote query per candidate edge. The first version can use ordered increments + a partial sparse snapshot cache; no distributed database or GPU map service for now. When an increment gap is detected, re-fetch the snapshot instead of clearing the whole map.

Map backups keep the full probabilistic state and metadata, e.g. complete `.ot`, checksums, coordinates, resolution, scene content version; saving only a display point cloud that lost the probability history is not acceptable. A new epoch is created when the run episode changes; only snapshots explicitly matching scene coordinates and content version may be restored. Transient trajectories/OHZ do not come back as valid state along with the static map.

Transient geometric occupancy reliably associated with a cooperating robot or a dynamic pedestrian is managed separately and is not restored as part of the static `.ot` snapshot; historical geometric evidence with uncertain association may be kept, but near-field execution after restore still needs fresh observations. Previously known occupancy must not be automatically flipped to free or unknown just because the current viewpoint is occluded.

## 6. One Environment Map, Two Traversability Models

The shared map stores physical occupancy and is not uniformly inflated by any one robot's footprint.

**Go2:** Combines local ground height, support, step/slope limits, body height, and the XY footprint into a 2D traversability layer fed to the Go2's Nav2 costmap view and SMART 2D. The ground itself must not become a body obstacle; an overhead beam blocks passage only if it intrudes into the body clearance. Phase 1 flat ground sets an explicit ground datum and does not quietly change the original Factory run configuration where support checking is disabled.

The new Go2 instance's geometric occupancy source is unified as the shared-map projection; the old standalone LiDAR accumulation layer no longer runs alongside as a second geometric authority, avoiding the case where the shared map has already cleared a cell but the old layer still blocks it. Manually drawn forbidden zones and short-term predictions are separate layers that keep their provenance, and each layer can only revoke constraints it created itself; the Nav2 exported view also records the corresponding map version.

**UAV:** Directly checks the 3D body and flight bounds. The first version uses a conservative envelope containing all propellers, measured against the actual rig; orientation-dependent boxes may come later. State, paths, and goals all include Z; collision checks cover the swept volume of the entire edge, including above and below — not just two endpoints or one center ray.

Each traversability view records the body/profile/calibration versions in use. Agree uniformly on the responsibility boundaries among object size, robot size, and safety margins, so that costmap inflation, SMART OHZ, and the executor do not each add the same margin three times.

Both allow building candidate trees in unknown areas; actual execution releases only near-field paths already observed free, and requires that robot's recent LiDAR to be fused and its latest-occupancy and peer-position checks to pass. The known-free classification is kept by default until new evidence changes it — a voxel is not automatically forbidden just because no ray has passed through it for 3 seconds; per-cell time limits may be enabled separately as an experimental policy. The near-field length is determined by actual speed, braking capability, and latency. An unscanned goal is not the same as unreachable; directions that cannot be observed should first be observed, not replaced by unverified forward motion.

## 7. True-3D SMART Integration and Tree Recovery

Pick one pinned SMART core as the shared dependency, recording the official source plus the existing static-increment/clearing-recovery/directional-OHZ extensions, to avoid arbitrary stitching from two checkouts. The official algorithm uses RRT* initialization with dynamic updates; the local true-3D example is a reference for XYZ types and interface organization. [Official SMART3D](https://github.com/LinksLabUConn/SMART3D)

The Go2 adaptation keeps `SMART<f32,2>`; the UAV adds `SMART<f32,3>`. They share the version protocol and generic collision interface, while state and edge checkers are implemented per platform. The existing 3D example's approach of adding whole ground-truth boxes to the map does not enter the formal perception chain.

Update order: install a consistent latest map view → locate newly occupied cells and prune the related invalid edges → re-validate historical edges/candidate connections over regions explicitly restored to free → apply current dynamic constraints → verify actual-position connectivity and the whole submitted path → hand over to the executor.

**Confirmed gap: the existing `maintain_static_clear_coverage` only runs at `N == 2`, using XY sampling.** First reuse the generic historical-edge and neighborhood reconnection; if 3D airspace lacks coverage, implement dimension-generic incremental point refills and connections inside that mechanism. Changing only the type parameter or lifting 2D edges must not be claimed as 3D tree recovery after obstacle removal.

Preserving the existing tree takes priority; appearing/disappearing obstacles trigger the relevant repairs. The first version records all scan, collision-check, and repair times; the 180 ms repair parameter does not mean every full-tree check is under the same hard budget. Only after proving the bottleneck, add edge/voxel spatial indexes, version caches, or batched maintenance.

Normal map checks do not send fixed-interval stops. New paths carry robot ID, goal version, map version, and path ID; the executor re-checks the handoff against the current measured position. Old path results must not override new tasks. When there is no path, give a diagnosable reason such as unknown, geometric blockage, peer occupancy, or compute budget.

Whole-tree reinitialization is only a controlled fallback for explicit failures/persistent unrecoverable states, with the reason recorded and the environment map kept; ordinary pedestrians passing by and map clearing updates do not trigger whole-tree restarts, and no clear-map watchdog is used.

## 8. UAV Carrier, Motion, and Model Integration

The AirSim raw body mesh bounding box is about 0.982×0.982×0.293 m, but it does not yet include the full propeller rig or safety margins and must not be hardcoded as the final collision size. First verify model scale, axes, materials, the four propeller positions, and the actual envelope, then register the independent UAV Actor.

The old `kinematic_drone` was a bodyless sensor rig with sensor transforms updated separately. The new chain uses one real body Actor with RGB, depth, and LiDAR rigidly attached; the sensor bridge only reads, and the single UAV executor moves that Actor.

The Actor must be explicitly set movable, with unneeded rigid-body dynamics disabled and a valid simple collision configured; swapping the mesh path on a default static prop must not count as UAV motion support. Initially keep roll/pitch level, with XYZ and yaw controlled by the kinematic adapter.

Simply disabling Go2 ground checks cannot fake flight: the current `Go2KinematicMovement` fixed-height mode still rejects vertical displacement. Add an independent UAV motion strategy, reusing the bounded-step and collision-sweep ideas, but fully checking XYZ, rotating envelope, speed limits, acceleration limits, and legal airspace.

Feedback includes at least measured XYZ, heading, actual speed, completed distance/angle, blocking reason, and timestamps. UE's actual transform is the simulation feedback; sent goals must not stand in for pose. After a collision, cameras and mapping must not keep being driven by the desired pose.

Remote control/pause/resume is also isolated per instance; initially only the needed XYZ/yaw commands are implemented. EGO, A*, MIGHTY, and PX4 may remain as later independent baselines, but this project's official UAV path source must be 3D SMART.

## 9. Shared Dynamic Information and Mutual Avoidance

Each robot broadcasts its actual timestamped body occupancy, velocity, and short-term confirmed path. Communicating one's own position is cooperative information; it is a different matter from reading UE pedestrian ground truth in place of visual estimation. Unknown external objects are still handled by sensors.

Its own point cloud is filtered by actual body and extrinsics; other robots are separated out of the static environment accumulation and keep transient collision constraints. When masking a robot's echoes, handle only its own occupancy — do not clear the occluded space behind it as free. Dynamic-constraint update/expiry and environment-map clearing are handled separately.

Two steps:

1. **Current position + short-term motion constraints:** each SMART avoids the peer's body and valid short-term trajectory; verify head-on, crossing, and different-altitude passages.
2. **Simple passage coordination:** only on narrow-passage conflicts, decide who waits and who goes by explicit priority and passage permission; with timeouts/revocation, preventing two robots infinitely yielding to each other. This module does not modify another robot's tree and does not directly send motion.

Experiment switches include environment sharing, peer intent, and passage coordination. Turning off sharing only affects cooperative features and must not incidentally disable local physical collision or the existing near-field avoidance. No global task auctioning, distributed consensus swarms, complex MAPF, or new VLM orchestration for now.

## 10. Multi-Robot Extension of SAM3 and OHZ

First LiDAR mapping and SMART navigation, then wire back the already-verified SAM3 ID + same-frame RGB-D + multi-frame position-derived velocity. Still no Kalman filtering; position differencing must not be dressed up as a SAM3 neural velocity head, and pedestrian ground truth must not be fed to estimators.

Implement one model service with an independent session per camera: weights resident once; video caches, object IDs, frame counts, conditioning history, and camera calibration/pose caches all isolated. The current single-camera service does not have this; cross-session contamination tests are required before going live. [Official SAM3 code](https://github.com/facebookresearch/sam3)

Scheduling uses bounded queues and fair round-robin/explicit priority; each camera keeps only a bounded set of pending latest frames with true capture times, avoiding long queues of stale frames behind slow inference. A single inference call must be bound to its session internally; concurrency must not mutate global tracker state. Weight sharing removes duplicate VRAM but not multi-stream inference compute or per-session CPU memory.

Person IDs in the first version use `robot_id/camera_id/session_id/local_track_id`; no cross-view global ReID is claimed. When two cameras see the same person, first keep the sourced constraints; potentially duplicated occupancy is handled conservatively and must not accumulate onto the permanent map. Later, spatiotemporal association only if measured redundancy is clear; no Qwen for person-ID merging for now.

Go2 keeps the verified planar pedestrian velocity and directional capsule OHZ. UAV people avoidance needs a bounded-height body occupancy range; a 2D circle must not be stretched infinitely into a whole-airspace obstacle. First estimate the body's 3D extent from valid depth and sweep along the ground-motion direction; in flat-ground pedestrian tests, state the `v_z=0` assumption and the height-uncertainty margin explicitly — no claim yet of predicting complex up/down-stair motion. A 3D ellipsoid may serve as an independent shape comparison but cannot replace range/velocity validation.

Peer robots have known profile dimensions and communicated poses, so 3D dynamic constraints are built directly; SAM3 does not need to re-recognize each cooperating robot.

SAM3 can be toggled per instance: all robots may run LiDAR navigation while only some enable people vision and share the results; this is an explicit perception configuration, not a pretense that every robot achieves the same vision update rate.

## 11. Implementation Order, Deliverables, and Exit Criteria

| Phase | Concrete work | Visible deliverable / pass criteria |
|---|---|---|
| P0 pinned baseline | New directory, Git/dependency locks, source manifest, existing-file hashes, port and OctoMap build checks; independent project/output paths | No changes to old projects; the extracted single-robot chain completes one cold start and shutdown in an isolated instance |
| P1 instance isolation | profile/instance config, replace the car1 hardcoding in the run entry, unified clock and TF, independent leases/logs | Two test instances with no cross-talk in topics, goals, or state; duplicate-identity startup rejected |
| P2 UAV minimal closed loop | Model import and rigging, independent Actor, XYZ executor, rigid sensor binding | The UAV can climb, turn, and stop on command; collisions effective above/below/left/right; actual pose matches RViz |
| P3 single-robot 3D mapping | First attach one LiDAR to OctoMap with exact scan poses, free/occupied/unknown, snapshot/increment | Wall positions match capture poses; adding/removing boxes updates correctly; the area behind occluders is not wrongly cleared |
| P4 UAV true-3D SMART | 3D worker, body sweep checks, incremental edge pruning and clearing reconnection | Actually flies over low walls/around beams; path Z varies; obstacle changes repeated 20 times without clearing the map or rebuilding the tree each time |
| P5 1+1 shared geometry | Dual-LiDAR fusion, Go2 ground projection, individual trees, remote obstacle appear/clear notifications | Under the same beam the Go2 passes below while the UAV is blocked at the corresponding altitude; a remote observation genuinely changes the other robot's planning view |
| P6 peer dynamic avoidance | Peer actual occupancy, short-term intent, then simple narrow-passage permissions | Crossing, head-on, and vertical separation without permanent robot shadows; the sharing and coordination switches each effective |
| P7 SAM3/OHZ | One stream regression → shared-weight two streams → add per performance; person height range | No cross-stream leakage of images/depth/poses/sessions; velocity and OHZ continuity meet the frozen single-stream/multi-stream criteria |
| P8 three/four-robot capacity | Add robots one by one in the same scene, then enable vision per stream, recording bottlenecks | Report the maximum accepted combination with sensor specs, scene, rates, and background load |
| P9 Factory and release | Independent factory instance, small scope → target scope, run package, cold start and backup | Empty scene and Factory each have clear start/stop entries; restart recovers the environment map and task configuration |

Parallelizable: one agent owns configuration/lifecycle, one owns the shared-map contract and offline replay, one owns the UAV Actor/executor; SMART 3D adaptation develops in parallel against fixed map fixtures. Integration proceeds by the P0→P9 dependencies; each agent has explicit file responsibility and no two agents modify the same core source simultaneously.

## 12. Behavioral Tests That Must Pass

| Test | Result to verify |
|---|---|
| Same wall scanned by two robots at different angles | Consistent position in the map; axis conversion, extrinsics, and capture pose correct |
| Resend, out-of-order, old episode | Resends not double-weighted, beyond the allowed window diagnosable, old-world data rejected |
| Near object in front of the LiDAR, unknown behind | Clearing rays stop at the surface and do not pass through the object to clear behind it |
| Delayed LiDAR callback during fast body turns | Map walls do not drag with callback delay; ray origins still match the capture pose |
| An edge free at both ends with an obstacle in the middle | The edge is rejected by body sweep, not endpoint-only checks |
| Obstacle removed, both ends of an old edge outside the cleared region | Related historical edges are still found via the intersection region and reconnected |
| A added and B removed simultaneously | Old edges must be revalidated against the new complete map and must not pass through the new A |
| Beams and low walls | From the same environment data Go2/UAV get different traversal outcomes, and the UAV path genuinely changes Z |
| Dynamic pedestrian/peer leaves | Predicted constraints update correctly; real free rays clean residues; occluded areas keep the last valid state and time; originally unknown stays unknown |
| Map updates while moving | No obstacle-free periodic holds; old paths must not re-enter from stale poses |
| Multiple cameras see the same/different people | tracker, depth, and capture poses do not cross sessions; global ID consistency not required in the first version |
| Stop one instance | Other robots keep running; the map and other control authorities are not deleted |
| Missing TF, sensor stalls, illegal actions, Actor/world swaps | Explicit status given and old commands stopped; recovery depends on fresh data and valid control authority, not map resets |
| Two clean cold starts | Assets, dependencies, map-restore identity, new sensor epochs, and startup order all correct |

Latencies for occupied→edge pruning and real-free→reconnection are tallied separately; all four timestamps — map publish, planner receipt, edge invalidation, path handoff — are recorded. Keep tree ID, node reuse counts, historical-edge recovery counts, new point counts, and whole-tree reset counts, proving the original tree is being repaired.

Experiment scoring may read UE box, pedestrian, and robot ground truth for collision-distance/velocity error measurement; the scoring process is separate from the formal sensor/estimator inputs, with provenance checkable in logs.

## 13. 4090 Capacity Testing and Budgets

First fix ambient light, render quality, camera specs, LiDAR, map extent, tree budget, and pedestrian-flow seed; change only robot count or vision-stream count at a time — lowering quality simultaneously and then claiming cost-free scaling is not allowed.

| Tier | Robots | Perception enablement order |
|---|---|---|
| B0 | 1 Go2 | Previously accepted chain as reference; new-map single-robot baseline |
| B1 | 1 UAV | Single-robot true-3D mapping, planning, and execution baseline |
| B2 | 1 Go2 + 1 UAV | All geometry → one SAM3 stream → two SAM3 streams |
| B3 | 2 Go2 + 1 UAV | All geometry → SAM3 streams added as needed |
| B4 | 2 Go2 + 2 UAV | Stress tier; no upfront commitment to full vision |

Each tier runs at least 15 minutes after warm-up, and final candidate combinations at least 1 hour; empty scene first, then Factory. Record peaks in both warm-up and long runs, state sample sizes explicitly in statistics; for shared-benefit comparisons, repeat multiple times with the same routes/pedestrian seeds — no conclusions from a single success.

Must record:

- GPU whole-card VRAM peak, model allocated/reserved memory, UE render time, actual output rate per camera stream.
- CPU per-core/thread pressure, RAM peak, map update queue, SAM3 per-session CPU cache growth.
- Simulation-time/wall-clock ratio; no claiming 10 Hz real-time while the simulation runs at half speed.
- Capture frame age p50/p95/p99 for each LiDAR, RGB-D, and people output stream; OHZ availability and longest gap.
- Mapping commit latency, map-version lag, full-check/prune/reconnect times separately, execution control rate and stalls.
- Observed voxel counts per map, tree node/edge counts, and RViz publish load; no extrapolating 2D empty-scene memory to the 3D factory.

Suggested initial engineering targets: VRAM headroom of about 2–3 GiB, at least 8 GiB RAM available; with a 20 Hz control target and 10 Hz LiDAR configuration, actually achieve no less than 8 Hz where possible and 5 Hz map updates; geometry frame-age p95 target no higher than 0.3 s. SAM3 keeps the original data-freshness threshold, comparing effective rate and prediction continuity per stream. These are **acceptance targets pending confirmation and freezing against the single-robot baseline**, not currently measured results or a universal safety guarantee.

If targets are missed, first locate whether UE camera rendering, CPU rays/mapping, SAM3 throughput, or 3D edge checks are overloaded; adjust camera count/rate, bounded caches, resolution, map extent/voxels, and bounded repair respectively. After changing specs, re-report capability — do not mask problems with larger timeouts, hidden stale data, or loading a model copy per robot.

Capacity results must finally be stated concretely, e.g. "in this scene, at this resolution and pedestrian flow, 2 Go2 + 1 UAV with 2 SAM3 streams sustained these rates and latencies for 1 hour straight" — never an unconditional "the 4090 can run 4 robots".

## 14. RViz, Startup Order, and Release Requirements

Keep this project free of a web console. RViz shows shared 3D occupancy, the Go2 2D traversability layer, each robot's real tree/path/body in its own color, individual LRZs, and valid people/peer OHZ and velocities. Panels state robot ID, map version, perception frame age, current goal, control authority, and blocking reason. Large-tree display may lower the publish rate, but drawn fake connections must not replace the planning tree.

The plan is one unified entry point supporting `plan`, `preflight`, `up --scene ... --team ...`, `status`, `snapshot`, `down`; the concrete commands are a later deliverable and not executable this round.

Startup proceeds by observable readiness conditions:

1. Parse configs, check sources/ports/dependencies and resources, and choose map restore or creation.
2. Start the single UE world and clock owner; confirm CARLA RPC, map identity, and advancing time.
3. Spawn each robot; verify real Actors, TF, and sensor frame pairing.
4. The mapping service starts, restores a legal snapshot, and publishes the first new-epoch snapshot.
5. Each traversability view and SMART worker starts; executors stay on standby at first.
6. Start SAM3 sessions, peer state, and coordination as needed; confirm message freshness.
7. Grant each robot control authority per the explicit task configuration and start patrol/navigation; RViz can be toggled separately.

Shutdown manages only the PIDs/services and Actors created by this run: first revoke motion permission and stop, save the environment map and task configuration, then shut down perception, planning, and mapping, and finally the owning UE. Broad `pkill python/Unreal` is forbidden. After restart, old leases, old dynamic predictions, and the old capture cache do not come back; trees may reinitialize from the saved environment map, and routes and task configuration load per an explicit recovery policy.

A release includes: source commits, pinned dependencies, model provenance, scene content version, resolved profile output, map snapshots, startup instructions, two cold-start evidences, capacity report, and rollback notes. New versions are written to a separate SSD backup directory, verified by manifest, then the "current version" index is updated, without deleting previously accepted versions. The GitHub push scope is confirmed separately later; Fab full assets or the user's original projects must not be uploaded as ordinary source.

## 15. First Acceptable Deliverable

The first deliverable is not four robots at full configuration, but: **in the same empty scene, the Go2 walks on the ground while the UAV flies at different altitudes; both observe one movable obstacle together — when it appears, each prunes its own related tree branches, and after it is removed each restores its connections, with the map preserved throughout; after stopping, everything can be brought back up through the same entry point.**

After this passes, accept peer avoidance, SAM3 cross-camera, the third/fourth robots, and the complex Factory scene. Every phase keeps independent switches and repeatable controls, so a failure never leaves map-wiping or a full system restart as the only options.

## 16. Main Local Evidence and References

- [Verification boundaries of the published source](/data/robotics/sam3-rgbd-smart3d-navigation/docs/CERTIFICATION.md)
- [Extracted source and launcher notes](/data/robotics/sam3-rgbd-smart3d-navigation/README.md)
- [SAM3/OHZ empty-scene continuity record](/data/robotics/ue58_ohz_test_go2/docs/verification/2026-09-27-ohz-continuity.md)
- [Existing Go2 navigation configuration](/data/robotics/ue58_ohz_test_go2/configs/car1_navigation_profile.json)
- [2D worker entry point](/data/robotics/sam3-rgbd-smart3d-navigation/runtime/native_smart3d/src/main.rs:30)
- [SMART incremental and clearing recovery source](/data/robotics/sam3-rgbd-smart3d-navigation/runtime/native_smart3d/vendor/smart3d_observed/src/smart/smart.rs)
- [Local 3D example](/data/robotics/smart3d/SMART3D-upstream/examples/smart_multi_agent_3d/main.rs:235)
- [Unified project blueprint and unfinished items](/data/robotics/semantic_smart3d/README.md)
- [UAV model package notes](/data/sim/assets/airsim/quadrotor1/1ca93f6f77e4/README.zh-CN.md)
- [Existing Go2 motion component](/data/sim/CARLA_UE58_Research/Carla-ue58/Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/Actor/Go2KinematicMovement.cpp)
- USB source: `/media/felix/FeliXFANG/drone1_full_code_backups/drone1_ue5_realtime_mapping_3fab808_20260701_210952/source_worktree`, pinned commit `3fab808af94e0f93f185ed3e605d5f1626a12e63`.
- New UAV branch: `codex/drone1-rolling-local-3d-fallback-20260701` of `FelixFtch/drone1-ue5-realtime-mapping`, previously verified at `e99f32a27e3b8c54ca8b9e394b0b7e46782e2943`; pin the exact commit at implementation time and re-review the selected diffs.

This plan treats only verified code as grounds for reuse; historical logs, goals stated in documents, or synthetic benchmarks are not evidence that the new system has already run successfully.
