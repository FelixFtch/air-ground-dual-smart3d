# Dual-robot shared OctoMap + SAM3 stable parameter record

Record date: 2026-09-29. Corresponds to the running version the user confirmed as "the parameters are good now, very smooth, and much faster after adding SAM3". This round only did read-only measurement, parameter saving, and speed-up direction assessment; no parameters were adjusted, no services restarted, no map/trees rebuilt.

This is a **configuration, source-identity, and performance-evidence baseline**, not a full backup including UE assets, weights, and the current map. A cold start can reuse the configuration, but identical random trees, transient obstacles, and performance numbers are not guaranteed.

## 1. Project and run identity

| Item | Current value |
|---|---|
| Program | `/data/robotics/air_ground_smart3d` |
| UE project | `/data/sim/AirGroundLab_UE58/CarlaUnreal/CarlaUnreal.uproject` |
| Level | `/Game/AirGroundLab/Maps/AirGroundLab_Main` |
| Scene identity | `air_ground_lab_v1`; content version `2026-09-29-v2-calibrated` |
| Current run epoch | `1ddaeeef8d1848909e9161e65dbba5ad` |
| Profile hash | `c7bb2aef1b98c82e3db2b82fcffd1bb9af0f62e6779adb74af0462f34e38be59` |
| CARLA / ROS | port 2400 / domain 45 |
| UE run | UE5.8, `-game -RenderOffScreen`, no audio, frame rate cap 30, 1280×800 |
| Render pools | Texture streaming 1800 MiB; Nanite streaming 512 MiB |
| Simulation clock | Asynchronous world; sensors keep capture frame and simulation time |
| Pedestrian flow | 6 people; configured speeds 0.5/0.7/0.9/1.0/1.2/1.4 m/s |
| Visual inference | SAM3 on; this dual-robot experiment does not additionally run YOLO, Qwen, or a new web console |

The old Town10, Factory, and single-Go2 empty-scene projects remain independent. See the project root `README.md` for start/stop order; `./lab start --rviz` brings up this experiment, and `./lab stop` manages only this experiment's own services. A default cold start re-observes the map; restoring a specific `.ot` requires an explicit `--resume-map`, and old free space must be re-observed.

## 2. Shared map, geometry, and execution

| Parameter | Value |
|---|---|
| Voxels | Real OctoMap, 0.25 m |
| Map extent | X/Y each −30 to 30 m, Z −0.5 to 8 m; 60×60×8.5 m |
| UAV planning extent | X/Y each −20 to 20 m; altitude 0.8–6 m |
| Coordinate transform | map x=CARLA x, map y=−CARLA y, map z=CARLA z−0.22 m |
| Measured ground | CARLA Z≈0.20125 m; map Z≈−0.01875 m |
| Occupancy probabilities | hit=0.9, miss=0.1, clamp=0.12–0.88 |
| Obstacle marking/clearing | LiDAR marks within 9 m; valid measured rays clear up to 40 m; physical range 45 m |
| Time handling | Applied strictly by capture time; old frames cannot overwrite newer evidence; hits take priority at the same time |
| Free retention | No TTL-based auto wall clearing; known free space is kept until new occupancy evidence appears |
| Unknown regions | Candidate trees allowed; actual near-robot motion must pass known-free and collision checks |
| Execution checks | Near end 0.4 m; max age of locally fused LiDAR 1.2 s; max age of planning results 3 s |
| Cooperative robots | Self/partner points filtered from the shared map; both sides' real positions and body envelopes act as temporary collision-avoidance constraints |

The fast hit/miss flipping here targets the current zero-noise simulation and is not yet a real-robot-validated parameter. Distant actually hit surfaces do not pass themselves off as free space; missing echoes do not fabricate clearing rays. Occluded regions not re-crossed by valid rays do not vanish into thin air.

## 3. Robot motion and patrol routes

| Parameter | Go2 | Bee UAV |
|---|---|---|
| Identity | `go2_01` / `go2_ground_v1` | `uav_01` / `fighter_uav_v1` |
| Motion/planning dimensions | XY + yaw; 2D SMART | XYZ + yaw; truly 3D SMART |
| Appearance scale | 1 | 0.35 |
| Planning body half-size | (0.43,0.43,0.28) m | (0.70,0.70,0.36) m |
| Max translation speed | 2 m/s | 1.5 m/s |
| Acceleration | 2 m/s² | 1.2 m/s² |
| Max rotation rate | 1 rad/s (about 57.3°/s) | 0.8 rad/s (about 45.8°/s) |
| Arrival tolerance | 0.55 m | 0.60 m |
| Height | Fixed map Z=0.33 m, flat-ground support check off | 0.8–6 m |
| Start point | (−8,−8,0.33) | (−8,−3,2) |

The planning half-sizes are conservative clearance envelopes and should not be treated as measured mesh-model dimensions. The UAV's UE entity collision half-size is (0.48,0.48,0.34) m; planning XY additionally covers the steering sweep. The Go2 provides no stair gait; the UAV flies kinematically.

Go2 cyclic patrol: `(−8,−8,0.33) → (8,−8,0.33) → (8,8,0.33) → (−8,8,0.33)`.

UAV cyclic patrol: `(8,−3,2) → (8,7,3.2) → (−8,7,2) → (−8,−3,2)`.

Path-following lookahead 0.7 m; rotate in place when heading error ≥0.65 rad (about 37°), otherwise slow down by cos(error). Control runs per UE tick with dt limited to 1–75 ms. The speeds above are upper bounds; obstacle avoidance/turning/waiting is slower.

## 4. Sensors and fusion

| Parameter | Go2 | UAV |
|---|---|---|
| LiDAR | 96 channels, 192000 points/s, 10 Hz, 45 m, zero noise | Same |
| Vertical FOV | −75 to 45° | −85 to 75° |
| LiDAR mount translation | (0,0,0.7) m | (0,0,0.4) m |
| LiDAR mapping target | 5 Hz each; every actually hit voxel keeps a real representative endpoint | Same |
| Forward RGB-D | 640×360, FOV100°, captured every UE frame | Same |
| Forward camera mount translation | (0.5,0,0.4) m | (0.5,0,0) m |
| Forward depth mapping target | 2 Hz, 6 m, stride20 | Same |
| Auxiliary depth | None | Six directions 96×96, FOV150°, captured at 5 Hz |
| Auxiliary depth mapping target | — | About 0.667 Hz per direction, 3 m, stride3 |

See `profiles/sensors/uav_rgbd_lidar.yaml` in the same directory for the exact six-direction mount poses. Fusion uses latest-frame with fair scheduling and a bounded pipeline; it does not queue large numbers of stale frames. Target rates are not guaranteed under combined load.

## 5. SAM3, person velocity, and OHZ

| Parameter | Current value |
|---|---|
| Model | `facebook/sam3`, revision `3c879f39826c281e95690f02c7821c4de09afae7` |
| Inference | Transformers Sam3VideoModel; CUDA BF16, non-quantized, SDPA |
| Instances | One shared-weight model, two independent video sessions; text query `person` |
| Input publishing | Up to 3 Hz per robot, full 640×360 same-frame registered RGB-D, at most 8 photo packages kept |
| Service processing cap | **2 Hz total**; the two cameras alternate, about 1 Hz each |
| Video memory | Up to 16 objects per session; re-detection every 16 frames; unconditional history 128 frames; up to 4 condition frames per object; lost-track retirement policy 30 s |
| Person position | Up to 2048 depth points per mask, 0.3–15 m valid depth; transformed to map with the capture pose |
| Velocity estimation | At least 4 valid same-source observations; up to 6 points, 4.5 s window; consecutive gap ≤1.5 s |
| Algorithm | Median of pairwise slopes of XY positions; max speed 3 m/s, residual threshold 0.6 m; no Kalman filter |
| Not estimated | Z velocity unreliable/not estimated; UE pedestrian heading or speed is not read as input |
| Geometry validity | 2 s; extrapolation at most 1 s, unlike the 30 s lost-video-track retention |
| OHZ | Prediction horizon 1 s; Go2 2D ellipse, UAV 3D ellipsoid; includes each body envelope |
| Error priors | Position margin 0.15 m; visible-person range padded 0.85 m above and below, **not yet experimentally calibrated** |
| Cooperative body filtering | Same-frame both-robot OBB + 0.08 m; rejected when the residual planar vertical span after filtering is <0.05 m |

Without a reliable velocity, directionless dilation is used and must not be interpreted as measured person motion. OHZ and RViz use the same actual planning constraints; cooperative robot collision avoidance is retained.

Note: `shared_model_inference:false` in the team YAML is a stale field the currently running code does not read; the actual state is explicitly `shared_model_instances=1` with two sessions. This round keeps the original file and records the discrepancy rather than silently modifying it.

## 6. SMART3D and display

| Parameter | Current value |
|---|---|
| Core | LINKS Rust `SMART<f32,2>` / `SMART<f32,3>`; provenance in `planning/CORE_PROVENANCE.json` |
| Initialization | Go2 up to 5000 iterations, UAV up to 7000; 1500 ms budget each, whichever comes first; iterations are not node counts |
| Goal update | One round every 0.4 s per robot |
| Local tree-repair budget | 3000 iterations or 50 ms; tree pruning, re-rooting, final checks, and export are not counted in the 50 ms |
| LRZ | 4 m for both robots |
| Clearing recovery | Per round 64 regions / 512 nodes / 64 connections / 10 ms cap |
| Free-space tree patching | Up to 64 attempts, 8 new nodes, 10 ms per round |
| State snapshot | 5 Hz target |
| RViz | Pose/path polled at 10 Hz, map 5 Hz, tree 2 Hz, camera 5 Hz, point cloud 10 Hz |
| Tree display | Connected edges width 0.025 m, alpha 0.35; disconnected edges 0.02 m, alpha 0.20 |
| Marker lifetime | 3 s; DELETEALL on first pass, afterwards only disappeared IDs are deleted / existing items updated |
| Display limits | Point cloud at most 5000 points per frame; voxels 100000; tree edges 30000; underlying map input not truncated |

This experiment does body clearance checks directly against the shared voxels; no separate Nav2 is started. The RViz 2D projection is not another Nav2 costmap. Trees may enter unknown regions; the execution side independently checks near-robot traversability.

## 7. Measurements this round

All measurements are read-only samples of existing files/counters; no new CARLA sensor listeners were added.

About 49.86-second sample:

| Metric | Go2 | UAV |
|---|---:|---:|
| LiDAR received | 10.03 Hz | 10.03 Hz |
| LiDAR actually mapped | **3.07 Hz** | **3.09 Hz** |
| Forward depth actually mapped | 1.22 Hz | 1.20 Hz |
| Latest fused LiDAR age mean / P95 | 0.411 / 0.675 s | 0.477 / 0.783 s |
| Distance moved during period | 38.92 m | 21.59 m |
| Patrol points completed during period | 2 | 2 |

RGB callback about 20.6 Hz; this is not an independently measured UE rendering FPS. Auxiliary depth actually mapped at about 0.44 Hz each. Display snapshot age mean 0.062 s, P95 0.248 s; this number must not be taken as LiDAR mapping latency or person clearing latency. The sampled map has about 965 thousand observed voxels. The 50-second sampler did not record `people_error`; its `errors=[]` does not prove the person pipeline had no anomalies over the whole period.

Follow-up ~19.67-second SAM3/planning sample: SAM3 at 0.915/0.966 frames per second each; model inference mean 229/256 ms, per-round perception including surrounding processing mean 323/338 ms. Sampled distinct planning results averaged about 379 ms for Go2 and about 989 ms for UAV, max 554/1724 ms; this is a subset sampled once per second and **not an unbiased distribution of all planning calls**. Pending clearing regions at the last sample were about 13.8k/29.0k; further work is needed to separate repeated regions from genuinely pending recovery.

Hardware instantaneous readings vary with the scene: two readings showed about 12.4–12.9 GiB VRAM (of about 24 GiB total), GPU compute utilization 44–58%; available memory about 21.6–27.7 GiB. These are not stress peaks and cannot guarantee adding several more robots. Precise hardware timestamps per pass and service usage are in `live-sample.json`.

Sporadic item pending investigation: this run's history had 7 person-size validation error events `person visible half extents outside [0,5]`; a dedicated ~29.7-second spot check and the later ~19.7-second sample did not reproduce them. No bad record was captured, so numeric error versus abnormal depth range cannot yet be determined, and it cannot be declared cured. Evidence in `people-bounds-check.json`.

## 8. Speed-up suggestions: independent comparisons in priority order

1. **Optimize the planner's repeated work first.** The current delta still copies about 960 thousand voxels and traverses the full tree for checks/export during updates; the 50 ms local tree-repair budget does not represent total cost. Add staged clone/diff/prune/export timing first, then consider a transactional incremental map, an affected-edge index, and clearing-region dedup/merging. Collision correctness and final path validation must be kept.
2. **Split high-frequency state from full-tree snapshots.** Poses and paths keep current update rates; the full tree is cached by changed version and output at the display cadence, avoiding moving the whole tree on every plan / every 5 Hz state write. Verify map/path/tree version consistency and disappearing-edge deletion before going live.
3. **Then run single-variable sensor/model frequency experiments.** The forward RGB-D is currently generated every UE frame while mapping actually runs at about 1.2 Hz and SAM3 at about 1 Hz per robot; test whether synchronized RGB/depth capture at 10 Hz saves UE cost. If person response still needs to be faster, then separately try SAM3 total 2→3 Hz. The current per-round average including surrounding processing is already about 0.33 s; 3 Hz may be near the service budget — tail latency and the other pipelines must be examined; no improvement is promised.

For now, robot driving speed is not raised, the 9/40 m observation range is not shrunk, voxel precision is not lowered, and higher frequencies are not obtained by reducing collision checks. Shadows/volumetric light/post-processing will get quality comparisons only if evidence shows UE rendering is the limiter; a GPU that is not saturated does not mean there is no CPU, serialization, or single-thread bottleneck.

None of these optimizations were implemented this round. The recommendation is to keep the currently user-approved parameters and use this record as the baseline for every comparison.

## 9. Evidence in the same directory

- `resolved_profiles.json`: the complete configuration actually resolved by the current run.
- `profiles/`: copies of the configuration source files; contains no run state, weights, or commercial assets.
- `source-sha256.json`: checksums of program, configuration, and native binaries; includes Git identity information but is not equivalent to a committed version.
- `fusion-sample.json` / `fusion-summary.json`: ~50-second raw sampling and summary.
- `live-sample.json`: ~20-second SAM3/planning sampling plus hardware and service snapshots; full trees removed, only counts and metrics kept.
- `people-bounds-check.json`: read-only spot check of the sporadic person-size issue.
- `manifest.json`: records time, run identity, and file checksums.
