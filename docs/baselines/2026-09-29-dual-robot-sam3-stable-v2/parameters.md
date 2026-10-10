# Dual-robot shared OctoMap + SAM3 stable baseline v2

Record date: 2026-09-29. Baseline ID: `2026-09-29-dual-robot-sam3-stable-v2`. Corresponds to the version the user approved after the LiDAR timing fix — current results acknowledged, updated save requested. This archival changes no run parameters, restarts no services, and clears no map. The old `2026-09-29-dual-robot-sam3-stable` baseline is retained.

**The entire current `profiles/` is byte-identical to the old baseline copy, and the current running `resolved_profiles.json` is also identical; what changed is code, the isolated UE module, and effective observation behavior.** Restoring the old YAML alone must not be considered restoring the current version.

This is a **configuration, source/native-binary/UE-patch identity, and performance evidence baseline**. The exact archived files are governed by the final manifest; it does not include the full UE engine, all commercial assets, model weights, or dependency environments, and does not pass off the live run's map as a consistency snapshot. This configuration archive **does not save the active map**. Restoring a stopped run's `.ot`/metadata and `state.json` is a separate operation. A cold start cannot guarantee identical random trees, transient obstacles, and performance numbers.

## 1. Project and run identity

| Item | Current value |
|---|---|
| Program | `/data/robotics/air_ground_smart3d` |
| UE project | `/data/sim/AirGroundLab_UE58/CarlaUnreal/CarlaUnreal.uproject` |
| Level | `/Game/AirGroundLab/Maps/AirGroundLab_Main` |
| Scene identity | `air_ground_lab_v1`; content version `2026-09-29-v2-calibrated` |
| Current run epoch | `d83ccef432d3410e8e573a93122096cb` |
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
| First obstacle marking/clearing | LiDAR first marks within 9 m; valid measured rays clear up to 40 m; physical range 45 m |
| Known-surface far re-observation | Voxels once established by real close-range hits can be refreshed or re-occupied by real hits at 9–40 m; distant unknown voxels do not gain first occupancy from this |
| Ray retention | `all_distinct_measured_rays`; only fully identical endpoints are deduplicated; distinct actual rays are not dropped by endpoint voxel |
| Same-frame evidence | All near/far real hit voxels take priority over same-frame free; at most one hit/miss per voxel per frame; keeping more rays does not amplify probability counts |
| Time handling | Applied strictly by capture time; old frames cannot overwrite newer evidence; hits take priority at the same time |
| Free retention | No TTL-based auto wall clearing; known free space is kept until new occupancy evidence appears |
| Unknown regions | Candidate trees allowed; actual near-robot motion must pass known-free and collision checks |
| Execution checks | Near end 0.4 m; max age of locally fused LiDAR 1.2 s; max age of planning results 3 s |
| Cooperative robots | Self/partner points filtered from the shared map; both sides' real positions and body envelopes act as temporary collision-avoidance constraints |

The fast hit/miss flipping here targets the current zero-noise simulation and is not yet a real-robot-validated parameter. Distant actually hit surfaces do not pass themselves off as this frame's free space; virtual endpoints clipped to range/bounds are not treated as real surfaces. Missing echoes do not fabricate clearing rays. Occluded regions not re-crossed by valid rays do not vanish into thin air. Far-range known surfaces are not permanently locked: they can still be cleared later when that hit is absent and a newer real passing ray exists. The discretization ambiguity of fine surfaces versus gaps within 25 cm voxels remains; it cannot be claimed that all thin-beam mis-clears and dynamic ghosts are cured.

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
| LiDAR mapping target | 5 Hz each; all distinct measured rays kept, only fully identical endpoints deduplicated | Same |
| Forward RGB-D | 640×360, FOV100°, captured every UE frame | Same |
| Forward camera mount translation | (0.5,0,0.4) m | (0.5,0,0) m |
| Forward depth mapping target | 2 Hz, 6 m, stride20 | Same |
| Auxiliary depth | None | Six directions 96×96, FOV150°, captured at 5 Hz |
| Auxiliary depth mapping target | — | About 0.667 Hz per direction, 3 m, stride3 |

See `profiles/sensors/uav_rgbd_lidar.yaml` in the same directory for the exact six-direction mount poses. Fusion uses latest-frame with fair scheduling and a bounded pipeline; it does not queue large numbers of stale frames. Target rates are not guaranteed under combined load.

**UE LiDAR timing fixed:** the isolated AirGroundLab's `RayCastSemanticLidar.h/.cpp` saves the sensor's true elapsed time in `PrePhysTick`, consumed once in `PostPhysTick`; the ordinary `RayCastLidar.cpp` uses the same interval and synchronizes the ordinary point cloud's `horizontal_angle` header. Previously `sensor_tick=0.1` yet the scan was computed with the ~1/30 s world frame interval, typically only about 120° per packet and about 64320 rays/s emitted; now, under this stable interval, the configured 192000 rays/s per LiDAR is restored. The first packet or interval-scheduling jitter can still produce partial-scan packets; **it is not a forced 360° per packet**. The observation diagnostic `observed_xy_azimuth_span_deg` measures actual echo coverage, not the emitted scan angle.

`mapping_sampling: representative_of_every_hit_voxel` remains in both YAMLs but is only an old description the code does not read; the actual policy is defined by the helper, the native implementation, and the runtime statistic `ray_geometry_policy`. To preserve the verified profile identity, the profile/hash was not modified this round just to reword text.

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

Without a reliable velocity, directionless dilation is used and must not be interpreted as measured person motion. Cooperative robot collision avoidance is retained.

**Current person OHZ display is now decoupled from slow planning receipts:** from the same latest valid person document and a common current simulation time, the runtime generates each robot's current OHZ per its own envelope; execution checks and RViz use this moment's data. The Rust planner consumes homologously constructed shapes per request, and its `planner.people_hazards` retains that planning round's evidence and must not be treated as the sole source of the current display. This change removes the extra display-waiting-for-planning delay; it did not raise the SAM3 inference frequency, extend the 2-second TTL, or guarantee a reliable velocity in every frame.

Shared identity only performs unambiguous spatiotemporally compatible association, not appearance-based ReID: cross-camera association distance 0.65 m, time difference 0.75 s, allowed visible-height center difference 0.8 m; dual-source valid velocity differences above 0.75 m/s are not merged for estimation. Visible-surface 3D boxes, planning OHZ, and velocity arrows mean different things.

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
| State snapshot | 5 Hz target; fast state/path separated from immutable full-tree files |
| Tree file reuse | Identical planning objects skip re-encoding; identical geometry reuses files; SHA256 and epoch/robot/session/tree-instance checks |
| Tree file retention | 6 s read grace + latest 4 versions per robot; display cache bounded; the controller's full paths/planning objects unchanged |
| RViz | Pose/path polled at 10 Hz, map 5 Hz, tree 2 Hz, camera 5 Hz, point cloud 10 Hz |
| Tree display | Connected edges width 0.025 m, alpha 0.35; disconnected edges 0.02 m, alpha 0.20 |
| Marker lifetime | 3 s; DELETEALL on first pass, afterwards only disappeared IDs are deleted / existing items updated |
| Display limits | Point cloud at most 5000 points per frame; voxels 100000; tree edges 30000; underlying map input not truncated |

This experiment does body clearance checks directly against the shared voxels; no separate Nav2 is started. The RViz 2D projection is not another Nav2 costmap. Trees may enter unknown regions; the execution side independently checks near-robot traversability.

The deployed Rust `Map::delta` now classifies this round's changed keys only into occupancy added/cleared/reverted-to-unknown; the full clone, validation order, fail-without-commit, and time-refresh semantics are kept. Full-tree and final path collision checks were not omitted. The neighbor-index crash caused by repeated coordinates in cyclic patrol was fixed inside this project's vendor with an exact adaptive-axis KD index, keeping all IDs/coordinates; provenance and the scope of deviation from upstream are in `planning/CORE_PROVENANCE.json`. This fork must not be called verbatim-unmodified official source.


## 7. Current-version measurements and limitations

Here, only **epoch `d83ccef432d3410e8e573a93122096cb` after the LiDAR timing fix** counts as the current measurement. The old baseline's ~3.07 Hz each and the earlier performance patch's 3.66/3.68 Hz both happened under the old scan timing and must not be mixed with the current roughly full-circle-per-packet workload as the same performance benchmark.

Evidence: [LiDAR timing acceptance](../../verification/2026-09-29-lidar-sensor-interval/README.md). Later 44.86-second window:

| Metric | Go2 | UAV |
|---|---:|---:|
| LiDAR received | 9.99 Hz | 10.01 Hz |
| LiDAR actually fused into map | **1.96 Hz** | **1.98 Hz** |
| Capture-to-map-apply mean / P95 | 0.230 / 0.457 s | 0.340 / 0.544 s |
| Map request mean / P95 | 0.103 / 0.150 s | 0.150 / 0.225 s |
| Distance moved in window | 31.46 m | 14.44 m |
| Tree nodes at end | 4997 | 4439 |

In the 60-second angular-coverage window, both robots' median per-packet actual echo azimuth span was 358.20°; packets matched to fusion exceeding 340° were 57/75 and 58/77 respectively. The earlier 60-second window includes startup, with fusion about 1.35/1.37 Hz; another 35.1-second window at about 110–145 s after start showed 1.74 Hz for both. In the latter, the last 20 packets' total processing time median was Go2 228 ms / UAV 298 ms, P95 about 374/437 ms. `cache_apply_s` includes waiting on the map lock, and `transport.await_s` includes native computation plus IPC/scheduling waits; they must not all be taken as a single function's CPU time.

Early windows showed brief `sensor_stale`; the 35-second window still had about 2 seconds of this UAV state, and the later 45 seconds had none. This indicates startup costs improve but sustained-throughput limits remain; this version must not be advertised as a 10 Hz map or as never pausing again over long periods. The 1.2-second execution freshness limit is unchanged.

The current model/sampling configuration is still 640×360, publishing at most 3 Hz each, SAM3 shared-model total cap 2 Hz. At archival verification the SAM3 state was one weight instance / two sessions, processing 627/621 frames cumulatively over about 630 seconds of running; this is a cumulative order-of-magnitude check including startup, not a substitute for a dedicated timing window or OHZ persistence scoring. This round did not re-quantify the full OHZ persistence ratio or GPU peaks, and does not reuse old hardware readings as current measurements.

The current 45-second sample had no fatal, map, people, or planner errors. The initial occupancy of the 10 sampled person-position ROIs all cleared to zero within the window; first clearing was 0.66–7.68 seconds from the **sampling start**, including time when the person kept walking inside the ROI — **not the clearing delay after confirmed departure**. Thin-beam ROI occupancy still fluctuates. These are short field observations, not whole-field ground truth or long-term stress certification.

Currently deployed items include full ray retention, real far-hit evidence, the UE sampling-interval fix, tree transport, and delta classification. Not yet deployed: Wavemap, TTL auto-decay, a whole-map transactional storage rewrite, or unconditionally skipping collision checks. The /tmp candidate changing the `observed` index to an unordered_map showed only about 0.4–0.8% benefit and was not deployed.

## 8. Runtime dependencies and launch/restore

| Item | Current source/value |
|---|---|
| Isolated UE project | `/data/sim/AirGroundLab_UE58/CarlaUnreal/CarlaUnreal.uproject` |
| Shared read-only engine | `/data/sim/CARLA_UE58_Research/UnrealEngine5_carla58` |
| This build's isolated Carla module SHA256 | `33d291317d125f548d4dfcd8ba7497d70480d84b8f775aed3336c63678803bf2` |
| CARLA Python environment | `/data/robotics/ue58_ohz_test_go2/.venv/`, CARLA 0.10.0 interface matching UE5.8 |
| SAM3 Python environment | `/data/robotics/sam3_video_people/.venv/`; offline-loads installed weights |
| ROS | `/opt/ros/humble`; domain45; RViz system Python |
| Private JSON dependency | `runtime/python_deps/ujson.cpython-310-x86_64-linux-gnu.so`, 5.12.1, SHA256 `2967cb08d9454085e9ed947e4d4598a59d50c817d383b8dc0c2d5fa4737eae3b` |
| Numeric library threads | Own services use 1 thread each for OPENBLAS/OMP/MKL, avoiding excess thread spawning for small matrix transforms |
| Service memory caps | UE 30G; runtime/ROS view default 8G; SAM3 12G; RGB-D capture 2G; LiDAR preview 1G; RViz 3G (caps are not actual usage) |

The launcher `simulation/launch_sim.py` also forcibly disables audio and directory watching and uses this project's private DDC/XDG directories; UE `carla.Go2.RequireSupport 0`, the 30 FPS cap, and the 1800/512 MiB render pool arguments are in the source. The default normal game mode does not start MCP; only scene-preparation mode enables 18183.

Cold start from a stopped state:

```bash
cd /data/robotics/air_ground_smart3d
./lab start --rviz
./lab status
```

Stopping manages only this experiment's `air-ground-lab-*` services and lets the runtime save a consistent exit map:

```bash
./lab stop
```

Restoring the map, robot poses, and patrol progress requires selecting a **compatible, normally stopped** run:

```bash
./lab start --rviz \
  --resume-map /absolute/stopped-run/shared/map.ot \
  --resume-state /absolute/stopped-run/state.json
```

Keep the metadata files beside the `.ot`; do not copy the `.ot` alone and ignore identity, probability validation, and the close-range hit history. Restore creates a new epoch, keeps map probabilities/known-surface history, but clears old observation times; old free space must be re-observed in the new epoch to be executable. Trees are reinitialized under the new worker; the original random trees are not loaded. `--resume-state` requires `--resume-map`; map-only restore does not restore the original robot poses/progress. Incompatible new/old scene/profile combinations cannot bypass validation.

`./lab start --rviz --visible` opens the UE window when launching from a stopped state; `--no-perception` is a launch option for pure-geometry comparison and does not shut down an already-running SAM3 online. Normally, do not start a second instance on the same port at the same time.

## 9. Differences from the old baseline and stale notes

1. **Configuration unchanged, code and UE module changed.** profiles and the resolved configuration are completely identical; the old source hashes cannot represent the new version. The new package must include the current `sim_io/lidar_rays.py`, runtime, mapper, Rust source/native binaries, the tree snapshot module, and the three UE LiDAR source files / corresponding module identity.
2. **Ray policy changed.** The old "representative point per hit voxel" loses free evidence from rays passing through different intermediate cells; now all distinct actual rays are kept. The original YAML description remains an unread historical field; the current README is updated, and the runtime statistic explicitly reports `all_distinct_measured_rays`.
3. **Far-range updating is more complete.** Same-frame real far-hit protection has been added; surfaces once close-range hit can be recovered by real echoes at 9–40 m, while unknown distant cells still do not gain first occupancy. `known_near_hit_keys` is persisted with map metadata, and an explicit forget clears this history too.
4. **sensor_tick time genuinely fixed.** The YAML still says 10 Hz/192 kpps, but only the new UE module computes from the true sampling interval; the old ordinary point-cloud angle header has also been synchronized and fixed. Replacing only the Python code while missing the UE module is not allowed.
5. **OHZ current display separated from planning history.** The current valid OHZ is constructed from the same person document at the current time; old planner historical shapes are not the current display. TTL, the velocity-estimation window, and shape parameters are unchanged.
6. **Tree transport / map diff optimization and the repeated-coordinate crash fix are done.** Old baseline section 8 listed these as to-do items and is outdated; the whole-map clone and every final collision check are still kept — full incremental storage must not be claimed as implemented.
7. **Residual documentation must not be taken as runtime fact.** `shared_model_inference:false` is not read by current code; the reality is one set of weights, two sessions. The `PersonRegistry`/service-class default TTL of 1.5 s is overridden by the launcher's explicit `--ttl-seconds 2.0`. The early "person OHZ pending migration" wording in `contracts` was updated this time; the running YAML and code stay as-is, and the historical parameter-field discrepancies are spelled out here.
8. **Not a Git history commitment.** The project's local Git still shows untracked directories, and the old manifest has no valid HEAD; the archive relies on actual files and SHA256 identity and cannot claim completed Git commits, pushes, or attestation.

## 10. Evidence to keep together and archive boundaries

- This directory's `profiles/`, `resolved_profiles.json`, source/native-binary/UE file hashes, and the final manifest; archived as `source-config.tar.gz`, `native-linux.tar.gz`, `ue-integration-delta.tar.gz`, with exact checksums in `manifest.json`.
- Current navigation source, tests, planning vendor provenance files, native mapper/worker; `.venv`, Cargo target caches, and general logs are not mixed into the source package.
- The isolated UE modified-source mirror, `simulation/uav_source_changes.json`, and the corresponding launch/configuration; model assets, the engine, and the full Content set need the original external backups/installs and do not come complete with a small parameter package.
- The private ujson runtime dependency with its provenance/license is included in `native-linux.tar.gz` and is not omitted by excluding runtime caches.
- Current acceptance: `docs/verification/2026-09-29-lidar-sensor-interval/`; previously deployed tree/OHZ/diff proof: `docs/verification/2026-09-29-performance-ohz-farhit/`; cyclic patrol fix: `docs/verification/2026-09-29-patrol-index-fix/`.
- This archival does not stop the simulation, save the active map, or change Town10, Factory, or the original single-Go2 empty-scene configuration. If an updatable restorable map is needed later, a separate consistency snapshot must be taken; a state being written must not be arbitrarily paired with an old map.

Evidence from different versions is labeled separately with epoch, time window, and workload. The new baseline is for repeated future experiments and configuration restore, not a claim that all dynamic/occlusion problems are cured.
