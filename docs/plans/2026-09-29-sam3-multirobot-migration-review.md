# SAM3 Multi-Robot People Tracking and OHZ Migration: Read-Only Source Review

2026-09-29. Source `/data/robotics/ue58_ohz_test_go2`; target `/data/robotics/air_ground_smart3d`. This review only inspected local source code, installed versions, and non-certified receipts; the old project was not modified, no model loaded, no GPU service started. The target still uses the existing Rust SMART; no CARLA pedestrian actor IDs, positions, or velocities were used as perception.

## Conclusions

The old tracking code and the downloaded models are still there; the new dual-robot instance has not yet been wired into this perception chain. The old SAM3 Video inference, bounded history, same-frame RGB-D projection, and velocity quality gates can be reused, but a new multi-source scheduler, persistent capture bundles, cross-camera association, and 3D risk adaptation are needed. Sharing model weights does not automatically share person identities.

Old chain: same-frame RGB + organized XYZ + capture-time TF → SAM3 Video local track ID/mask → PeopleProjector → capture-time map position and XY velocity → transient people constraints/OHZ → consumed by the old Go2 planner.

The most critical current gap is that the target's `sim_io/runtime.py:187` saves only the matched frame's `latest_rgb.jpg/json`, not full-pixel depth/XYZ, and there is no immutable capture bundle for the async model to read at a fixed point. `sim_io/geometry.py:25` decodes sparse mapping points; those points cannot be used to per-pixel project a full SAM3 mask.

## Boundaries for Independent Reuse

Source files are all under `/data/robotics/ue58_ohz_test_go2/scripts/`. Target directory names are suggestions, not existing ones. Full source paths, SHA-256s, and test files are in the [copy manifest](/tmp/sam3_multirobot_copy_candidates.json).

| Source file / key entry points | Suggested use | Boundaries that must change or be kept |
|---|---|---|
| `car1_sam3_video_people.py:71` VideoModel; `:114` reset; `:126` infer; `:230` FrameSequence | `perception/sam3_backend.py` + service | Split the single `self.session/frame_index/session_id` into per-camera storage; load weights and processor only once. Extract overlay and frame-order checks; do not carry over the old main, HTTP fetch, or lock/output. |
| `car1_sam3_video_memory.py:167/192/199/314` | standalone vendored memory policy | Keep the model source-hash check, per-stream session quota, and the order of lost-track cleanup before history trimming. Configure the model once, and each new session once. |
| `car1_sam3_people_projection.py:87/123/191` encode/decode/PeopleProjector | `registered_rgbd.py` + `people_projection.py` | Keep the strict shape/dtype/ZIP/hash/capture-time checks; parameterize camera, size, frame, epoch. One projector/history per stream. |
| `car1_person_observations.py:29/105/137` | new observation validation and transient people layer | Reuse the evidence gates, rewrite multi-source identity, timing, and shared-OctoMap adaptation; do not carry over the old LocalCostmap wrapper. |
| `car1_ohz_geometry.py:23/42/52` | Go2 2D OHZ geometry and comparison tests | Explicitly XY-only; the radius already includes the robot envelope, so the UAV must not re-interpret it with the Go2 radius. |
| `car1_prediction_error.py:14/37` | fixed calibration budget reading | The new environment starts as uncalibrated; the old lens/resolution/scene margins must not be silently reused. |
| `car1_sam3_rviz_image.py:41` Sam3OverlayReader | per-camera RViz overlay reading | Keep refusing to display on image/status hash mismatch; move to the new schema, paths, and camera identity. |
| `car1_dynamic_policy.py:122` CrossingWaitPolicy | optional second stage | Migrate only after valid observations, OHZ geometry, and planner/guard agree; this is not a new planner. |

Not suitable for wholesale copying:

- `car1_vlm_evidence.py`: `:212` exact frame pairing and `:235` the map→optical/body TF at the same capture stamp are worth borrowing; but the ROS loop, old costmap state, and car1 control ownership coupling do not fit the new project.
- `car1_rgbd_app.py`: importing it runs the old prep guard and instantiates the old VlmEvidence; the old ports/HTTP routes need not be kept.
- `car1_native_planner.py`: `:678` onward is useful for understanding the old fusion/OHZ wiring, but do not port the whole old planner or controller.
- `car1_sam3_node.py`: this is the road/sidewalk/grass/cone static semantics node, not the people Video tracker; do not load a second copy of the model because of it. Its ExactFrameBuffer can only serve as a pairing-test reference.

## Single Weight, Multiple Video Sessions

The old `VideoModel` already infers via `model(inference_session=..., frame=..., frame_idx=..., reverse=False)` (`:134`). The installed Transformers source `Sam3VideoInferenceSession` keeps object IDs, track history, prompts, and cache inside the session; the processor's `init_video_session` creates independent objects. Minimal implementation:

1. One process, one Sam3VideoModel, one processor, and the same verified local weights. Load offline once at startup; model configuration happens once.
2. `sessions[(scene_epoch, robot_id, camera_id)]` each hold a session UUID, a contiguous processed-frame counter, a FrameSequence, a PeopleProjector, and lost-track state. A calibration change or clock rollback also requires an isolated reset.
3. At most one pending latest complete bundle per source, scheduled by fair round-robin/deadline; GPU inference is serial, processing one session at a time. Thread-concurrent model-call safety has not been verified — do not call the same model in parallel yet.
4. Old capture frames may be skipped, but SAM3's session frame index stays contiguous from 0 for that source; velocity uses the original capture time, not the inference completion time. Resetting one stream must not wipe another stream's history.
5. The old 4 Hz was a single-stream limit and does not guarantee 4 Hz each for two streams. Measure single-frame latency and VRAM first, then set per-stream rates; record queue wait, capture age, inference latency, and drop reasons separately.

The old memory policy is per-stream history 128, max objects 16, re-conditioning every 16 frames, conditioning context 4, lost TTL 30 seconds. Two streams grow session state, so "one weight" must not be advertised as "VRAM completely unchanged". The policy deliberately bounds upstream context; do not claim numerical equivalence with unbounded history. Slow inference alone must not trigger a "stream-lost reset"; existing FrameSequence tests cover this distinction.

Local dependencies were verified via file metadata and source hashes: Python 3.10 environment `/data/robotics/sam3_video_people/.venv`; torch `2.14.0+cu126`, transformers `5.17.0`, kernels `0.16.0`, numpy `1.26.4`, Pillow `11.1.0`, safetensors `0.8.0`. Model revision `3c879f39826c281e95690f02c7821c4de09afae7`; the local model/receipt can be reused, and the weights need not be copied into the new project or re-downloaded. Keep the two Transformers source SHA checks at `car1_sam3_video_memory.py:19` and the kernel receipt check; the new service uses its own outputs and locks, avoiding tampering with the old service's directories.

## Single-Robot Assumptions That Must Change

- `video_people.py:17–21` old ROOT/OUTPUT, HTTP 20280/20287, `/api/fubao/v1/…`; `:45` `X-Go2-*`, `d435i_rgb`, 848×480.
- `people_projection.py:22/43` hardcoded 848×480, camera ID, two-element old map_epoch, body TF child `car1/base_link`; all of these should come from the current instance/sensor profile. The new front-facing camera is 640×360; scaling the RGB alone while leaving depth pixels and intrinsics untransformed is not acceptable.
- `video_people.py:188/267/272` a single output directory, worker.lock, Publisher/FrameSequence/session; each camera's output is separate, and the service lock belongs to the new project.
- `person_observations.py:79` forbids duplicate bare-integer track IDs in the merged list. Native ID 1 appearing on both streams is normal; the full local identity must be kept first — never concatenate the old lists directly.
- `person_observations.py:87` map_stamp refers to the old costmap publish time. The new shared map watermark is not a unified "current prediction time"; replace it with an explicit `prediction_reference_sim_time`, and separately record the consumed map revision and capture time.

## Persistent Registered RGB-D Bundle Input Contract

Recommend replacing the old factory schema with a versioned `air_ground.registered_rgbd/v1`, reusing the old NPZ validation ideas. Every bundle must contain, together:

- Identity: `scene_id, scene_epoch, content_version, frame_id, robot_id, camera_id, calibration_id`.
- Capture: `frame_seq, capture_sim_time, capture_stamp`; RGB, depth, camera transform, and body pose must share the capture frame/time. `receive_wall_time/receive_monotonic_time` is receipt time and must not replace capture time.
- Pixels: the RGB image and `width,height,K`; a full float32 `[H,W,3]` optical XYZ on the same pixel grid, NaN for invalid pixels; record the valid depth range and color encoding. Mapping stride/downsampling must not apply to this input.
- Poses: capture-time `camera_to_map` and `body_to_map` with axes and units declared; filling in old images with a latest-pose lookup is forbidden.
- Integrity: `frame_token` covering source identity, frame, and calibration; RGB/XYZ hashes packaged and verified together with the payload. Output observations keep the token, capture time, and original bundle path/hash.

Use a separate filename `<frame_seq>-<token>.npz`, write to a temporary file first, then atomically rename, then update a small latest manifest; keep a bounded ring per source (e.g. 8–16 bundles), and bundles under inference must never be reclaimed. Sample bundles and results kept for review can be retained separately by configuration, avoiding unbounded video archives. This enables async replay, velocity auditing, and overlay without backfilling depth from new frames. The old `encode_rgbd_capture/decode_rgbd_capture` has ZIP_STORED, member-count/size, Numpy dtype/header, and JPEG hash boundary checks worth keeping and retesting at 640×360.

Coordinate conversion is a real migration risk: the current `geometry.depth_points` outputs CARLA sensor axes `+X forward,+Y right,+Z up`; the old projector reads z-depth in optical axes `+X right,+Y down,+Z forward`. Use `xyz_optical=[localY,-localZ,localX]`. If `S=diag(1,-1,1)` is CARLA world→map and `C=[[0,0,1],[1,0,0],[0,-1,0]]` is optical→CARLA sensor, then `R_map_optical=S @ R_carla_sensor @ C` and `t_map=S @ (t_world-world_origin)`. The coordinate basis changes together — the Y-negating reflection matrix alone must not be passed off as a quaternion. Run numerical tests with known pixels, camera rotation/UAV pitch, and static pedestrian world points.

Production of SAM3 front-view bundles must be independent of the mapper's depth downsampling/sampling success. The current runtime saves RGB only after fusion completes; copying that as-is would wrongly bind perception availability to the mapper queue. The UAV's 96×96 six-way auxiliary depth has no RGB pairing and must not serve as the people video stream.

## Shared Person IDs and Velocity

A local identity is at least `(scene_epoch,robot_id,camera_id,tracking_session_uuid,native_track_id)`. The old `PeopleProjector:259` has only `sessionUUID:nativeID`, and the existing `normalize_people` constraints cannot directly become a multi-robot registry.

Build a new shared person registry: associate different sources by map-space positions synchronized to a common reference time, position quality, time offsets, and velocity consistency, storing the evidence mapping from global ID to each source's local key. Within one camera it must be one-to-one; across cameras the same global person may be supported. On ambiguity from two people approaching/occlusion/different-time observations, keep it pending or assign a new ID — never force "nearest point is the same person". No unimplemented appearance ReID is introduced; identity continuity after long disconnection cannot be guaranteed.

The old projection is the median and 10–90% percentile range of visible surface points, not a full body center/bounding box. Surface centers shift across viewpoints, so centers from the two robots must not be interleaved into one velocity sequence. Estimate velocity per source by capture time first, then fuse compatible valid velocities; on association changes or anomalous jumps, clear the corresponding velocity evidence. The old velocity function `people_projection.py:197` requires at least 4 observations, sufficient time span, robust slope, and velocity/residual gates; keep these validity flags.

The old chain measured only XY velocity; Z velocity was not estimated. The new output must carry explicit `velocity_xy_valid` and `velocity_z_valid=false`; do not fill zeros and claim a full XYZ velocity was measured. The shared registry publishes raw observations/uncertainty, and each robot adds its own profile envelope — avoid sharing a risk already inflated for Go2 and inflating it again for the UAV.

## How OHZ Reattaches to the Existing SMART

The people layer is a time-stamped, sourced transient perception/prediction constraint; the scene OcTree still keeps the actual depth/LiDAR obstacles. Do not write predicted pedestrian paths as permanent walls, and do not bulk-delete real map points for "pedestrian moved"; the old `person_observations.py:182–208` likewise explicitly refuses to disguise historical occupied cells as moving pedestrians.

The new worker's vendored `planning/vendor/smart3d_observed/src/smart/obstacle.rs:11/30/38` already has `SphericalObstacleWithOhz<F,N>`, sweep, ellipse; `hazard_zone.rs:7/40/87` has generic-N sphere/capsule/ellipsoid checks. But the people hazard list is currently empty when calling core update; the existing `dynamic_boxes` are peer body occupancy and do not mean SAM3/OHZ is wired in.

The minimal follow-up wiring is a new `dynamic_people` contract: global ID, source contributors, capture/reference time, measured map geometry, valid velocity components, horizon, uncertainty/calibration state, expiry; converted per Go2/UAV profile into verifiable risk geometry, with the planner, near-field execution guard, and RViz all using that same geometry. Keep the real SMART<f32,3> XYZ tree. The old 2D ellipse can serve directly as the Go2 comparison; the UAV must explicitly handle observed z-range, incomplete visible surfaces, and unknown vertical velocity. If a conservative body-height prior is adopted, label it a profile prior — do not claim SAM3 measured the full body. Do not stretch an XY ellipse infinitely and pass it off as 3D perception.

The old prediction-error calibration is a fixed held-out budget, not online ground-truth input. After new lenses, resolutions, or UAV viewpoints, start uncalibrated and re-evaluate on new perception tracks; never quietly reuse old calibration margins.

## Minimal Implementation Order and Acceptance

1. Create the standalone `perception/`, copying the bounded source/tests with hashes; implement the bundle writer/reader and complete coordinate boundaries first, and enable the model only after CPU tests pass.
2. One weight, two-stream sessions tracking in shadow mode without affecting current motion; output per-source overlay, mask+depth quality, map positions, velocity validity, and frame evidence. Measure latency, fairness, VRAM, and frame drops.
3. Attach the shared registry and RViz global ID/source display; verify that simultaneous native ID 1s do not collide, a single-stream reset does not hurt the other stream, the same person seen by both robots can merge, and crossing pedestrians are not forcibly merged.
4. First verify offline/in shadow that OHZ/native/guard/display geometry agree on the same tracks, then enable the effect of people risk on execution and SMART tree repair. Loss/expiry must trigger transient-layer clearing, never clearing of real occupancy.

Ten CPU tests can be inherited; see the copy manifest for the list. Must be added: wrong-frame/wrong-calibration/wrong-epoch rejection; deformable 640×360 shapes; bundle publish/read with no frame mixing; UAV pitch/body rotation producing no false velocity against static pedestrians; independent dual-stream frame index/session reset; cross-view visible-surface offset creating no velocity; ambiguous identity not forcibly merged; fair scheduling without starvation; unknown vertical motion not passed off as valid 3D velocity; the tree recovering after expired people geometry is cleared; and each robot type inflating its own envelope exactly once.

This review rests on source code and local dependency checks. No dual video model or performance test was run, so no per-stream frame rate, VRAM budget, cross-camera identity accuracy, or Factory real-time motion performance is promised.
