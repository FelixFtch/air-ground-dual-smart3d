# Shared people service (SAM3 or YOLO26, one at a time)

Normally `./lab start` / `./lab people sam3|yolo26` runs it. By hand, from this project:

```sh
/data/robotics/sam3_video_people/.venv/bin/python -m perception.service --backend sam3 --run-root /absolute/current/run
/data/robotics/ue58_ohz_test_go2/.venv/bin/python -m perception.service --backend yolo26 --max-hz 6 --run-root /absolute/current/run
```

## Back ends

Both only find people and give each camera's tracks an ID. Everything after that (depth projection, visible-surface bounds, speed fit, cross-camera registry, `shared/people.json`, the runtime's OHZ) is the same code for both.

- `sam3`: SAM3 video model, text query `person`; identities from its per-camera video memory (lost tracks kept 30 s). About 230–300 ms per frame on the RTX 4090, so `--max-hz 2` shares about 1 frame/s per camera.
- `yolo26`: `yolo26m-seg`, class 0 (person) only, confidence 0.25, one detection pass per frame. Weights are refused unless size and SHA-256 match `provenance/yolo26_people.json`. Identities come from `perception/map_tracker.py`: each detection's depth-projected map XY is matched to a constant-velocity prediction inside a walking-speed gate (0.5 m + 1.5 m/s × gap, at most 5 m). Lost tracks are kept 20 s; after a gap over 1.5 s the old ID is reused only if the clothing colour histogram also matches (Bhattacharyya ≥ 0.6). About 11 ms per frame including association, so `--max-hz 6` processes every captured frame (about 2.7 Hz per camera; RGB-D capture is capped at 3 Hz).

`shared/people.worker.lock` admits one people service per run whatever its back end, because both write `shared/people.json`.

Omitting `--run-root` resolves `runtime/latest_run.json` once. Restart the service for each run; it never carries sessions to another scene epoch. `--max-hz 4` caps **total** processed frames across cameras, not each camera's rate. Model initialization is offline, checks source/checkpoint/kernel hashes, and loads one model and processor. Only the root integration lifecycle starts this process. This module has no simulator actor, control, HTTP, road-segmentation, Qwen, or motion API.

Each `(scene_epoch, robot_id, front_rgbd)` has an independent tracking session UUID (SAM3 video session or YOLO26 map tracker), contiguous processed frame index, frame sequence guard and projector. The GPU is called serially with fair round robin and one latest manifest per source. Older pending frames are replaced, never appended to a backlog. Each inference reads a fully validated immutable bundle into RAM, so later producer ring rotation cannot alter the frame being projected. A calibration change or source clock restart resets only that source. Slow inference is not itself a camera interruption.

Input is `<run>/<robot_id>/rgbd/latest.json` through `registered_rgbd.load_capture()`. The current reference time comes from `<run>/state.json`, supplemented by fresh matching `<run>/shared/sensor_clock.json`. No wall-time extrapolation of simulation time is used. A capture slightly ahead of telemetry is deferred without deleting history. Expired captures cannot publish active geometry. Missing/corrupt input is reported as unavailable; a model failure is reported as failed and drops that camera's session before retrying a later frame.

## Outputs

- `<run>/<robot_id>/<backend>/latest.jpg` and `latest.json` (`<backend>` is `sam3` or `yolo26`): atomic image then status replacements. `ros_view` republishes whichever back end wrote last as `/air_ground/<robot_id>/people/image`. `overlay_sha256` (alias `image_sha256`) is the JPEG hash; readers must verify it and the scene epoch, capture expiry, status and source identity before displaying. Status contains `frame_token`, `capture_sim_time`, `frame_seq`, `robot_id`, `camera_id`, `tracks_count`, native tracks, global IDs, session UUID/index, memory/lifecycle receipt and timing diagnostics. `source_rgb_sha256`, `source_bundle_path`, `source_payload_sha256` identify the exact source bundle. A stale image is never re-labelled as a new source frame.
- `<run>/shared/people.json`: `schema: air_ground.people/v1`, `scene_id`, `scene_epoch` (alias `epoch`), `frame_id: map`, `reference_sim_time`, `generated_wall_time`, `state`, `perception_backend`, `sources`, `people`.
- `<run>/shared/<backend>_status.json`: compact model and per-source health, including `shared_model_instances: 1` and camera-session count.

Each person contains `id` / `global_id`, `center_xyz`, `half_extents_xyz`, `velocity_xyz`, `velocity_xy_valid`, `velocity_z_valid: false`, `capture_sim_time`, `position_reference_sim_time`, `valid_until_sim_time`, `source_keys`, `quality`, and the contributing `source_observations`. Map positions and dimensions are metres; simulation times are seconds. `center_xyz` describes the merged visible-surface box at `position_reference_sim_time`. It has already been extrapolated by at most one second; consumers must **not** apply capture-to-current displacement a second time. After that bound, `position_reference_sim_time` stays at the bounded prediction time. Geometry validity expires 1.5 seconds from capture by default; the merged box expires at the earliest contributor expiry. A later snapshot can retain still-valid contributors separately.

Source keys are JSON-encoded `[scene_epoch, robot_id, camera_id, tracking_session_uuid, native_track_id]` strings. Two cameras returning native ID 1 do not collide. A new cross-camera observation is merged only when the full spatial candidate graph is one-to-one, timestamps differ by at most 0.75 seconds, predicted XY centers are within 0.65 m, vertical centers within 0.8 m, and available valid velocities agree. Ambiguous matches remain separate IDs. Briefly missing masks retire their geometry immediately; scoped identity links survive only within their existing capture TTL. Long occlusion and ambiguous new identities are not retrospectively reconciled.

## Measurement and limits

Projection erodes the mask by one pixel, samples at most 2,048 registered optical points (0.3–15 m), rejects weak/mixed depth, transforms with the capture-time camera pose and retains 10–90% visible-surface quantiles. These are **partial observed 3D bounds**, not a complete human body. `human_height_prior_applied: false` is explicit. Robot-specific risk adapters may add an explicitly labelled prior and their own body envelope once.

XY speed requires at least four same-source, same-native-ID capture measurements, sufficient span, robust pairwise slopes, and speed/residual quality gates. The fitting evidence window is bounded to 4.5 seconds and six samples to support the measured approximately 1 Hz per-camera rate. Consecutive capture gaps above 1.5 seconds still reset the fit; widening the evidence window does not extend geometry TTL or the one-second extrapolation bound. Alternating surfaces from different cameras are never used as displacement samples. Compatible source velocities can be averaged. No Z speed is estimated: the numeric Z placeholder is zero and `velocity_z_valid` is always false. Camera pose compensation is tested under translation and pitch. Visible-surface bias can still affect position or speed when the person's pose, occlusion, or visible surface changes. Cross-camera association uses no appearance; the only appearance check is YOLO26's per-camera colour match after a gap. No ground-truth person input is used.

The default policy reuses the audited upstream-sensitive limits: 128 nonconditioning frames, four conditioning frames per object, correction every 16 processed frames, maximum 16 native objects per session, lost-track TTL 30 seconds with native keep-alive exhaustion. It deliberately bounds the upstream conditioning-pointer context and is not numerically equivalent to unlimited history. One weight copy does not make per-session state free. Throughput, two-camera peak GPU memory and identity accuracy require live verification; this document does not claim a guaranteed frame rate. Prediction-error calibration remains `uncalibrated` for the new camera views.

CPU verification: `test_people_perception.py` plus the inherited exact-source `test_sam3_video_lifecycle.py` and `test_sam3_reconditioning_memory.py`. The inherited tests inspect pinned upstream methods without importing/loading a GPU model. Copy/extraction records are in `provenance/sam3_people.json`.
