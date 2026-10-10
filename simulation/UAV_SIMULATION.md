# AirGroundLab UAV simulation contract

Implementation: 2026-09-29. Isolated project `/data/sim/AirGroundLab_UE58/CarlaUnreal` only. Factory and previous OHZTest source/maps are unchanged. The shared UE5.8 engine is reused; the new Carla plugin was built with `-NoEngineChanges`. Initial build succeeded (114.29 s); the LiDAR parent exclusion incremental build also succeeded (27.14 s); live tests are separately recorded: all 11 bounded acceptance cases passed on 2026-09-29 (21.07 s), report `runtime/uav_actor_smoke.json`. This does not yet establish the full shared-map navigation chain.

## Actor and appearance

CARLA blueprint `robot.uav.fighter_drone`, default role `ag_uav_01`. A single box root drives the complete Fighter skeletal mesh rigidly; attached sensors use CARLA rigid attachment. The visual mesh uses its reference pose; flight/propeller animations and aerodynamic or PX4 simulation are not implemented.

The original mesh bounds are centered at approximately UE cm `(0, 33.025, 158.447)` with half extents `(85.066,113.962,80.937)`. Scale `.35` and yaw `-90°` produce a visual half envelope approximately `.399 × .298 × .283 m`. The actor automatically centers using the loaded asset's actual bounds and rejects appearances extending outside the collision box. Collision half extents default `.48 × .48 × .34 m`. The planner must cover every yaw, e.g. XY half extent `.70 m`; this is separate from the actual rotating UE box.

The root blocks all UE trace/collision channels, including sensor traces. The visual mesh has collision disabled. Thus remote LiDAR can observe the UAV body. In this isolated project, the LiDAR raycast excludes only its actual attachment parent chain, allowing rays to pass its own carrier. Other robots and obstacles remain visible. Post-filtering self hits alone cannot recover rays already blocked by the carrier.

## Spawn attributes

All fields except `role_name` are floats. Metres and radians are used by attributes, and CARLA API locations are world metres. Only UE internals use cm and degrees.

| Attribute | Default | Meaning |
|---|---:|---|
| `role_name` | `ag_uav_01` | Per-robot identity |
| `body_half_x_m`, `body_half_y_m`, `body_half_z_m` | `.48`, `.48`, `.34` | Actual collision half extents |
| `visual_scale` | `.35` | Asset scale, validated against body bounds |
| `visual_yaw_degrees` | `-90` | Local asset +Y aligned to actor +X |
| `max_speed_mps` | `1.5` | 3D speed norm cap |
| `max_acceleration_mps2` | `2.0` | Velocity-vector slew cap |
| `max_yaw_rate_rps` | `.8` | Absolute yaw-rate cap |
| `max_yaw_acceleration_rps2` | `2.0` | Yaw-rate slew cap |
| `max_request_translation_m` | `.15` | Maximum single XYZ displacement request |
| `flight_min_x_m`, `flight_min_y_m`, `flight_min_z_m` | `-10`, `-10`, `.2` | World bounds of the **complete body** |
| `flight_max_x_m`, `flight_max_y_m`, `flight_max_z_m` | `10`, `10`, `6.2` | World bounds of the **complete body** |

Flight bounds are not center-position bounds. Convert the scene's map coordinates to CARLA world once. When robot profiles specify center altitude limits, account for body extents before setting these attributes. Do not pass the larger yaw-conservative planning extents as UE body attributes: that changes the actual simulated body.

Spawn checks level pose (zero roll/pitch), full body flight-bounds containment, loadable mesh, appearance containment and no initial world overlap. Invalid spawn is refused rather than leaving a mesh-less actor.

## Commands and receipts

```python
# The controller computes a bounded next pose from ACTUAL snapshot pose.
responses = client.apply_batch_sync(
    [carla.command.ApplyTransform(uav.id, desired_world_transform)],
    False,  # no client tick in our asynchronously advancing world
)
error = responses[0].error
# Always read the subsequent snapshot/actual pose; accepted != target achieved.
```

`actor.set_transform()` is asynchronous and discards the error response. Use the existing batch API for receipts; no CARLA Python wheel change was needed. CARLA transform rotation and `get_angular_velocity()` use degrees/degrees per second; convert explicitly for ROS radians.

A normal command arrives about every 50 ms. Each request is limited to `.15 m` total XYZ movement and at most 45° requested yaw; actual speed and yaw-rate limits usually make the accepted displacement much smaller. Timing uses simulation time, not wall time. Duplicate requests in one simulation tick are rejected. After a command gap >.2 s acceleration starts from rest. There is no automatic residual integration: no commands means no movement, and requesting the current pose stops immediately.

Every translation and yaw uses short (≤2.5 cm / ≤2°) substeps, a rotation-inflated swept box, endpoint overlap checks and the UE root sweep. The full swept envelope must remain inside the geofence. X, Y and Z movements use the same collision checks; there is no ground support ray or fixed-height rule.

Rate clipping is successful and the batch response contains no error. Confirm progress through actual pose and measured velocity. On a collision, a partial safe substep may already have been applied before the error; use actual pose for the next decision. Success states (`rate_limited`, `reached_requested_step`) are internal labels, not currently exposed through a separate RPC status endpoint.

Typical returned errors use the prefix `uav_motion:`:

- `swept_body_blocked` / `root_sweep_clamped`: actual body would collide.
- `flight_bounds` / `flight_bounds_swept_envelope`: full body would leave the configured bounds.
- `request_exceeds_bounded_move`: oversized translation/yaw request.
- `same_simulation_tick`: command frequency exceeds new world frames.
- `finite_unit_scale_level_pose_required`: invalid transform, scale, roll or pitch.
- `query_body_unavailable` / `dormant_actor`: actor cannot currently apply collision-aware movement.

`get_velocity()` and `get_angular_velocity()` report accepted displacement divided by simulation dt; they expire to zero after .15 simulation seconds without motion commands. They are kinematic feedback, not measured aerodynamic dynamics. The bridge should also retain timestamped pose deltas for independent progress checks.

## Source and build

`simulation/ue_source/` mirrors the nine changed/new Carla files; `simulation/uav_source_changes.json` records original and new hashes. This is an integration patch set, not a standalone replacement CARLA source tree. Existing Go2 movement code is not changed. The server's new error branches are restricted to the UAV actor.

```bash
DLSS_SDK=/data/sim/CARLA_UE58_Research/SDKs/DLSS \
/data/sim/CARLA_UE58_Research/UnrealEngine5_carla58/Engine/Build/BatchFiles/Linux/Build.sh \
  CarlaUnrealEditor Linux Development \
  -Project=/data/sim/AirGroundLab_UE58/CarlaUnreal/CarlaUnreal.uproject \
  -NoEngineChanges -NoHotReloadFromIDE -MaxParallelActions=6
```

Build output: the isolated project's `Plugins/Carla/Binaries/Linux/libUnrealEditor-Carla.so`; log: `runtime/build/uav_editor_build.log`. The UBT output line naming the shared UnrealEditor binary is normal reuse, not a new separate engine build.

## Live acceptance

Use the verified UE5.8 client `/data/robotics/ue58_ohz_test_go2/.venv/bin/python`; the older semantic_smart3d CARLA wheel is incompatible.

Run `simulation/uav_actor_smoke.py` only after the isolated AirGroundLab scene is running on port 2400. Pick a known-empty 4 m cube with ground >1 m below the chosen center. It refuses other map names and synchronous worlds, creates only its own temporary actor/obstacles, uses no `world.tick()`, and cleans up its own actors. It records free XYZ/yaw motion, sensor rigid attachment, stopping, oversized-command rejection, all six directional collisions, yaw sweep, full-body geofence, recovery after obstacle removal, and LiDAR ego exclusion with an external obstacle still visible.

The scene bridge separately verifies LiDAR/RGB-D timestamps, shared-map alignment, complete UAV profile separation and 3D SMART3D routes over/under obstacles. Passing this actor test alone does not establish those higher-level results.

## LiDAR sampling interval correction (2026-09-29)

The isolated LiDAR now consumes actual sensor actor tick elapsed time from PrePhysTick instead of the post-physics world-frame delta. Ordinary LiDAR also copies the updated scan angle into its own output header. The parent-chain exclusion remains intact. These changes are mirrored in `simulation/ue_source/Sensor/` and recorded in `uav_source_changes.json`. Incremental build succeeded in 84.98 seconds. See `docs/verification/2026-09-29-lidar-sensor-interval/README.md` for live evidence and limitations; this does not guarantee every packet spans a full turn.
