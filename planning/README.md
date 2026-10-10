# Air/ground SMART worker

One JSON request per stdin line; one JSON response per stdout line. Run one worker
process per robot. UAV uses `SMART<f32,3>` and Go2 uses `SMART<f32,2>`; both use the
same revisioned physical voxel map. No simulator, ROS, remote per-edge service,
semantic labels, or synthetic scene obstacle generator is used by this worker.

```sh
cargo build --release --offline --locked --manifest-path planning/Cargo.toml
cargo test --offline --locked --workspace --manifest-path planning/Cargo.toml
planning/target/release/air-ground-smart3d-worker < planning/examples/uav.jsonl
```

## Core identity and local changes

`CORE_PROVENANCE.json` pins the copied core to local source commit
`815eb64befb080c0b3d7a8aaffed2c4b472787a1` and records original and installed hashes.
The source project/vendor is untouched. Cargo.lock pins all dependency versions.
The local reference checkout has origin `https://github.com/LinksLabUConn/SMART3D.git`
at commit `cbe475dfe51a479c8cbb3e56d0dde57a8ee74320`; the provenance file records
which common source files remain byte-identical (including RRT*). This is an
observed-map derivative of that N-dimensional SMART core, not an unmodified
upstream release.
The new copy's `src/smart/smart.rs` retains these local changes:

- True 3D bounded clear coverage uses a 33-position XYZ stencil through native
  SMART insertion. The original 13-position XY behavior remains for N=2.
- A native sampling repair iteration tries at most 256 samples before returning
  to the outer iteration/time termination condition. Arena growth caps at 250000.
- `refresh_current_path` rechecks the actual-current connector.
- `reroot_goal` keeps existing sample coordinates and valid undirected edges,
  reorients their parents/costs from the new goal, and retains disconnected
  components and historical cut recovery. SMART reserves ID 0 for its goal, so
  the old ID 0 moves to the previous arena length; metrics explicitly disclose
  this remap. This O(arena + edges) pass rebuilds its NN index, not a new RRT*.

Temporary people OHZ also adds an explicitly specified affine ellipsoid variant
in `src/smart/hazard_zone.rs` and the corresponding runtime constructor in
`src/smart/obstacle.rs`. The existing sphere, capsule and ellipse behavior remains.
[PEOPLE_HAZARDS.md](PEOPLE_HAZARDS.md) defines the shared Python/native geometry,
source evidence, footprint, identity and expiry contract.

Initial RRT* and subsequent SMART both use a deterministic Halton sampling
adapter (bases 2/3/5), with the core's existing connection/repair rules. Time
budgets can still make node counts hardware dependent. There is no A*, EGO,
AirSim planner, grid route search, or direct-path fallback.

## Input contract

See `examples/uav.jsonl` and `examples/go2.jsonl`. Their empty map is intentionally
unknown and is only a protocol fixture, never an execution certificate.

`init` requires `protocol:"air_ground_smart3d.v1"`, `robot_id`, `session_id`,
`dimension` (2 or 3), `goal_revision`, `start[N]`, `goal[N]`, `ranges[N][min,max]`,
`profile`, and `map_snapshot`. Optional limits are `initial_iterations` (1200),
`initial_budget_ms` (1500), `repair_iterations` (128), `repair_budget_ms` (50).
The optional limits are bounded; `include_tree` defaults to true.

The profile has `profile_id`, `body_half_extents_m[3]`, `body_offset_m[3]` and
`planar_z` (required, unused for N=3). Robot orientation is not a planner state:
use an envelope conservative for every allowed yaw/attitude. The initial Go2
profile is half extents `[.43,.43,.28]`, zero offset, planar z `.349`; the UAV
profile uses `[.68,.68,.34]`. Do not discard positive-height obstacles. With the
root bridge's z translation `carla.z-.201`, the measured floor near map z=-.001
lands in voxel [-.25,0] and does not intersect Go2's bottom at .069.
Ranges constrain the reference position, not the outer body; the execution
bounds guard must check the full body against the operating boundary.

`plan`, `update`, `set_goal`, and `state` require matching `robot_id`, `session_id`
and `goal_revision`. All except `state` require actual `current[N]`. `set_goal`
also requires `goal[N]`; a changed goal requires a strictly newer goal_revision.
An unchanged goal can advance the revision without rerooting. A blocked target
returns `ok:false, reason:"goal_blocked_or_out_of_bounds"`; the proposed request
is rejected before changing the session. `init` explicitly starts a new tree;
ordinary map and goal updates keep `tree_instance_id`.

On `plan`/`update`/`set_goal`, supply either `map_delta` or `map_snapshot`, or omit
both to keep the map. The nested object is the mapper's **flat** response:
scene_id, scene_epoch, content_version, frame_id, resolution, revision,
capture_sim_time_watermark plus `cells` for snapshots, or base_revision and
changed_cells for deltas. Signed keys are `floor(mapXYZ / resolution)`.
Occupied/free are stored, missing means unknown; unknown changes delete cells.
Restored cells may have `last_observed_sim_time:null`: their saved occupancy
still constrains planning, but their time is stored as `None`, and saved free
cells cannot make `path_all_known_free` true until new current-epoch evidence
refreshes them. Null time never grants execution freshness.
A revision gap returns `snapshot_required`; a scene/epoch/frame/resolution change
requires init. Compact `refreshes` update observed times, but their probabilities
remain the last transmitted values. Collision decisions use the explicit state,
not cached log odds. A full recovery snapshot must include the watermark.

Candidate planning allows unknown; all occupied voxels block. Collision is exact
closed-segment versus voxel AABB inflated by the axis-aligned body envelope,
including the body offset, throughout the whole segment. Touching blocks. Long
sweep enumerations over 2 million candidate cells fail closed. Clear/expiry
regions queue same-tree recovery; expiry to unknown grants no free evidence.

## Short-lived partner layer

Optional `dynamic_boxes` completely replaces a separate partner layer:

```json
{"sim_time":4.0,"dynamic_revision":7,"dynamic_boxes":[{"id":"go2_01","center_xyz":[1,2,0.349],"half_extents_xyz":[0.43,0.43,0.28],"valid_until_sim_time":4.5}]}
```

Boxes are measured partner envelopes supplied by cooperative communication;
exclude self. The checker inflates them by this robot's own envelope. A changed
live list (including a lease renewal) requires a newer dynamic_revision. Geometry
comparison uses ID, center and half extents; renewing valid_until_sim_time alone
updates the lease without pruning or queueing clear work. Actual geometry moves,
resizes and removals still trigger the retained-tree obstacle/clear path. When omitted, the last list stays
until its validity deadline. Expiration/movement queues clear repair, without
inserting/removing any OctoMap cell. This is instantaneous spatial separation,
not a space-time reservation or a guarantee against a moving/stale partner; the
executor must continue live partner checks and hold on missing fresh poses.

## Temporary people layer

Optional `people_hazards` contains normalized `air_ground.people_hazard/v1`
ellipses/ellipsoids generated for this robot by `perception/hazards.py`. The
worker validates profile, epoch, scene, finite geometry, velocity evidence,
footprint and simulation times before mutation. An empty list removes the layer;
omission retains only entries before their expiry. Expiry/motion restores the
same retained SMART tree through bounded clear-region repair. A metadata-only
renewal does not trigger pruning, clearing, or a new `people_revision`. Prediction
geometry remains separate from actual sensor occupancy. Native path checks,
execution guard and RViz use the same pre-inflated shape. See
[PEOPLE_HAZARDS.md](PEOPLE_HAZARDS.md) for exact equations and profile allowances.

## Output and performance semantics

`path_xyz` is null or begins at `current_xyz` and ends at `goal_xyz`. The path and
current connector are checked against the full current physical/dynamic world.
`path_status` explains candidate_path, no_connected_path, blocked current/goal,
or validation failure. `map` contains exact map identity and revision.
`path_id`, goal_revision, session_id and tree_instance_id identify the result.
`execution_authorized` is always false. `path_all_known_free` is only a spatial
classification diagnostic; it checks neither freshness nor partner motion.
The root execution guard must check current revision, age, operating bounds,
fresh free evidence and current measured partner envelopes on the next segment.

`tree_nodes` are `{id,xyz,tree_id,parent}` with MAIN=0, PRUNED=-1 and positive
forest IDs. `tree_edges` are real currently valid `[child_id,parent_id]` pairs.
Use these edges directly for visualization. A detected invalid parent edge
returns no path and `tree_invariant_failed`. With include_tree=false, the arrays
are empty and tree_exported=false, but invariant checking remains active.

`dynamic_changed` means geometric occupancy changed, while
`dynamic_metadata_changed` includes lease updates and `dynamic_ttl_only_refresh`
identifies a metadata-only renewal. `prune_scan_triggered`,
`map_clear_regions_queued` and `dynamic_clear_regions_queued` distinguish new work
from maintenance of older queued work. A TTL-only update can still drain an
already pending recovery queue or repair an actual-current connector; such work
is not attributed to the renewal.

Metrics include actual node counts, cuts/restores/coverage work, reroot remap and
retained counts, native status, cumulative collision counters and total wall
milliseconds. The repair budget is a soft bound around native primitives.
Full pruning, reroot, path finalization, whole-tree validation and output are
outside that budget; total_wall_ms exposes these costs. No hard realtime claim.
`state` reads the last measured current, not a newly observed pose. JSON parse
errors are responses; an unexpected native panic invalidates the session.

## Validation

`test_results.log` records the locked offline test run: 81 retained core tests
plus 13 new tests. New cases cover true-Z clear coverage, finite rejection
sampling, 2D/3D current connectors, goal reroot/retained IDs and edges, obstacle
addition/clear after reroot, blocked goals, transactional revision rejection,
partner lease renewal without geometric changes, moves/resizes/expiry with real recovery,
signed keys, thin diagonal/body collision, floor/low obstacles,
unknown versus free and compact timestamp refresh. These are component tests,
not Factory FPS or closed-loop simulator acceptance.
