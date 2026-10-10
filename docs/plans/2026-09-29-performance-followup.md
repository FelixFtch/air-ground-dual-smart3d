# Dual-Robot SMART3D Performance Follow-Up Plan

Date: 2026-09-29. Status update: the user has authorized and, as of today, deployed the state/tree separation and changed-key diff; also fixed the OHZ display waiting on stale planning results and far-range same-frame rays wrongly clearing cells. See [deployment verification](../verification/2026-09-29-performance-ohz-farhit/README.md). The map chunking, clearing-work queue, and sampling-rate experiments below are still follow-up plans, not deployed; keep 640×360 and SAM3 total 2 Hz.

## 1. Established Costs and Statistical Boundaries

Reference `docs/baselines/2026-09-29-dual-robot-sam3-stable/`: about 50 seconds of sampling, roughly 965k observed voxels in the map, dual LiDAR actual ingest at 3.07/3.09 Hz against a configured target of 5 Hz each; latest fused-LiDAR age P95 about 0.675/0.783 seconds. In a separate ~20-second window with one distinct planning result sampled per second, full Go2/UAV planning averaged about 379/989 ms. This is not an unbiased distribution over all planning calls and cannot be directly attributed to any single stage.

Source-code findings:

- `planning/src/voxel.rs:167` copies the entire `HashMap` on every increment before `diff` walks the new map and then walks the old map again; the input is already incremental, yet processing still does whole-map work.
- `planning/src/lib.rs` full-tree pruning, clearing maintenance, final path validation, and whole-tree edge validation are not all counted inside the 50 ms partial tree-repair budget. `include_tree=false` **still performs full tree-edge collision validation**, only skipping node/edge JSON construction — this cannot be claimed to have solved planning latency.
- `sim_io/runtime.py` issues every planning request with `include_tree=true`, and the 5 Hz state file contains the full tree; `observability/ros_view.py` already publishes the tree at 2 Hz, but the tree signature includes `received_wall/path_id`, so every planning reply invalidates the tree cache.
- This is a read-only instantaneous sample: the Go2 planner was still in failure recovery, leaving only the UAV tree of 7,985 nodes/7,880 edges; `state.json` was 889,343 B, of which the UAV tree occupied 868,391 B. The equivalent compact state with the tree array removed is about 20,983 B, a reduction of about 97.6%. This proves the redundant transfer exists; it does not mean 97.6% of total runtime can be saved.
- Clearing regions are deduplicated only by exact float equality of center/radius; every cleared voxel enqueues a recovery region. The Go2 2D `clear_center` drops Z yet still receives clearing events at all heights. The native recovery queue has no such 256 cap; the 256 cap belongs to a different re-sampling queue. The two must not be conflated, nor may queue tails simply be dropped.
- RGB-D is generated per simulation frame with callbacks at about 20.6 Hz here; depth is actually consumed at about 1.2 Hz/robot and SAM3 at about 0.9–1 Hz/robot. SAM3's current total cap is 2 Hz; one round including peripheral processing averages about 323/338 ms, so 3 Hz is already close to the average processing budget.

## 2. Do First: Separate State from Full Tree, Keep All Collision Validation

This is the first low-risk optimization: **no changes to the SMART3D algorithm, the map, or input rates; do not disable tree export or collision checks** — first remove the duplicated tree bytes from the 5 Hz state.

### 2.1 Data Contract

The high-frequency `state.json` keeps providing pose, current path, goal, health, map/people/dynamic revisions, and planning timings; each robot gains a structured reference, for example:

```json
{
  "tree_ref": {
    "schema": "air_ground.tree_ref/v1",
    "scene_epoch": "...",
    "robot_id": "go2_01",
    "session_id": "...",
    "tree_instance_id": 1,
    "geometry_revision": 42,
    "relative_path": "trees/go2_01/42.json",
    "sha256": "...",
    "validated_map_revision": 123,
    "validated_dynamic_revision": 77,
    "validated_people_revision": 18,
    "validated_sim_time": 150.0,
    "status": "validated"
  }
}
```

A separate immutable file stores only the actually exported nodes, parent-child edges, and the scene/robot/session/tree-instance/geometry identity. The geometry revision advances only when node coordinates, parent-child relations, tree_id, or actually visible edges change; it can first be implemented from a fingerprint of the canonicalized export content, with no need to guess whether the underlying tree was modified. When the map revision updates but the geometry content is identical, reuse the same geometry file; new validation receipts stay in the high-frequency state reference.

Write order: receive and check the successful native planning result → write the full tree to a temporary file and atomically replace the immutable versioned file → publish the state pointing to it. Never publish the reference before the file exists. Immutable files have a small retention window; garbage collection must not delete the currently referenced version, nor versions still inside a read grace period. On version race/missing file, re-read the latest state and retry; never treat half a tree as a valid tree.

A reference is not execution authority. Execution keeps using the current native path, current map, and live execution checks; an RViz snapshot must not become a control input.

### 2.2 RViz Compatibility and No Flicker

- `AtomicJsonCache` caches the high-frequency state and the immutable tree files separately. Support reading the old embedded-tree format first, then switch the runtime to the new reference contract, so deployment ordering cannot produce an empty tree.
- Validate that epoch, robot_id, session_id, tree_instance_id, geometry_revision, and the file hash all match; paths must stay inside that run's tree directory.
- Tree publishing stays at 2 Hz; identical geometry revisions reuse the already-built MarkerArray, **keeping the 2 Hz ADD heartbeat**, so the 3-second Marker lifetime does not expire just because the tree geometry is unchanged. Pose/path keep the existing 10 Hz polling and the state source stays 5 Hz; do not fabricate 10 Hz localization data that does not exist.
- A new version deletes only the marker IDs that actually disappeared; per-round DELETEALL is forbidden. When epoch/session becomes invalid or a native panic definitively loses the valid tree, explicitly mark it invalid and remove the corresponding display — never keep the old tree forever pretending to have recovered.
- A transient read race must not publish an empty tree: retry the same complete version. Only invalidate on a confirmed invalid contract, scene change, or exceeding the health deadline, with the health deadline coordinated with the existing 3-second planning freshness.
- The map updating faster than the tree is an existing asynchronous fact; the display should keep the validated version and age, and must not claim the cached tree is validated against map updates the planner has not seen. Native final path validation and execution collision checks remain unchanged.

### 2.3 Acceptance

Run at least 10 seconds continuously with no geometry change, with the 2 Hz tree heartbeat unbroken and zero non-initialization DELETEALLs; edge deletion/reroot/new session/epoch switches all correctly remove the old display. With the tree file not yet on disk, a wrong hash, a stale state reference, or a worker failure, no tree from another robot or an old session may leak in. Comparing identical native responses before and after the optimization, the `actual_tree_edges` set matches, and path and collision-check counts match.

Measure state-file bytes, bytes written per second, JSON encode/decode time, publication P95, RViz callback P95, and process CPU. The benefit target is a significant drop in these metrics; smaller files alone do not establish that planning time has improved.

## 3. Second Step: Increment Classification Removes the Full-Map Diff, Keep the Clone for Now

After `Map::delta` fully parses and validates all changed/refresh inputs, build `Changes` using only the old and new values of this round's changed keys: free/unknown→occupied is added; occupied→free is cleared; occupied→unknown is expired; time refreshes produce no geometry change. Keep the sorting contract, duplicate-key rejection, revision/identity checks, and the validate-then-commit transaction semantics. Keep the full clone for now, to avoid changing storage, transactions, and collision queries all at once.

Add stage timing: JSON parsing, delta parse/clone/apply/diff, prune, clear maintenance, coverage, reroot/update, final path validation, whole-tree validation, JSON construction/encoding. The current `total_wall_ms` is recorded before the native return object is complete; the outer `dump/write` time must be recorded separately; runtime round-trip latency can serve as an end-to-end metric.

Acceptance: with randomly generated snapshots/increments, compare final cells and added/cleared/expired item by item against the original `clone+full diff` oracle; cover unknown-withdrawal, first observation after restore, empty delta, refresh-only, duplicate keys, expired timestamps, revision gaps, illegal refresh, and failure leaving the original map unchanged. Do release offline benchmarks at a million voxels with tens/thousands of changed keys, recording per-stage P50/P95 and allocations. Under the same map/dynamic constraints, the final path and every exported visible edge must still pass full collision validation.

## 4. Only Later: Map Copying, Clearing Queue, Whole-Tree Recheck

### 4.1 If the Clone Really Dominates Cost

The current `Arc<Map>` is held by the native checker, so a plain `Arc::make_mut` can still trigger a full copy; do not treat this as solved. A chunked immutable store could be compared, copying only changed chunks, with old checkers seeing the old snapshot and failed batch validation not committed. The impact of the added indirection on high-frequency collision queries must be measured; do not make every voxel query slower just to save the clone. Without measured significant benefit, keep the previous step.

### 4.2 Enqueue Clearing Work by the Region Possibly Affecting Each Robot

First do precise relevance filtering: a voxel AABB must possibly intersect that robot's legal center range plus body envelope; Go2 fixed-Z uses `planar_z + body_offset ± body_half_extent`, UAV uses the flight altitude range plus body envelope. Use the same boundary tolerance as the existing collision checks. Only skip maintenance that cannot affect this planner; **voxels of the shared map are not deleted**.

Only in a second stage conservatively merge nearby clear regions into fixed spatial chunks: the merged query range must cover the union of the original regions, keeping the latest dirty revision; chunks that change again while being processed must not lose work. Do not just round the center to a grid and reuse the old radius, or edges of old trees will be missed. The existing deterministic coverage sampling points depend on region center/radius; recovery queries and re-sampling must not be changed together without verification. Near-robot/current-path work goes first, but far-away work must have fair rotation and never starve permanently.

Acceptance: historical-edge recovery latency P50/P95 after obstacle removal, whether pending regions stay bounded over time, region/node scan counts, new node counts, current-path stability; all existing static_forest_tests pass, plus coverage of high-altitude clearing not affecting Go2, Go2's real altitude band still recovering, voxels at legal flight altitudes still affecting the UAV, boundary-region merging missing nothing, and blocked old edges never reviving while the blocker persists. Do not hide queue backlog by enlarging the per-round budget.

### 4.3 Whole-Tree Edge Validation Cache Is an Even Later Optimization

The final execution path keeps being validated each round with the full latest checker. To reduce whole-tree scans, an edge AABB/spatial-chunk dependency index with invalidation of affected edges must be built; new/rewired edges, goal reroots, unknown→occupied, and dynamic robot/people geometry changes must all be covered. Candidate filtering can only be a miss-free broad phase; exact collision checks must not be canceled. Visible edges must carry validation evidence under current geometry constraints; against a full-scan oracle across random map insert/delete, people movement, and long edges crossing obstacles, no false negative may appear. Not doing this in the first small step.

## 5. Split Camera and SAM3 into Two Independent Experiments

The user's latest decision: keep 640×360 and do not switch to 640×480, to avoid increasing SAM3 load. The 640×480 resolution comparison is withdrawn and will not be run. The 10 Hz capture and SAM3 total 2→3 Hz below remain only as optional follow-up experiment proposals; no rate change has been approved or executed this round; the current priority is the root cause of tree disappearance.

Experiment A: only change both robots' forward RGB and registered depth `sensor_tick` from 0 to 0.1 simultaneously; do not change resolution, camera mounting, LiDAR, auxiliary depth, mapping consumption rates, or SAM3 total 2 Hz. The same value does not automatically guarantee RGB/depth same-frame pairing — the existing same-frame_seq and capture-pose checks must keep rejecting mismatches, and the actual pairing rate must be measured. Sensor creation parameters generally require rebuilding the corresponding sensors/this experiment's instance; do it first in a controlled window with a map snapshot and restore point, and never accidentally wipe the current map for an experiment.

Experiment B: fixing experiment A's winning configuration or reverting to the original one, change only SAM3 total 2→3 Hz; the single-model two-session fair rotation stays. The current ~0.33 s average full processing is already close to the 0.333 s per round, so 3 Hz may merely remove margin and widen tail latency. Do not raise input resolution or tracked-object counts at the same time.

Pre-warm each candidate, then fix pedestrian routes/speeds/robot routes for an A-B-A comparison, each segment at least 60–120 seconds, repeated multiple times. Record actual, not configured:

- UE simulation/wall-clock ratio, RGB/depth actual callback Hz, same-frame pairing failure rate, latest frame capture age P95/P99.
- SAM3 actual processing Hz per source, fairness, full-processing P95, queue wait, geometry TTL expiry/rejection share, time to establish valid velocity, and OHZ effective coverage.
- Dual LiDAR actual fused Hz and latest-evidence age P95/P99, full planning call stage breakdown P95/P99, control RPC latency, sensor_stale/planner_stale counts.
- GPU compute/VRAM peaks, CPU and RSS, real driving progress, closest distance, and collision/blocking counts. UE pedestrian ground truth, if used, is offline scoring only and must not enter estimators.

Retention conditions: no functional contract regression, no drop in OHZ continuity, no significant worsening of LiDAR/planning tail latency, and a reproducible gain in resources or end-to-end latency. If 3 Hz stays overloaded, keep 2 Hz; a separate 2.5 Hz trial can come next — do not pass configured numbers off as speedups. If camera 10 Hz yields no gain, revert to 0 and keep the currently user-approved visual quality.

## 6. Explicit Implementation Order

1. Independently finish the current panic correctness fix and confirm both robots' motion/tree recovery, preserving the map.
2. State/tree file separation with version caching, with native full validation fully preserved.
3. Stage timing + changed-key diff (clone kept), deployed after offline equivalence verification.
4. Depending on measured cost, choose map chunked copying or clearing relevance filtering/merging — not both major changes at once.
5. Finally A/B camera 10 Hz and SAM3 total 3 Hz separately.

For each step, keep source-file checksums, independent tests, and same-condition performance data. Do not modify the old Town10, Factory, or single-robot empty-scene projects; do not raise robot driving speed; do not shrink the 9/40 m perception range; do not reduce voxel/tree density; do not weaken real body and transient peer collision avoidance.
