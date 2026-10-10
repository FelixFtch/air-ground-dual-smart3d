# Shared physical occupancy service

This is a single C++ authority backed by **OctoMap 1.9.8 `octomap::OcTree`**.
It is an independent offline-capable component; it starts no ROS or simulator.
Python's `MapCache` is a versioned consumer copy, not a second occupancy mapper.

Build from the project root:

```sh
cmake -S mapping -B mapping/build -DCMAKE_BUILD_TYPE=Release
cmake --build mapping/build -j2
OCTOMAP_SERVICE_BIN="$PWD/mapping/build/octomap_service" python3 -m unittest discover -s tests -p 'test_mapping*' -v
```

Dependencies are existing system CMake/C++17, `/opt/ros/humble` OctoMap,
nlohmann/json 3.10.5, and OpenSSL libcrypto for SHA-256. No downloads are needed.
The executable reads one request per stdin line and emits one response per stdout
line; diagnostics from OctoMap use stderr. The optional `request_id` is echoed.
One bridge owns one executable and serializes all sensor requests with
`mapping.client.OctomapClient`. No per-robot service is needed.

## Ordered live application pipeline

Native requests to the single authority remain serialized. The live bridge may
prepare and request the next latest sensor frame while one previous delta is
being applied to `MapCache` and the incremental display index. There is one
apply worker and at most one submitted apply: before submitting a new delta,
the producer drains the previous future. This is overlap, not an old-frame FIFO.

Capture identity, timestamp, camera transform and timing metadata are bound to
the individual frame. The existing map lock and exact base-revision checks still
guard each cache commit. Sensor freshness and fused counters advance only after
the corresponding application succeeds. A planner response ahead of the cache
revision cannot authorize movement. Shutdown and preparation/RPC failures drain
submitted work; application failure stops fusion rather than skipping a delta.

`apply_queue_s` measures response-to-worker-entry delay; `cache_apply_s` measures
worker entry through the cache/index commit, including map-lock wait.
`arrival_to_apply_wall_s` remains the end-to-end sensor-arrival-to-commit age.
The latest measurements and limitations are recorded in the
[real-time verification report](../docs/verification/2026-09-29-octomap-realtime.md);
target rates must not be read as achieved rates.

## Requests

Initialize exactly once. A new episode requires a new process and epoch:

```json
{"op":"init","scene_id":"lab","scene_epoch":"run-20260929-a","content_version":"lab-v1","frame_id":"map","resolution":0.25,"bounds":{"min":[-40,-40,-2],"max":[40,40,10]},"out_of_order_window_s":0.25,"history_limit":128}
```

Submit capture-time **map-frame** endpoints and the matching map-frame sensor
origin. The bridge applies the captured extrinsic/pose transform before IPC.
The map service never substitutes the most recent robot pose. All points must be
actual finite hits; remove invalid returns and the self/cooperating-robot body
geometry upstream. Human obstacle measurements remain in the physical map.

```json
{"op":"integrate","scene_id":"lab","scene_epoch":"run-20260929-a","robot_id":"go2_01","sensor_id":"lidar","frame_seq":12,"capture_sim_time":1.2,"receive_wall_time":1790654400.0,"calibration_id":"go2-lidar-v1","origin":[0.125,0.125,1.125],"points":[[2.125,0.125,1.125]],"max_range_m":12.0}
```

Optional finite depth free-only rays use the same capture origin, range and frame
identity. They clear the traversal and endpoint without marking a hit:

```json
{"op":"integrate","scene_id":"lab","scene_epoch":"run-20260929-a","robot_id":"uav_01","sensor_id":"depth","frame_seq":12,"capture_sim_time":1.2,"receive_wall_time":1790654400.0,"calibration_id":"uav-depth-v1","origin":[0.125,0.125,2.125],"points":[],"max_range_m":12.0,"free_rays":[{"endpoint":[12.125,0.125,2.125],"source":"registered_depth_measured_beyond_range"}]}
```

Accepted sources are `registered_depth_measured_beyond_range`,
`lidar_measured_beyond_range`, and the explicit simulator-only
`carla_depth_far_plane`. `mapping.client.FREE_RAY_SOURCES` exposes these names.
For `lidar_measured_beyond_range`, the bridge must have a **real finite measured
hit** (the initial profile uses hits 12–30 m away) and explicitly crop its ray to
the 12 m integration range. For example, a measured hit 20 m along +X from
`origin:[0,0,2]` permits `{"endpoint":[12,0,2],"source":"lidar_measured_beyond_range"}`.
The terminal at 12 m is free, not occupied; the unseen interval beyond it is
unchanged. This is a provenance assertion by the bridge: the service does not
receive raw depth images or the original out-of-range LiDAR hit to independently
prove it. Missing LiDAR returns never create free rays. Observations outside the
configured range or bounds are ignored; they are not silently clipped into free
observations. Valid hits stop the ray at their surfaces and do not clear behind.
The protocol examples below use a 12 m range. The current live profiles use
9 m LiDAR marking and measured free-ray prefixes up to 40 m, at 0.25 m
resolution. A farther actual hit contributes a clearing prefix with
`include_endpoint:false`, never a fake near-range obstacle or erased surface.

Complete LiDAR observations also include original `protected_hit_points` with
an explicit `mark_range_m`. Their real measured endpoints are excluded from all
free rays in that frame. For cells previously marked by an accepted in-range
hit, a real far hit adds ordinary positive log-odds evidence and refreshes its
capture time, including recovery from free to occupied. Far returns do not create
occupied cells that have only been observed free or never marked. `points` are
also bounded by `mark_range_m` when supplied. Thus the live policy is new marking
within 9 m, but both positive and negative updates to known surfaces within 40 m.
No permanent lock or time-to-live is added; later measured free rays still clear
a departed object. This fixes asymmetric evidence, not subvoxel thin-surface
aliasing: a hit and a different ray through empty space in the same 25 cm cell
can still conflict across frames.

`accepted_hits` counts in-range hit cells. `protected_hit_cells` counts real far
endpoint cells; `refreshed_protected_hits` counts accepted far positive updates
to previously marked cells, and `recovered_far_hit_cells` counts their transitions
to occupied. The latter two counters exclude older evidence rejected per cell.

Other operations:

```json
{"op":"snapshot"}
{"op":"changes","since_revision":1}
{"op":"query","points":[[1.125,0.125,1.125],[2.125,0.125,1.125],[3.125,0.125,1.125]]}
{"op":"status"}
{"op":"save","path":"/absolute/runtime/shared/map.ot"}
{"op":"load","path":"/absolute/runtime/shared/map.ot"}
{"op":"forget","scene_id":"lab","scene_epoch":"run-20260929-a","keys":[[8,0,4]],"reason":"explicit operator region removal"}
```

`forget` is explicit maintenance, not automatic aging. It removes nodes to
**unknown**, never free. No decay or rolling crop is enabled. Occupied nodes are
cleared by actual miss evidence and may need repeated independent frames to
overcome accumulated probability.

## Responses and cache contract

Every successful response includes `ok`, `schema`, `scene_id`, `scene_epoch`,
`content_version`, `frame_id`, `resolution`, `revision`, and
`capture_sim_time_watermark`. `snapshot` and `query` add `cells`. `integrate`,
`changes` and `forget` add `base_revision` and `changed_cells`, with no extra nested
wrapper. Example cell:

```json
{"key":[8,0,4],"state":"occupied","log_odds":0.8472978,"last_observed_sim_time":1.2}
```

For bounded IPC bandwidth, opt in with `compact:true` on `integrate`, `changes`
or `forget`. `changed_cells` then contains only new cells and occupancy-class
transitions. Same-state observations appear in `refreshes`:

```json
{"capture_sim_time":1.2,"keys":[[0,0,4],[1,0,4]]}
```

`refreshes` is a list of these groups. Merged deltas retain the newest timestamp
per cell; unknown deletions still appear as full changed cells. `MapCache.apply`
supports both formats. The planner can ignore refreshes for candidate collision
checks, while the execution cache uses them to preserve free-space freshness.
The authority always updates full probabilities in OcTree. A compact consumer's
`log_odds` is only the last transmitted value, **not current probability**;
request a full snapshot/query or read the saved `.ot` for current probabilities.
Default responses and all snapshots keep the original full-cell format.

The live bridge uses `compact:true, refresh_encoding:"i16le_base64"` for
`integrate`. Only refresh coordinates are packed as signed little-endian 16-bit
XYZ triples, then Base64 encoded (`keys_i16le_b64`, `key_count`). Signed OctoMap
keys are exactly within this range; no resolution or coordinate precision is
lost. Every group still carries its full capture timestamp. `MapCache` validates
length, encoding, bounded expansion, membership and time before applying any
changes. Default key-array responses remain available. This removes thousands
of temporary JSON arrays per scan; it does not discard observations.

Planner `changes` requests use `compact:true, refresh_semantics:"presence_only"`.
This view sends occupancy changes and the first current-epoch observation of a
restored cell, without repeated same-state timestamp refreshes. Both native
planners only need this distinction. The execution cache rejects this reduced
view and continues receiving full freshness through packed integrate responses.

Inner occupancy probabilities are updated only along ancestors of leaves whose
actual log-odds changed. A large update (more than 4096 changed leaves) uses the
original full-tree propagation. Leaves remain at the configured resolution;
automatic OctoMap pruning is not enabled. Full `.ot` byte parity, snapshot and
restoration checks are recorded in the real-time verification report.

Keys are signed `floor(map_coordinate / resolution)` indices, cell center is
`(key + 0.5) * resolution`. No origin shift is needed. Missing snapshot cells are
unknown. Query/deletion cells can explicitly have `state:"unknown"` with null
log-odds/time. Cell states use OctoMap's probability threshold (default 0.5).
All geometry remains uninflated; body and support rules belong to each robot's
planning view. Root's ground projection consumes the same physical cells.

`MapCache.apply` or the worker requires matching map identity and exact
`base_revision`. Missing/expired history returns `error:"snapshot_required"`;
fetch a fresh snapshot without resetting the map or other robots. Initial map
revision is 0. Every touched scan produces a revision including observation
freshness, even if its classification stays unchanged. A duplicate or empty scan
does not increase it. A `changes` request merges all touched cells since its base
into their latest state, including deletion-to-unknown. History stores compact
keys, not repeated full JSON snapshots.

Per-frame hit and miss cells are deduplicated; hits win conflicts inside the same
frame. Default probabilities are hit 0.7/miss 0.4, clamped to [0.12, 0.97]. The
unique observation key is epoch/robot/sensor/frame_seq. The last 4097 sequence
numbers per stream are deduplicated; older sequences are rejected, never treated
as new. The global maximum capture time allows 0.25 s out-of-order input by
default. Accepted reordering adds evidence but cannot make freshness regress.
These log-odds updates accumulate evidence, not last-message-wins occupancy.

## Persistence and validation boundary

Saving writes complete `.ot` probabilities plus `.ot.metadata.json`, with SHA-256,
scene/content/frame/resolution, source config, observation metadata and
`known_near_hit_keys` marking provenance. The latter includes previously marked
cells that are currently free, so later far measurements can restore them. It
never includes a cell solely because a free ray observed it. Explicit `forget`
removes this provenance as well as occupancy. When loading an older snapshot
without this field, only its currently occupied cells initialize the provenance;
old free cells need a fresh in-range hit before far-positive recovery is enabled.
Malformed provenance is rejected before the in-memory map is replaced. The pair
is staged to temporary files and renamed; a crash between renames is detected by
checksum on load. A restore requires an empty process, matching scene/content/
frame/resolution and **a new epoch**. Restored probabilities survive; old sensor
timestamps are deliberately absent from the new epoch's cells until reobserved,
so they cannot grant fresh near-term execution. Restore returns a full snapshot
at revision 1. Historical dynamic constraints and observation dedupe are not
restored. The bridge excludes the two cooperating robot bodies. People still enter the
physical occupancy map through measured rays and are cleared by subsequent free
evidence; SAM3/OHZ is an additional temporary constraint. Restoring a saved map
can therefore retain old person occupancy until measured clearing rays revisit it.

Meaningful tests exercise the actual C++ executable and linked OctoMap: capture
origin, duplicate and invalid-frame atomicity, same-frame hit priority, shared
sensor evidence, free/occupied/unknown, occlusion, real miss clearing, explicit
depth rays, out-of-order rejection/freshness, revision gaps, unknown deletion,
probability-preserving restore, stale-time invalidation and checksum rejection.
These are offline behavior checks, not a claim of live multi-robot throughput or
simulator acceptance. Later live acceptance is recorded separately in the
real-time verification report.

Offline stress measurement on this workstation: 1,500 synthetic directions over
a full 12 m sphere touched 87,194 cells. Compact first-frame request took 253 ms;
five steady requests had median 94 ms, plus median 24 ms Python cache application.
This is a deliberately broad synthetic geometry, not measured CARLA throughput.
That initial stress test is not a live acceptance result. Current profiles and
later shared-map measurements are documented under `docs/verification`; compare
per-source fusion counts and capture age rather than publication rate alone.
