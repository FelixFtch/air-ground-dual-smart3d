# Dual-robot OctoMap real-time performance and near-marking / far-clearing acceptance

Date: 2026-09-29. Current running epoch: `1ddaeeef8d1848909e9161e65dbba5ad`.

## Distance and clearing rules

- Both the Go2's and the UAV's LiDARs use **9 m obstacle marking, 40 m measured-ray clearing, 45 m sensor range**.
- References the original single-robot 9/40 m configuration in `configs/car1_nav2_semantic.yaml`.
- Each clearing ray excludes its own real hit endpoint and does not extend behind the echo; space without echoes is not erased out of thin air. Adjacent far-end rays sharing coarse voxels can still affect each other; only hits within 9 m enjoy full-frame hit-wins protection.
- The current zero-noise simulation probability parameters allow a single new valid hit/free observation to flip occupancy; cross-sensor capture-time ordering and same-time hit priority are still kept.
- Forward depth is still 6 m, and the UAV's six-direction close-range blind-spot coverage is still 3 m. This round did not change all depth to 40 m.

## Actual run results

The following is computed from each sensor's actual `fused` count; RViz publish frequency is not taken as map fusion frequency.
The latest fused-frame age includes the sampling period, fusion, and state-publish lag; it is not the clearing delay after a specific person left.
Baseline and this round differ in map size and driving positions; these are actual run checks, not a strictly controlled A/B experiment.

| LiDAR | Measured fusion Hz at start | Measured fusion Hz this round | Latest fused-frame mean age s: before→after | This round P95 age s |
|---|---:|---:|---:|---:|
| go2_01 | 1.704 | 3.286 | 0.959 → 0.377 | 0.663 |
| uav_01 | 1.671 | 3.286 | 1.032 → 0.455 | 0.723 |

This round sampled 49.9 seconds. The configured target is 5 Hz per robot; whether it was met must be judged from the table above.
Mean display data age 0.066 seconds.
Map/main-run anomalies: `[]`.
Actual motion and reached-goal counts during the period are in `progress_during_sample` of `*-summary.json`.

## Root causes and implementation

The synchronous CARLA motion RPC held the Python GIL while waiting on UE; a separate restricted motion subprocess now waits and returns the actual collision receipt.
Map display no longer scans the full map and copies all coordinates every round; the immutable geometry snapshot updates only when membership changes.
The planner receives only occupancy changes and this run's first observations, while the execution cache still receives full observation timestamps.
Execution refresh coordinates use lossless signed 16-bit XYZ encoding, reducing large JSON arrays and garbage collection.
OctoMap internal probabilities propagate only along ancestors of actually changed leaves; large batches fall back to the original whole-tree propagation, keeping fixed-resolution leaves.
Already-found saturated leaves and timestamp iterators are reused directly, avoiding repeated lookups; the original OctoMap probabilities and saved data stay consistent.
Native fusion of the next frame can overlap the previous frame's cache application; only one cache application is in flight, commits are strictly ordered, and freshness advances only after the cache update completes.
The fast JSON dependency is isolated inside this project, falling back to the standard implementation when absent; the shared Python environment was not modified.

Map resolution was not lowered and close-range obstacle measurement points were not truncated; every real hit voxel keeps a representative measured point.
Latest-frame and bounded processing are used; unbounded backlogs of old frames are not accumulated. Execution checks, map revision validation, and entity collision receipts are kept.
The `.ot` is saved when navigation stops; this round's multiple reloads all resumed from the existing map — latency was not hidden by clearing the map.

## Evidence and limitations

- The deployment candidate's full Python/native-interface tests: 279 passed; the Rust planning algorithm was not modified or re-verified this round.

- `*-live.json`: this round's raw state samples; `*-baseline.json`: starting samples.
- `*-ros.json`: counts from real ROS subscriptions of both robots' point clouds, SAM3, person boxes, OHZ, LRZ, and velocity arrows.
- `*-source-sha256.json`: deployed source and map-service binary identity.
- Same-directory native ancestor, native leaf reuse, packed refresh, JSON codec, display snapshot, planner presence verification records: exact map/timestamp consistency and offline performance.
- Timed deletion was not substituted for real free observation. Old occluded voxels not crossed by new rays can still remain; 40 m is a clearing upper bound, not an unconditional whole-sphere erase.
- No UE pedestrian ground truth was added as input to the person estimator; this round also does not claim to have verified person velocity accuracy or sustained ID association quality.

The change scope is `/data/robotics/air_ground_smart3d`. Town10, Factory, and the original single-robot experiment configurations were not replaced.

An additional offline experiment compared 5 ms vs 1 ms Python thread-switch intervals; the four throughput rounds overlap substantially and did not prove sufficient to close the current 5 Hz gap, so that adjustment was not put online. See `2026-09-29-octomap-thread-interval-experiment.json`.
