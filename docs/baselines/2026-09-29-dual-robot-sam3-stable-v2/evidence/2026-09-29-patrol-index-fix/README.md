# Dual-robot cyclic patrol neighbor-index crash and tree display continuity fix

Date: 2026-09-29. Scope is `/data/robotics/air_ground_smart3d` only. The 640×360 cameras, SAM3 total 2 Hz, and original map/sensor/speed/tree-repair parameters are kept; the frequency adjustments and map algorithm optimizations from the performance plan were not implemented.

## Root cause and actual evidence

When the user reported the Go2 tree suddenly disappearing, the running epoch was `1ddaeeef8d1848909e9161e65dbba5ad`. The Go2 had completed 124 patrol points, goal version 125, `planner={}`, status waiting_path. Native stderr was Kiddo 5.0.3 `construction.rs:223`:

```
Too many items with the same position on one axis.
```

SMART keeps the old root node when changing the goal root. Long-term four-corner patrol keeps producing same-coordinate nodes; the original neighbor index's fixed 32-item leaves and cyclic splitting axes cannot accommodate such degenerate distributions. The error triggers when an update appends to the index, not necessarily immediately on every set_goal.

After catching the panic, the native layer drops the internal session and returns `native_panic_session_invalidated`. The Python layer treats it as an unrecoverable contract error, clears the planner, and ends that robot's planning thread. The shared map, LiDAR, and UE keep running; the problem is not GPU exhaustion or RViz failing to draw.

## Fix contents

1. `planning/vendor/smart3d_observed/src/rrt/neighbors.rs`: exact adaptive-axis Kd index. Splits only across dimensions with coordinate spread, between distinct values; fully coincident coordinates go into dynamic leaves, keeping all node IDs. No random coordinate jitter, no dropping duplicate nodes, and not just raising 32 to wait for the next overflow. Nearest-k and radius queries were checked against naive exact distance; the radius boundary includes equal points.
2. `sim_io/runtime.py`: reinitialize that robot's planning session only on an explicit `native_panic_session_invalidated`. Immediately revoke the old path, use the latest map snapshot, current position, and current goal, with an independent session generation and bounded backoff; no clear/load/restart of the shared mapper. Other contract errors are still reported; bad data is not masked by endless retries. Both pre-execution identity checks use the currently active session, preventing rejection by stale session names after recovery.
3. Goal switching only revokes the old path and keeps the still-valid real tree. When the goal is temporarily blocked, the native update keeps validating the old tree against the latest map and then displays that tree; the old path is null, and the goal-version gate does not allow executing the old goal. Panic or illegal results still clear the invalid tree; a cache must not be used to pretend the planner is healthy.
4. `sim_io/restart_state.py` and the launch entry point: support explicit `--resume-map ... --resume-state ...`. Only accepts states from the same scene, same profiles, same robot identities, and already stopped; positions/headings/counts must be valid, and the goal must uniquely match the original patrol route. Both robots' positions, orientations, goals, and cumulative progress are kept, and a valid tree and path are regenerated. Old free space still needs new sensor evidence; old motion commands are not restored.

The original LINKS source is recorded elsewhere; this change is only in the dual-robot project's vendored copy, and `planning/CORE_PROVENANCE.json` has been supplemented with the neighbor-fix provenance. Town10, Factory, and the old single-Go2 empty-scene source were not modified.

## Verification

- Rust workspace: 104 tests pass. Among them, 160 repeated four-corner goal switches in both 2D and 3D: always the same tree instance, original nodes never lost, valid paths, no illegal tree edges; also 512 duplicate/collinear/coplanar points, mixed 1024 points, empty index, and radius boundaries.
- Final Python: 330 tests pass. Covers explicit panic recovery, panic during blocked-goal recovery, a valid non-empty path after recovery actually being executable, stale sessions being rejected, tree retention on goal change / old-path revocation, snapshot restore, and existing map/person/cooperative-avoidance contracts.
- Deployment used the stopped snapshot to save the map and both robots' poses/patrol progress. There were several staged runtime deployments during this period, with UE and RViz processes kept running; deployment times and restored positions are in the checkpoint JSONs. The first map save contained 970511 observed voxels, fully retained on restore; observation accumulation continued afterward.
- About 44.9 seconds and 124 status samples after the tree-display patch deployed: the Go2 advanced 35.25 m and completed 2 goals; the UAV advanced 18.35 m and completed 1 goal. Empty-tree samples were 0 for both robots and planning errors were 0. Encountering people/geometric obstacles may still cause temporary waits under the original checks.
- In the same period, real ROS tree markers in RViz received 70 messages over about 35 seconds: 70 ADDs and 0 DELETEs for each main tree; maximum interval 1.05 s, below the 3-second marker lifetime. This verifies display continuity and does not treat the drawn tree as extra execution permission.
- After the final session-gate patch deployed, actual displacement, trees, and perception were independently verified again; results are in `final-deployment-live.json`. The above short online verification and long-term repeated-goal regression cannot replace hours of stress testing, nor do they mean all planner anomalies are eliminated.

## Boundaries of the speed-up work

The follow-up plan is in `docs/plans/2026-09-29-performance-followup.md`. Priorities: separate high-frequency state from full-tree snapshots, then compute diffs of incremental keys only, then evaluate map copying and the clearing queue. 640×480 was reverted per the user's request; the camera stays at 640×360, and neither capture frequency nor SAM3 total 2→3 Hz was implemented.

One 35-second sample after an early recovery showed both LiDARs entering the map at about 3.86 Hz each, but the rebuilt tree size and person positions differed from before the fix, so this number must not be taken as a performance-optimization gain. This round mainly fixed correctness and recovery behavior.

## Explicit restore procedure

Restore the map and both robots' patrol progress from a stopped compatible run snapshot:

```bash
cd /data/robotics/air_ground_smart3d
./lab start --rviz \
  --resume-map /absolute/stopped/run/shared/map.ot \
  --resume-state /absolute/stopped/run/state.json
```

`--resume-state` must be paired with `--resume-map`; the adjacent `resolved_profiles.json` must exist. The default `./lab start --rviz` behavior is unchanged. Source identity, error logs, run checks, and ROS marker evidence for this round are in this directory.
