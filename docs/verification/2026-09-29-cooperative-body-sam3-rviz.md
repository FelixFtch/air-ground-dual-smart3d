# Dual-robot false stops and RViz display fix

Date: 2026-09-29. Isolated project `/data/robotics/air_ground_smart3d` only.

## Actual root cause

The UAV's SAM3 local frame 156 recognized the white Go2 as a person. The person projection center was about (-8.53,-2.93,0.25), about 0.12 m from the Go2's planar position. Verified with the current actual OHZ collision function: the same misidentification simultaneously blocked the Go2's current position and the UAV goal (-8,-3,2). The two robots respectively reported current_blocked_or_out_of_bounds and goal_blocked_or_out_of_bounds. LiDAR, map, and trees all kept updating.

## Changes

- RGB-D reuses the existing depth callback, saving each robot's body pose and dimensions from the same frame and same capture time; it does not query the current pose to patch old photos and does not read pedestrian ground truth.
- Body data is included in the original photo-package hash verification, with bounded history. When the same frame lacks complete data for both robots, a bounded number of frames is kept for pairing, then the latest complete frame is selected.
- Before fitting position and velocity, SAM3 mask projection removes points that fall inside known robot body volumes. The bounding box adds only a 0.08 m surface margin; external human points in mixed masks are kept.
- If, after body removal, the remaining support points are only a planar fragment with vertical span under 0.05 m, it is not expanded into a human OHZ; this applies only to masks that actually had robot points removed this round, not a global human-height threshold.
- Removed original tracking sources immediately exit the person geometry, and their velocity history is cleared. Overlays are explicitly labeled robot surface excluded.
- Between robots, real-time body-volume temporary constraints are still used. Person filtering does not cancel SMART3D, execution-segment checks, or UE entity collision.
- RViz main-tree line width/transparency is 0.025 m/0.35, broken branches 0.02 m/0.20; only real tree edges are displayed. The first pass cleans up old publisher markers, after that only disappeared markers are deleted, and the heartbeat no longer repeatedly issues DELETEALL.

## Verification

The full test suite passes 317 items, covering robot bodies, mixed real-person masks, leftover floor, cross-frame pose rejection, old-input compatibility, inter-robot physical avoidance, and display marker lifecycle.
A dedicated execution test verified: with the person list empty and the voxel free, the partner's body still blocks forward motion; motion resumes once the partner moves away.

Only RGB-D, SAM3, and the RViz publisher were reloaded; UE, the navigation main program, the map, and both trees were not restarted. The running epoch remained `1ddaeeef8d1848909e9161e65dbba5ad`, both tree instance IDs were 1; at check time the Go2's cumulative patrol goals rose from 91 to 98 and the UAV's from 55 to 58. About 960 thousand observed voxels in the map were retained. The SAM3 service ran, and both new-frame streams carried same-frame body data for both robots; actual person output is allowed to be empty as the field of view changes.

A further 10-second ROS subscription after deployment: 20 tree and 39 map messages, none with periodic DELETEALL; about 7187 Go2 tree edges and 7429 UAV tree edges continuously published, with the new line width and transparency in effect.

User feedback: the current version is smooth. Existing speed, tree parameters, SAM3, and the 9 m marking / 40 m ray clearing configuration are kept.

## Checked but not changed

The UAV entity does return echoes in the raw LiDAR; the Go2's BlockAllDynamic collision configuration does not correctly cover SensorTrace, which is ignored by default. Before fusion, partner body points are still excluded per the existing design, to avoid treating the robots themselves in the shared map as permanent walls. If switching to pure-LiDAR mutual avoidance later, the per-robot self-exclusion view of the shared map and motion ghosts must be handled together; the partner filter and temporary volume constraints must not simply be deleted.

Accumulated ground free space was not wholesale cleared. Some occupied voxels changed due to real-ray occupied/free evidence transitions; this round's fix fixed display clearing flicker and does not disguise dynamic occupancy as permanent accumulation.

Evidence is in `2026-09-29-cooperative-body-evidence/` in the same directory. Frozen overlays confirm the misidentification; original SAM3 masks of old frames were not saved, and offline depth-fragment statistics come from approximate colored masks and bounding-box upper bounds, not from original exact mask measurements.
