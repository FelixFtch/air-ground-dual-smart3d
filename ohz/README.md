# Changing the person hazard shape (OHZ)

The OHZ (the keep-out area the planner avoids around a person) is now a **velocity box**: the person's measured 3D box, inflated by the same margin the planner uses for clustered voxels (0.35 m by default, `cluster_inflation_m`), and stretched forward along the walking direction by half the predicted travel on each side of the shift. A standing person gets just the inflated box. This replaced the ellipse/ellipsoid on 2026-10-10.

The legacy ellipse is still in the code as a rollback: set `people_hazard: {shape_kind: ellipse}` in a robot profile under `profiles/robots/`.

One warning first. This pipeline runs live today. Do not touch the RRT* (the tree-growing path planner) or LRZ code in `planning/` unless the shape interface itself is too limiting.

## Terms

| Term | Meaning |
|---|---|
| Go2 | the robot dog |
| Bee | the drone |
| OHZ / hazard zone | the keep-out area the planner avoids around a person |
| YOLO26 | the people detector |
| MapTracker | our tracker that keeps a person's identity across frames by matching map positions |
| mask | the pixel outline of a person in an image |
| depth | how far each pixel is |
| 3D box | the person's measured extent in the map |
| track ID | the number that marks one person across frames |
| RRT* | the tree-growing path planner |
| pruning | cutting tree branches that enter the hazard zone |

## How the data flows

The chain in one line: `perception/yolo26_backend.py` -> `perception/people_projection.py` -> `perception/hazards.py` -> `planning/src/people.rs`.

Step by step:

1. `perception/yolo26_backend.py`: YOLO26m detects people only. MapTracker assigns each person a track ID, per camera.
2. `perception/people_projection.py`: each mask plus its depth is projected into a 3D box in map coordinates. On the same track ID, a horizontal speed is estimated.
3. `perception/hazards.py`: `build_hazards` folds the person's radius and this robot's radius into one hazard zone. Go2 gets a ground ellipse. Bee gets an ellipsoid.
4. `planning/src/people.rs`: accepts only `ellipse` and `ellipsoid`, and prunes the tree with them. When the person leaves or the hazard expires, the same clearing logic reattaches the pruned branches.

The robot's `dimension` field already splits 2D from 3D. Go2 reads the ground ellipse. Bee reads the ellipsoid.

## The shape, as implemented

**Person standing still** (no reliable speed yet): the measured 3D box plus the unified inflation margin on every side. No elongation.

**Person walking** (speed estimated): the same box, rotated so its long axis points along the walking direction, and stretched forward. The forward stretch grows linearly with speed: `speed x horizon_s`, applied as half the travel added on each side of the shifted center. Default horizon is 1 s, so a person at 1.4 m/s stretches the box 1.4 m ahead. When the person stops, the stretch collapses back to the standing box.

Go2 (2D) reads the box's ground footprint at its plane; the Bee (3D) reads the full box. Z keeps the human-height prior (0.85 m per side) because depth only sees the near surface of a person.

Rules that stay in force:

1. People are never written into the occupancy map. Tree pruning and reattachment keep using the dynamic clearing path.
2. The shape is produced in `perception/hazards.py` (`shape: "box"`) and consumed by `planning/src/people.rs`, which now accepts both `box` and the legacy ellipse family.
3. Rollback to the ellipse: `people_hazard: {shape_kind: ellipse}` in the robot profile.

If you want to try a different shape (for example a ground-only rectangle), add it as a third kind in those two files. Do not quietly bend the box math.
