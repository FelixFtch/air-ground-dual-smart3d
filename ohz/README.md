# Changing the person hazard shape (OHZ)

Today the OHZ (the keep-out area the planner avoids around a person) is an ellipse. This guide is for the person replacing that ellipse with the new shape. It shows you where the shape is made, where it is consumed, and what to change.

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

## The replacement shape (agreed spec)

**Person standing still.** No elongation. The zone is the person's radius plus this robot's radius. Go2 gets a disc on the ground. Bee gets a vertical cylinder of the same radius, with height.

**Person walking.** Pull a rectangle on the ground from the feet, along the walking direction. Width equals the person's diameter plus the robot's diameter. Length follows speed and grows forward from the feet. The whole rectangle prunes the tree. When the person stops, the rectangle disappears and only the disc or cylinder remains.

## What to change

The rectangle is not wired in yet. `people.rs` currently rejects everything except the ellipse family. Rules for the change:

1. Add the new shape as a second output kind in `perception/hazards.py`. Keep the ellipse too, for side-by-side comparison.
2. Teach `planning/src/people.rs` to accept the new kind.
3. Never write people into the occupancy map.
