# People OHZ: for anyone changing the hazard shape

This pipeline is what runs live today. Do not modify the RRT* and LRZ code in `planning/` unless the shape interface itself is too limiting.

## How the data flows

1. `perception/yolo26_backend.py`: YOLO26m detects people only; MapTracker assigns per-camera track IDs.
2. `perception/people_projection.py`: masks plus depth are projected into map-frame 3D boxes, and horizontal speed is estimated on the same track ID.
3. `perception/hazards.py`: `build_hazards` folds the person's radius and this robot's radius into a hazard zone. Go2 gets a ground ellipse; Bee gets an ellipsoid.
4. `planning/src/people.rs`: accepts only `ellipse` / `ellipsoid` and prunes the tree with it. When the person leaves or the hazard expires, the same clearing logic reattaches the pruned branches.

2D and 3D are already split by the robot's `dimension`. Go2 reads the ground ellipse; Bee reads the ellipsoid.

## The replacement shape

Person standing still: no elongation. The hazard zone is the person's radius plus this robot's radius. For Go2 that is a disc on the ground; for Bee a vertical cylinder of the same radius with height.

Person walking: pull a rectangle on the ground from the feet along the velocity direction. Width equals the person's plus the robot's diameter; length follows speed, extending forward from the feet. The whole rectangle prunes the tree. When the person stops, the rectangle disappears and only the disc/cylinder remains.

The rectangle is not wired in yet. `people.rs` currently rejects anything but the ellipse family. Add the new shape as a second output kind in `perception/hazards.py` and teach `people.rs` to accept it. Keep the ellipse for side-by-side comparison. Never write people into the occupancy map.
