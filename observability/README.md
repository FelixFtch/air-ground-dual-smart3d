# Read-only air-ground RViz view

This viewer follows `runtime/latest_run.json` and publishes display-only ROS data
in domain **45**. It cannot start CARLA, move robots, change maps or goals. It does
not start RViz automatically and does not create a web console.

Run from an independently prepared terminal after the runtime has started:

```sh
source /opt/ros/humble/setup.bash
cd /data/robotics/air_ground_smart3d
ROS_DOMAIN_ID=45 /usr/bin/python3 -m observability.ros_view
```

Open the saved view separately:

```sh
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=45 rviz2 -d /data/robotics/air_ground_smart3d/observability/air_ground.rviz --ros-args -p use_sim_time:=false
```

`--latest-run /absolute/path/latest_run.json`, `--rate-hz 2`,
`--max-tree-edges 30000` (per robot) and `--max-display-voxels 100000` are optional.
The process does not publish `/clock`. It uses a common wall-publication timestamp
for TF and markers; their geometry always comes from measured runtime state and
the label separately reports `sim_time`. This display stamp is not a sensor
capture stamp and never feeds mapping or execution.

Input pointer: `{"run_root":"/absolute/runtime/scene/run"}`. The run directory
contains `state.json` and optional `shared/display.json`, written by atomic rename.

```json
{"scene_id":"lab","epoch":"episode-a","sim_time":12.5,"robots":[{"robot_id":"go2_01","position_xyz":[1,2,0.349],"yaw":0.2,"body_half_extents_m":[0.43,0.43,0.28],"body_offset_m":[0,0,0],"status":"following","goal_xyz":[5,2,0.349],"planner":{"tree_instance_id":"tree-a","tree_exported":true,"path_xyz":[[1,2,0.349],[5,2,0.349]],"tree_nodes":[{"id":0,"xyz":[5,2,0.349],"parent":null,"tree_id":0},{"id":1,"xyz":[1,2,0.349],"parent":0,"tree_id":0}],"tree_edges":[[1,0]]}}]}
```

```json
{"scene_id":"lab","scene_epoch":"episode-a","revision":25,"resolution":0.25,"occupied":[[10,4,5]],"free_xy":[[4,8],[5,8]],"blocked_xy":[[10,4]],"ground_z":0,"sim_time":12.5}
```

The 3D keys are signed `floor(coordinate/resolution)` indices. Projection keys
are `[ix,iy]` from the root's ground/body projection, not a naive collapse of all
3D occupied cells. `free_xy` and `blocked_xy` may be omitted; unknown stays unknown.
The map is hidden if its scene/epoch does not match the state. A state file older
than 3 seconds is labeled **STALE**. Invalid/unavailable files produce a waiting
status and clear old markers instead of continuing a false live display.

Each robot has separate `robot_id/body_envelope`, `robot_id/heading`,
`robot_id/actual_path` and `robot_id/tree/*` marker namespaces. TF is
`map -> robot_id/base_link`. A transparent, axis-aligned box is explicitly labeled
**body envelope**; no unverified Go2/UAV mesh or fabricated RobotModel is shown.
The heading arrow and TF use actual yaw. The envelope covers the same all-yaw
planner bounds and is not rotated into a smaller safety footprint.

The viewer draws **only** `planner.tree_edges` and validates each edge's node IDs
and actual `parent` value. It never derives connections by nearest-neighbor,
connects disconnected roots or fills omitted tree exports. No export means no
tree markers until the next real export. Main edges use the robot color,
disconnected/pruned edges are gray, and the label reports omitted entries.
Both Go2 and UAV use their actual exported XYZ, so a 2D Go2 path remains planar
while UAV height variation is preserved. A malformed path is discarded as a whole
rather than bridging across a missing point. Marker lifetimes are 3 seconds.

Independent display topics:

- `/air_ground/robot_markers` — measured bodies, heading, goals and status.
- `/air_ground/tree_markers` — actual tree edges, capped without invented edges.
- `/air_ground/path_markers` — actual planner paths.
- `/air_ground/shared_map_markers` — physical occupied cells; display-only sampling
  is labeled if the configured cap is exceeded.
- `/air_ground/ground_projection` — `nav_msgs/OccupancyGrid` with free 0, blocked
  100, unknown -1. Blocking wins any duplicate projection entries.
- `/air_ground/go2_01/rgb/image` and `/air_ground/uav_01/rgb/image` — decoded
  `sensor_msgs/Image` BGR8 from each same-run `sensor/latest_rgb.jpg` (or
  `latest_rgb.jpg`). The saved RViz configuration includes both Image displays.
  The header uses the image file's wall receipt time, not an invented capture
  timestamp; this topic never feeds mapping or perception. Files older than
  three seconds are explicitly labeled STALE in the scene status. New RGB-D
  capture sidecars remain the runtime's authoritative registration record.

A valid planner response with `path_xyz:null` means no candidate path; the viewer
still displays that robot's measured body and actual exported tree. Nullable tree
arrays are treated as empty, without adding edges. To add the new image panels to
an already-open RViz, reload this saved configuration without restarting the
runtime. The viewer never closes RViz.

Offline geometry/contract tests are in `tests/test_observability.py`; they do not
launch ROS or RViz. Live verification on 2026-09-29 received both independent
measured LiDAR topics at about 6 Hz, both RGB image topics, and the real Go2 XY
and UAV XYZ trees. The window-only verification capture is
`docs/verification/rviz-live.png`. This display remains a body-envelope view,
not a claim that a robot mesh or SAM3 tracking has been integrated.
## Live measured LiDAR

The runtime atomically writes `<run_root>/<robot_id>/latest_lidar.npz` with
`points` (finite N×3 map-frame XYZ float32), scalar nonnegative integer `frame`,
and scalar simulation-seconds `capture_sim_time`. A separate 10 Hz observer
timer publishes only new capture identities on
`/air_ground/<robot_id>/lidar/points` as XYZ float32 `PointCloud2`, with
Best Effort / Volatile / Keep Last 1. The header uses the `map` frame and current
display wall time; this display topic is not an input to mapping or planning.
The RViz configuration distinguishes Go2 cyan and UAV orange measured clouds.

Robot names and two-metre vertical locator lines start at the measured poses;
the body envelope remains at its true center. Actual SMART edges are deliberately
faint and paths remain bright. Occupied cells in or below the measured ground
layer are drawn transparent grey, while higher occupied cells remain red; this
is display styling only and does not change occupancy or collision checking.
