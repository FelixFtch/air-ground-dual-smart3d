# Shared frame, robot and evidence contracts

Each run creates a new `scene_epoch`; no saved map is silently loaded into a new
world. Scene/content/profile identity is captured with the run. Robot IDs are
unique and generate role, TF, sensor, worker and log ownership. Appearance is a
separate profile from movement/body/sensors. A mesh swap does not alter mobility.

- **Frame:** map X=CARLA X, map Y=-CARLA Y, map Z=CARLA Z minus the measured scene
  origin. All values are metres/radians/simulation seconds. Each sensor uses its
  own capture transform. Profile calibration and environment-specific floor
  height must be measured before operation.
- **Observation:** scene+epoch+robot+sensor+frame sequence+capture time+capture ray
  origin+calibration ID. Real hit rays update occupied endpoints/free interiors.
  Valid measured returns beyond the fusion range are explicitly clipped free
  rays; missing data never means free. The shared C++ OcTree owns probabilities.
- **Map:** signed floor(XYZ/resolution) keys, occupied/free/unknown, monotone
  revision, capture-time watermark. Consumers require matching identity and
  contiguous deltas. Full `.ot` snapshots preserve probabilities and metadata.
- **Tree/path:** one pinned native SMART worker per robot: `SMART<f32,2>` or
  `SMART<f32,3>`. Session/robot/goal/map revisions accompany every response.
  The unknown-space tree is a candidate. RViz renders real node XYZ and real
  parent edges only. Recovery and goal reroot keep the tree instance.
- **Motion:** the owner checks fresh sensors, the latest map, full-body swept
  known-free volume (retained until new occupancy evidence, optional per-cell TTL) and the live partner envelope. UE additionally applies
  physical collision sweeps. Commands are bounded XYZ/yaw changes; actual pose
  deltas, not commanded displacement, produce odometry feedback.
- **People:** physical walkers are scenario actors. Their true poses are never
  passed to mapping or prediction. Human occupancy comes from measured geometry. The active SAM3 video pipeline
  estimates per-person visible position from registered depth and capture pose,
  then estimates XY velocity from valid per-source observations. One shared model
  serves two independent camera sessions; associated people produce temporary
  profile-specific 2D/3D OHZ constraints. Z velocity remains unknown.
- **Cooperation:** partner robot odometry is permitted cooperative data, is
  temporary and expires, and never becomes a permanent semantic wall. Captured
  partner-body hits are masked only with matching capture-time robot poses.

The runtime has bounded latest measurement queues and bounded RGB frame pairing.
Only latest RGB preview files are retained; navigation events and final map
snapshots live under the explicit run directory. There is no hidden API/model
call, no web console, and no automatic global map reset watchdog.
