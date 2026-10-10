# Velocity-Shaped OHZ: Passing Behind Pedestrians

Date: 2026-10-04. Status: discussion conclusions, **no code changed yet**. The current OHZ is still the fixed 1-second sweep ellipse in `perception/hazards.py`.

## Observation

When a pedestrian crosses in front of the robot, SMART3D often detours around the front of the pedestrian (the direction they are heading). The pedestrian keeps walking and blocks the newly chosen path again, so the path keeps being pushed in the pedestrian's direction of travel, and the robot ends up taking the long way around.

## Cause

- Current OHZ: sweeps the sum of pedestrian and robot radii ρ along the velocity direction for 1 second, with the center shifted forward by half the sweep; only ρ remains behind, and v×1 s extends ahead.
- SMART treats the OHZ as a static region: nodes falling inside are pruned, edges crossing it are invalidated, until the next refresh. It does not know "where the pedestrian will be when".
- The robot typically takes several seconds to reach the crossing point. When the OHZ front first touches the robot's route, going around the front is shortest, so the front is chosen; each OHZ refresh pushes it further forward, producing a chase-around-the-pedestrian behavior.
- The original SMART (Shen et al., RA-L 2023, arXiv 2305.06487) uses an OHZ that is a circle centered at the obstacle's current position, r = v·T_OH + r_i + r_R, T_OH = 0.4 s, assuming obstacle heading is unknown.

## Idea

1. For pedestrians with a velocity estimate, the OHZ becomes a teardrop that is long in front and short behind; pedestrians without velocity stay circular. The shape is determined solely by the velocity estimate, with no additional confidence adjustment.
2. The teardrop covers positions the pedestrian could reach within [0, T] under a bounded acceleration a: at time t the circle center is p + v·t with radius ρ + ½·a·t².
   - Front: ρ + v·T + ½·a·T²
   - Sides: ρ + ½·a·T²
   - Behind: keep ρ. For t ≤ 2v/a, a pedestrian moving forward cannot get back behind (about 1.3 s for v = 1 m/s, a = 1.5 m/s²).
3. Use this variant only inside the LRZ, with T on the order of the time the robot needs to cross the LRZ. Outside the LRZ, keep the current approach.
4. For the checks, approximate the teardrop as a capsule whose radius grows along the predicted direction; both point and edge checks have closed forms, with cost comparable to the ellipse.
5. Channels form behind every moving pedestrian in a crowd: channels behind same-direction flows link into followable "lanes"; a crossing flow can be traversed by passing behind pedestrians one after another.

## Validity Conditions and Risks

- **Most pedestrians must have velocity**: pedestrians without velocity are treated as circles, with no channel behind them. From a 100 s recording in the empty scene on 2026-10-04 compared against CARLA ground truth: YOLO26m + map association gives velocity for about 64% of correct detections, online SAM3 at 1 Hz about 26%.
- **Time window**: passing behind a pedestrian within roughly 2v/a seconds after they walk past has physical justification; any later it only relies on "pedestrians generally do not suddenly turn around".
- **Braking margin**: the robot must be able to stop at any time. The pedestrian body is also in the LiDAR voxels in this stack, and execution is additionally backed by a sweep check against measured geometry.
- **Density**: when gaps are smaller than the robot size there is no channel, and the problem degenerates into the frozen robot problem (Trautman & Krause, IROS 2010; Trautman et al., IJRR 2015).
- **Pedestrian reaction**: pedestrians slow down or sidestep near robots, so the ½·a·t² widening on the sides cannot be dropped.
- **Comfort**: passing tight behind a pedestrian is safe but uncomfortable; a separate minimum distance could be set.

## Possible Further Steps (Crossing Scenarios)

- Space-time checks: tree nodes convert cost into arrival time and compare against the pedestrian's position at that time (SIPP, Phillips & Likhachev ICRA 2011; velocity obstacles, Fiorini & Shiller IJRR 1998).
- Path-velocity decomposition: a crossing pedestrian is a diagonal band in the "distance-along-path vs. time" plot; slowing to yield means passing behind the pedestrian, and only oncoming pedestrians require a path change (Kant & Zucker IJRR 1986; the ST graph in Baidu Apollo's EM Planner).

## Validation Plan

- Scenario: a medium-density crossing pedestrian flow in UE, planning to add characters via MetaHuman Crowd.
- Compare three approaches: the current ellipse, the velocity teardrop, and the teardrop plus speed-adjusted yielding.
- Metrics: crossing time, minimum distance to pedestrians, detour length, fraction of passes made behind pedestrians, number of replans.
- Before integrating MetaHuman characters, confirm:
  - Whether they carry collision bodies. CARLA ray-based LiDAR relies on collision for intersection; without a collision body they will not appear in LiDAR or OctoMap, leaving only the camera stream able to see them.
  - Where ground truth comes from. They are not CARLA walkers, so `world.get_actors().filter('walker.*')` will not return them; positions and velocities need to be exported from the UE side.

## References

- Koschi, Pek, Beikirch, Althoff. Set-Based Prediction of Pedestrians in Urban Environments Considering Formalized Traffic Rules. ITSC 2018.
- Vassallo et al. How do walkers avoid a mobile robot crossing their way? Gait & Posture 2017 (arXiv 1609.07955); plus the 2018 follow-up.
- Kirby, Simmons, Forlizzi. COMPANION. RO-MAN 2009 (asymmetric Gaussian personal space, wide in front and narrow behind).
- Mavrogiannis et al. Core Challenges of Social Robot Navigation: A Survey. ACM THRI 2023.
