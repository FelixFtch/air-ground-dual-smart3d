# Empty-scene dual-robot LiDAR timing fix verification

Date: 2026-09-29. Status: deployed in the isolated dual-robot empty scene and verified with a short run. The scan timing error is fixed; short samples must not be taken as a guarantee of no ghosts in all occlusion and dynamic scenarios.

## Problem and confirmed cause

After a person left, old occupancy in some directions was not cleared promptly. A normal LiDAR reception rate does not prove those directions were re-observed by the map.

In this machine's CARLA, `ASensor::Tick` receives the actual time accumulated by UE per `sensor_tick`, then calls `PrePhysTick`. However, what `SensorManager` passes in `PostPhysTick` is only the current world frame time. The original rotating LiDAR used the latter to compute emission counts and scan angles, losing the time between two sensor samples.

The current configuration is 96 channels, 192000 points/s, rotation_frequency=10, sensor_tick=0.1, with the scene capped at 30 FPS. At a world frame of about 1/30 s, the original path requested only 67 rays/channel per packet and scanned about 120°, instead of 200 rays/channel per ~0.1 s. Since the map takes only the latest packet, sectors that were not sampled could go a long time without new evidence. This is the confirmed timing error and coverage gap; it has not been proven that all person ghosts were caused by this one item alone.

Another confirmed issue: the original scanner updated the angle in `SemanticLidarData`, while the ordinary LiDAR sends a separate `LidarData` without a synchronized angle header. Therefore the old ordinary `horizontal_angle` cannot directly serve as reliable evidence of the full scan range.

## Pre-fix measurements

Evidence: `/tmp/lidar_sector_phase_1790725792.json`. Observed for 26.013 seconds of wall-clock time, epoch `4eae8624f29d4bfcb7bdc8ce0f1413ad`; read existing point-cloud previews and fusion state, adding no sensors and reading no pedestrian ground truth.

| Metric | Go2 | UAV |
| --- | --- | --- |
| Median per-packet actual echo azimuth span | 118.2099° | 118.2099° |
| Fused packets successfully matched to preview | 57/68, 83.8% | 59/68, 86.8% |
| Angular coverage gap across consecutively observed, successfully matched fused packets | 2.010373 sim seconds | 2.694560 sim seconds |
| Corresponding consecutive packets and map direction | 5 packets, 70–80° | 8 packets, 100–110° |

Angular spans are computed from point-cloud echoes; 118.21° is consistent with "theoretical emission of 120°, discrete endpoints excluding the next ray". The gaps above use `conservative_missing_angular_support_run` with a 10° edge margin and exclude seen fusion timestamps with no preview match; however, the old script did not save fusion counters, so intermediate fused packets unobserved during status-publish gaps cannot be fully ruled out. These are therefore angular coverage gaps of continuously observed samples, not the maximum gap of full per-packet ground truth. The 4.798/4.074 seconds in that evidence are only re-observation intervals of a limited matched subset and **must not be reported as the true gap of the complete fusion chain**. All directional gaps above are capture-time differences, i.e. sim seconds, not wall-clock seconds.

## This change

Only three files under the isolated project `/data/sim/AirGroundLab_UE58/CarlaUnreal/Plugins/Carla/Source/Carla/Sensor/` were modified:

- `RayCastSemanticLidar.h/.cpp`: save the true elapsed time in `PrePhysTick`, consume it once in `PostPhysTick`, then clear it; the scan formula and `SimulateLidar` signature are unchanged.
- `RayCastLidar.cpp`: use the same true elapsed time; after the scan completes, synchronize the actual scan angle into the ordinary point-cloud header.

When there is no PrePhys call, the original world-frame-time fallback is kept; non-finite or non-positive times do not publish the old point cloud. The attached-parent ignore logic is kept, so other robots can still produce LiDAR collisions. `HSSLidar`'s separate fixed-sector implementation is unchanged.

Map content, patrol points, robot profiles, LiDAR range, FOV, noise, dropoff, map probabilities, and clearing TTL are not modified; deployment keeps the current map and poses. Factory, Town10, and the shared Engine core are out of scope.

## Boundaries and load

- **Not a forced 360° per packet.** UE's first interval tick uses the single world frame time and may still be only 120°; scheduler compensation can produce 0.08/0.12 s intervals, corresponding to 288/432°. The fix follows actual time and does not fabricate a fixed 0.1 s.
- Normal scan geometry and timing at `sensor_tick=0` keep the original behavior; the ordinary angle header is still correctly fixed.
- **No echo does not mean free evidence obtained.** No missing rays are generated this time. Directions facing the sky with no return, occluded by objects ahead, or not actually measured still have no evidence to clear old occupancy. The issue of fine surfaces and gaps inside a 25 cm voxel being alternately observed by different rays is also unresolved.
- Normal emission with zero hits outputs a genuinely empty packet. The original boundary where "a positive but tiny time rounds the emission count to 0" would return early and resend the old buffer is still unchanged; triggering it in this configuration requires `dt<0.00025 s`, inconsistent with normal 30 FPS / 0.1 s intervals.
- Estimated at 30 FPS and 10 packets/s, requested rays per LiDAR recover from about 64320 to 192000 per second, and for the two robots from about 128640 to 384000 per second, roughly 2.99x the ray computation. Echo counts, map processing, and IPC may also increase; this does not predict the total machine time exactly tripling — it must be measured.

## Build and map retention

The main thread completed the incremental build of this isolated project successfully, taking 84.98 seconds; 425 project tests passed. The pre-deployment backup is at `/data/robotics/air_ground_smart3d/runtime/deployment_backups/2026-09-29-sensor-elapsed-195904`; the map snapshot retains 1,043,664 observed cells, of which 19,315 are occupied. Both robots' positions and patrol progress were restored from that snapshot; the map was not cleared. The running instance was checked to have loaded the isolated project's new module.

## Post-deployment verification

New epoch: `d83ccef432d3410e8e573a93122096cb`. Isolated UE process PID 1573527; the mapped module comes from the AirGroundLab project, not marked deleted; module SHA256: `33d291317d125f548d4dfcd8ba7497d70480d84b8f775aed3336c63678803bf2`.

| Metric | Go2 | UAV |
| --- | --- | --- |
| 60-second median per-packet echo azimuth span | 358.20° | 358.20° |
| Matched fused packets exceeding 340° | 57/75 | 58/77 |
| LiDAR reception rate, later 45 seconds | 9.99 Hz | 10.01 Hz |
| Map fusion rate over the same period | 1.96 Hz | 1.98 Hz |
| Sample-to-map-apply mean / P95 | 0.230 / 0.457 s | 0.340 / 0.544 s |
| Motion distance over the same period | 31.46 m | 14.44 m |
| Tree nodes at end | 4997 | 4439 |

- The 60-second window includes cold start: fusion about 1.35/1.37 Hz; brief sensor_stale and larger initialization cost appeared. Another 35-second window still caught about 2 seconds of UAV sensor_stale; the later 45-second window had none. This does not mean it will never happen over long periods.
- Matched fused packets with counter-continuity checking still have about 1.48/1.50-second angular coverage exclusion windows; it cannot be claimed that every packet is a full circle or that directional gaps have fully disappeared. The first frame and UE interval-scheduling jitter can produce partial-scan packets.
- The initial occupancy of 10 sampled person ROIs in the later 45 seconds all cleared to zero within the window; first clearing was 0.66–7.68 seconds after the sampling start. That timing includes time when the person was still moving within the ROI and **is not the clearing delay after confirmed departure**. This is only an on-site sample of SAM3-associated positions, not a whole-field per-person ground-truth scoring.
- The thin-beam evaluation ROI occupied 71–133 cells, 129 at the end; rules unchanged. This range of fluctuation still does not prove all discretization issues of fine surfaces are solved.
- The 45-second window had no fatal, map, people, or planner errors. Earlier there were two person-snapshot timestamp-race warnings; when new person data's time is later than an earlier planning time it is rejected and the still-valid cache is used. No UE person positions were fed into the estimator.
- Both robots kept moving, and the SAM3 service continued using the original 2 Hz and 640×360 input; this round did not separately re-quantify the OHZ persistence display ratio.

Different windows have different positions, robot heights, and pedestrian relative positions, so this is not a strictly controlled A/B experiment. 10 Hz reception and about 2 Hz actual map fusion are different metrics; this round makes no claim of a 10 Hz map.

## Evidence files

- `coverage_after.json`: 60-second angular coverage, fusion counter continuity, and latencies including startup.
- `map_after.json`: first 55-second map and dual-robot progress.
- `map_steady.json`: later 45-second steady-state sample.
- `coverage_before.json`: 26-second angular coverage under the old timing; note it lacks fusion counter-continuity verification.
- Build logs and pre-write module/source backups are in the deployment directory. The candidate diff is also saved there.

## Rejected performance candidate

Also compared in /tmp: changing the observed time index to an unordered_map: 82 tests passed, 21 request responses and the saved snapshot were equivalent, but on fixed synthetic inputs the median request time improved only 0.41% with a slightly worse P95, within noise. It was therefore not deployed and nothing was restarted again. The candidate is kept at `/tmp/octomap_observed_hash_20260929_200454`.
