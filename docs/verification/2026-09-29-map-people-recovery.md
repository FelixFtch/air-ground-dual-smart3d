# 2026-09-29 shared-map / people recovery

**Historical recovery-stage report.** The runs, rates and implementation state
below describe the earlier map/person recovery phase. Subsequent ordered
fusion/application pipelining, immutable display snapshots and transport/GC
optimizations are documented in the [OctoMap real-time verification
report](2026-09-29-octomap-realtime.md). Do not use this report's older timings as
current performance or as an end-to-end human clearing benchmark.

## Observed failure and fix

Map restore intentionally retained geometry but invalidated capture freshness from the old epoch. Its `observed: null` was incompatible with the native worker's numeric-only parser. Both workers repeatedly attempted initialization, requesting the full ~434k-cell map and delaying fusion. Native cells now accept an optional observation time; occupied geometry still blocks, while missing-time free space is not executable without new evidence. Structural/protocol failures no longer trigger an unbounded full-map retry. No map reset was used to hide this issue.

A second issue was occupancy hysteresis: a saturated free cell required three hits under default probabilities, often more than a walking person supplied. This simulator profile now uses hit0.9/miss0.1 and clamp0.12/0.88. Timestamp ordering prevents delayed sensor frames reversing newer cell evidence; same-time hits take priority. This is a simulator setting, not a calibrated hardware model. Free-ray evidence is still required to clear occupancy.

## Bounded live evidence

Raw40-second telemetry and its machine-readable summary sit beside this document. Mean displayed map age0.288s/max0.625s is measured against the published sensor simulation clock; it is not a human-ground-truth marking/clearing latency benchmark. Fusion processing means ranged about54–82ms. Both robots advanced2goals, Go2 travelled31.29m and UAV16.55m. Map and planner errors were absent during this sample.

Actual ROS subscription over18seconds received20Go2/18UAV SAM3 images,95/96LiDAR clouds and38people marker arrays. Latest array contained one measured surface box, two appliedOHZs and two nativeLRZs. No valid velocity arrow was present in that final sample. SAM3 visible-surface boxes are partial depth measurements, not complete learned3D human boxes.

SAM3 uses one cached model and separate video sessions, with a total2Hz processing limit and latest-frame replacement. Each camera achieved roughly1Hz. During this40-second sample no active person had a valid velocity estimate; position/OHZ integration is verified, but stable speed continuity and cross-camera identity quality are not claimed. XYvelocity needs at least4same-source observations; Zvelocity is explicitly invalid. Uncertainty margins remain uncalibrated.

## Verification and lifecycle scope

162Python tests pass, including map restoration, timestamp ordering, dynamic occupancy, execution guards, RGB-D pairing, people projections and native hazard contracts. NativeRust suite99tests passed. Actual map save/restore to2D/3Dworker initialization was verified.

All seven owned services were active after this repair. Navigation/perception were restarted against the already-running isolatedUE instance; this is not a claim of a new completeUEcold-start test for the addedSAM3 lifecycle. The existing `.ot` was retained. OriginalTown10, Factory and single-Go2lab configurations were not changed.

An incremental body-height projection helper was separately tested and benchmarked but is not yet wired into the running publisher; it is not counted in the measured gains above.

## Follow-up corrections in the same repair

The2-second velocity fitting history could not hold the required4measurements at the actual1Hz/camera serving rate. The bounded fitting window is now4.5seconds, at most6samples; minimum4measurements,1.5second consecutive-gap reset and residual/speed gates are unchanged. This does not extend active geometryTTL or prediction horizon. New regression covers1Hz accurate motion, history bounds, missing-frame reset/reacquisition and stale captures.

Blocked set-goal recovery now reuses the exact rejected delta with the accepted goal revision, rather than fetching another full map. Fresh current pose/time/dynamics are attached. Contract failures stop the affected planner instead of silently amplifying full-map retries. Four focused regressions pass.

## Final deployed check

The final restart preserved prior `.ot` (2,883,940bytes) and resumed it in epoch7e7ba4a073174464bc0e182b3ab278c3. UE remained running; robots restarted at their configured starting locations. Final40second sample: mapdisplay age mean0.2967s/max0.6089s, people present32of40samples, validXYvelocity present11of40samples (previous sample0of40). Bothrobots moved and each completed a goal. This confirms the fitting-window defect is fixed, not guaranteed continuous tracking. Allmap/planner errors were absent. Rawdata and summary are in `2026-09-29-final-live*.json`.

Final18second ROSsubscription verified actual delivered markers: up to4measured person surfaceboxes,8robot-specificOHZs,2LRZs and2validvelocityarrows. The lastreceived array had1personbox,2OHZs,2LRZs and1velocityarrow. Eachcamera delivered18/20SAM3images; eachLiDAR87/86clouds. This is transport/output verification, not a person-tracking accuracy benchmark. See `2026-09-29-final-ros-markers.json`.
