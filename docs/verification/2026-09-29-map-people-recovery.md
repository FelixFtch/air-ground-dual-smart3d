# 2026-09-29 shared-map / people recovery

**Historical recovery-stage report.** The runs, rates and implementation state
below describe the earlier map/person recovery phase. Subsequent ordered
fusion/application pipelining, immutable display snapshots and transport/GC
optimizations are documented in the [OctoMap real-time verification
report](2026-09-29-octomap-realtime.md). Do not use this report's older timings as
current performance or as an end-to-end human clearing benchmark.

## Observed failure and fix

Map restore intentionally retained geometry but invalidated capture freshness from the old epoch. Its `observed: null` was incompatible with the native worker's numeric-only parser. Both workers repeatedly attempted initialization, requesting the full ~434k-cell map and delaying fusion. Native cells now accept an optional observation time; occupied geometry still blocks, while missing-time free space is not executable without new evidence. Structural/protocol failures no longer trigger an unbounded full-map retry. No map reset was used to hide this issue.

A second issue was occupancy hysteresis: a saturated free cell required three hits under default probabilities, often more than a walking person supplied. This simulator profile now uses hit 0.9 / miss 0.1 and clamp 0.12/0.88. Timestamp ordering prevents delayed sensor frames reversing newer cell evidence; same-time hits take priority. This is a simulator setting, not a calibrated hardware model. Free-ray evidence is still required to clear occupancy.

## Bounded live evidence

Raw 40-second telemetry and its machine-readable summary sit beside this document. Mean displayed map age 0.288 s / max 0.625 s is measured against the published sensor simulation clock; it is not a human-ground-truth marking/clearing latency benchmark. Fusion processing means ranged about 54–82 ms. Both robots advanced 2 goals; Go2 travelled 31.29 m and the UAV 16.55 m. Map and planner errors were absent during this sample.

An actual ROS subscription over 18 seconds received 20 Go2 / 18 UAV SAM3 images, 95/96 LiDAR clouds and 38 people marker arrays. The latest array contained one measured surface box, two applied OHZs and two native LRZs. No valid velocity arrow was present in that final sample. SAM3 visible-surface boxes are partial depth measurements, not complete learned 3D human boxes.

SAM3 uses one cached model and separate video sessions, with a total 2 Hz processing limit and latest-frame replacement. Each camera achieved roughly 1 Hz. During this 40-second sample no active person had a valid velocity estimate; position/OHZ integration is verified, but stable speed continuity and cross-camera identity quality are not claimed. XY velocity needs at least 4 same-source observations; Z velocity is explicitly invalid. Uncertainty margins remain uncalibrated.

## Verification and lifecycle scope

162 Python tests pass, including map restoration, timestamp ordering, dynamic occupancy, execution guards, RGB-D pairing, people projections and native hazard contracts. The native Rust suite passed 99 tests. Actual map save/restore through 2D/3D worker initialization was verified.

All seven owned services were active after this repair. Navigation/perception were restarted against the already-running isolated UE instance; this is not a claim of a new complete UE cold-start test for the added SAM3 lifecycle. The existing `.ot` was retained. The original Town10, Factory and single-Go2 lab configurations were not changed.

An incremental body-height projection helper was separately tested and benchmarked but is not yet wired into the running publisher; it is not counted in the measured gains above.

## Follow-up corrections in the same repair

The 2-second velocity fitting history could not hold the required 4 measurements at the actual 1 Hz/camera serving rate. The bounded fitting window is now 4.5 seconds with at most 6 samples; the minimum of 4 measurements, the 1.5-second consecutive-gap reset, and the residual/speed gates are unchanged. This does not extend the active geometry TTL or the prediction horizon. New regressions cover 1 Hz accurate motion, history bounds, missing-frame reset/reacquisition and stale captures.

Blocked set-goal recovery now reuses the exact rejected delta with the accepted goal revision, rather than fetching another full map. Fresh current pose/time/dynamics are attached. Contract failures stop the affected planner instead of silently amplifying full-map retries. Four focused regressions pass.

## Final deployed check

The final restart preserved the prior `.ot` (2,883,940 bytes) and resumed it in epoch 7e7ba4a073174464bc0e182b3ab278c3. UE remained running; robots restarted at their configured starting locations. Final 40-second sample: map display age mean 0.2967 s / max 0.6089 s, people present in 32 of 40 samples, valid XY velocity present in 11 of 40 samples (previous sample 0 of 40). Both robots moved and each completed a goal. This confirms the fitting-window defect is fixed, not guaranteed continuous tracking. All map/planner errors were absent. Raw data and summary are in `2026-09-29-final-live*.json`.

A final 18-second ROS subscription verified actually delivered markers: up to 4 measured person surface boxes, 8 robot-specific OHZs, 2 LRZs and 2 valid velocity arrows. The last received array had 1 person box, 2 OHZs, 2 LRZs and 1 velocity arrow. Each camera delivered 18/20 SAM3 images; each LiDAR 87/86 clouds. This is transport/output verification, not a person-tracking accuracy benchmark. See `2026-09-29-final-ros-markers.json`.
