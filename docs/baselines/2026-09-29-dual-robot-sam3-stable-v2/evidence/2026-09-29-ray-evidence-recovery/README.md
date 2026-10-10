# 40 m clearing: real-ray coverage and positive-evidence recovery of old surfaces

2026-09-29. Only the isolated air_ground_smart3d experiment was modified; Town10, Factory, the UE scene, cameras, SAM3 frequency, motion speed, probability parameters, and planning parameters were not changed. The user did not want new sensor recording/playback work, so this round only did isolated synthetic regressions and a short read-only runtime metrics check; no new raw image/point-cloud sequences were saved.

## Confirmed and fixed

1. The running dual-robot clearing range had already been restored to 40 m, with first marking still at 9 m. Earlier residue cannot be blamed on unrestored YAML.
2. The original helper deduplicated by endpoint voxel. Different actual rays can share the endpoint cell yet pass through different intermediate cells; dropping one loses real clearing evidence. Now only fully identical endpoints are removed, keeping both close-range hit rays and far-range free rays. The native mapper deduplicates hit/free keys after traversal, so each cell is still updated at most once per frame.
3. The original far-hit could only protect current occupancy and could not recover old surfaces once wrongly cleared. Real close-range hit history is now recorded separately: first obstacle marking within 9 m; real echoes at 9–40 m can add normal positive hit evidence to previously established positions, including free→occupied. Free-only history cannot be upgraded to marked history; distant cells never established gain nothing.
4. Later real free can still clear these cells; no TTL, permanent protection zones, or UE person ground truth are added. forget clears the history; new snapshots save the history; old snapshots migrate conservatively with current occupancy only, and old cleared cells must be re-observed close-up to regain history eligibility.

## Verification and deployment

- Full Python regression: 417 passed; including a real OctoMap subprocess, near/far ray-omission reproduction, same-frame hit priority, dual-robot out-of-order, duplicate frames, recovery, and forget.
- Deployed native binary SHA256: 55431a0e1b37446fa37bf1b4209edfdc3df1b8d4b7b7dff6e29b2c57dacf43f7. Verified via the running process's /proc/PID/exe, not just the disk file.
- New epoch: 4eae8624f29d4bfcb7bdc8ce0f1413ad. The old map retains 1,032,354 cells; original robot poses/patrol progress restored from the stopped checkpoint. UE PID 774790, ROS view, and RViz were not restarted.
- Runtime metrics reported ray_geometry_policy=all_distinct_measured_rays, clear_range_m=40, and actual recovered_far_hit_cells. The old profile's mapping_sampling description string remains a historical label with no part in execution; actual behavior is governed by the new metrics and code, and the profile hash was not changed this round just to reword a description.

45-second uncontrolled field sample:

| Metric | Go2 | UAV |
|---|---:|---:|
| Received LiDAR Hz | 10.03 | 10.03 |
| Fused LiDAR Hz | 3.57 | 3.57 |
| Capture-to-map-apply mean / 95th percentile | 0.132 / 0.302 s | 0.176 / 0.344 s |
| Distance traveled | 41.40 m | 17.85 m |
| Tree nodes at end | 4983 | 4389 |

Of 9 sampled areas near SAM3 persons with initial occupied cells, all had been cleared 0.19–3.06 s after sampling and were also 0 at the end; this is a result for these areas, not a whole-field no-residue guarantee, nor a precise "person-left-to-cleared" latency. SAM3 ROIs may be incomplete and associations may change. The map added 586 revisions; map/perception/planning anomalies were empty. Both robots walked normally, and the UAV waited at times due to real geometry/OHZ constraints; waiting periods were not removed to exaggerate speed.

The static-beam scoring ROI occupied 63–125 cells during this check, 100 at the end, still fluctuating. The scoring ROI comes from known scene geometry and is for checking only; it never entered the mapper or planning inputs.

## Explicitly retained limitations

- This round fixed omitted rays and the far-range positive/negative evidence asymmetry; it does not claim all trailing ghosts or sparse-ray surface aliasing are solved.
- Within a 25 cm cell, "solid somewhere in the cell" and "another ray passes through the cell's gap" can still hold simultaneously. A single miss can currently flip occupancy; thin beams may keep flickering across frames, which cannot be cured by tuning 40/15 m or ordinary probability weights.
- Missing echoes are not free space; positions not reliably re-observed cannot be marked free out of thin air. Going further would need subdivided-surface / continuous sensor models with an explicit measurement footprint; real static obstacles must not be deleted wholesale over time.
- Far-range recovery allows positions "with a real historical close-range hit" to be re-occupied at distance by new real objects; this is an explicit update-policy change and does not mean all historical free space can remotely create new obstacles.

Backup and deployment manifest: /data/robotics/air_ground_smart3d/runtime/deployment_backups/2026-09-29-ray-evidence-193838.
