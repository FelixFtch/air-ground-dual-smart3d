# Blue overhead beam far-range erasure: follow-up forensics

2026-09-29. This round changed no map probabilities, marking/clearing ranges, planning, or SAM3; no map clearing or navigation restart. To trace measurement rays precisely, only `capture_origin_xyz` was added to the existing LiDAR preview NPZ and that preview service was restarted. Its ~22-second preview interruption is this known diagnostic restart and must not be treated as a mapping LiDAR fault.

## Confirmed

- Over 75 seconds and 375 samples, blue-beam ROI occupancy fell from a maximum of 152 cells to 14; the number of ROI markers RViz actually produced always exactly matched display occupied, stride=1, with no clipping by robot distance.
- With the nearest robot <5 m from the beam the average was 135 cells, at 5–9 m about 85 cells, and with both robots >9 m about 29 cells. Real beam echoes still occur at range, e.g. 8 points for Go2 at 13.35 m and 2 points at 17.38 m.
- 75-second existing-LiDAR-preview forensics with exact capture origins: Go2 552 frames, UAV 570 frames. The known beam geometry from the test scenario was used only for offline scoring; rays that pass through the real solid yet report a farther endpoint number 0; rays grazing the beam's outer bounding voxels without passing through the beam solid number 13807 and 7986 respectively, of which 1465 and 952 are farther than 9 m. The preview is not an audit log of all mapper-accepted frames; these counts must not be called final clearing counts or attributed to a single sensor.
- Standard RGB-D axial depth, half-pixel, and coordinate transforms match the local UE/CARLA source; ground reconstruction height error is within millimeters. UAV frame 626872 has 424 valid depth pixels on the blue beam with median distance 12.35 m, while current forward depth only fuses/clears 6 m and auxiliary 3 m, so that frame's depth cannot clear the distant beam.

## Root-cause mechanism and scope of the previous fix

A 25 cm voxel being crossed by any single free ray does not mean all space in the cell is empty. A thin beam occupies only part of the voxel, and sparse beams at long range can graze the remaining gaps; both Go2 and the UAV can produce such free evidence. With the current prob_miss=.1, a single negative observation (-2.197) can directly flip an occupied cell already saturated at clamp_max=.88 (+1.992). Long range adds no occupancy, and the previous far protection only keeps cells with real hits in the same frame; it does not protect cells without echoes in later frames, nor add positive evidence to occupancy probability.

Minimal reproduction on real OctoMap: a new ray 13.26 cm from the old real surface — entirely able to miss a 5 cm thin object — yet crosses the same 25 cm cell and clears it in a single update. Another new ray passes through the voxel center while still 14.18 cm from the old surface, proving that requiring passage through the voxel center is not a cure either. The previous round's same-frame hit-priority fix was correct, but it cannot be claimed to have cured the blue beam's cross-frame erasure.

## Fix trade-offs (not yet implemented)

Restoring symmetric positive/negative probability evidence would mitigate this: keeping hit .9 and max .88, miss .25/.3/.4 would require 2/3/5 valid free frames respectively; real far hits should also add positive evidence, not merely refresh timestamps. Changing probabilities alone would still eventually suffer long-term grazing mis-clears and is not a cure; dynamic ghost removal would also become slower.

If both the static thin beam and fast clearing of real dynamic targets must be kept, more accurate spatial evidence is needed: retain actually hit surfaces / finer obstacle sub-voxels, and avoid treating a ray that grazes the empty part of a large voxel as the whole obstacle disappearing. Static beam, a person staying in place then leaving, and dual robots from different directions with long-range low density should each be tested separately. The blue beam must not be written as a permanent forbidden zone, Go2 forbidden from clearing high up, or UE obstacle ground truth read to fake fixed perception.
