# Live scene and depth registration evidence

Date: 2026-09-29. These captures are verification evidence, not map or planner inputs. Existing robot/sensor actors, goals, scene geometry and world settings were not modified. Every temporary observer camera was destroyed after capture.

## Live overview

- `air_ground_overview.png`: 960 × 720, frame 21524, existing `OHZTest_Daylight` exposure preset, observer ray tracing disabled.
- `air_ground_overview_rt.png`: same view/settings with ray tracing enabled, frame 22933. This confirms the preset alone fixes washed-out RGB output.
- Corresponding `.json` files include image capture simulation time, camera pose, and same-frame world snapshots of Go2, the Bee and all six people. Actor truth is recorded only in these verification files.
- The three saved collision objects are the red low wall, blue overhead beam and yellow box; flat colored squares belong to the base scene. Images are direct simulator captures, with no compositing or annotation.

The standard RGB blueprint in this local UE5.8 CARLA build exposes `post_process_profile`, not individual ISO/shutter/exposure-mode attributes. The existing profile `Content/Carla/Config/PostProcess/OHZTest_Daylight.json` differs from `Default.json` only in exposure bounds and bias: override minimum/maximum enabled, range 10–12 instead of −10–20, bias 0 instead of 1.2. The default preset produced an almost completely white frame. Both ray-tracing settings produced a clear image using the daylight preset. Formal RGB sensors can therefore keep their existing ray-tracing choice and use `enable_postprocess_effects=true`, `post_process_profile=OHZTest_Daylight`; depth sensors remain unchanged.

## Floor/depth probe before correction

`floor_depth_registration_probe.json` copies the original read-only measurement report, including paired RGB/depth frame 9270 and the reference camera pose. It predates the later map-origin and pixel-center corrections.

- Physical ground rays at (−8,8), (−7.2,8), (−8,8.8) measured world Z ≈ 0.20124996 m.
- The old origin Z=0.201 m placed actual ground slightly above map Z=0, therefore in the first nonnegative occupancy layer.
- 297 floor pixels projected with integer pixel coordinates yielded world Z=0.203397–0.210163 m (median 0.204720 m).
- Using pixel centers `(u+0.5,v+0.5)` yielded Z=0.200915–0.201063 m (median 0.200988 m), consistent with the ground to submillimetre precision.
- Ordinary `sensor.camera.depth` encodes planar depth along camera-forward X, not radial distance. Normalizing the ray before multiplying by this depth is incorrect. Local upstream evidence: `Docs/ref_sensors.md:674`, `PythonAPI/examples/nvidia/cosmos/client/carla_cosmos/visibility.py:23`; the Linux depth material uses `MaterialExpressionSceneDepth`.

The root task separately implements and validates the map-origin and projection fixes; this directory preserves the measurement that motivated them.
