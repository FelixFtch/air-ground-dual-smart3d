# Set up the dual-robot stack on a fresh Ubuntu machine

This guide takes a fresh Ubuntu machine from zero to a running stack: the Go2 robot dog and the Bee drone moving in CARLA (the simulator, built on Unreal Engine 5.8), started with `./lab start --rviz`.

The repository carries all the first-party code: the Rust planner, the C++ map service, the YOLO26 weights (the neural network that finds people in camera images), and the handmade UE levels. Three heavy dependencies cannot travel through Git:

1. The CARLA UE 5.8 project and engine.
2. A Python 3.10 environment.
3. ROS 2 Humble.

Steps 1 through 3 below cover those three. First, pick a route for the UE side.

## Pick a route: restore or build

| | Route A: restore the snapshot | Route B: build from upstream |
|---|---|---|
| Speed | Fastest | Slowest |
| Requires | Access to the team backups | Internet access only |
| You end up with | The exact machine state the recordings were made on | Your own equivalent build |

### Route A: restore the reference environment

1. Unpack `/media/felix/MoFang2T1/AirGround_Dual_Go2_Bee_Full_2026-10-05_0001_EDT` (75.5 GB). It contains the UE project, the engine, the shader caches, and both Python environments.
2. Restore the roughly 32 GB of reused engine and SDK archives listed in the `00_README*` restore guide at the snapshot's root. That guide is written in Chinese.
3. Overlay the code snapshot `AirGround_Dual_Go2_Bee_Code_2026-10-10_0015_EDT` on top.

You are done when: the restored tree matches the machine the recordings were made on. Continue with step 1 below.

### Route B: assemble from CARLA upstream

Use this route when you cannot reach the backups.

1. Build the CARLA fork of UE 5.8 (Epic's engine with CARLA's changes). Clone `github.com/carla-simulator/UnrealEngine` and check out the branch matching CARLA 0.10.x, for example `carla58`.
2. Get a CARLA 0.10.x source checkout. It provides the `CarlaUnreal` project and the plugins.
3. Import the levels from this repo's `ue/` folder. Step 3 below has the details.

The handmade levels use only engine cubes plus the five materials under `Content/OHZTest/Materials`. The single external piece is CARLA's own `CarlaGameMode`, which every CARLA source build already has.

> **Two different UE 5.8s. Do not confuse them.** Epic's vanilla UE 5.8 editor (on Windows or Linux) is enough to *edit* the levels; see Option 1 in [`ue/README.md`](../ue/README.md). *Running* the robots is different. The stack drives the editor through the CARLA Python API, and that needs the CARLA fork of UE 5.8. Vanilla UE will not run the stack.

## 1. System prerequisites

1. Install Ubuntu 22.04.
2. Install an NVIDIA GPU and driver. The stack runs UE offscreen. About 24 GB of VRAM is comfortable; less works for the empty scene.
3. Install ROS 2 Humble into `/opt/ros/humble` with apt: `ros-humble-desktop`.
4. Install the Rust toolchain (rustup, stable) plus `build-essential` and `cmake`.
5. Confirm Python 3.10 is present. It is the Ubuntu 22.04 default.
6. Confirm the systemd user session works (the default on Ubuntu desktop). The launcher relies on systemd user services (how it starts and names its background processes) and creates its own `air-ground-lab-*` units. You never write unit files by hand.

You are done when: ROS 2 Humble, Rust, `cmake`, and Python 3.10 are all installed and the NVIDIA driver is loaded.

### Ubuntu 24.04 and RTX 50-series caveats

- Ubuntu 24.04: install Python 3.10 manually, and change the two `/opt/ros/humble` paths in `simulation/lifecycle.py` for Jazzy.
- Newer GPUs (RTX 50 series): replace torch with a CUDA 12.8+ (cu128) build. See `docs/environment/requirements-core.txt`.

## 2. Python environment

```bash
python3.10 -m venv .venv
.venv/bin/pip install -r docs/environment/requirements-core.txt
```

That creates a venv (a self-contained Python environment) at `.venv` and fills it from the requirements file.

Two packages need no pip install: `lap` and `ujson`. Prebuilt cp310 wheels are vendored at `runtime/python_deps/`, and the stack puts them on `PYTHONPATH` itself.

The YOLO26 weights are already in the repo at `runtime/models/yolo26/`. Their hash is recorded in `provenance/yolo26_people.json`. Set `YOLO_OFFLINE=1` to keep ultralytics from phoning home.

You are done when: the pip command finishes without errors and `.venv/` exists.

## 3. UE project and levels

1. Put a CARLA UE 5.8 `CarlaUnreal` project somewhere on disk. Route A gives you the restored `/data/sim/AirGroundLab_UE58`; Route B gives you your own CARLA source build.
2. Copy this repo's `ue/Content/AirGroundLab` and `ue/Content/OHZTest` into that project's `Content/` directory. Levels hand off between Windows and Linux unchanged.
3. Point the launcher at your paths. Four constants sit at the top of `simulation/launch_sim.py`:
   - `ROOT`: this repository's absolute path.
   - `PROJECT`: your `CarlaUnreal` directory.
   - `ENGINE`: your CARLA UE 5.8 engine directory.
   - `DLSS_SDK` (line ~45): the SDKs/DLSS path inside your CARLA checkout.
4. Point `simulation/lifecycle.py` line ~10 (`PYTHON`) at the venv from step 2 above.
5. If you plan to run the SAM3 backend, also update the SAM3 venv path at line ~43. The YOLO26 backend needs nothing extra.

You are done when: all four constants in `launch_sim.py` point at real directories on your machine, and `lifecycle.py` points at your `.venv` Python.

## 4. Build the native components

The first two commands build the C++ map service. The third builds the Rust planner.

```bash
cmake -S mapping -B mapping/build -DCMAKE_BUILD_TYPE=Release
cmake --build mapping/build -j4
cargo build --release --offline --locked --manifest-path planning/Cargo.toml
```

You are done when: both builds finish and `mapping/build/` and `planning/target/release/` contain fresh binaries.

## 5. Run and verify

```bash
./lab start --rviz --people yolo26   # empty scene, 6 pedestrians, RViz on ROS domain 45
./lab status                          # both robots should leave waiting_path and start accruing goals
./lab stop
```

RViz (the 3D viewer) opens on ROS domain 45. After startup, run the sanity checklist:

- Go2 spawns at (-8, -8, 0.33).
- The Bee spawns at (-8, -3, 2.0).
- `people_backend` reports `yolo26`.
- RViz shows the two LiDAR point clouds and the cyan/orange trees.

Then run the test suite:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  .venv/bin/python -m pytest -q tests -p no:cacheprovider
```

You are done when: both robots leave `waiting_path` and accrue goals, every checklist item passes, and pytest reports no failures.

## Working from Windows

Scene authoring is fully possible on Windows: import `ue/Content` into your own CARLA UE 5.8 project, edit, and hand the `.umap` back (see [`ue/README.md`](../ue/README.md)). The runtime itself (systemd units, ROS 2, the Linux CARLA build) runs on Linux. Reproduce it with this guide, or see [`WINDOWS_TEAMMATE.md`](WINDOWS_TEAMMATE.md) for the four ways to work from a Windows machine.
