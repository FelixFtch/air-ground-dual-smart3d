# Reproducing the live dual-robot stack from this repository

This guide takes a fresh Ubuntu machine from zero to `./lab start --rviz` with Go2 and the Bee flying on your own copy. The repository carries all first-party code, the Rust planner, the C++ map service, the YOLO26 people weights, and the handmade UE levels. Three heavy dependencies cannot travel through Git and are covered in steps 1–3 below: the CARLA UE 5.8 project + engine, a Python 3.10 environment, and ROS 2 Humble.

Two routes for the UE side:

- **Route A — restore the reference environment** (fastest, exact): unpack `/media/felix/MoFang2T1/AirGround_Dual_Go2_Bee_Full_2026-10-05_0001_EDT` (75.5 GB; includes the UE project, engine, shader caches, both Python envs) plus the ~32 GB of reused engine/SDK archives listed in its `00_README_恢复说明.md`, then overlay the code snapshot `AirGround_Dual_Go2_Bee_Code_2026-10-10_0015_EDT`. You land on the exact machine state the recordings were made on.
- **Route B — assemble from CARLA upstream** (no access to the backups): build CARLA's UE 5.8 fork from source, create a CarlaUnreal project, and import the levels from this repo's `ue/` folder. The handmade levels only use engine cubes plus the five materials under `Content/OHZTest/Materials`, so any CARLA UE 5.8 source checkout can open them; the only external piece is CARLA's own `CarlaGameMode`, which every CARLA source build has.

## 1. System prerequisites

- Ubuntu 22.04, NVIDIA GPU + driver (the stack runs UE offscreen; ~24 GB VRAM comfortable, less works for the empty scene)
- ROS 2 Humble (`/opt/ros/humble`) — apt install `ros-humble-desktop`
- Rust toolchain (rustup, stable) and `build-essential`, `cmake`
- Python 3.10 (Ubuntu 22.04 default)
- systemd user session (default on Ubuntu desktop; the launcher creates `air-ground-lab-*` user units itself, no manual unit files)

On Ubuntu 24.04 you additionally need Python 3.10 installed manually and the two `/opt/ros/humble` paths in `simulation/lifecycle.py` changed for Jazzy. On newer GPUs (RTX 50 series) replace torch with a CUDA 12.8+ build (see `docs/environment/requirements-core.txt`).

## 2. Python environment

```bash
python3.10 -m venv .venv
.venv/bin/pip install -r docs/environment/requirements-core.txt
```

`lap` and `ujson` need no pip install — prebuilt cp310 wheels are vendored at `runtime/python_deps/` and the stack puts them on `PYTHONPATH` itself. YOLO26 weights are already in the repo at `runtime/models/yolo26/` (hash in `provenance/yolo26_people.json`); set `YOLO_OFFLINE=1` to keep ultralytics from phoning home.

## 3. UE project and levels

1. Place a CARLA UE 5.8 `CarlaUnreal` project somewhere (Route A: restored `/data/sim/AirGroundLab_UE58`; Route B: your own CARLA source build).
2. Copy this repo's `ue/Content/AirGroundLab` and `ue/Content/OHZTest` into that project's `Content/`. Levels hand off between Windows and Linux unchanged.
3. Point the launcher at your paths — four constants at the top of `simulation/launch_sim.py`:
   - `ROOT` — this repository's absolute path
   - `PROJECT` — your `CarlaUnreal` directory
   - `ENGINE` — your CARLA UE 5.8 engine directory
   - `DLSS_SDK` (line ~45) — the SDKs/DLSS path inside your CARLA checkout
4. Point `simulation/lifecycle.py` line ~10 `PYTHON` at the venv from step 2. If you plan to run the SAM3 backend, also update the SAM3 venv path at line ~43; the YOLO26 backend needs nothing extra.

## 4. Build the native components

```bash
cmake -S mapping -B mapping/build -DCMAKE_BUILD_TYPE=Release
cmake --build mapping/build -j4
cargo build --release --offline --locked --manifest-path planning/Cargo.toml
```

## 5. Run and verify

```bash
./lab start --rviz --people yolo26   # empty scene, 6 pedestrians, RViz on ROS domain 45
./lab status                          # both robots should leave waiting_path and start accruing goals
./lab stop
```

Sanity checklist after startup: Go2 spawns at (-8, -8, 0.33), the Bee at (-8, -3, 2.0); `people_backend` reports `yolo26`; RViz shows the two LiDAR point clouds and the cyan/orange trees. Then run the test suite:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  .venv/bin/python -m pytest -q tests -p no:cacheprovider
```

## Windows teammates

Scene authoring is fully possible on Windows: import `ue/Content` into your own CARLA UE 5.8 project, edit, hand the `.umap` back (see [`ue/README.md`](../ue/README.md)). The runtime itself — systemd units, ROS 2, the Linux CARLA build — runs on Linux; reproduce there via this guide, or work against the shared Linux machine.
