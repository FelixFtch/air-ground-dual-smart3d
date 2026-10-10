# Running the stack from a Windows machine

Windows cannot run the launcher natively (systemd user units, ROS 2 Humble, Linux CARLA). This page lists the realistic ways a Windows teammate gets from "edited a scene" to "watched Go2 and the Bee run on it", ordered by effort.

## Option 0 — remote into a Linux machine (recommended, zero setup)

The full stack already runs on the team's Linux hosts. From Windows:

- **VS Code Remote-SSH** into the Linux machine — full edit/build/run: terminal, `cargo build`, `pytest`, `./lab start`.
- **RViz**: the Linux machines run RViz on their desktops (remote desktop), or forward it with `ssh -X` + VcXsrv/X410 on Windows, or `rviz2` inside WSLg (below).
- Scene files hand off both ways: copy `.umap` in, restart `./lab start`, watch.

This is the lowest-friction path and what we suggest starting with.

## Option 1 — hybrid: CARLA on Windows + Python stack in WSL2 (local, proven pattern)

The simulator and the Python stack are separate processes talking over TCP — they do not have to share an OS. The community-proven Windows layout is exactly this split (see sources in the repo discussion): **run the CARLA server natively on Windows** (native Vulkan drivers, officially supported platform) and **run everything else inside WSL2** (ROS 2, venv, planner, map service), connected over localhost.

Windows side:
1. Build or obtain CARLA UE 5.8 for Windows and open the `CarlaUnreal` project with this repo's `ue/Content` imported (`ue/README.md`, Option 2). Levels are cross-platform.
2. Start the editor with map `AirGroundLab_Main` and CARLA RPC port 2400.

WSL2 side (Ubuntu 22.04):
1. Enable systemd (`/etc/wsl.conf` → `[boot] systemd=true`) and `loginctl enable-linger $USER` so `systemctl --user` works.
2. Follow `docs/SETUP_LINUX.md` steps 1–2 and 4 (prereqs, venv, cargo/cmake builds). RViz works out of the box under WSLg on Windows 11; on Windows 10 use VcXsrv.
3. Point the stack at the Windows host instead of localhost for the CARLA connection, and skip the launcher's own UE spawn: the startup probe in `simulation/lifecycle.py` already accepts any CARLA on port 2400 serving `AirGroundLab_Main`, so attaching to the Windows-side simulator needs only a small launcher flag (ask and we will add it — one line change; the probe logic is already there).

Caveats, honestly stated: WSL2 localhost forwarding usually makes `127.0.0.1:2400` on the WSL side reach the Windows host (mirrored networking mode on Windows 11 is the most reliable); RViz under WSLg is fine but heavier scenes tax it; and the Windows CARLA build is a large download/compile. Everything except UE also runs fine in WSL2 alone — the perception/OHZ inner loop (`pytest`, `perception/`) never touches the simulator, so day-to-day shape work needs only the WSL2 half.

## Option 2 — everything inside WSL2 (not recommended)

Running the UE editor itself inside WSL2 is unreliable: WSL2's GPU paravirtualization targets compute (CUDA) first, and UE/CARLA under WSL2 hit Vulkan driver incompatibilities ("Incompatible Vulkan driver found", CUDA context faults, llvmpipe fallbacks). Driver pinning sometimes helps, but treat this route as experimental — use Option 1's split instead, which keeps Vulkan on native Windows drivers.

## Option 3 — dual-boot Ubuntu

Always works, zero compromises — it is just the Linux setup in `docs/SETUP_LINUX.md` on bare metal. Costs a partition and a reboot to switch.

## What each role actually needs

| Role | Minimum viable setup | Full experience |
|---|---|---|
| UE scene author | Vanilla UE 5.8 editor on Windows (`ue/README.md` Option 1) | + any Option above to watch scenes run |
| OHZ shape work | WSL2 alone: venv + pytest — no simulator needed for the inner loop | + Option 0/1 for integration runs on robots |
