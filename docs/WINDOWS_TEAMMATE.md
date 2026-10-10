# Run the stack from a Windows machine

Windows cannot run the launcher natively. The launcher depends on systemd user services (how it starts and names its background processes), on ROS 2 Humble, and on a Linux build of CARLA (the simulator, built on Unreal Engine 5.8).

This page lists the four realistic ways to get from "edited a scene" to "watched Go2 and the Bee run on it". They are ordered by effort. Start with Option 0.

## Option 0: remote into a Linux machine (recommended, zero setup)

The full stack already runs on the team's Linux hosts. Work on those from your Windows machine.

1. Connect with VS Code Remote-SSH. You get the full edit/build/run loop: terminal, `cargo build`, `pytest`, `./lab start`.
2. Pick one way to see RViz (the 3D viewer): the remote desktop of the Linux machine; `ssh -X` with VcXsrv or X410 (X servers for Windows); or `rviz2` inside WSLg (see Option 1).
3. Hand scene files over in either direction: copy the `.umap` in, restart with `./lab start`, watch.

You are done when: you can edit, run `./lab start`, and see RViz without leaving your Windows desk. This is the lowest-friction path.

## Option 1: hybrid, CARLA on Windows + Python stack in WSL2 (local, proven pattern)

The simulator and the Python stack are separate processes talking over TCP. They do not have to share an OS. The community-proven Windows layout is exactly this split (see sources in the repo discussion): run the CARLA server natively on Windows, with native Vulkan drivers on an officially supported platform, and run everything else inside WSL2 (ROS 2, the venv (the Python environment), planner, map service), connected over localhost.

Windows side:

1. Build or obtain CARLA UE 5.8 for Windows. Running the robots needs the CARLA fork of UE 5.8 (Epic's engine with CARLA's changes); vanilla UE is only enough for editing scenes.
2. Open the `CarlaUnreal` project with this repo's `ue/Content` imported (see [`ue/README.md`](../ue/README.md), Option 2). Levels are cross-platform.
3. Start the editor with map `AirGroundLab_Main` and CARLA RPC port 2400.

You are done when: the editor serves `AirGroundLab_Main` and port 2400 answers.

WSL2 side (Ubuntu 22.04):

1. Enable systemd in `/etc/wsl.conf` with `[boot] systemd=true`.
2. Run `loginctl enable-linger $USER` so `systemctl --user` works.
3. Follow `docs/SETUP_LINUX.md` steps 1-2 and 4 (prerequisites, venv, cargo/cmake builds).
4. RViz works out of the box under WSLg on Windows 11. On Windows 10, use VcXsrv.
5. Point the stack at the Windows host instead of localhost for the CARLA connection, and skip the launcher's own UE spawn. The startup probe in `simulation/lifecycle.py` already accepts any CARLA on port 2400 serving `AirGroundLab_Main`. Attaching to the Windows-side simulator needs only a small launcher flag; ask and we will add it (a one-line change, since the probe logic is already there).

You are done when: the WSL2 half of `docs/SETUP_LINUX.md` builds and runs, and your Windows editor listens on port 2400.

Caveats, honestly stated:

- WSL2 localhost forwarding usually makes `127.0.0.1:2400` on the WSL side reach the Windows host. Mirrored networking mode on Windows 11 is the most reliable.
- RViz under WSLg works, but heavier scenes tax it.
- The Windows CARLA build is a large download and compile.

Everything except UE also runs fine in WSL2 alone. The perception/OHZ inner loop (`pytest`, `perception/`) never touches the simulator, so day-to-day shape work needs only the WSL2 half.

## Option 2: everything inside WSL2 (experimental, not recommended)

Running the UE editor itself inside WSL2 is unreliable. WSL2's GPU paravirtualization targets compute (CUDA) first, so UE and CARLA under WSL2 hit Vulkan driver problems: "Incompatible Vulkan driver found", CUDA context faults, llvmpipe fallbacks. Driver pinning sometimes helps.

Treat this route as experimental. Use Option 1's split instead; it keeps Vulkan on native Windows drivers.

## Option 3: dual-boot Ubuntu

Always works, with zero compromises. It is just the Linux setup from `docs/SETUP_LINUX.md` on bare metal. It costs a partition and a reboot to switch.

## What each role actually needs

| Role | Minimum viable setup | Full experience |
|---|---|---|
| UE scene author | Vanilla UE 5.8 editor on Windows ([`ue/README.md`](../ue/README.md) Option 1) | Plus any Option above, to watch scenes run |
| OHZ shape work | WSL2 alone: venv + `pytest`. No simulator needed for the inner loop. | Plus Option 0 or 1, for integration runs on the robots |
