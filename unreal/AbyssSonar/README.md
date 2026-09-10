# Sonar Observatory: Unreal presentation

**Source prototype. Not compiled or run in Unreal yet.** This Mac has Command Line
Tools but no Unreal Editor or full Xcode installation. The independent C++ bridge
checks pass; they do not validate Unreal APIs, shaders, packaging or frame rate.

This project compiles the existing repository's `core/` directly. It does not
substitute an optical depth buffer or single collision ray for forward-scan sonar.
Keep this project in `sonar_sim/unreal/AbyssSonar`; the relative core path is intentional.
The original Cython, Pygame, demos and measurements are preserved.

## Build on Mac

1. Install Unreal Engine from [Epic](https://www.unrealengine.com/download).
2. Install the full Xcode version supported by that engine and your macOS version;
   [Epic's Mac requirements](https://dev.epicgames.com/documentation/unreal-engine/macos-development-requirements-for-unreal-engine)
   and the installed engine's Apple SDK configuration are authoritative.
3. Select full Xcode with `xcode-select` if Command Line Tools is still selected.
4. From this directory:

```sh
UNREAL_ENGINE_ROOT='/Users/Shared/Epic Games/UE_5.8' ./Scripts/build_mac.sh
```

Replace the example path with your actual engine root. The script builds the editor
module and launches Unreal with `Scripts/bootstrap.py`, which creates materials and
`/Game/Maps/Canyon` if absent. Open that map and press Play. Engine compatibility,
bootstrap APIs and runtime rendering remain unverified until this build succeeds.

## Controls

- Space: run/pause. Enter: one ping. R: reset selected experiment.
- 1–4: ambiguity, shadow, multipath, concave geometry.
- Arrow keys: move and turn. Page Up/Down: vertical motion. Comma/period: roll.
- Right mouse drag: orbit camera, without changing acoustic pose.
- E: flip target elevation. K: carve from current captured pose. P: export raw ping.
- F: request a 1440×900 borderless viewport. Escape: standard editor stop.
- Click the sonar fan to choose the A-scan bearing.

## Scientific boundary

The core supplies first-hit elevation integration, diffuse incidence, r^-4
point-scatterer spreading, Thorp absorption and optional approximate multipath.
Its metre-based, X-right/Y-forward/Z-up frame is converted explicitly to Unreal's
centimetre-based X-forward/Y-right/Z-up frame. OBJ winding is reversed for display
when axes are swapped. The same loaded OBJ triangles feed both the core and display.

Raw intensity is separate from gain, log compression and exponential display
speckle. The sweep is slowed to 1.2 seconds for inspection; physical round-trip
travel time is 2r/c. Fog, lights and camera orbit have no acoustic effect. The
sphere is analytically intersected and tessellated only for display. The seabed is
acoustically infinite but its drawing is finite.

Carving uses the existing segmentation-dependent conservative feasible shadow rule.
It is a visual hull from known poses, not unique recovered geometry. The native
prototype does not reproduce the 2024 ghost-removal objective or the 2020 paper's
complete stereo reconstruction. Existing Python experiments retain the quantitative
validation and Monte Carlo results. See `../../docs/paper_mapping.md` from the repo
root documentation tree, and `../../docs/mac3d_progress.md` for this run's status.

The native UI currently exposes four experiments, camera/pose motion, naive versus
integrated comparison, multipath, speckle, raw export and current-pose carving.
It does not yet implement the brief's full parameter inventory, cinematic wreck
assets, physically rendered shadow volumes, measured Unreal performance, or all
panels from the existing desktop console.

## Check the shared bridge without Unreal

```sh
./Tests/run_core_checks.sh
```

Checks cover coordinate conversion, exact parity with the unchanged core render,
elevation ambiguity, deterministic speckle, raw/display separation, OBJ rendering,
and disjoint highlight/feasible-shadow masks. Reports are saved in `Saved/checks/`.

Visual direction: the user's Neural Observatory reference at
https://wistful-rocket-bbsd.here.now/ (observed 9 September 2026), plus Higgsfield
job `2ecaad5b-acf5-4207-80b9-a16c4d58ec84`. The generated image is concept art,
not evidence of a working simulator or physically correct sonar.
