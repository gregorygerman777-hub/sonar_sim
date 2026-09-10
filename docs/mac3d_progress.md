# Animated Mac / Unreal integration, 9 September 2026

## Failures, corrections and blockers

1. The initial request was interpreted as a website before the user specified Unreal.
   A starter exists at `/Users/gregsobe/Projects/abyss-sonar`; no Site was registered,
   deployed or opened. It is not a deliverable and has no running server.
2. An early standalone Unreal ray-trace sketch exists at
   `/Users/gregsobe/Projects/AbyssSonarUE`. After the supplied brief identified the
   existing repository, that approach was abandoned. The deliverable Unreal source
   is under this repository's `unreal/AbyssSonar` and compiles the existing core.
3. Unreal Editor, Epic Games Launcher and full Xcode were not found. `xcode-select -p`
   reports `/Library/Developer/CommandLineTools`. The build launcher exits 2 with
   a concrete engine-path message. No Unreal compilation, launch, graphics or
   packaging result is claimed.
4. The first Pygame perspective preview emitted NumPy RuntimeWarnings at small
   matrix products. The render completed, but the warning cause was not proven.
   Replacing those three-coordinate BLAS products with explicit elementwise sums
   removed the warnings. Subsequent renderer checks treat RuntimeWarnings as errors.
   No warnings were suppressed and no acoustic equations were changed.
5. Browser/web fetches of the DOI targets and MDPI full text failed with safety/
   availability errors and HTTP 429. Institutional records verified the 2016
   image model and 2020 stereo paper metadata/abstracts. This run does not claim
   to have read every full-text derivation. The existing paper mapping and its
   explicit unimplemented boundary remain in force.
6. The supplied Neural Observatory URL failed in the web fetch tool but loaded in
   the browser. Its dark laboratory layout, guided experiment, parameter column,
   central visualization and inspector were inspected successfully.
7. Native UI inspection was blocked because the Mac is locked. The preview process
   started and loaded the checked extension, but visible on-device operation was
   not verified. Unlocking the Mac is required for that last check.

## Deliverables

- `python/observatory_3d.py`: animated, native Pygame perspective preview, explicitly
  labeled as a Mac research preview. It is not an Unreal render.
- `launch_observatory.command`: launch using the fresh, checked Cython environment;
  compares core files with that snapshot to prevent silent use after source changes.
- `unreal/AbyssSonar`: Unreal C++ source project with explicit frame conversion,
  shared-core build wrappers, native experiment HUD, geometry, visual sweep,
  sonar fan, A-scan, current-view feasible carving and raw ping export.
- `output/observatory_3d.png`, `output/observatory_3d.gif`: actual preview output.
- `output/observatory_mesh.png`, `output/observatory_multipath.png`: experiment views.
- `tests/test_observatory.py`, `tests/check_observatory_ui.py`: model-state checks and
  actual Pygame input event checks.
- Higgsfield concept completed, job `2ecaad5b-acf5-4207-80b9-a16c4d58ec84`.
  Provider returned model `nano_banana_2`, 2752×1536, despite the requested model
  being `nano_banana_pro`. Receipt is in `unreal/AbyssSonar/Concept/higgsfield.json`.
  Concept art is not geometry and does not drive raw acoustic data.

## Clean integration result

Fresh source copy and new virtual environment:
`/Users/gregsobe/sonar_sim_mac3d_audit_20260909`.

Exact command from that directory: `.venv/bin/python python/reproduce.py`.
Full logs, source hashes, dependency versions, compiler, source snapshot, output
artifacts and manifest:
`results/2026-09-09_114703_113805/` in that clean copy.

**PASS: all 30 stages**, including independent Cython and CMake builds,
43 analytic assertions, 6 research tests, demos 1–17, the 38-cell notebook,
console input events, 1280×800 and 1920×1200 console captures, and 120 animation
frames. Python actually imported the freshly built
`sonar_sim_mac3d_audit_20260909/sonar.cpython-39-darwin.so`.

| Measurement | Observed result |
|---|---:|
| Existing +elevation/-elevation relative image difference | 2.428e-14 |
| Plane intensity maximum relative error, 1–10 m | 2.277e-16 |
| Speckle standard deviation / mean | 0.9999 |
| Speckle fraction below 0.1 of mean | 0.0946 (theory 0.0952) |
| Chirp half-power range width | 11.06 mm (theory 11.08 mm) |
| Space carving IoU, 4 / 8 / 16 / 32 views | 0.4475 / 0.6308 / 0.6936 / 0.7207 |
| New bridge +elevation/-elevation relative difference | 5.9803e-15 |

The bridge test compiles on the local AppleClang toolchain independently of Unreal:
`unreal/AbyssSonar/Tests/run_core_checks.sh`. It passes core parity, coordinate
round-trip, raw/display separation, deterministic speckle, OBJ and mask checks.
Its configuration is 97 bearings × 256 ranges × 512 elevation rays, 600 kHz,
top-hat response, one thread.

The new preview tests pass 5 cases with RuntimeWarnings treated as errors:

```sh
SONAR_CORE_DIR=/Users/gregsobe/sonar_sim_mac3d_audit_20260909 \
/Users/gregsobe/sonar_sim_mac3d_audit_20260909/.venv/bin/python \
-W error::RuntimeWarning tests/test_observatory.py
```

Replace the final path with `tests/check_observatory_ui.py` for event checks.

## Performance scope

Preview captures use 97 bearings × 256 ranges × 256 elevation rays, four acoustic
threads, 600 kHz, 60° horizontal field, 30° vertical beam, top-hat response.
The sphere scene has one analytic sphere and one plane. A representative measured
acoustic render was 0.397 ms. The concave scene has one plane and 612 OBJ triangles;
its representative acoustic render was 44.46 ms during simultaneous capture checks.
These are individual samples, not a benchmark distribution.

The 90-frame capture and GIF path took 6.38 s (14.11 loop iterations/s), including
software drawing, resizing and image recording. That is not Unreal FPS or an
isolated acoustic throughput number. Other preview session files record their own
configuration and wall time. The baseline benchmark has separate 20-sample
statistics and explicitly excludes GUI overhead from render throughput.

## Implemented / approximate / pending

| Component | Current state |
|---|---|
| Existing C++/Cython model and experiments | Preserved, clean rebuild passed |
| Native Mac animated preview | Runs; captured offline; desktop inspection blocked by lock |
| Reference-inspired layout | Implemented in preview and drafted in Unreal HUD |
| Same triangle assets in acoustics and 3-D display | Implemented; analytic sphere uses display tessellation |
| Elevation flip, shadows, multipath and concave targets | Interactive preview experiments |
| Unreal integration | Source only; engine compilation pending |
| UE cinematic canyon, detailed submarine and wreck | Not completed; Higgsfield concept only |
| Real-ocean or DIDSON calibration | Not implemented |
| 2024 paper's ICP/IRLS / patch-motion E_mu / ghost removal | Not implemented |
| 2020 paper's full stereo pipeline | Not implemented; existing point recovery stays separate |
| Full new-console control/panel inventory from the long brief | Partial; existing research console preserved |

The scientific model retains the existing assumptions: idealized beam response,
diffuse coefficients, point-scatterer r^-4 spreading, Thorp absorption, approximate
planar multipath and roughness, no reflected-leg visibility, no complete concavity
reverberation, known poses and segmentation-dependent feasible volumes. A dark bin
is not automatically a measured shadow. Synthetic validation is not calibration.

## Reopen and demonstrate

From `/Users/gregsobe/sonar_sim`, run `./launch_observatory.command`.
Space pauses, Enter pings, 1–4 selects experiments, E flips elevation,
A/D shifts sideways, W/S shifts vertically, arrows change sonar roll,
right-drag orbits the display camera, P saves a screenshot, F toggles fullscreen,
and Q/Escape quits. Click the fan to select the A-scan bearing.

First show experiment 1, pause and flip elevation; explain range and bearing stay
unchanged although height changes. Then show experiment 2 and move sideways to
change the highlight/shadow pattern. For multi-view feasible-volume reconstruction,
use the preserved `python/research_console.py` and K; the new preview does not
pretend to expose the entire reconstruction workflow yet.

No commits, pushes or deployments were made. The repository already had extensive
uncommitted work; it was preserved. This turn adds the files listed above and a
short README entry, rather than claiming ownership of existing changes.
