# Boat-mounted sonar survey laboratory

Run `./launch_survey.command`, or `.venv/bin/python python/survey_lab.py` from the repository.

This native Pygame perspective viewer uses the existing C++/Cython acoustic engine.
It adds no dependencies and preserves the earlier consoles. The scene, range-bearing
image and feasible volume share the exact acquired position, axes and scene geometry.
The vessel travels a straight survey line along its selected heading at the selected
speed. The white surface track shows its acquired positions. Acquisition runs at up
to four pings per simulated second; movement commits with a ping so the displays do
not mix old measurements and new poses. Camera motion is independent.

## Controls

| Action | Control |
|---|---|
| Run/pause | Space or Pause/Run |
| Move boat in world X/Y | A/D and W/S, including while paused |
| Heading and sonar tilt | Labeled sliders, degrees |
| Orbit and zoom | Right-drag scene and mouse wheel |
| Perspective / top camera | 1 / 2 or camera buttons |
| Sphere, cylinder, pipe, wreck | M or Target button |
| Speckle / planar multipath | N / G or buttons |
| Scientific transparent-water view | C or Clear water |
| Reset position and accumulated volume | R or Reset |
| Capture image and data | P or Capture |
| Export data | E or Export |
| Record PNG sequence, at most 300 frames | V or Record |
| Fullscreen / quit | F / Q or Escape |

Range, frequency, FOV, vertical beam width, target, speckle and multipath changes
start a fresh volume and recording of measurements. Heading, tilt and boat movement
add observations to the same volume. Gain and speed leave the current measurement
untouched. Gain affects display only; the intensity legend is relative to each
ping's peak before gain. Frequency is kHz in the control and Hz in the backend.

## Reconstruction and scientific boundaries

The ROI is fixed in world coordinates, approximately X [-1.5,1.5], Y [3,6],
Z [-3,-1.1] metres, sampled at 0.12 m. It is an operator-selected search region,
not a truth-occupancy mask. The selected target geometry is overlaid in amber for
comparison and never passed to the carving algorithm. Cyan points are observed,
surviving voxel centres. Unobserved voxels are retained but hidden, with their
count stated. The renderer shows every second surviving centre to reduce clutter.

Every displayed linear-intensity measurement, including optional speckle and
multipath, is thresholded at 3% of its peak. Highlight plus conservative farther-
range masks feed the existing `sonar.carve` function with one-bin tolerances.
This accumulates actual survey pings, not a separate target-only orbit. Seabed,
clutter, imperfect masks, limited view diversity and multipath can leave extra
volume or remove true voxels. No calibrated shape accuracy or unique solution is
claimed. Multipath is not removed before carving.

Sphere scattering is analytic; its visual triangle tessellation approximates that
sphere. Cylinder, pipe, wreck and rocks share the same triangle geometry in both
views. A flat acoustic seabed corresponds to the displayed grid at Z=-3 m.
The grid is 1 m; its visible extent is finite, while the acoustic plane is infinite.
The translucent fan uses the true spherical range and angular support in the
acquired sonar frame. It is a field-of-view overlay, not a sound-speed animation.

The boat, water contours, surface tint and track are presentation only. The
transducer is 0.65 m below water, below the 0.28 m hull keel. No vessel scattering,
wave-driven motion, refraction or hydrodynamics is simulated. The existing ideal
top-hat beam, diffuse first-hit angular mean, attenuation and optional planar
multipath remain uncalibrated approximations. This is a research visualization,
not a photorealistic renderer or a hardware-calibrated sonar model.

## Evidence and exports

Each session saves under `results/survey_TIMESTAMP/`. `survey.npz` contains raw
returns, the noise-enabled linear images actually carved, per-ping positions and
axes, ROI voxel centres, kept mask and observed mask. `survey.json` records the
acoustic configuration and approximation boundaries. The session caps at 500
stored pings; export and reset to continue. PNG recording uses wall-clock capture
at up to 10 fps, independent of simulated time, and is not a video container.

Run `.venv/bin/python tests/test_survey_lab.py` for synchronization, replay,
parameter isolation, noise provenance, coordinate geometry, data export and
native control checks. Existing `tests/test_units.py`, `tests/test_research.py`
and `tests/test_observatory.py` remain applicable.

For repeatable windowless rendering:

```
SDL_VIDEODRIVER=dummy .venv/bin/python python/survey_lab.py --frames 12 --fixed-dt 0.1 --screenshot output/survey_preview.png
```

The interface uses a native desktop layout with a minimum 1000x625 window,
not a mobile web page. Design QA uses offscreen app renders and native input
events; browser DOM checks do not apply.

Verified 10 September 2026: all 11 survey tests, the analytic unit checks,
6 research tests and 5 observatory tests passed. Rendered and inspected sphere,
wreck, tinted-water and minimum-size top views; exercised all sliders and main
buttons through native input events. Capture/recording outputs and exported data
were checked. Final default 12-frame capture completed with 5 synchronized pings.
Text contrast, clipping, keyboard hints and the separation of measured versus
presentation controls were reviewed. No acoustic core changes were required.
