# Integration audit, 8 September 2026

Baseline: commit c488012e76db6ea2408dde335af768ae6f1aadc8, clean working tree.
Fresh local clone: `/Users/gregsobe/sonar_sim_integration_20260908`.
Fresh isolated environment: Python 3.9 (system python3); original environment is a
symlink to `/Users/gregsobe/fls_sim/.venv` and has not been removed.
Baseline logs and measurements are preserved in that clone.

## Findings and corrections

- Dependency installation initially failed with DNS errors inside the sandbox.
  Retried with approved network access; installation succeeded.
- `run_all.sh` process-substitution logging failed at `/dev/fd/62` inside the
  sandbox. Baseline run used approved execution. New reproduction driver uses
  Python subprocess logging to avoid this shell dependency.
- README installation omitted pygame, Pillow and notebook execution dependencies.
  Add an explicit requirements file.
- Baseline reproduction excludes notebook/CMake/UI checks and records no manifest.
  Add those stages and capture failures, interpreter, imported extension, source
  hashes, revision, environment and per-stage timings in a unique run directory.
- Camera centroid emits spurious divide/overflow/invalid matmul warnings on this
  NumPy/macOS environment, despite finite results. Replace small dot products
  with explicit elementwise reductions; represent a genuinely empty mask with
  NaN coordinates instead of division by zero.
- Console moves a fixed distance per displayed frame, including after slow
  renders, and recomputes paused pings. Use elapsed seconds and parameter caching.
- Renderer sums samples without quadrature normalization: increasing elevation
  samples increases raw power. Add a normalized elevation average, with a named
  legacy-sum option to reproduce earlier measurements. This is an angular mean
  of the stated patch-intensity approximation, not calibrated receiver power.
- Multipath bearing indexing truncates negative floating values toward zero,
  incorrectly admitting arrivals just outside the left field boundary. Use floor.
- Existing multipath is seeded only from direct-visible patches and does not test
  occlusion on the reflected leg. Document this limitation explicitly.
- Existing thread-equality test uses zero roll so it cannot establish general
  bitwise equality with cross-bearing mirror deposition. Test rolled geometry
  with a declared roundoff tolerance.
- New mesh code initially failed on Python 3.9's macOS deployment target because
  `std::filesystem` required a newer OS. Replaced its sole use with simple relative
  OBJ-material path resolution; C++17 builds then passed.
- Notebook execution was blocked by sandbox local socket restrictions. Approved
  execution passed. The runner explicitly selects the current interpreter to
  avoid an unrelated Jupyter kernel environment.
- New notebook prose initially used non-raw Python strings for LaTeX, turning
  theta/alpha/frac/right into escape characters. Corrected the stored notebook
  cells and inspected equation strings before final execution.
- Research-console display controls initially triggered a new speckle draw.
  Separated physical and display/camera cache keys; an event-driven test verifies
  changing gain and pages leaves one acoustic ping across eight display frames.
- Original shadow demo cropped at 9.5 m although the configured range was 10 m.
  Extended the plotted limit to the full configured range.
- Added target-relative coordinates for sphere/cylinder texture translation;
  the old world-coordinate field stayed fixed when these targets moved.
- Corrected voxel surface export to emit only exposed faces with shared vertices,
  instead of retaining internal faces of boundary cubes.
- The legacy console is retained as a historical front end. Elapsed-time motion
  and caching corrections apply to the new primary `research_console.py`.
