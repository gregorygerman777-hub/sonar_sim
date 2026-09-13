# Eigenray multipath: surface occlusion fix and a new seabed boundary, 13 September 2026

Prompted by comparing this simulator against the Acoustics Toolbox
(BELLHOP/KRAKEN/SCOOTER/RAM, M. B. Porter) and Attia et al.'s *Towards
Realistic 3D Sonar Simulation* (arXiv:2606.06130), which names those solvers
as the physical-realism standard a real-time sonar simulator is still
approximating. See [paper_mapping.md](paper_mapping.md) for the full citation
and an honest account of how far this falls short of them.

## What changed

- **Reflected-leg occlusion, both boundaries.** The surface ghost/mirror paths
  previously assumed the bounce always reached the sonar; `docs/scientific_status.md`
  and `demo17`'s own metrics said as much ("reflected-leg occlusion is not
  evaluated"). `geometry.h/.cpp` gained `segment_occluded`, and both boundaries
  now check the two real-space legs of the fold (target to bounce point, bounce
  point to sonar) against the whole scene before depositing anything. A hull or
  seabed feature between a target and a boundary now correctly removes that
  reflection instead of always counting it.
- **A seabed boundary**, built with the identical image-method construction as
  the surface (`SonarConfig::bottom_enabled/bottom_z`), but with a real
  fluid-fluid reflection coefficient instead of an assumed pressure-release -1:
  `physics.h`'s `bottom_reflection_coefficient` implements the standard
  Rayleigh two-fluid formula (impedance and Snell's law in grazing form), which
  is what gives it a critical grazing angle, `acos(c_water / c_bottom)`, below
  which the boundary reflects totally -- the same cutoff BELLHOP's ray trace
  shows over a fast bottom. `bottom_speed_mps`, `bottom_density_kgm3`,
  `water_density_kgm3` and `bottom_rms_height_m` are the new physical knobs;
  `bottom_ghost_enabled`/`bottom_mirror_enabled` isolate its two components the
  same way the surface's `ghost_enabled`/`mirror_enabled` already did.
- `simulator.cpp`'s surface-only multipath code was refactored into
  `deposit_boundary`, parameterized by boundary kind (surface or seabed), so
  both paths are now one reviewed implementation instead of two.
- New unit tests (`tests/test_units.py`): the Rayleigh coefficient at normal
  incidence and at/around the critical angle against an independent Python
  re-derivation, the roughness factor against its closed form, `segment_occluded`
  against a hand-built blocking wall, a floor that should (and does) remove a
  seabed reflection, and the existing 1-thread/8-thread bit-exactness check
  extended to cover the new boundary.
- New demo (`python/demo18_bottom_multipath.py`): sweeps the sediment sound
  speed at *fixed geometry* (a single boresight ray against a plane target, so
  the hit point and grazing angle are exact, not beam-averaged), which holds
  every geometric factor constant and isolates the reflection coefficient
  itself. Measured rendered energy is checked against the closed-form R² to
  within 1%, not just plotted alongside it.

## What this is not

- Not a sound-speed-profile ray trace. Both boundaries are still flat planes;
  there is no refraction, and BELLHOP's actual value over the image method is
  precisely for range-dependent, depth-varying sound speed, which this does
  not attempt.
- Single bounce per boundary only. Surface-then-bottom or bottom-then-surface
  combination paths (second order and beyond) are a natural extension of the
  same `deposit_boundary` construction -- the grazing angle is provably
  constant along the whole folded path between two parallel horizontal
  boundaries, which is what would make a recursive version tractable -- but
  are not implemented here. At this simulator's frequencies and ranges
  (`max_range_m` of order 10 m, absorption already significant), higher-order
  paths are expected to fall well below the noise floor represented by
  anything already modelled; that expectation is not itself validated.
- No coherent phase. `bottom_reflection_coefficient` returns a signed
  amplitude, but only `fabs`/its square is ever used; the phase flip a real
  reflection carries is discarded, consistent with how this simulator has
  always summed multipath as incoherent intensity rather than a complex field.
- The default bottom parameters (1650 m/s, 1900 kg/m^3) are a generic
  "sandy sediment" stand-in, not measured values for any real site or dataset.
