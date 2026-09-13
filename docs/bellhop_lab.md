# BELLHOP propagation laboratory

The Acoustics Toolbox archive contains Fortran source and Windows executables.
The `.exe` suffix is also used by its Unix Makefiles, but the supplied Windows
binaries cannot run on Apple silicon. The installation script compiles the
Fortran source with `gfortran` under `~/.local/opt`, outside this MIT repository.
Both supplied archives declare GPL-3.0, so their source is not copied into the
project.

The simulator adapter follows the same workflow demonstrated by PYAT:

1. Define frequency, water column, source, receivers, seabed and ray fan.
2. Write a BELLHOP environment file.
3. Execute the external solver for a ray trace and for arrivals.
4. Parse `.ray` path vertices and `.arr` amplitude, phase, delay, launch/receive
   angle and surface/bottom bounce counts.
5. Pair reciprocal one-way eigenrays into monostatic two-way paths.
6. Deposit direct/direct, mixed and reflected/reflected energy into object,
   ghost and mirror range bins of a geometric FSS image.

The default sound-speed profile is piecewise linear between its surface and
bottom controls. BELLHOP bends paths through that profile and handles surface
and seabed interactions. This is materially richer than reflecting the source
through one flat plane.

`demo21_bellhop_fss.py` performs the first working two-way coupling. For each
lit geometric range bin it selects the nearest BELLHOP receiver range. Every
outgoing arrival is paired with every reciprocal return arrival. The apparent
range is `c_ref (tau_out + tau_back) / 2`, and the path intensity is
`|A_out A_back|^2`, normalized to the direct/direct path.

This coupling is range-depth and incoherent. Because elevation has already
collapsed in the FSS image, the demo assigns all lit bins one configured target
depth. It does not preserve complex phase, model broadband pulse interference,
or solve a separate environment for every triangle patch. Those are required
before calling it a complete BELLHOP target-scattering simulator.

The first assumption likely to fail in real water is the range-independent,
linear sound-speed profile. Temperature and salinity structure can vary with
both depth and horizontal position, moving caustics and arrival times.
