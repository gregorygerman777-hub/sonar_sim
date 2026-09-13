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
5. Plot paths in range-depth space and arrivals as a delay-amplitude stem plot.

The default sound-speed profile is piecewise linear between its surface and
bottom controls. BELLHOP bends paths through that profile and handles surface
and seabed interactions. This is materially richer than reflecting the source
through one flat plane.

The forward-scan image renderer has not yet been converted into a BELLHOP
two-way scattering model. Such a conversion needs eigenrays from the sonar to
each target patch, reciprocal return propagation, complex phase, target
scattering strength and coherent or incoherent path summation. Calling the
current propagation display a complete BELLHOP FSS image simulation would be
incorrect.

The first assumption likely to fail in real water is the range-independent,
linear sound-speed profile. Temperature and salinity structure can vary with
both depth and horizontal position, moving caustics and arrival times.
