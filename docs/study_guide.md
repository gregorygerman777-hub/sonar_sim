# Your sonar research interview study guide

Prepared for Gregory German on 9 September 2026, for the planned meeting with
Professor Shahriar Negahdaripour on 11 September. This guide is grounded in the
local simulator and its recorded experiments, not a prediction of exactly what
the professor will ask. No single packet covers all underwater acoustics; this
one covers the project you should be ready to explain and defend.

## How to use this packet

Your aim is to explain a problem, show evidence, recognize a limitation, and
suggest a sensible next experiment. You do not need to sound like a PhD student.
When a word is unfamiliar, stop and explain it in your own language before
memorizing the equation around it.

| Priority | What you should be able to do |
|---|---|
| Essential: explain without notes | Range/bearing/elevation; why elevation disappears; first-hit rays; highlights and shadows; raw versus displayed intensity; what carving can and cannot recover |
| Important: explain with a drawing or equation | Coordinate transforms, incidence cosine, two-way spreading, absorption, speckle, known poses, IoU and uncertainty, direct/ghost/mirror geometry |
| Advanced: recognize and discuss honestly | Jacobian rank and conditioning, coherent roughness, array sidelobes, ICP/IRLS and the paper's patch-motion objective |

**If you have only two hours:** spend 25 minutes on geometry and the opening,
25 on the rendering loop, 25 on carving and results, 20 on noise and multipath,
15 on limitations and authorship, and 10 rehearsing aloud. Answer from memory,
then check the text. Re-reading alone can make material feel more familiar than
you can actually explain.

**A useful answer structure:** give the short answer, explain its mechanism,
point to a test or figure, then state the limitation. Example: "More views
constrain the feasible volume. Each view rejects incompatible voxel projections.
My sphere IoU rose from about 0.45 to 0.72 with 4 versus 32 poses. That still
left excess volume, so I do not claim exact geometry."

## A true 30-second opening

"I built this simulator to study why recovering 3-D shape from forward-scan
sonar is difficult. A pixel records range and left-right bearing, but combines
returns over elevation. I model that with several vertical rays and first-hit
visibility, then use known views to carve a feasible volume. My focus is testing
simple models and understanding their limits."

Use the longer version below if invited to expand.

## Your expanded project explanation

“I built a forward-scan sonar simulator to understand why reconstructing 3-D shape from sonar images is difficult. The sonar records distance and left-right bearing, but it does not directly record elevation. My simulator sends several virtual rays through the sonar’s vertical beam, finds the first surface each ray hits, and combines those echoes into range-bearing bins. It can generate highlights, shadows, speckle, and simplified sea-surface ghost and mirror returns. I then use several known sonar poses to test which 3-D voxels are consistent with all the images.”

Practice this slowly. It may take closer to 40 seconds at a comfortable speaking pace.

## Start with ordinary objects

Imagine standing in a dark room with a flashlight shaped like a tall, thin fan.
You know which way it points left or right, and you can measure how far away an
object is. But the return combines everything above and below within the fan.
A bright spot could come from high up or low down. That is the core problem.

Sound is a moving pressure disturbance. Water carries that disturbance. A sonar
transmits a pulse and times the echo. At 1500 metres per second, an echo arriving
0.004 seconds later represents 1500 × 0.004 / 2 = 3 metres. The division by two
accounts for the trip out and the trip back.

A camera pixel usually fixes two angles but not distance. A forward-scan sonar
pixel fixes distance and one angle but not elevation. Neither is automatically a
3-D point. An intensity is how strong a received signal is, not a depth coordinate.

## Symbols, read aloud before using them

| Symbol | Say | Meaning and units |
|---|---|---|
| P | point | Three coordinates, in metres |
| X, Y, Z | x, y, z | Right/starboard, forward, up |
| r | range | Distance from sonar, metres |
| θ | theta | Horizontal bearing; left-right angle |
| φ | phi | Elevation; up-down angle |
| λ | lambda | Wavelength, metres per cycle |
| f | frequency | Cycles per second, hertz; Thorp uses kilohertz |
| c | sound speed | Usually 1500 m/s in this simulator; optical c means attenuation instead |
| t | time | Seconds since transmission |
| I | intensity | Relative echo power in this simulator |
| R, sometimes μ | reflectivity | Assumed material coefficient; not calibrated target strength |
| n | normal | Unit arrow perpendicular to a surface |
| u | ray direction | Unit arrow pointing outward from sonar |
| n·(-u) | dot product | Facing-direction cosine; one head-on, zero grazing |
| α | alpha | Absorption loss in dB/km |
| TL | transmission loss | Here, two-way absorption in dB |
| N | element count | Number of ideal array elements |
| d | spacing | Element separation in metres |
| B(φ) | beam power | Angular power weight; one at boresight |
| B in c/(2B) | bandwidth | Chirp frequency span in Hz; different from beam B |
| T | duration | Pulse length in seconds |
| σ | sigma | Standard deviation, unless a surface-height subscript is present |
| σ_h | sigma h | RMS water-surface height, metres |
| k | wavenumber | 2π/λ, radians per metre |
| g | grazing angle | Angle measured above the reflecting plane |
| γ | gamma | Approximate coherent amplitude retention |
| U | uniform draw | Random number between zero and one |
| exp(x), ln(x) | exponential, natural log | Inverse mathematical operations |
| π | pi | About 3.14159; half a circle is π radians |
| β | beta | Sonar tilt in the paper's parameter estimation |
| D | depth | Distance of sonar below the surface |
| E_μ | objective E mu | The paper's patch-motion objective; not implemented here |

A **vector** is a short list of numbers, such as a point's three coordinates.
A **unit vector** has length one, so it describes direction alone. A **matrix** is
a table of numbers; a rotation matrix stores three sensor-axis directions. The
world-to-sonar transform subtracts sonar position and takes dot products with
those directions. The point has not disappeared; only its description changed.

A **radian** measures angle using arc length divided by radius. A complete circle
is 2π radians or 360 degrees. Code must convert degrees before using sine and
cosine. A **bin** is a measurement interval, like a labeled drawer. A **voxel** is
a small cube in a grid. A **mesh** is a surface assembled from triangles.

## Understand the equations one piece at a time

### Position and lost elevation

P = r [cos(φ) sin(θ), cos(φ) cos(θ), sin(φ)]. Brackets contain X, Y and Z.
At bearing zero, X is zero. At elevation zero, Z is zero. Range is the distance
from the origin, not just the forward Y coordinate.

The image address uses only r and θ. For fixed r and θ, changing φ moves the
point around an arc. That is a many-to-one mapping. For symmetric targets and an
even beam, changing +φ to −φ mirrors the scene while keeping the same image.
That claim requires symmetric beam/scene/material/visibility conditions. It is
not a claim that every arbitrary object above and below always looks identical.

Floating-point arithmetic stores finitely many digits. Adding the same numbers
in reverse order may change the last few bits. We test a numerical tolerance,
not an impossible promise that every equivalent calculation has identical bits.

### Echo brightness

I = R × max(0, n·(-u)) / r⁴ × 10^(−TL/10).

Read it as: material strength times how directly the surface faces the sonar,
weakened by distance and absorption. Doubling distance gives a factor of 1/16
before absorption in this chosen point-scatterer approximation. The outward
wave spreads, and the returning wave spreads again. Extended surfaces and
calibrated sonar systems can have different effective range dependence.

Thorp estimates frequency-dependent absorption. Frequency is converted from Hz
to kHz, and range from metres to kilometres. TL = 2αr_km counts two legs.
The factor 10^(−TL/10) converts a power loss in decibels to a linear multiplier.
For amplitude, the corresponding decibel denominator would be 20.

The raw array is relative linear intensity. Taking logs reveals weak signals.
Normalizing a display helps eyes but changes the apparent contrast; it must not
silently change reconstruction input. More display gain does not create more
information or stronger physical echoes.

### Beams and sampling

Wavelength equals sound speed divided by frequency. At 1.8 MHz and 1500 m/s,
wavelength is about 0.833 mm. The array formula adds regularly spaced elements'
phases, then squares amplitude. At boresight every phase aligns, so normalized
beam power is one. The zero-over-zero expression must be replaced by its limit.

Hann shading weakens the edge elements. It reduces sidelobes while widening the
main lobe. These are measured, not just drawn. Our ideal elevation array is not
an actual calibrated DIDSON lens response.

Many elevation sub-rays sample the missing angle. They are not extra camera
pixels. The first surface blocks the rest of that ray. Averaging samples gives
a converging approximation of an angular mean. Too few rays leave striped or
missing seabed bins. In one recorded scene 4096 rays gave about 0.92% relative
L1 error versus 8192; that threshold is specific to its configuration.

### Ghost and mirror

A flat surface lets us replace a reflected path with a straight line to a
virtual, mirrored sonar. Let direct distance be r_d and reflected-leg distance
be r_m. A path with one leg of each kind has apparent range (r_d+r_m)/2.
Two reflected legs give the mirror apparent range r_m. Mirrored geometry can
project into a different bearing after sonar roll. A direct-associated ghost
can persist while the mirror leaves the vertical field of view.

This implementation has limitations: it starts from direct-visible patches,
does not check obstacles on the reflected leg, and does not model phases or
full rough-surface scattering. The simplified coherence factor reduces coherent
amplitude; squaring converts it to power. At short wavelengths small roughness
can matter greatly. The decorative animated water does not alter this flat
acoustic interface.

### Speckle and averages

A Gaussian distribution is the familiar bell-shaped distribution. The real and
imaginary parts of many random-phase scatterers can approach Gaussian values.
Their combined magnitude has a Rayleigh distribution. Squaring it gives an
exponential intensity distribution. Multiplying true intensity by −ln(U)
simulates that distribution. Its mean and standard deviation are both one.

A probability distribution describes repeated outcomes, not a fixed texture.
Independent looks are separate realizations. Averaging L independent looks
reduces contrast approximately to 1/sqrt(L). Spatially correlated samples are
not independent extra evidence. Filtering complex amplitude preserves the
Gaussian model before squaring, whereas arbitrary blurring of intensity changes
the single-look marginal distribution.

### Reconstruction and uncertainty

A voxel survives if all observing views allow its projection. A highlight allows
a surface; a feasible shadow allows material hidden behind it. Unobserved water
provides no constraint. Our conservative mask permits all farther cells behind
a highlight on the same bearing, so it can retain too much volume.

IoU is overlap divided by union. A normalized volume error of +0.39 means 39%
more volume than truth. False-positive volume is outside truth; false-negative
volume is lost truth. A hollow or deeply concave object may have the same
silhouettes as a fuller object. Therefore this is a visual hull, not guaranteed
true shape.

Monte Carlo means repeating an experiment with random noise. The mean describes
average error, standard deviation describes spread, and a confidence interval
expresses uncertainty in a statistic such as the mean. Our bootstrap resamples
trials to estimate that interval. A 95% interval for the mean is not a promise
that 95% of individual errors lie inside it.

A Jacobian is a sensitivity table. Rank counts locally distinguishable point
motions. Small singular values signal weak information and noise amplification.
Coplanar sideways views retain a global above/below ambiguity even when a
nonzero-elevation point has local rank three. Vertical motion can break it.

### Chirp and pulse compression

A chirp changes frequency over time. Duration T determines how long it lasts;
bandwidth B is how much the frequency changes. BT is the time-bandwidth product.
The matched filter slides a copy of the known transmitted signal along the
received signal and scores similarity. A strong peak estimates echo delay.

Range resolution is approximately c/(2B), because greater bandwidth localizes
time better and range still counts a round trip. With B = 60 kHz it is 12.5 mm.
For a rectangular chirp, half-power peak width is approximately 0.886 times that
scale, about 11.08 mm. A long unmodulated pulse of the same duration spreads
returns over approximately cT/2 and can merge two echoes the chirp resolves.

The WAV demonstrations play slowed/sonified signals. A human does not directly
hear a MHz carrier. The waveform model is separate from the image renderer, so
changing its bandwidth does not secretly sharpen the image pixels.

## Flash cards and likely questions

| Professor asks | Short, honest answer |
|---|---|
| Why is elevation lost? | All elevation patches in a range/bearing cell contribute to the same recorded bin. |
| Why do +φ and −φ match? | Symmetry of geometry and even beam weights gives the same measurements, up to rounding. |
| Why several sub-rays? | To approximate the vertical accumulation and visibility within a bearing. |
| Why does intensity fall with range? | Two-way spreading and absorption weaken the return. |
| Why r⁴? | A declared point-scatterer two-way spherical-spreading approximation, not a universal measured law. |
| What does Thorp model? | Empirical acoustic absorption versus frequency, with explicit kHz and dB/km units. |
| What is incidence angle? | How a ray meets a surface relative to its normal. |
| What creates a highlight? | A visible, sufficiently reflective surface facing the sonar. |
| What creates a shadow? | A nearer first hit blocks farther surfaces along that ray. |
| What creates the ghost? | A longer mixed direct/reflected route whose delay is interpreted as range. |
| What is the mirror? | A component corresponding to the mirrored geometry and reflected path. |
| Why does roll help? | It changes which elevation plane collapses and shifts projected mirror geometry. |
| Why exponential speckle? | Squared Rayleigh amplitude from complex Gaussian scattering. |
| Why correlated speckle? | Finite resolution, beam/pulse response, interpolation and processing correlate neighbors. |
| What is space carving? | Remove voxels whose measured projections are inconsistent. |
| Why only a hull? | The data can allow multiple shapes, particularly hidden concavities. |
| Why do concavities cause trouble? | Their interior may never constrain a silhouette or receive visible rays. |
| Did you reproduce equation (6)? | No. I do not implement its contour-to-3-D-patch-motion and mirror-consistency construction. |
| How does this differ from DIDSON? | No full calibration of lenses, beams, pulse footprint, materials, noise or intensity scale. |
| What was analytically validated? | Ray intersections, array limits, absorption units, plane brightness, ambiguity and waveform delay/width. |
| What broke? | The clean run exposed environment/logging restrictions and centroid warnings; inspection found unnormalized ray sums and frame-dependent motion. The audit records corrections. |
| Did you remove real ghosts? | No; I generate approximate forward artifacts and label the missing inverse problem. |
| Did you write it alone? | I used AI as a coding assistant while testing the models and learning the physics. |

## Trick questions to practice

1. **Does twice as many bins mean twice the physical resolution?** No. Binning is
   sampling; waveform bandwidth and beam response limit physical resolution.
2. **Does a black pixel prove there is no object?** No. It may be shadow, low
   reflectivity, outside the beam, noise loss, or no observed surface.
3. **Does rank three prove a unique solution?** No. It is local; a distant mirror
   solution can still exist.
4. **Do more looks always improve this IoU?** No. The noisy-maximum threshold can
   change masks nonmonotonically. Show the measured curve and explain the rule.
5. **Does matching an equation validate the ocean?** No. It validates the
   implementation of that chosen equation under its assumptions.
6. **Does the console's FPS prove real-time mesh rendering?** Only for the stated
   geometry, resolution, rays, threads and machine. Paused display FPS is not
   physics throughput.
7. **Is the decorative water the scattering surface?** No. The acoustic interface
   remains planar; RMS roughness controls only the stated coherent attenuation.
8. **Are 32 synthetic views equivalent to 32 independent field measurements?** No.
   Shared model errors and correlated speckle can persist across views.

## Two-minute demonstration

- 0:00–0:30: Show `demo1_ambiguity.png`. Point at the two different 3-D positions
  and identical sonar images. Read the printed maximum difference.
- 0:30–1:00: Show `demo2_shadow.png`. Explain first-hit blocking and the full range
  axis. Mention sub-ray convergence rather than pretending stripes are texture.
- 1:00–1:30: Show `demo15_carving.png`. More poses shrink excess volume, but the
  result remains larger than truth. Name IoU and one limitation.
- 1:30–2:00: In the console, change roll and toggle ghost/mirror. Explain that
  these are forward artifacts, not a ghost-removal algorithm.

## Five-minute technical walkthrough

Spend one minute on coordinates and the lost elevation arc; one minute on the
C++ hit/beam/intensity/bin loop; one minute on analytic checks and raw versus
display intensity; one minute on voxel consistency, IoU and confidence intervals;
and one minute on multipath simplifications and the missing patch-motion work.
Open `core/reconstruction.cpp` to show the short consistency function. If asked
about equation (6), open the paper mapping instead of improvising a derivation.

## Your plan for September 9-11

**September 9: geometry and the code path.** Memorize the opening. Draw X/Y/Z, r/θ/φ and an ambiguity arc.
Calculate a 3 m round-trip delay. Run demos 1, 2 and 3. Explain a normal and the
dot product aloud. Write down anything you cannot yet explain.

**September 10: noise, reconstruction and rehearsal.** Run demos 4, 15 and 16. Define IoU, false
positive, false negative, standard deviation and mean confidence interval.
Explain why sideways poses can fail and why increasing views does not recover
every concavity. Practice the trick questions without reading answers.

**September 11, before the meeting: a short review.** Run the two-minute demonstration once. Practice
five minutes with code open. Review multipath and chirp figures. Read the failure
audit and scientific-status table. Prepare one thoughtful question: “Which
calibration measurement would most improve the validity of this forward model?”

## Practice ladder

Easy: convert 100 kHz to Hz; name the missing sonar coordinate; compute r for a
4 ms delay; explain a highlight in one sentence.

Intermediate: predict the pre-absorption intensity ratio at 2 m and 4 m; explain
why raw data must survive display scaling; compute wavelength at 1.8 MHz;
identify false positives on the carving slice.

Hard: derive the first array null from N d sin(φ)/λ = 1; explain why normalized
quadrature is necessary; distinguish local rank from global uniqueness; explain
why correlated speckle reduces effective sample count; defend a conservative
shadow mask without pretending darkness is direct evidence.

Research discussion: describe the missing patch correspondence and 3-D motion
steps before depth/tilt optimization; propose a calibration experiment with a
known cylinder and measured sonar poses; identify which synthetic metrics may
look good despite an incorrect real scattering law.

## What not to claim

Do not claim eight months of work, no assistance, exact paper reproduction,
institutional affiliation or endorsement, research-grade accuracy, ghost removal,
unique 3-D reconstruction, or direct human audibility of MHz sonar. Say what was
implemented, what was measured, and what you would validate next. It is fine to
say: “I do not know yet; I would test that with a controlled measurement.”

## The geometry you should be able to draw

### Bearing, elevation and the inverse formulas

For a point in the sonar frame, horizontal distance is sqrt(X² + Y²), and full
range is sqrt(X² + Y² + Z²). A square root reverses squaring. Bearing is
atan2(X, Y), and elevation is atan2(Z, sqrt(X² + Y²)). The two-input atan2
function chooses the correct quadrant; ordinary atan(X/Y) can confuse a point
ahead with a point behind. The argument order follows this project's Y-forward
convention and differs from the usual mathematical atan2(y, x).

Yaw turns the sensor left/right about up. Pitch or tilt points its nose up/down.
Roll rotates around its forward axis. In this project's helper, positive tilt
points downward. The sign of yaw follows the implemented right-handed rotation;
do not assume every marine compass convention uses that same sign.

Suppose the sonar is at world coordinates (1, 0, 0), and a target is at (1, 4, 0).
Subtract the sonar position: the relative point is (0, 4, 0). With aligned axes,
range is 4 m, bearing is zero, and elevation is zero. If the sonar rotates, take
the relative vector's dot product with each of its three unit axes to get the
local X, Y and Z. Translation changes the origin; rotation changes the axes.

### Why +elevation and -elevation can match

Cosine is even: cos(-φ) = cos(φ). Sine changes sign: sin(-φ) = -sin(φ). Thus
flipping elevation preserves X and Y and reverses Z. Range and bearing stay
fixed. If the whole relevant scene and scattering conditions are symmetric,
the corresponding vertical rays have equal weights and path lengths.

The *pixel address* ambiguity holds geometrically. Equality of the *whole image*
additionally needs that symmetry. A seabed, asymmetric shape, texture, beam or
surface reflection can break the intensity equality. Intensity may provide
model-dependent elevation clues without making sonar a direct elevation sensor.

### Why multiple views help, and when they do not

Each range/bearing pair allows an arc of possible points. Another known pose
creates another arc. Their common points can constrain the missing coordinate.
But the same surface point must be associated across views. **Correspondence**
means knowing which measurements come from the same physical feature; our point
experiment assumes it instead of solving the real matching problem.

Two sensors at the same depth looking sideways have a particularly important
failure: reflecting a target above/below their common horizontal plane preserves
both ranges and bearings. Near zero elevation, small height changes barely
change range, making height estimation sensitive to noise. Vertical displacement
can make the two signs produce different ranges. It is not true that any two
views always recover a unique, stable 3-D point.

## Readable code walkthrough

### One rendered image

Read this as pseudocode; the actual implementation also handles motion,
threading and component switches.

```text
for each horizontal bearing:
    for each elevation sample inside the beam:
        make the local ray direction from bearing and elevation
        rotate the ray into world coordinates
        find the closest surface intersection
        if there is a visible hit within range:
            compute reflectivity, incidence, spreading and absorption
            multiply by beam power and sampling weight
            add to the hit's range-bearing bin
            optionally add the approximate reflected components
```

"Add" matters: two elevations can arrive in the same bin. "Closest" matters:
a blocked farther patch does not contribute along that ray. Dividing sample
contributions by their count makes the default a normalized angular average.
This keeps the chosen intensity scale from growing just because more rays were
used. It is still not an integral of calibrated acoustic power over patch area.

### One ray and one triangle

A ray is origin + t × direction. With unit direction, t is distance. An
intersection routine finds a positive t where the ray reaches a surface.
For a plane, substitute the ray into the plane equation. For a sphere, substitute
into the equation saying distance from its centre equals its radius, yielding
a quadratic equation. Select the nearest positive solution.

A triangle hit must lie in the triangle's plane *and* inside its edges. The
Moller-Trumbore method solves for t and two barycentric coordinates. Barycentric
coordinates are weights telling where the point lies between the three corners;
nonnegative weights summing to one place it inside. A nearly parallel ray is
rejected when the determinant is near zero. The triangle routine uses two-sided
patches and flips the computed normal toward the incident ray. Consequently,
successful rendering alone is not proof that a mesh has correct winding.

OBJ vertices have no guaranteed unit scale. Here scale converts them to metres.
Normals come from the cross product of two triangle edges. A cross product gives
an arrow perpendicular to both edges, with direction depending on vertex order.
Connectivity tells which three vertex indices form each triangle. Holes,
nonmanifold edges and self-intersections are different defects; our asset audit
does not exhaustively detect arbitrary self-intersections.

### One voxel and one view

```text
project voxel centre into the sonar frame
if it is outside this view's observed support:
    keep it for this view
otherwise:
    keep it if a highlight or feasible-shadow mask permits the bin
    allow the explicitly configured neighboring-bin tolerance
remove a voxel if any observing view rejects it
```

The test is on a voxel's centre, not every point of its cube. A finite voxel size
and bin tolerances therefore affect boundary accuracy. Halving voxel edge length
in a fixed 3-D region gives roughly eight times as many voxels. Roughly doubling
poses doubles consistency-test work, although implementations can skip already
rejected voxels.

### Where each idea lives

| File | Be ready to explain |
|---|---|
| `core/geometry.cpp` | Plane, sphere and cylinder intersections; closest surface; texture coordinates |
| `core/mesh.cpp` | Triangle intersections, triangular OBJ loading, material proxy and transforms |
| `core/physics.cpp` | Array response, Hann shading and Thorp units |
| `core/simulator.cpp` | Bearing/elevation loops, midpoint weighting, visibility, direct/ghost/mirror deposition |
| `core/reconstruction.cpp` | Explicit range/bearing projection and the consistency rule |
| `bindings/sonar.pyx` | Python-to-C++ interface; array conversion and compiled extension |
| `python/reconstruction.py` | Threshold masks, orbit poses, grid, metrics and exposed voxel faces |
| `python/point_recovery.py` | Measurement prediction, finite-difference Jacobian and least-squares updates |
| `python/speckle.py` | Complex Gaussian field, spatial filtering and squared magnitude |
| `core/camera.cpp` | Pinhole projection, attenuation and veiling radiance |
| `core/waveform.cpp` | Chirp generation, echoes and matched filtering |
| `python/research_console.py` | Controls, display copies, cached pings and synthetic carving acquisition |
| `python/reproduce.py` | Builds, tests, demos, notebook and recorded run identity |

C++ handles repeated geometric work efficiently. Cython lets Python call that
compiled work and exchange numerical arrays. Python makes experiments and plots
easy to inspect. Releasing Python's global interpreter lock during C++ work
allows other Python threads to proceed; it does not itself create more workers.
The C++ code creates its own threads over bearings. Multipath can write into a
neighboring bearing, so private accumulators are combined afterward. Floating-
point summation order can change with thread count; use a declared tolerance.

The mesh renderer currently scans triangles linearly. With A bearings, E
elevations and T triangles, its intersection work grows approximately as
A × E × T. A bounding-volume hierarchy, or BVH, would group triangles into
bounding boxes to skip groups a ray cannot reach. It is a plausible optimization,
not something this code already provides. Profile before adding it.

## Optical comparison and sensor fusion

A pinhole camera maps local coordinates to a pixel using forward Y as depth:
horizontal pixel = centre_x + focal_pixels × X/Y; vertical pixel = centre_y -
focal_pixels × Z/Y. The minus sign appears because image rows increase downward.
A target at or behind the camera is not projected by this model.

For the approximate radiance equation, **L** means observed light, **J** source
strength, **ρ (rho)** reflectivity, **i** incidence angle, **a** water attenuation
per metre, and **B_inf** distant veiling light. Using a here avoids confusing
optical attenuation with the sound-speed symbol c:

L = J ρ cos(i) exp(-2ar)/r² + B_inf [1 - exp(-ar)].

The first term weakens target light along the outgoing and returning path. The
second term adds a veil that reduces contrast. This is a simplified active-light
model, not a complete underwater radiative-transfer solution. The source code
calls a `attenuation_per_m`. Turbidity in the console changes this optical model;
it does not secretly change acoustic scattering.

A camera ray can supply elevation information and a sonar measurement can
supply range, when extrinsic calibration and feature association are known.
**Extrinsic calibration** is the relative sensor positions and orientations;
**intrinsic calibration** concerns internal parameters such as focal length.
An offset camera must be transformed into the sonar/world frame. Never simply
combine two pixel coordinates as though both sensors have the same origin.
As optical contrast disappears, the fusion measurement may be unavailable.
Reporting a missed detection is better than inventing a precise elevation.

## The numbers you can defend

These values are from the completed local run
`results/2026-09-08_163327_188239/`, not measurements made in the ocean. Its
manifest records a modified worktree based on commit
`c488012e76db6ea2408dde335af768ae6f1aadc8`, source hashes, compiler, interpreter
and all stage outcomes. Quote that run when discussing its results.

| Check | Recorded result | What it supports |
|---|---|---|
| Integration | 30 stages passed; 43 analytic checks, 6 research tests, 17 demos, 38 notebook cells | That recorded software state runs together |
| Elevation ambiguity | Maximum absolute difference 3.266e-21; relative to image peak 2.428e-14; tolerance 1e-12 | Numerical symmetry under the chosen scene/beam assumptions |
| Plane brightness | Worst relative error 2.277e-16 over 1-10 m at 600 kHz | Correct implementation of the selected intensity equation |
| Single-look speckle | Mean factor 1.0003; contrast 0.9999; 9.46% below 0.1 | Agreement with the chosen exponential model by sampling |
| Chirp width | 11.06 mm measured versus 11.08 mm half-power prediction | Pulse-compression check for that waveform |
| Array versus Hann | Width 1.590° versus 2.625°; sidelobes -13.25 versus -31.47 dB | Taper tradeoff for 64 half-wavelength-spaced elements |
| Sub-ray convergence | Relative L1 error 0.00923 at 4096 rays versus 8192-ray reference | Numerical convergence for that scene and binning |

**L1 image error** means add all absolute pixel differences, then divide by the
reference image's total intensity. A finer-ray reference is a numerical reference,
not exact continuous ground truth. Relative difference "to image peak" is a
different metric: largest absolute pixel difference divided by the peak.

### Sphere carving: improving, still too large

The target radius was 0.45 m, voxel edge length 0.08 m, with known orbit poses,
a top-hat beam, threshold 0.03 of each image's peak, and one-bin tolerances.

| Poses | IoU | Excess volume relative to truth | False-positive volume | False-negative volume |
|---|---|---|---|---|
| 4 | 0.448 | 123.4% | 0.4664 m³ | 0 |
| 8 | 0.631 | 58.5% | 0.2212 m³ | 0 |
| 16 | 0.694 | 44.2% | 0.1669 m³ | 0 |
| 32 | 0.721 | 38.8% | 0.1464 m³ | 0 |

The evaluation truth is the sphere sampled at voxel centres. The zero false-
negative result is for this noiseless experiment, not a general guarantee of
carving. The estimated region includes substantial space outside the sphere.
Changing voxel size, masks, pose layout or tolerances changes these numbers.

The degradation experiment uses 20 trials per setting and bootstrap intervals
for the mean IoU. It sweeps looks, contrast, correlation length, threshold and
SNR. Here SNR specifies an exponential receiver-power floor relative to the
true image's peak; it is not a calibrated hardware SNR measurement. Some noise
settings can improve IoU by carving excess hull volume. This is evidence of the
interaction between the threshold rule and this target, not proof that noise
improves reconstruction generally.

### Point recovery: the geometry of the viewpoints matters

Noise settings: position 2 mm, attitude 0.05°, range 3 mm, bearing 0.05°.
Each configuration used 300 trials; failure means error above 10 cm.

| Views | Mean 3-D error | Standard deviation | 95% bootstrap interval for mean | Failure rate |
|---|---|---|---|---|
| Two sideways | 30.26 cm | 21.89 cm | 27.84-32.68 cm | 83.7% |
| Two vertical | 2.71 cm | 2.04 cm | 2.48-2.94 cm | 0.67% |
| Eight mixed | 1.98 cm | 1.48 cm | 1.81-2.14 cm | 0 of 300 trials |

Say "zero observed failures in 300 trials," not "the failure probability is
zero." The optimizer starts on a positive-elevation branch and assumes known
correspondence. Comparing the two two-view layouts isolates viewpoint geometry
more clearly than comparing two views with eight. Eight mixed views also have
more measurements, so their result cannot be attributed to pose direction alone.

### A defensible performance answer

The recorded benchmark used 97 bearings × 400 range bins, 512 elevation rays,
900 kHz, a 30° vertical beam, a 9 m maximum range and multipath, on the recorded
macOS arm64 machine. After two warmups, it timed 20 repetitions. For an analytic
sphere plus plane, rendering averaged 1.636 ms with one thread and 0.549 ms with
eight. For a 360-triangle coral mesh plus plane it averaged 101.685 ms and
22.257 ms, respectively. The Cython timing includes scene conversion and mesh
loading, and excludes optical rendering, speckle and display. These are timing
observations under that workload, not a guarantee of sustained application FPS.

## Papers: know the contribution and the boundary

**Aykin and Negahdaripour, 2016:** *Modeling 2-D Lens-Based Forward-Scan Sonar
Imagery for Targets With Diffuse Reflectance*, IEEE Journal of Oceanic
Engineering 41(3), 569-582; DOI 10.1109/JOE.2015.2503818. It develops a diffuse
image model including finite-pulse effects. This simulator uses a simpler
first-hit patch model. Its image bins do not include that finite-pulse model.

**Aykin and Negahdaripour, 2017:** *Three-Dimensional Target Reconstruction From
Multiple 2-D Forward-Scan Sonar Views by Space Carving*, IEEE Journal of Oceanic
Engineering 42(3), 574-589; DOI 10.1109/JOE.2016.2591738. This is the appropriate
space-carving reference. Our conservative masks and synthetic sphere evaluation
do not reproduce all of its experiments.

**Liu and Negahdaripour, 2024:** *Ghost Removal from Forward-Scan Sonar Views near
the Sea Surface for Image Enhancement and 3-D Object Modeling*, Remote Sensing
16(20), 3814; published 14 October 2024; DOI 10.3390/rs16203814. It combines
parameter and shape refinement. This simulator generates simplified forward
components but does not perform the paper's ghost removal or joint optimization.

**The advanced gap, in ordinary language:** a contour is an object's image
boundary. ICP, iterative closest point, repeatedly matches nearby boundary
points and estimates a transformation aligning them. IRLS, iteratively reweighted
least squares, changes how much influence residuals receive to reduce the effect
of poor matches. Neither name by itself guarantees correct correspondences.
Recovering 3-D patch-centre motions requires redundant views and associations;
object/mirror consistency adds a further geometric constraint. A single 2-D
contour-distance score skips that construction. Do not call it equation (6).
The current code implements none of that complete objective or depth/tilt search.

The requested IEEE document number 8516375 remains identity-unverified in this
project's audit because the publisher blocked access. Do not memorize an inferred
title or assert it is the 2020 stereo paper. Verified bibliographic details and
primary-source links are in [paper_mapping.md](paper_mapping.md).

## Worked practice: cover the answer before looking

### 1. Round-trip time

An echo arrives 8 milliseconds after transmission. What is the range at 1500 m/s?

**Answer:** 8 milliseconds = 0.008 seconds. Range = 1500 × 0.008 / 2 = 6 m.
Forgetting the division by two gives the total traveled distance, 12 m.

### 2. One physical point, two possible heights

Take r = 5 m, θ = 0°, φ = +30°. Find its coordinates, then reverse elevation.

**Answer:** X = 0; Y = 5 cos(30°) ≈ 4.330 m; Z = 5 sin(30°) = 2.5 m.
For -30°, X and Y stay the same and Z becomes -2.5 m. Both have image address
(5 m, 0°), provided that elevation is within the configured beam. This is a
geometry exercise; ±30° lies outside the usual narrow beam used in the demos.

### 3. Incidence cosine

The outgoing ray is (0, 1, 0). Which normal faces the sonar: (0, -1, 0) or
(1, 0, 0)? What factors enter the diffuse intensity model?

**Answer:** The first gives n·(-u) = 1, a head-on hit. The second gives zero,
a grazing direction. The dot product is the sum of matching coordinate products.

### 4. Range and decibels

Ignoring absorption, how much weaker is a return at 4 m than at 2 m under r⁻⁴?
What does a 10 dB power loss mean?

**Answer:** (2/4)⁴ = 1/16. A 10 dB power loss multiplies power by 10^(-10/10)
= 0.1. A 20 dB power loss gives 0.01. Power and amplitude decibels use different
conversion factors because power is proportional to amplitude squared.

### 5. Absorption with explicit units

Use α = 34.0687 dB/km and range 100 m. Find two-way absorption loss.

**Answer:** 100 m = 0.1 km. TL = 2 × 34.0687 × 0.1 = 6.81374 dB.
The retained power fraction is about 10^(-0.681374) ≈ 0.208. This is absorption
alone; include geometric spreading separately in the implemented equation.

### 6. Wavelength and the first array null

Find wavelength at 1.8 MHz, then the first null for 64 elements spaced λ/2.

**Answer:** λ = 1500 / 1,800,000 = 0.0008333 m = 0.8333 mm.
The first numerator zero occurs at N d sin(φ)/λ = 1, so sin(φ) = 2/64.
Therefore φ ≈ 1.791°. The denominator is nonzero there. At φ = 0 both vanish,
but the ratio's limit gives unit power instead of a null.

### 7. Bins and their boundaries

A 10 m range is divided into 100 bins; a 40° horizontal field has 80 bins.
Which zero-based indices contain range 4.23 m and bearing 0°?

**Answer:** Range spacing is 0.1 m, so floor(4.23/0.1) = 42. Bearing spacing
is 0.5°, so floor((0 + 20)/0.5) = 40. The axis starts at -20°. With an even
number of bearing bins, zero lies at a boundary, not a bin centre. Exactly 10 m
maps to index 100 and is outside this half-open bin domain.

### 8. Ghost delay

Let direct range be 4 m and reflected-leg range be 6 m. What are the three
apparent ranges in this simplified model?

**Answer:** Direct: 4 m. Ghost: (4 + 6)/2 = 5 m. Mirror: 6 m.
For the ghost, total travel is 10 m, so delay is 10/1500 seconds. Converting
that delay back to range by ct/2 returns 5 m.

### 9. Looks and contrast

What single-look fraction lies below one tenth of the mean? What contrast is
predicted after averaging 16 independent looks?

**Answer:** 1 - exp(-0.1) ≈ 0.09516, or 9.52%. Contrast is approximately
1/sqrt(16) = 0.25. The independent-look assumption matters. Correlated looks
do not provide the same reduction.

### 10. Carving metrics

Truth has 100 occupied voxels. The reconstruction keeps 120, including 80 true
ones. Find false positives, false negatives, IoU and normalized volume error.

**Answer:** TP = 80, FP = 40, FN = 20. Union = 80 + 40 + 20 = 140.
IoU = 80/140 ≈ 0.571. Volume error = (120 - 100)/100 = +20%.
Excess total volume does not mean there are no missed true voxels.

### 11. Chirp resolution

For c = 1500 m/s, bandwidth 60 kHz and duration 2 ms, calculate resolution,
uncompressed length scale and time-bandwidth product.

**Answer:** c/(2B) = 0.0125 m = 12.5 mm. cT/2 = 1.5 m. BT = 120.
The matched filter uses frequency variation to localize the echo despite the
long transmit duration. It does not physically shorten the transmitted pulse.

### 12. A numerical tolerance and a scientific conclusion

Your ambiguity images differ by 2.4e-14 relative to their peak. Does that prove
your simulator is accurate to 14 decimal places in real water?

**Answer:** No. It measures numerical agreement for a symmetry check. Beam,
material, propagation and calibration errors can be vastly larger in reality.

## A 15-minute mock interview

Answer aloud before reading the response target. These are practice questions,
not a claim to know this professor's interview script.

| Time | Question | A strong response should include |
|---|---|---|
| 0-1 min | What problem are you studying? | Measurement ambiguity, why reconstruction is difficult, one demonstrated result |
| 1-3 min | Draw a sonar pixel's possible 3-D origins. | Range/bearing arc, unknown elevation, beam support |
| 3-5 min | Walk through one rendered bearing. | Multiple elevations, nearest hit, beam weight, accumulation and units |
| 5-7 min | How do you know the code is right? | A hand-checkable case and a convergence test; numerical versus physical validity |
| 7-9 min | Explain your 32-view reconstruction. | Masks, known poses, IoU 0.721, 38.8% excess volume and concavity limitation |
| 9-11 min | Why could two views fail? | Correspondence, weak geometry, noise and above/below ambiguity |
| 11-13 min | What part of my paper did you implement? | Specific forward-model inspiration and explicit missing inverse steps |
| 13-15 min | What would you do next in a lab? | One controlled calibration experiment, predicted trend and measurable error |

For each answer score yourself 0, 1 or 2: zero means guessing, one means the
right idea with a missing mechanism, and two means a clear mechanism plus a
test or limitation. The score is a rehearsal tool, not an interview pass mark.
Spend your next study block on the two weakest answers.

## Questions about you and research readiness

**Why are you interested in this research?** Use your own reason. A possible
starting point: "I am interested in problems where the measurement itself leaves
information missing. This project made me want to understand how geometry,
physics and multiple views can recover some of it." Do not claim interests or
experience you do not have.

**What did you personally learn?** Name a specific thing you can demonstrate:
"I learned that adding elevation samples must not automatically increase raw
power," or "I learned to distinguish local rank from a global mirror ambiguity."
Explain the test you examined and the conclusion, rather than listing features.

**How much help did you use?** "I used AI as a coding assistant. I am responsible
for checking the model assumptions and understanding the tests I present. The
project is a learning tool, and I can show which parts I understand and which
still need work." Do not claim you personally performed a test until you have
actually opened its code or result and can explain it.

**What broke, and what did you learn?** A strong example is the raw sample-count
problem: more rays changed the arbitrary brightness scale. The default now uses
a normalized midpoint average, and the legacy mode preserves earlier results.
Another is display gain unintentionally causing a new speckle draw; separate
cache keys now preserve the ping when changing the display. Distinguish these
code defects from environment restrictions such as blocked notebook sockets.
The camera-centroid warning was avoided with elementwise reductions; do not
claim the low-level numerical-library cause was conclusively isolated.

**What are your availability and goals?** Decide truthful answers before the
meeting: hours per week, class constraints, duration you can commit, and whether
you seek course credit, volunteering or another arrangement. None of those
personal details can be inferred from the simulator. Bring a question about
expectations rather than assuming an offer or a particular role.

## A small research proposal you can defend

**Question:** How much of the observed range-dependent intensity change is
explained by the simulator's simple law for one known target?

**Experiment:** If appropriate equipment and supervision are available, image a
known simple target at several measured ranges with fixed orientation and sonar
settings. Record material, dimensions, positions, frequency and every gain or
processing setting. Retain raw data if available. Repeat pings to estimate
variability. A pool is useful for control but can introduce wall/surface echoes.

**Prediction:** With the declared point-scatterer approximation, spreading gives
r⁻⁴ and absorption adds the frequency-dependent decay. An extended target may
depart from that prediction, which is a result to investigate.

**Evaluation:** Define a consistent target region and intensity statistic before
comparing ranges. Separate background/noise, fit any scale parameter on one
subset and test on held-out ranges. Plot residuals and variability. Do not
adjust a different reflectivity at each range and call that validation.

**Follow-up:** Ask whether beam calibration, a different scattering law, finite
pulse modeling, segmentation or pose uncertainty is the most useful next step.
Change one assumption at a time so a better result has an interpretable cause.

## Questions worth asking the professor

- Which first calibration measurement would make this forward model more useful?
- Which simplified assumption is most misleading for the data your group uses?
- Would a small project on beam response, masks or pose uncertainty fit your work?
- What background should I strengthen first to contribute effectively?
- What would a useful first-month result look like for an undergraduate?

Ask two or three that fit the conversation. Avoid delivering them as a checklist.

## Demonstration logistics and a fallback

From `/Users/gregsobe/sonar_sim`, launch with
`.venv/bin/python python/research_console.py --reconstruct --animate`.
Space pauses, F toggles fullscreen, M changes target, B changes beam, K acquires
a synthetic target-only direct-path orbit for carving, and P saves a screenshot.
K is a controlled experiment: it does not reconstruct the currently displayed
seabed/multipath scene. Say this if demonstrating it. Gain and dynamic range are
display controls; roughness changes the coherent surface model; turbidity is
optical; bandwidth reports a separate waveform prediction.

Have the saved ambiguity and carving figures open before the meeting. If the
live console fails, say "I have saved results from the reproducible run" and
use those figures. Do not spend the meeting trying to repair a graphics issue.
Lead with **demo 1, elevation ambiguity**, then **demo 15, feasible-volume carving**.
The shadow and multipath figures are good follow-ups. Practice the controls so
you can explain a change before making it.

## Sources and evidence to bring

- Local experiment identity and stage outcomes: `results/2026-09-08_163327_188239/manifest.json`.
- Geometry, waveform and statistics: the corresponding `demo*.log` and `demo*_metrics.json` files in that same directory.
- Implementation versus approximation: [scientific_status.md](scientific_status.md).
- Development failures and corrections: [integration_audit.md](integration_audit.md).
- Verified primary bibliographic sources: [paper_mapping.md](paper_mapping.md), including University of Miami institutional records and the MDPI publisher page.

Before using any numerical claim, open its figure and explain the axes, units,
what varied, what stayed fixed, and what would make the conclusion fail. That
exercise is more valuable than memorizing every decimal in this packet.
