# Forward-scan sonar simulator: what it does and why I trust it

## The demo I would start with

A target at range `r`, azimuth `theta`, elevation `+phi`, and the same target at
`(r, theta, -phi)`. Two genuinely different places in the water. The simulator
renders them and the two images agree to **1.1e-15 of peak**, which is
double-precision round-off.

That is not a coincidence I engineered. The image is addressed by range and
bearing only, and the vertical beam weighting is even in `phi`, so nothing in the
pipeline can distinguish them. The residual is not zero only because the sub-ray
that strikes the `+phi` target is `s` while for `-phi` it is `n-1-s`, so the
contributions are summed in reverse order and floating-point addition is not
associative.

I take this as the sharpest available check that the ambiguity is a property of
the model rather than something bolted on: if I had a bug that leaked elevation
into a pixel address, this test would fail.

## Why the intensity model is grounded rather than decorative

Each sub-ray's first hit contributes

```
I = R * cos(incidence) / r^4 * 10^(-TL/10),   TL = 2 * alpha * (r/1000)
```

with `alpha` from Thorp's formula. I validated it directly: a flat plane swept
from 1 to 10 m, one boresight sub-ray so the geometry matches the equation
exactly, and the measured peak agrees with the closed form to **relative error
0.000e+00** at every range.

Getting there caught a real subtlety. With an even azimuth bin count no beam
lies on boresight; the nearest sits 0.23 degrees off, the ray strikes at
`r/cos(theta)`, and the validation sits 4e-5 off. An odd bin count fixes it. I
mention it because it is the kind of thing that quietly biases a simulator.

The frequency dependence is worth a sentence on its own: Thorp gives 34 dB/km at
100 kHz and 935 dB/km at 1.8 MHz. That is the resolution-versus-range trade in
one number, and it is why high-frequency units are short-range instruments.

## Why the shadows are real

Nothing implements shadowing. Each sub-ray stops at its first surface and
contributes to no bin beyond it, so occlusion is the residue. Because different
elevation sub-rays are blocked at different ranges, the shadow edge is soft by
construction. A sphere on the seabed removes 5.0% of the energy beyond it.

## What one view cannot do, and what two can

The natural sequel to the ambiguity demo. Two targets at the same `(r, theta)`
and opposite elevation, with the sonar translated along a line:

```
cross-range motion:  separation 0.0000, 0.0000, 0.0000, 0.0000 m
vertical motion:     separation 0.0000, 0.1248, 0.2460, 0.3606 m
```

Cross-range motion never separates them, because only `z^2` enters the range.
Vertical motion does, immediately. The practical reading is that a survey line
holding constant depth cannot resolve elevation however many pings it collects.

## Putting the elevation back with a camera

The reason I built the ambiguity demo first is that it sets up the interesting
question: what does it take to undo it. A sonar bin is a range and a bearing, so
it is consistent with a one-parameter family of 3-D points, the arc swept by
elevation across the vertical beam. A camera pixel is the complement: two angles
and no range, and in turbid water not even that.

I added a pinhole camera 30 cm above the sonar head with the underwater direct
plus veiling-backscatter model, projected the sonar's ambiguity arc into it, and
took the elevation where the projected arc meets the pixel the target was
detected in. Both detections are made off the rendered images, not read out of
the scene:

```
one sonar bin alone leaves        0.959 m of arc open
fused with the camera detection   elevation 3.486 deg against a true 3.500
                                  3-D position error 2.4 cm
```

The turbidity sweep is the other half of the argument. The direct term decays as
`exp(-2 c r)` while the veiling glow grows as `1 - exp(-c r)`, so the camera's
contrast on the target falls 14.0, 2.9, 0.47, 0.023 as `c` goes 0.02, 0.08, 0.20,
0.40 per metre. Past `c = 0.2` the detection is gone and the pair falls back to
the sonar's 96 cm arc, while the sonar image itself is unchanged. That is the
regime that makes acoustics necessary in the first place, and it is the honest
version of "fusion helps".

## Sea-surface multipath

Reflection off the air-water interface adds a ghost at the mean of the direct and
bounced ranges and a mirror at the full bounced range. Sweeping the roll:

```
roll     ghost/object   mirror/object
  0.0       0.1185         0.0000
 22.5       0.1203         0.0001
 45.0       0.1052         0.0015
 67.5       0.1249         0.0051
 90.0       0.5330         0.0550
```

The ghost is on essentially every view; the mirror is absent at zero roll and
grows monotonically as the sonar rolls. That is not something I put in by hand: it
falls out because the ghost keeps the object's bearing while the mirror arrives
from the mirrored direction, which for a shallow target sits outside the vertical
beam until roll converts that elevation offset into a bearing offset. It is also
why rolling the sonar is the standard way to separate the two.

Two bugs on the way there are worth mentioning because they were both real. I
first gated the ghost and the mirror behind the same in-beam test, which wrongly
killed the ghost whenever the mirror left the beam. And I deposited the mirror at
the cast ray's azimuth, which made roll do nothing.

## The signature a detector actually keys on

A cylinder lying on the seabed, the standard stand-in for a pipe or a mine, next
to a rock of similar size. What comes out is the pairing an operator reads first:
a bright return off the flank turned toward the sonar and a black wedge behind
it. The wedge is a measurement, not just an appearance. Similar triangles give

```
h_o = h_s L / (d_o + L)
```

and inverting the measured shadow length against the truth over a sweep of object
heights:

```
true height   0.120  0.180  0.240  0.340  0.460 m
recovered     0.128  0.196  0.260  0.371  0.434 m
```

0.8 to 3.1 cm out. The rock, at the same range, gives a rounder highlight and a
shorter shadow, which is most of why the pair is diagnostic rather than merely
visible.

## Texture and speckle are not the same noise

Uniform reflectivity gives a seabed that is a smooth ramp with nothing on it, so
I gave every surface a band-limited noise field multiplying its reflectivity.
The point is not that it looks better. It is that texture is attached to the
ground and speckle is not, and only one of them is usable.

I imaged the same seabed from two positions 5 cm apart along the boresight,
shifted the second image by the 5 range bins that corresponds to, and correlated
what is left after dividing out the deterministic `1/r^4` field:

```
texture only         contrast 0.133   view-to-view r = 0.996
speckle only         contrast 0.990   view-to-view r = 0.006
texture + speckle    contrast 1.008   view-to-view r = 0.023
```

Single-look speckle buries the texture completely. Averaging looks recovers it,
and the combined contrast follows `sqrt(sigma_T^2 (1 + 1/L) + 1/L)` to three
decimals at every `L` I tried, which makes me believe the two mechanisms are
composing the way they should rather than by accident. At 32 looks the
correlation is back to 0.359 and still well short of the 0.996 ceiling: this is
the cost, in pings, of anything that has to register two views of the same
seabed.

## Motion inside one sweep

A mechanically scanned head visits bearings one at a time, so bearing `a` is
formed at `t_a = T (a + 1/2) / N_theta` and rendered from the pose at that
instant. Surging slides targets in range across the fan; yawing stretches the fan
in bearing, because the sweep and the vehicle are turning at once. I predicted
every displacement from the geometry first and then measured it off the rendered
image, and they agree to 0.07 degrees and 10 mm across all four cases.

Two honest caveats. This is a warp, not a blur: each bearing is a single instant,
so a compact target stays sharp and simply lands in the wrong place, and real
blur would need finite dwell inside one bin. And it is the scanned head, not the
multibeam fan, which forms every bearing from one ping and is frozen within it,
so for that sensor the same distortion shows up between pings when images are
mosaicked.

## What I know is missing

No volume reverberation, and the energy a rough surface scatters out of the
specular direction simply disappears rather than becoming diffuse reverberation. The transmit pulse has zero length, so the range point spread is a
delta rather than a compressed pulse. `B(phi)` is applied across elevation
although the array-factor formula properly describes the beamformed axis; being
even in `phi`, it does not affect the ambiguity either way, but it is a taper I
chose rather than derived.

The sub-ray count deserves flagging too. 48 is fine for a compact target and
leaves a grazing seabed 85% empty; the seabed demos use 3072 on the strength of a
measured coverage curve rather than a guess.

The texture field is band-limited lattice noise, not a K or lognormal draw with a
measured seabed spectrum, and the camera has no lens blur, no vignetting and no
sensor noise. In the fused result, the 2.4 cm is dominated by the sonar measuring
the target's near surface while the camera centroids its whole disc, one radius
apart; I report the number both with and without that correction rather than
quietly applying it.

On speed, since it matters for anything iterative: the core threads over bearing
and comes back bit for bit identical to the serial result, which the test suite
checks in the awkward case too, where multipath makes one bearing deposit into a
neighbour's column. A 193x700 frame at 3072 sub-rays with multipath went from
61.4 ms to 6.0 ms.
