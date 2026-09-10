# Scientific status

This is an educational simulator with analytic and synthetic validation. It is
not a calibrated DIDSON, a real-data reconstruction result, or an exact paper
reproduction. The integration report provides measured results and failed runs.

| Component | Implemented | Approximation / remaining work |
|---|---|---|
| Geometry | C++ plane, sphere, capped cylinder, triangle OBJ; mesh translation/rotation/scale | Linear triangle search; no BVH; only triangular OBJ faces accepted |
| Asset checks | Units, dimensions, triangle count, signed volume, boundary/nonmanifold/winding edges | Exhaustive arbitrary self-intersection detection and repair absent; table comprises touching closed boxes |
| Materials | Per-object coefficient and MTL Kd mean per triangle | Kd is an explicit visual-to-acoustic proxy, not physical material calibration |
| Image formation | Multi-elevation first-hit rays, normalized midpoint angular mean, range/bearing bins | Relative intensity, not watts; no finite pulse footprint, coherent phase summation, lens aberration or real array calibration |
| Beam | Uniform array, top-hat, Hann; measured sidelobes and width | Ideal elevation array; not a measured transmit/receive directivity product |
| Scattering | Incidence cosine, r^-4, Thorp two-way absorption | Diffuse point-scatterer model, incomplete sonar equation, MHz use uncalibrated |
| Shadows | First-hit occlusion and canonical sphere/seabed | Reconstruction's farther-range feasible region is conservative; dark water is not independently classified as empty |
| Multipath | Independent component switches, projected mirror bearing, roll, coherent roughness sweep | Direct-visible patches only; reflected-leg blockage, diffuse rough-surface energy, pressure phase, seabed multipath and concavity reverberation incomplete |
| Speckle | Seeded exponential single look; filtered complex Gaussian spatial correlation; looks/contrast sweeps | Periodic spatial boundary; optional contrast mixture is not exponential; no persistent correlated temporal process |
| Texture | Deterministic sphere/cylinder translation-attached field; seabed field | Analytic texture has no explicit body orientation; mesh texture map absent |
| Reconstruction | C++ voxel consistency; 4/8/16/32 views; sphere IoU, volume errors, FP/FN; raw voxel export | Synthetic known poses and threshold masks; no uniqueness; exposed voxel-face surface with shared vertices; no smooth surface fitting |
| Degradation | Five parameter sweeps, 20 trials each, 95% bootstrap mean intervals | One synthetic sphere, fixed grid/tolerances; not real-data statistics |
| Point recovery | Two/eight views, Jacobian/SVD, 300 trials, position/attitude/range/bearing noise | Known correspondence and initial branch; no global uniqueness guarantee |
| Waveform | C++ chirp, tone, delayed echoes, matched filter; WAV and width validation | Separate from the image renderer; audio is slowed playback/sonification |
| Optical | Pinhole, direct attenuation and veiling light, turbidity/fusion demos | Approximate illumination, no complete underwater optical calibration |
| Console | Resizable/fullscreen Pygame, parameter pages, scene/sonar/beam/trace/optical/waterfalls/carving, capture and GIF | 3-D scene is orthographic wireframe; no polished Blender scene, general mesh shadow-volume extraction or pose timeline editor |
| Four-way comparison | Images, differences, timings, shared reference | Static experiment; not a four-way live console tab |
| Paper diagnostics | Verified contribution mapping and limitations | Equation (6), ICP/IRLS patch motions, ghost removal and depth/tilt optimization unimplemented |

## Rules for interpreting results

Raw intensity is immutable under display gain or log normalization. A bin address
retains range and bearing; elevation is only a beam-support check. An unobserved
voxel survives rather than being treated as known empty water. Highlight masks
are formed before carving, with named bin tolerances. Synthetic truth is used
only after reconstruction to score it.

`legacy_elevation_sum=True` restores endpoint rays and unnormalized accumulation.
The default is midpoint rays divided by their count: an angular **average**, not
an angular integral in calibrated units. This correction changes raw magnitudes
and sampling locations; compare against the preserved baseline rather than
silently treating old raw values as interchangeable.

Single-ray mode uses boresight and preserves the analytic point-hit equation.
Increasing range-bin count changes sampling, not physical pulse resolution.
`SonarSimulator.range_resolution_m` is historically named and means range-bin
spacing; `ChirpSonar.range_resolution_m` means c/(2B). The console distinguishes
bandwidth theory from image binning.

Noise can occasionally improve this conservative hull's IoU by removing excess
volume. A threshold relative to a noisy maximum couples segmentation and looks.
Therefore we report the curves as measured, even when they are nonmonotonic.

The pose-recovery sensitivity matrix scales ranges by 3 mm and bearings by
0.05 degrees. Comparing singular values without these units would be misleading.
Positive-elevation initialization does not establish global identifiability.
