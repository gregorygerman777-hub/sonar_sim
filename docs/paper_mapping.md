# Paper verification and implementation boundary

Verified 8 September 2026 using the publisher and University of Miami's
institutional records. Titles and dates below distinguish journal issue dates
from years embedded in DOI strings. No claim of equation-level reproduction is
made where full derivations could not be inspected.

| Work and source | Paper contribution | Simulator | Simplifications and missing parts |
|---|---|---|---|
| Aykin & Negahdaripour, *Modeling 2-D Lens-Based Forward-Scan Sonar Imagery for Targets With Diffuse Reflectance*, IEEE JOE 41(3), 569–582, July 2016; [institutional record](https://scholarship.miami.edu/esploro/outputs/journalArticle/Modeling-2-D-Lens-Based-Forward-Scan-Sonar-Imagery/991031578677202976), DOI 10.1109/JOE.2015.2503818 | Diffuse patch image model, finite pulse width, simultaneous arrivals, cylinder measurements and bottom multipath | Diffuse first-hit elevation accumulation | No finite pulse image convolution, lens calibration or real cylinder validation; not an exact implementation of its equations |
| Aykin & Negahdaripour, *Three-Dimensional Target Reconstruction From Multiple 2-D Forward-Scan Sonar Views by Space Carving*, IEEE JOE 42(3), 574–589, July 2017; [institutional record](https://scholarship.miami.edu/esploro/outputs/journalArticle/Three-Dimensional-Target-Reconstruction-From-Multiple-2-D/991031576679702976), DOI 10.1109/JOE.2016.2591738 | Known-pose sequential empty-space removal, roll diversity; synthetic and physical targets | C++ range-bearing voxel consistency with highlight/feasible-shadow masks | Conservative farther-range mask, fixed bin tolerances, synthetic sphere metrics; no real-data reconstruction |
| Liu & Negahdaripour, *Ghost Removal from Forward-Scan Sonar Views near the Sea Surface for Image Enhancement and 3-D Object Modeling*, Remote Sensing 16(20), 3814, **14 October 2024**; [publisher](https://www.mdpi.com/2072-4292/16/20/3814), DOI 10.3390/rs16203814 | Joint sonar-depth/tilt and shape refinement; contour registration, patch-centre motions and mirror consistency | Separate approximate forward direct/ghost/mirror paths and roughness sweep | No ICP/IRLS, patch-motion objective, parameter grid search, ghost removal or iterative shape refinement |

The 2024 publisher's indexed full text was accessible; direct page/PDF requests
also returned 429 errors. Its section 2.3 describes contour registration using
IRLS ICP and recovery of 3-D patch-centre motions from multiple 2-D motions.
That construction is absent here. A contour distance alone would not reproduce
equation (6). We do not calculate or relabel such a proxy.

## IEEE document 8516375

The [requested IEEE URL](https://ieeexplore.ieee.org/document/8516375) returned a
JavaScript robot-verification page. Its document-number identity and equations
could not be verified. It remains **unresolved**, rather than being assigned a
title by inference.

Separately verified: Shahriar Negahdaripour, *Application of Forward-Scan Sonar
Stereo for 3-D Scene Reconstruction*, IEEE JOE 45(2), 547–562, April 2020,
DOI 10.1109/JOE.2018.2875574, [institutional record](https://scholarship.miami.edu/esploro/outputs/journalArticle/Application-of-Forward-Scan-Sonar-Stereo-for/991031578702602976).
The record describes stereo geometry, degeneracies and reconstruction methods.
Our finite-difference least-squares point experiment illustrates sensitivity;
it does not implement that paper's complete stereo algorithms. We have not
verified that this DOI corresponds to the requested IEEE document number.
