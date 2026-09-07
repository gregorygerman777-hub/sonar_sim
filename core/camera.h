#pragma once

#include "geometry.h"
#include "simulator.h"

// A pinhole optical camera sharing the sonar's frame convention: Y along the
// optical axis, Z up, X to starboard. It exists here for one reason, which is
// that it sees the elevation the sonar throws away.
//
// Underwater the direct term is not the whole story. Light leaves the vehicle
// light, travels to the surface and comes back, so it is attenuated over both
// legs and spreads as 1/r^2, while the water between camera and scene scatters
// a veiling glow back that grows with range. The Jaffe-McGlamery reduction of
// that is
//
//   L = J rho cos(incidence) exp(-2 c r) / r^2  +  B_inf (1 - exp(-c r))
//
// with c the beam attenuation coefficient and J the vehicle light's intensity.
// Raise c and the second term buries the first: that is turbidity, and it is
// why nobody navigates on cameras alone.
struct Camera {
    Pose pose;
    int width = 320;
    int height = 240;
    double focal_px = 260.0;
    double attenuation_per_m = 0.0;  // c, in 1/m
    double veiling_radiance = 0.35;  // B_inf
    double light_intensity = 20.0;   // J, the lamp the vehicle carries
};

// Pixel that a world point lands on. Returns false when the point is behind the
// camera, where the projection is meaningless rather than merely off-frame.
bool project_point(const Camera& camera, const Vec3& world, double* u, double* v);

// Unit ray leaving the pinhole through pixel centre (u, v).
Vec3 camera_ray(const Camera& camera, double u, double v);

// Row-major width * height radiance image.
void render_camera(const Camera& camera, const Scene& scene, double* image);
