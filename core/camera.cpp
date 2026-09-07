#include "camera.h"

#include <cmath>

static Vec3 to_camera(const Camera& camera, const Vec3& world) {
    const Vec3 offset = world - camera.pose.position;
    return {dot(offset, camera.pose.x_axis), dot(offset, camera.pose.y_axis),
            dot(offset, camera.pose.z_axis)};
}

bool project_point(const Camera& camera, const Vec3& world, double* u, double* v) {
    const Vec3 local = to_camera(camera, world);
    if (local.y <= 1e-9) return false;  // behind the pinhole

    // Pixel rows run downward, so up in the world is up the image.
    *u = 0.5 * camera.width + camera.focal_px * local.x / local.y;
    *v = 0.5 * camera.height - camera.focal_px * local.z / local.y;
    return true;
}

Vec3 camera_ray(const Camera& camera, double u, double v) {
    const double x = (u - 0.5 * camera.width) / camera.focal_px;
    const double z = (0.5 * camera.height - v) / camera.focal_px;
    const Vec3 local{x, 1.0, z};
    return normalize(camera.pose.x_axis * local.x + camera.pose.y_axis * local.y +
                     camera.pose.z_axis * local.z);
}

void render_camera(const Camera& camera, const Scene& scene, double* image) {
    for (int row = 0; row < camera.height; ++row) {
        for (int column = 0; column < camera.width; ++column) {
            const Vec3 direction = camera_ray(camera, column + 0.5, row + 0.5);
            const Hit hit = intersect_scene(scene, camera.pose.position, direction);

            double radiance = 0.0;
            if (hit.valid) {
                const double cos_incidence = -dot(hit.normal, direction);
                if (cos_incidence > 0.0) {
                    radiance = camera.light_intensity * hit.reflectivity * cos_incidence *
                               std::exp(-2.0 * camera.attenuation_per_m * hit.t) /
                               (hit.t * hit.t);
                }
                radiance += camera.veiling_radiance *
                            (1.0 - std::exp(-camera.attenuation_per_m * hit.t));
            } else {
                // Open water: nothing but the veiling glow, saturating at B_inf.
                radiance = camera.veiling_radiance;
            }
            image[row * camera.width + column] = radiance;
        }
    }
}
