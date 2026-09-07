#include "geometry.h"

#include <cmath>

Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(const Vec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }

Vec3 normalize(const Vec3& a) {
    const double n = norm(a);
    return n > 0 ? a * (1.0 / n) : a;
}

// Integer hash of a lattice cell, returned on [0, 1). Deterministic in the cell
// index and the seed, so the texture is a property of the surface and not of
// the order the rays happen to arrive in.
static double lattice_value(int i, int j, int k, unsigned int seed) {
    unsigned int h = seed * 374761393u;
    h += static_cast<unsigned int>(i) * 3266489917u;
    h += static_cast<unsigned int>(j) * 2246822519u;
    h += static_cast<unsigned int>(k) * 668265263u;
    h ^= h >> 15;
    h *= 2654435761u;
    h ^= h >> 13;
    return h * (1.0 / 4294967296.0);
}

static double smoothstep(double t) { return t * t * (3.0 - 2.0 * t); }

// Trilinear value noise with a smoothstep ramp, mean 1/2.
static double value_noise(double x, double y, double z, unsigned int seed) {
    const double fx = std::floor(x), fy = std::floor(y), fz = std::floor(z);
    const int i = static_cast<int>(fx), j = static_cast<int>(fy), k = static_cast<int>(fz);
    const double u = smoothstep(x - fx), v = smoothstep(y - fy), w = smoothstep(z - fz);

    double accumulated = 0.0;
    for (int dk = 0; dk < 2; ++dk) {
        const double wk = dk ? w : 1.0 - w;
        for (int dj = 0; dj < 2; ++dj) {
            const double wj = dj ? v : 1.0 - v;
            for (int di = 0; di < 2; ++di) {
                const double wi = di ? u : 1.0 - u;
                accumulated += wi * wj * wk * lattice_value(i + di, j + dj, k + dk, seed);
            }
        }
    }
    return accumulated;
}

double texture_factor(const Vec3& point, const Texture& texture) {
    if (texture.amplitude <= 0.0 || texture.scale_m <= 0.0) return 1.0;

    // Three octaves at half the scale and half the weight each: enough spectrum
    // that the patches do not all come out the same size, which is what a single
    // octave looks like and what gives away a synthetic image.
    double sum = 0.0, weight = 0.0, frequency = 1.0 / texture.scale_m, gain = 1.0;
    for (int octave = 0; octave < 3; ++octave) {
        sum += gain * value_noise(point.x * frequency, point.y * frequency,
                                  point.z * frequency, texture.seed + octave);
        weight += gain;
        frequency *= 2.0;
        gain *= 0.5;
    }

    const double factor = 1.0 + texture.amplitude * 2.0 * (sum / weight - 0.5);
    return factor > 0.0 ? factor : 0.0;
}

Hit intersect_plane(const Plane& plane, const Vec3& origin, const Vec3& direction) {
    const double denominator = dot(plane.normal, direction);
    if (std::fabs(denominator) < 1e-12) return {};  // parallel to the plane

    const double t = dot(plane.normal, plane.point - origin) / denominator;
    if (t <= 0) return {};

    Hit hit;
    hit.valid = true;
    hit.t = t;
    // Face the normal back toward the ray, so the incidence angle is the angle
    // to the side actually being looked at.
    hit.normal = denominator < 0 ? plane.normal : plane.normal * -1.0;
    hit.reflectivity = plane.reflectivity * texture_factor(origin + direction * t, plane.texture);
    return hit;
}

Hit intersect_sphere(const Sphere& sphere, const Vec3& origin, const Vec3& direction) {
    // |origin + t*d - centre|^2 = radius^2, with d a unit vector so a = 1.
    const Vec3 offset = origin - sphere.centre;
    const double b = 2.0 * dot(offset, direction);
    const double c = dot(offset, offset) - sphere.radius * sphere.radius;
    const double discriminant = b * b - 4.0 * c;
    if (discriminant < 0) return {};

    const double root = std::sqrt(discriminant);
    double t = 0.5 * (-b - root);
    if (t <= 0) t = 0.5 * (-b + root);
    if (t <= 0) return {};

    const Vec3 point = origin + direction * t;
    Hit hit;
    hit.valid = true;
    hit.t = t;
    hit.normal = normalize(point - sphere.centre);
    hit.reflectivity = sphere.reflectivity * texture_factor(point, sphere.texture);
    return hit;
}

Hit intersect_cylinder(const Cylinder& cylinder, const Vec3& origin, const Vec3& direction) {
    const Vec3 axis = normalize(cylinder.axis);
    const Vec3 offset = origin - cylinder.centre;

    // Everything happens in the plane perpendicular to the axis, so strip the
    // axial component out of both the offset and the direction first.
    const Vec3 d_perp = direction - axis * dot(direction, axis);
    const Vec3 o_perp = offset - axis * dot(offset, axis);

    double best_t = 0.0;
    Vec3 best_normal;
    bool found = false;

    const double a = dot(d_perp, d_perp);
    if (a > 1e-15) {
        const double b = 2.0 * dot(o_perp, d_perp);
        const double c = dot(o_perp, o_perp) - cylinder.radius * cylinder.radius;
        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant >= 0.0) {
            const double root = std::sqrt(discriminant);
            for (int sign = 0; sign < 2; ++sign) {
                const double t = (sign ? -b + root : -b - root) / (2.0 * a);
                if (t <= 0.0) continue;
                const Vec3 point = origin + direction * t;
                const double axial = dot(point - cylinder.centre, axis);
                if (std::fabs(axial) > cylinder.half_length) continue;
                if (!found || t < best_t) {
                    best_t = t;
                    best_normal = normalize(point - cylinder.centre - axis * axial);
                    found = true;
                }
                break;  // the nearer root is the one that matters
            }
        }
    }

    // The two end caps, which is what turns a broadside pipe into a finite
    // object rather than an infinite tube through the scene.
    const double along = dot(direction, axis);
    if (std::fabs(along) > 1e-15) {
        for (int end = 0; end < 2; ++end) {
            const double signed_half = end ? cylinder.half_length : -cylinder.half_length;
            const double t = (signed_half - dot(offset, axis)) / along;
            if (t <= 0.0) continue;
            const Vec3 point = origin + direction * t;
            const Vec3 radial = point - cylinder.centre - axis * signed_half;
            if (dot(radial, radial) > cylinder.radius * cylinder.radius) continue;
            if (!found || t < best_t) {
                best_t = t;
                best_normal = axis * (end ? 1.0 : -1.0);
                found = true;
            }
        }
    }

    if (!found) return {};

    Hit hit;
    hit.valid = true;
    hit.t = best_t;
    hit.normal = dot(best_normal, direction) < 0 ? best_normal : best_normal * -1.0;
    hit.reflectivity = cylinder.reflectivity *
                       texture_factor(origin + direction * best_t, cylinder.texture);
    return hit;
}

Hit intersect_scene(const Scene& scene, const Vec3& origin, const Vec3& direction) {
    Hit nearest;
    for (const Plane& plane : scene.planes) {
        const Hit hit = intersect_plane(plane, origin, direction);
        if (hit.valid && (!nearest.valid || hit.t < nearest.t)) nearest = hit;
    }
    for (const Sphere& sphere : scene.spheres) {
        const Hit hit = intersect_sphere(sphere, origin, direction);
        if (hit.valid && (!nearest.valid || hit.t < nearest.t)) nearest = hit;
    }
    for (const Cylinder& cylinder : scene.cylinders) {
        const Hit hit = intersect_cylinder(cylinder, origin, direction);
        if (hit.valid && (!nearest.valid || hit.t < nearest.t)) nearest = hit;
    }
    return nearest;
}
