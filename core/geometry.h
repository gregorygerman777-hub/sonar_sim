#pragma once

#include <vector>

struct Vec3 {
    double x = 0, y = 0, z = 0;
};

Vec3 operator+(const Vec3& a, const Vec3& b);
Vec3 operator-(const Vec3& a, const Vec3& b);
Vec3 operator*(const Vec3& a, double s);
Vec3 cross(const Vec3& a, const Vec3& b);
double dot(const Vec3& a, const Vec3& b);
double norm(const Vec3& a);
Vec3 normalize(const Vec3& a);

// Spatial modulation of the backscatter coefficient. Real seabed and real hull
// plating are not uniform Lambertian: patch to patch the strength varies by
// several dB, which is what gives sonar imagery its mottled look. This is band
// limited lattice noise standing in for that; a rigorous model would draw the
// local strength from a K or lognormal distribution instead.
//
// The distinction that matters is against speckle. Texture is attached to the
// surface, so it is identical from every viewpoint and survives re-imaging;
// speckle is redrawn per realisation. Anything that registers two views has to
// live on the first and tolerate the second.
struct Texture {
    double amplitude = 0.0;  // fractional swing about the mean, 0 disables it
    double scale_m = 0.25;   // size of the coarsest blob
    unsigned int seed = 1;
};

double texture_factor(const Vec3& point, const Texture& texture);

// A ray hit: t is the distance along a unit direction, so it is also the range.
struct Hit {
    bool valid = false;
    double t = 0.0;
    Vec3 normal;
    double reflectivity = 0.0;
};

struct Plane {
    Vec3 point;
    Vec3 normal;
    double reflectivity = 0.05;
    Texture texture;
};

struct Sphere {
    Vec3 centre;
    double radius = 1.0;
    double reflectivity = 0.5;
    Texture texture;
};

// A capped finite cylinder, which is the standard stand-in for a pipe, a mine
// or a length of debris. Its signature is the point of the whole exercise: a
// bright return off the flank turned toward the sonar, then nothing behind it.
struct Cylinder {
    Vec3 centre;
    Vec3 axis{0, 1, 0};
    double radius = 0.1;
    double half_length = 0.5;
    double reflectivity = 0.8;
    Texture texture;
};

struct Scene {
    std::vector<Plane> planes;
    std::vector<Sphere> spheres;
    std::vector<Cylinder> cylinders;
};

Hit intersect_plane(const Plane& plane, const Vec3& origin, const Vec3& direction);
Hit intersect_sphere(const Sphere& sphere, const Vec3& origin, const Vec3& direction);
Hit intersect_cylinder(const Cylinder& cylinder, const Vec3& origin, const Vec3& direction);

// Nearest hit over the whole scene, which is what makes a sub-ray stop at the
// first surface and contribute nothing beyond it.
Hit intersect_scene(const Scene& scene, const Vec3& origin, const Vec3& direction);
