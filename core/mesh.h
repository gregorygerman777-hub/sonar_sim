#pragma once
#include "geometry.h"
#include <string>
#include <array>

// OBJ units are unspecified. scale explicitly converts source units to metres.
// MTL Kd mean is only a fallback acoustic reflectivity proxy, not calibration.
TriangleMesh load_obj(const std::string& path, double scale, const Vec3& translation,
                      double reflectivity);
Hit intersect_triangle(const Vec3& a, const Vec3& b, const Vec3& c,
                       double reflectivity, const Vec3& origin, const Vec3& direction);
Hit intersect_mesh(const TriangleMesh&, const Vec3&, const Vec3&);

void transform_mesh(TriangleMesh&, const Vec3&, const Vec3&, const Vec3&, const Vec3&);
