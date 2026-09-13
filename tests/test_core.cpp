#include "geometry.h"
#include "physics.h"

#include <cmath>
#include <iostream>

namespace {

int failures = 0;

void check_close(const char* name, double actual, double expected, double tolerance) {
    const bool pass = std::fabs(actual - expected) <= tolerance;
    std::cout << (pass ? "PASS  " : "FAIL  ") << name << "  got " << actual
              << "  expected " << expected << '\n';
    if (!pass) ++failures;
}

void check_true(const char* name, bool actual) {
    std::cout << (actual ? "PASS  " : "FAIL  ") << name << '\n';
    if (!actual) ++failures;
}

}  // namespace

int main() {
    const double water_speed = 1500.0;
    const double water_density = 1000.0;
    const double bottom_speed = 1650.0;
    const double bottom_density = 1900.0;

    // At normal incidence the two-fluid expression reduces independently to
    // the familiar impedance step (Z2-Z1)/(Z2+Z1).
    const double z1 = water_density * water_speed;
    const double z2 = bottom_density * bottom_speed;
    check_close("normal-incidence bottom coefficient",
                bottom_reflection_coefficient(M_PI / 2.0, water_speed, bottom_speed,
                                              water_density, bottom_density),
                (z2 - z1) / (z2 + z1), 1e-12);

    const double critical = std::acos(water_speed / bottom_speed);
    check_close("below-critical reflection has unit magnitude",
                bottom_reflection_coefficient(critical - 0.05, water_speed, bottom_speed,
                                              water_density, bottom_density),
                1.0, 1e-12);

    Scene wall_scene;
    wall_scene.planes.push_back({{0.0, 3.0, 0.0}, {0.0, -1.0, 0.0}, 0.05});
    check_true("wall blocks a finite reflected leg",
               segment_occluded(wall_scene, {0.0, 1.0, 0.0}, {0.0, 5.0, 0.0}));
    check_true("short segment before wall remains clear",
               !segment_occluded(wall_scene, {0.0, 1.0, 0.0}, {0.0, 2.0, 0.0}));

    if (failures != 0) {
        std::cerr << failures << " core check(s) failed\n";
        return 1;
    }
    std::cout << "all standalone C++ core checks passed\n";
    return 0;
}
