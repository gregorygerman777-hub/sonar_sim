#include "physics.h"

#include <cmath>

double beam_pattern(double phi_rad, int element_count, double element_spacing_m,
                    double wavelength_m) {
    const double x = M_PI * element_spacing_m * std::sin(phi_rad) / wavelength_m;
    const double denominator = element_count * std::sin(x);

    // On boresight, and wherever the grating denominator vanishes, the ratio
    // tends to one.
    if (std::fabs(denominator) < 1e-12) return 1.0;

    const double amplitude = std::sin(element_count * x) / denominator;
    return amplitude * amplitude;
}

double thorp_absorption_db_per_km(double frequency_hz) {
    const double f = frequency_hz / 1000.0;  // kHz
    const double f2 = f * f;
    return 0.11 * f2 / (1.0 + f2) + 44.0 * f2 / (4100.0 + f2) + 2.75e-4 * f2 + 0.003;
}

double transmission_loss_db(double alpha_db_per_km, double range_m) {
    return 2.0 * alpha_db_per_km * (range_m / 1000.0);
}
