#pragma once

// Array factor of a uniform line array, squared for intensity:
//
//   B(phi) = [ sin(N_e * pi * d * sin(phi) / lambda)
//              / (N_e * sin(pi * d * sin(phi) / lambda)) ]^2
//
// Note on where this belongs: conventionally this is the pattern along the
// beamformed axis, which for a forward-scan sonar is bearing, not elevation.
// Applied across elevation it acts as a vertical aperture taper. It is even in
// phi either way, so the +phi / -phi ambiguity survives it untouched.
double beam_pattern(double phi_rad, int element_count, double element_spacing_m,
                    double wavelength_m);

// Thorp's absorption coefficient in dB per km, frequency in Hz.
//
//   alpha = 0.11 f^2/(1 + f^2) + 44 f^2/(4100 + f^2) + 2.75e-4 f^2 + 0.003
//
// with f in kHz.
double thorp_absorption_db_per_km(double frequency_hz);

// Two-way transmission loss in dB at a range in metres.
double transmission_loss_db(double alpha_db_per_km, double range_m);
