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
double shaded_beam_pattern(double phi, int elements, double spacing, double wavelength, int mode);

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

// Coherent-reflection reduction from Ogilvy's (1991) Rayleigh-roughness
// scattering: an interface with RMS height sigma reflects a fraction
// exp(-Ra^2 / 2) of the specular amplitude, Ra = 2 k sigma sin(grazing) being
// the Rayleigh roughness parameter. A flat interface (sigma = 0) leaves this
// at 1; a grazing angle near zero also leaves it near 1, since a shallow ray
// barely samples the relief. This is the same reduction the surface ghost and
// mirror paths already used inline; it is shared here so the bottom boundary
// uses an identical, separately validated roughness model.
double roughness_coherence_factor(double wavenumber, double rms_height_m, double sin_grazing);

// Plane-wave pressure reflection coefficient at a fluid-fluid interface
// (the Rayleigh two-fluid seabed model that BELLHOP/KRAKEN's "half-space"
// bottom boundary condition uses), for a grazing angle measured up from the
// interface. Subscript 1 is the upper medium (water), 2 the lower (sediment).
// With Snell's law in grazing form, cos(theta1)/c1 = cos(theta2)/c2, and
// impedance Z_i = rho_i * c_i:
//
//   R = (Z2 sin(theta1) - Z1 sin(theta2)) / (Z2 sin(theta1) + Z1 sin(theta2))
//
// When the sediment is faster than water (the ordinary case for sand or
// rock), there is a critical grazing angle theta_c = acos(c1 / c2) below
// which theta2 has no real solution: the coefficient is complex and has unit
// magnitude. This real-valued API returns that magnitude (1) below critical
// and the signed real coefficient above critical. A coherent field model
// would also need the omitted phase; this simulator sums incoherent intensity,
// so its renderer uses |R|^2.
double bottom_reflection_coefficient(double grazing_rad, double water_speed_mps,
                                     double bottom_speed_mps, double water_density_kgm3,
                                     double bottom_density_kgm3);
