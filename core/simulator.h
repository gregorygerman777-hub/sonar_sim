#pragma once

#include "geometry.h"

struct SonarConfig {
    double frequency_hz = 1.8e6;
    int num_azimuth_bins = 256;
    int num_range_bins = 512;
    double horizontal_fov_deg = 30.0;
    double vertical_beamwidth_deg = 14.0;
    double max_range_m = 10.0;
    double speed_of_sound_mps = 1500.0;
    int num_elevation_subrays = 48;
    int array_element_count = 64;
    double array_element_spacing_m = 0.0;  // zero means half a wavelength

    // Sea-surface multipath. The bounced leg has the same length and launch
    // direction as the straight line to the sonar mirrored through the surface,
    // so one reflection gives both extra components:
    //
    //   object  out direct, back direct    -> r_d
    //   ghost   one leg each way           -> (r_d + r_m) / 2
    //   mirror  both legs bounced          -> r_m
    //
    // A rough surface scatters part of the field out of the specular direction,
    // so the coherent reflection falls off with the Rayleigh parameter
    // R_a = 2 k sigma_h sin(grazing), amplitude factor exp(-R_a^2 / 2). The
    // ghost bounces once and the mirror twice, so the mirror dies as the square.
    bool multipath_enabled = false;
    double surface_z = 0.0;
    double surface_reflectivity = 1.0;   // amplitude; a pressure-release surface is 1
    double surface_rms_height_m = 0.0;   // zero is a flat mirror

    // Platform motion during one image. Bearing bin a is formed at
    // t = sweep_duration_s * (a + 1/2) / num_azimuth_bins, and the pose used for
    // that bin is the pose at that instant. Setting sweep_duration_s to zero
    // freezes the platform and reproduces the static sensor exactly.
    //
    // The assumption behind this: it is the mechanically scanned head, where
    // bearings really are visited one after another. A multibeam fan forms every
    // bearing from one ping and is frozen within a ping, so for that sensor the
    // same smear appears between pings, when consecutive images are mosaicked,
    // rather than inside one.
    Vec3 platform_velocity_mps;          // world frame
    double platform_yaw_rate_dps = 0.0;  // about world Z
    double sweep_duration_s = 0.0;

    // Poses per bearing bin. One is an instantaneous sample, which displaces a
    // target without widening it: a warp, not a blur. A bearing bin is really
    // open for a dwell of sweep_duration_s / num_azimuth_bins, and sampling that
    // dwell more than once is what actually smears a moving return across range
    // and bearing the way an AUV-mounted head does.
    int motion_samples_per_bin = 1;

    // Threads over bearing. Zero asks the hardware how many it has.
    int num_threads = 0;
    int beam_mode = 0; // 0 uniform array, 1 top-hat, 2 Hann-shaded array
    bool legacy_elevation_sum = false;
    bool direct_enabled = true;
    bool ghost_enabled = true;
    bool mirror_enabled = true;
};

// Columns of the sonar rotation, following the forward-scan convention: Y_s runs
// forward along the acoustic axis, Z_s is up, and X_s completes a right-handed
// frame as X = Y x Z, which puts it to starboard for a vessel heading along Y.
struct Pose {
    Vec3 position;
    Vec3 x_axis{1, 0, 0};
    Vec3 y_axis{0, 1, 0};
    Vec3 z_axis{0, 0, 1};
};

// Renders into an array of num_azimuth_bins * num_range_bins, azimuth-major.
//
// For each azimuth bin the vertical beamwidth is swept by num_elevation_subrays
// sub-rays. Each sub-ray stops at its first surface and deposits
//
//     B(phi) * R * cos(incidence) / r^4 * 10^(-TL/10)
//
// into the range bin of that hit, and contributes nothing to any bin beyond it.
// Shadowing is that last clause; there is no separate occlusion pass.
//
// Bearings are independent unless multipath is on, in which case a bearing can
// deposit a mirror into a neighbour's column, so that case gets a private
// accumulator per thread and one reduction at the end.
void render(const SonarConfig& config, const Scene& scene, const Pose& pose, double* image);

// Single-look speckle: intensity is exponentially distributed about its mean,
// so multiplying by -ln(U) with U uniform on (0,1] is exact, not an approximation.
void apply_speckle(double* image, int count, unsigned int seed);
