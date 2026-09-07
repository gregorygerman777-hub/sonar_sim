#include "simulator.h"

#include "physics.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <thread>
#include <vector>

namespace {

// One entry per elevation sub-ray. None of it depends on bearing, so it is built
// once instead of num_azimuth_bins times; at 3072 sub-rays that removes about
// six hundred thousand sine and beam-pattern evaluations per frame.
struct SubRay {
    double cos_phi;
    double sin_phi;
    double weight;
};

Vec3 yaw_world(const Vec3& v, double cos_psi, double sin_psi) {
    return {cos_psi * v.x - sin_psi * v.y, sin_psi * v.x + cos_psi * v.y, v.z};
}

// Where the platform is when bearing bin a is formed.
Pose pose_at(const Pose& base, const SonarConfig& config, double time_s) {
    if (config.sweep_duration_s <= 0.0) return base;

    const double psi = config.platform_yaw_rate_dps * M_PI / 180.0 * time_s;
    const double c = std::cos(psi), s = std::sin(psi);

    Pose moved;
    moved.position = base.position + config.platform_velocity_mps * time_s;
    moved.x_axis = yaw_world(base.x_axis, c, s);
    moved.y_axis = yaw_world(base.y_axis, c, s);
    moved.z_axis = yaw_world(base.z_axis, c, s);
    return moved;
}

struct Beam {
    double fov;
    double beam;
    double d_theta;
    double inv_d_range;
    double wavelength;
    double spacing;
    double absorption_per_m;  // exp(-k r) is 10^(-TL/10) with TL Thorp's two-way loss
    double alpha;
};

void deposit_bearing(const SonarConfig& config, const Scene& scene, const Pose& pose,
                     const Beam& beam_data, const std::vector<SubRay>& subrays, int a,
                     double theta, double sample_weight, double* image) {
    const int n_az = config.num_azimuth_bins, n_r = config.num_range_bins;
    const double sin_theta = std::sin(theta), cos_theta = std::cos(theta);

    for (const SubRay& ray : subrays) {
        const Vec3 local{ray.cos_phi * sin_theta, ray.cos_phi * cos_theta, ray.sin_phi};
        const Vec3 direction = normalize(pose.x_axis * local.x + pose.y_axis * local.y +
                                         pose.z_axis * local.z);

        const Hit hit = intersect_scene(scene, pose.position, direction);
        if (!hit.valid || hit.t > config.max_range_m) continue;

        // The normal was turned to face the ray, so this is the cosine of the
        // incidence angle at the surface.
        const double cos_incidence = -dot(hit.normal, direction);
        if (cos_incidence <= 0.0) continue;

        const double r2 = hit.t * hit.t;
        const double intensity = hit.reflectivity * cos_incidence / (r2 * r2) *
                                 std::exp(-beam_data.absorption_per_m * hit.t);

        const int bin = static_cast<int>(hit.t * beam_data.inv_d_range);
        if (bin >= 0 && bin < n_r)
            image[a * n_r + bin] += sample_weight * ray.weight * intensity;

        if (!config.multipath_enabled) continue;

        const Vec3 point = pose.position + direction * hit.t;
        const Vec3 mirrored_sonar{pose.position.x, pose.position.y,
                                  2.0 * config.surface_z - pose.position.z};
        const Vec3 to_mirror = mirrored_sonar - point;
        const double bounced = norm(to_mirror);
        const double cos_bounced = dot(hit.normal, normalize(to_mirror));
        if (cos_bounced <= 0.0 || bounced <= 0.0) continue;

        // Grazing angle at the surface: the bounce point lies on the straight
        // line to the mirrored sonar, so its sine is the vertical separation
        // over the bounced range.
        const double rise = std::fabs(2.0 * config.surface_z - point.z - pose.position.z);
        const double rayleigh = 2.0 * (2.0 * M_PI / beam_data.wavelength) *
                                config.surface_rms_height_m * (rise / bounced);
        const double gamma = config.surface_reflectivity * std::exp(-0.5 * rayleigh * rayleigh);

        // The bounced path arrives from the direction of the mirrored point, so
        // that is the elevation the beam pattern must be evaluated at.
        const Vec3 mirrored_point{point.x, point.y, 2.0 * config.surface_z - point.z};
        const Vec3 offset = mirrored_point - pose.position;
        const Vec3 local_offset{dot(offset, pose.x_axis), dot(offset, pose.y_axis),
                                dot(offset, pose.z_axis)};
        const double mirror_elevation = std::asin(local_offset.z / norm(local_offset));

        // The bounced arrival may fall outside the vertical beam while the direct
        // one does not. That is not a reason to drop the ghost: the ghost has two
        // reciprocal paths, and the one that goes out direct and returns bounced
        // still arrives from the direct bearing. Gating both on the mirror's
        // elevation is why almost every view carries a ghost while only some
        // carry a mirror.
        // Under roll the mirrored arrival is displaced in bearing as well as
        // elevation, so it must be deposited at its own azimuth bin rather than
        // the cast ray's. This is what lets a rolled sonar see a mirror that an
        // unrolled one cannot.
        const double mirror_azimuth = std::atan2(local_offset.x, local_offset.y);
        const int mirror_az_bin =
            static_cast<int>((mirror_azimuth + 0.5 * beam_data.fov) / beam_data.d_theta);
        const bool mirror_in_beam = std::fabs(mirror_elevation) <= 0.5 * beam_data.beam &&
                                    mirror_az_bin >= 0 && mirror_az_bin < n_az;
        const double mirror_weight =
            mirror_in_beam ? beam_pattern(mirror_elevation, config.array_element_count,
                                          beam_data.spacing, beam_data.wavelength)
                           : 0.0;

        // Lambert is a backscatter law, so the bistatic term is taken as the
        // geometric mean of the two monostatic cosines: a simplification.
        const double bistatic = std::sqrt(cos_incidence * cos_bounced);
        const double ghost_range = 0.5 * (hit.t + bounced);

        const double ghost = hit.reflectivity * bistatic * gamma * gamma /
                             (r2 * bounced * bounced) *
                             std::exp(-beam_data.absorption_per_m * ghost_range);
        // Two reciprocal paths of equal length: one arrives along the direct
        // bearing, the other along the mirrored one, each carrying half.
        const int ghost_bin = static_cast<int>(ghost_range * beam_data.inv_d_range);
        if (ghost_bin >= 0 && ghost_bin < n_r) {
            image[a * n_r + ghost_bin] += 0.5 * sample_weight * ray.weight * ghost;
            if (mirror_in_beam)
                image[mirror_az_bin * n_r + ghost_bin] +=
                    0.5 * sample_weight * mirror_weight * ghost;
        }

        const double b2 = bounced * bounced, g2 = gamma * gamma;
        const double mirror = hit.reflectivity * cos_bounced * g2 * g2 / (b2 * b2) *
                              std::exp(-beam_data.absorption_per_m * bounced);
        const int mirror_bin = static_cast<int>(bounced * beam_data.inv_d_range);
        if (mirror_in_beam && mirror_bin >= 0 && mirror_bin < n_r)
            image[mirror_az_bin * n_r + mirror_bin] += sample_weight * mirror_weight * mirror;
    }
}

void render_bearing(const SonarConfig& config, const Scene& scene, const Pose& base,
                    const Beam& beam_data, const std::vector<SubRay>& subrays, int a,
                    double* image) {
    const int n_az = config.num_azimuth_bins;
    const double theta = -0.5 * beam_data.fov + (a + 0.5) * beam_data.d_theta;
    const double sweep_time = config.sweep_duration_s * (a + 0.5) / n_az;

    const int samples = config.motion_samples_per_bin > 1 ? config.motion_samples_per_bin : 1;
    const double dwell = config.sweep_duration_s / n_az;
    const double sample_weight = 1.0 / samples;

    for (int k = 0; k < samples; ++k) {
        // Sample centres spread symmetrically about the bin's nominal instant, so
        // one sample reduces exactly to the instantaneous case.
        const double offset = samples > 1 ? ((k + 0.5) / samples - 0.5) * dwell : 0.0;
        deposit_bearing(config, scene, pose_at(base, config, sweep_time + offset), beam_data,
                        subrays, a, theta, sample_weight, image);
    }
}

}  // namespace

void render(const SonarConfig& config, const Scene& scene, const Pose& pose, double* image) {
    const int n_az = config.num_azimuth_bins, n_r = config.num_range_bins;
    const int pixels = n_az * n_r;
    std::fill(image, image + pixels, 0.0);

    Beam beam_data;
    beam_data.wavelength = config.speed_of_sound_mps / config.frequency_hz;
    beam_data.spacing = config.array_element_spacing_m > 0.0 ? config.array_element_spacing_m
                                                             : 0.5 * beam_data.wavelength;
    beam_data.alpha = thorp_absorption_db_per_km(config.frequency_hz);
    // 10^(-TL/10) with TL = 2 alpha r / 1000 is exp(-r alpha ln10 / 5000), which
    // is one exponential instead of a pow per hit.
    beam_data.absorption_per_m = beam_data.alpha * std::log(10.0) / 5000.0;
    beam_data.fov = config.horizontal_fov_deg * M_PI / 180.0;
    beam_data.beam = config.vertical_beamwidth_deg * M_PI / 180.0;
    beam_data.d_theta = beam_data.fov / n_az;
    beam_data.inv_d_range = n_r / config.max_range_m;

    const double d_phi = config.num_elevation_subrays > 1
                             ? beam_data.beam / (config.num_elevation_subrays - 1)
                             : 0.0;
    std::vector<SubRay> subrays(std::max(config.num_elevation_subrays, 0));
    for (int s = 0; s < config.num_elevation_subrays; ++s) {
        // A single sub-ray means boresight, not the bottom edge of the beam,
        // which is what makes the one-ray case comparable to the analytic
        // intensity equation.
        const double phi =
            config.num_elevation_subrays > 1 ? -0.5 * beam_data.beam + s * d_phi : 0.0;
        subrays[s].cos_phi = std::cos(phi);
        subrays[s].sin_phi = std::sin(phi);
        subrays[s].weight = beam_pattern(phi, config.array_element_count, beam_data.spacing,
                                         beam_data.wavelength);
    }

    int threads = config.num_threads > 0
                      ? config.num_threads
                      : static_cast<int>(std::thread::hardware_concurrency());
    threads = std::max(1, std::min(threads, n_az));

    if (threads == 1) {
        for (int a = 0; a < n_az; ++a)
            render_bearing(config, scene, pose, beam_data, subrays, a, image);
        return;
    }

    // Without multipath a bearing writes only into its own column, so the
    // threads can share one buffer. With multipath a bearing can deposit a
    // mirror into a neighbour, so each thread accumulates privately and the
    // results are summed once at the end.
    const bool shared = !config.multipath_enabled;
    std::vector<double> scratch(shared ? 0 : static_cast<size_t>(pixels) * (threads - 1), 0.0);

    std::vector<std::thread> workers;
    workers.reserve(threads);
    for (int w = 0; w < threads; ++w) {
        double* target = shared || w == 0
                             ? image
                             : scratch.data() + static_cast<size_t>(pixels) * (w - 1);
        workers.emplace_back([&, w, target]() {
            for (int a = w; a < n_az; a += threads)
                render_bearing(config, scene, pose, beam_data, subrays, a, target);
        });
    }
    for (std::thread& worker : workers) worker.join();

    if (!shared)
        for (int w = 1; w < threads; ++w) {
            const double* part = scratch.data() + static_cast<size_t>(pixels) * (w - 1);
            for (int i = 0; i < pixels; ++i) image[i] += part[i];
        }
}

void apply_speckle(double* image, int count, unsigned int seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    for (int i = 0; i < count; ++i) {
        // 1 - U so the draw is on (0, 1] and the logarithm stays finite.
        image[i] *= -std::log(1.0 - uniform(rng));
    }
}
