#include "waveform.h"

#include "physics.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <thread>
#include <vector>

int waveform_sample_count(const ChirpConfig& config) {
    return static_cast<int>(config.chirp_duration_s * config.sample_rate_hz);
}

void generate_chirp(const ChirpConfig& config, double* out, int count) {
    const double rate = config.chirp_bandwidth_hz / (2.0 * config.chirp_duration_s);
    for (int n = 0; n < count; ++n) {
        const double t = n / config.sample_rate_hz;
        out[n] = std::cos(2.0 * M_PI * (config.frequency_hz * t + rate * t * t));
    }
}

void generate_tone(const ChirpConfig& config, double* out, int count) {
    for (int n = 0; n < count; ++n) {
        const double t = n / config.sample_rate_hz;
        out[n] = std::cos(2.0 * M_PI * config.frequency_hz * t);
    }
}

void receive_echoes(const ChirpConfig& config, const Reflector* reflectors, int reflector_count,
                    const double* transmit, int transmit_count, double* out, int out_count) {
    std::fill(out, out + out_count, 0.0);
    const double alpha = thorp_absorption_db_per_km(config.frequency_hz);

    for (int index = 0; index < reflector_count; ++index) {
        const Reflector& reflector = reflectors[index];
        if (reflector.range_m <= 0.0) continue;

        const double delay_s = 2.0 * reflector.range_m / config.speed_of_sound_mps;
        const double r2 = reflector.range_m * reflector.range_m;
        const double intensity = reflector.reflectivity * reflector.cos_incidence / (r2 * r2) *
                                 std::pow(10.0, -transmission_loss_db(alpha, reflector.range_m) / 10.0);
        // The record is a pressure, the imaging pipeline's model is an intensity,
        // so the two are tied together by one square root and the matched-filter
        // peak comes out proportional to sqrt(I).
        const double amplitude = std::sqrt(intensity);

        const double delay_samples = delay_s * config.sample_rate_hz;
        const int whole = static_cast<int>(std::floor(delay_samples));
        const double fraction = delay_samples - whole;

        for (int n = 0; n < transmit_count; ++n) {
            // Linear interpolation between neighbouring samples, so a delay that
            // falls between two samples is not rounded onto the grid. Without it
            // every recovered range would be quantised to c / (2 fs).
            const int near = whole + n, far = near + 1;
            if (near >= 0 && near < out_count) out[near] += amplitude * (1.0 - fraction) * transmit[n];
            if (far >= 0 && far < out_count) out[far] += amplitude * fraction * transmit[n];
        }
    }
}

void add_gaussian_noise(double* signal, int count, double sigma, unsigned int seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> gaussian(0.0, sigma);
    for (int n = 0; n < count; ++n) signal[n] += gaussian(rng);
}

void matched_filter(const double* received, int received_count, const double* reference,
                    int reference_count, double* out, int num_threads) {
    int threads = num_threads > 0 ? num_threads
                                  : static_cast<int>(std::thread::hardware_concurrency());
    threads = std::max(1, std::min(threads, received_count));

    // Lags are independent, so this splits cleanly and needs no reduction.
    auto correlate = [&](int begin, int step) {
        for (int lag = begin; lag < received_count; lag += step) {
            const int overlap = std::min(reference_count, received_count - lag);
            double sum = 0.0;
            for (int n = 0; n < overlap; ++n) sum += received[lag + n] * reference[n];
            out[lag] = sum;
        }
    };

    if (threads == 1) {
        correlate(0, 1);
        return;
    }
    std::vector<std::thread> workers;
    workers.reserve(threads);
    for (int w = 0; w < threads; ++w) workers.emplace_back(correlate, w, threads);
    for (std::thread& worker : workers) worker.join();
}
