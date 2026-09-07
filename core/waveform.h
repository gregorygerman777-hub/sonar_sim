#pragma once

// Time-domain acoustics, kept deliberately apart from the geometric imaging
// pipeline in simulator.{h,cpp}. Nothing here writes into a beam-bin image and
// nothing there calls into this; the only shared code is physics.h, whose
// absorption and spreading terms both layers use so the two agree on how loud a
// return at range r ought to be.
//
// Pulse compression, following Urick, *Principles of Underwater Sound*, 3rd ed.,
// ch. 2 and 9. A plain pulse of duration T resolves two targets only if their
// echoes do not overlap, which needs c T / 2 of separation: getting fine range
// resolution that way means a short pulse, and a short pulse carries little
// energy. A linear FM chirp breaks that tie. It sweeps B hertz across the same
// T seconds,
//
//     s(t) = cos(2 pi (f0 t + (B / 2T) t^2)),   t in [0, T]
//
// so its instantaneous frequency f0 + (B/T) t rises linearly, and correlating
// the echo against the transmitted copy compresses it to a pulse of width about
// 1/B. The range resolution becomes
//
//     delta_r = c / (2 B)
//
// independent of T, so energy and resolution are set by two different knobs. The
// improvement factor is the time-bandwidth product B T.

struct ChirpConfig {
    double frequency_hz = 300e3;        // f0, the start of the sweep
    double chirp_bandwidth_hz = 60e3;   // B
    double chirp_duration_s = 2e-3;     // T
    double sample_rate_hz = 1.2e6;
    double speed_of_sound_mps = 1500.0;
};

// A point reflector as the waveform layer sees it: how far away it is and how
// strongly it scatters. cos_incidence is the same Lambert term the imaging
// pipeline applies, carried across so the two layers scale returns identically.
struct Reflector {
    double range_m = 0.0;
    double reflectivity = 1.0;
    double cos_incidence = 1.0;
};

int waveform_sample_count(const ChirpConfig& config);

// The transmitted sweep. Its first sample is cos(0) = 1 by construction.
void generate_chirp(const ChirpConfig& config, double* out, int count);

// The same duration and start frequency with no sweep at all, which is the
// thing the chirp has to beat.
void generate_tone(const ChirpConfig& config, double* out, int count);

// Sum of the transmitted waveform delayed by tau = 2 r / c and scaled by the
// pressure amplitude of the existing intensity model, sqrt(R cos(theta) / r^4)
// with Thorp absorption folded in. Fractional sample delays are interpolated, so
// a reflector's delay is not quantised to the sample grid.
void receive_echoes(const ChirpConfig& config, const Reflector* reflectors, int reflector_count,
                    const double* transmit, int transmit_count, double* out, int out_count);

// Ambient and receiver noise, zero mean and Gaussian.
void add_gaussian_noise(double* signal, int count, double sigma, unsigned int seed);

// Cross-correlation of the record against the transmitted copy:
//
//     y[k] = sum_n received[n + k] * reference[n]
//
// The peak sits at the delay in samples. Threaded over lag.
void matched_filter(const double* received, int received_count, const double* reference,
                    int reference_count, double* out, int num_threads);
