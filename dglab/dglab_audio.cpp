// dglab_audio.cpp
//
// Platform-independent audio analysis (FFT dominant-tone detection, frequency
// mapping, smoothing) and the capture-thread lifecycle. The platform-specific
// capture thread itself lives in dglab_audio_backend_win.cpp /
// dglab_audio_backend_macos.mm / dglab_audio_backend_none.cpp.

#include "dglab_audio.h"

#include <complex>
#include <cstdio>
#include <mutex>
#include <thread>

#include "dglab_audio_backend.h"

namespace dglab {

namespace {

// In-place iterative radix-2 Cooley-Tukey FFT. a.size() must be a power of 2.
void fft_inplace(std::vector<std::complex<double>>& a) {
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; i++) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            std::swap(a[i], a[j]);
        }
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        const double ang = -2.0 * kPi / static_cast<double>(len);
        const std::complex<double> wlen(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> w(1.0, 0.0);
            for (size_t k = 0; k < len / 2; k++) {
                const std::complex<double> u = a[i + k];
                const std::complex<double> v = a[i + k + len / 2] * w;
                a[i + k] = u + v;
                a[i + k + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
}

// Capture lifecycle globals: only ever one capture at a time.
std::mutex g_capture_mutex;
std::thread* g_capture_thread = nullptr;
AudioState* g_capture_state = nullptr;

}  // namespace

ToneInfo dominant_tone(const std::vector<float>& samples, double sample_rate,
                       double freq_min_hz, double freq_max_hz,
                       std::vector<float>* spectrum_bands) {
    ToneInfo result;
    if (samples.size() < 256) {
        return result;
    }

    const size_t n = kFftSize;
    std::vector<std::complex<double>> a(n, std::complex<double>(0.0, 0.0));
    const size_t count = samples.size();
    for (size_t i = 0; i < count; i++) {
        const double w = 0.5 * (1.0 - std::cos(2.0 * kPi * static_cast<double>(i) /
                                               static_cast<double>(count - 1)));  // Hann
        a[i] = std::complex<double>(static_cast<double>(samples[i]) * w, 0.0);
    }
    fft_inplace(a);

    size_t lo = static_cast<size_t>(freq_min_hz * n / sample_rate);
    size_t hi = static_cast<size_t>(freq_max_hz * n / sample_rate);
    if (hi >= n / 2) {
        hi = n / 2 - 1;
    }
    if (lo >= hi) {
        return result;
    }

    size_t peak_bin = lo;
    double peak_mag = 0.0;
    for (size_t i = lo; i <= hi; i++) {
        const double mag = std::abs(a[i]);
        if (mag > peak_mag) {
            peak_mag = mag;
            peak_bin = i;
        }
    }

    // Log-spaced magnitude spectrum for display: band i covers
    // [fmin * r^i, fmin * r^(i+1)] and takes the max bin magnitude in it,
    // normalized like `level` (a full-scale sine gives ~1.0 at its bin) and
    // then mapped onto a -60..0 dBFS scale so typical levels read bright.
    if (spectrum_bands != nullptr) {
        const double fmin = std::max(1.0, freq_min_hz);
        const double fmax = std::max(fmin + 1.0, freq_max_hz);
        const double r = std::pow(fmax / fmin, 1.0 / static_cast<double>(kSpectrumBands));
        spectrum_bands->resize(kSpectrumBands);
        const double norm = 0.5 * static_cast<double>(samples.size());
        double f_lo = fmin;
        for (size_t i = 0; i < kSpectrumBands; i++) {
            const double f_hi = f_lo * r;
            size_t b_lo = static_cast<size_t>(f_lo * static_cast<double>(n) / sample_rate);
            size_t b_hi = static_cast<size_t>(f_hi * static_cast<double>(n) / sample_rate);
            if (b_lo >= n / 2) {
                b_lo = n / 2 - 1;
            }
            if (b_hi > n / 2 - 1) {
                b_hi = n / 2 - 1;
            }
            double mag = 0.0;
            for (size_t j = b_lo; j <= b_hi; j++) {
                const double m = std::abs(a[j]);
                if (m > mag) {
                    mag = m;
                }
            }
            const double level = std::clamp(mag / norm, 0.0, 1.0);
            // dB scale with a -60 dBFS floor: 0 dBFS -> 1.0, -20 dBFS -> ~0.67,
            // -40 dBFS -> ~0.33, below -60 dBFS -> 0 (true black).
            // TODO(idea): if quiet input still looks dim, try per-row peak
            // normalization instead: divide each frame by its own max band
            // (floored, e.g. max(peak, 0.01), so silence can't amplify noise).
            // That makes the loudest band full-bright regardless of level,
            // at the cost of losing absolute loudness.
            const double db = 20.0 * std::log10(std::max(level, 1e-6));
            (*spectrum_bands)[i] =
                static_cast<float>(std::clamp((db + 60.0) / 60.0, 0.0, 1.0));
            f_lo = f_hi;
        }
    }

    if (peak_mag < kSilenceThreshold) {
        return result;
    }
    result.hz = static_cast<double>(peak_bin) * sample_rate / static_cast<double>(n);
    // A sine wave of amplitude A produces a peak bin magnitude of ~A * count * 0.5
    // (Hann window coherent gain), so divide by that to recover the amplitude.
    result.level = std::clamp(peak_mag / (0.25 * static_cast<double>(count)), 0.0, 1.0);
    return result;
}

double audio_freq_ratio(double hz, double freq_min_hz, double freq_max_hz) {
    if (hz <= 0.0 || freq_max_hz <= freq_min_hz) {
        return 0.0;
    }
    const double f = std::clamp(hz, freq_min_hz, freq_max_hz);
    const double ratio = (std::log10(f) - std::log10(freq_min_hz)) /
                         (std::log10(freq_max_hz) - std::log10(freq_min_hz));
    return std::clamp(ratio, 0.0, 1.0);
}

double smooth_level(double* smooth_s, double sample, double smooth_ms, int tick_ms) {
    if (smooth_ms <= 0.0) {
        *smooth_s = sample;
        return sample;
    }
    const double alpha = 1.0 - std::exp(-static_cast<double>(tick_ms) / 1000.0 / smooth_ms);
    *smooth_s += (sample - *smooth_s) * alpha;
    return *smooth_s;
}

bool start_audio_capture(AudioState* st, double freq_min_hz, double freq_max_hz) {
    std::lock_guard<std::mutex> lock(g_capture_mutex);
    if (g_capture_thread != nullptr) {
        return g_capture_state != nullptr && g_capture_state->capture_ok.load();
    }

    st->running = true;
    st->freq_min_hz = static_cast<float>(freq_min_hz);
    st->freq_max_hz = static_cast<float>(freq_max_hz);
    g_capture_state = st;

    g_capture_thread = new std::thread(&dglab_audio::audio_capture_thread,
                                       st, freq_max_hz);
    // Give the capture stream a moment to come up.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    return st->capture_ok.load();
}

void stop_audio_capture(AudioState* st) {
    std::lock_guard<std::mutex> lock(g_capture_mutex);
    if (g_capture_thread == nullptr) {
        return;
    }
    st->running = false;
    g_capture_thread->join();
    delete g_capture_thread;
    g_capture_thread = nullptr;
    g_capture_state = nullptr;
}

}  // namespace dglab
