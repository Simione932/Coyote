// dglab_audio.h
//
// Audio capture + analysis for the DG-LAB Coyote apps: dominant-tone
// detection (Hann window + 8192-point FFT), audio-to-wire frequency mapping,
// and intensity smoothing.
//
// Capture: WASAPI loopback on the default render device (Windows), the
// 'BlackHole 2ch' input device via an AUHAL monitor (macOS), a PortAudio
// tap on the default output's PulseAudio/PipeWire monitor stream (Linux),
// and a no-op stub on other platforms. start_audio_capture() launches the
// capture thread and stop_audio_capture() tears it down.

#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <vector>

#include "dglab_protocol.h"

namespace dglab {

// Analysis parameters.
constexpr double kPi = 3.14159265358979323846;
constexpr double kAnalysisWindowS = 0.1;      // 100 ms of audio per spectrum
constexpr size_t kFftSize = 8192;             // zero-padded FFT length
constexpr double kAudioFreqMinHzDefault = 20.0;  // default lower limit (20 Hz)
constexpr double kAudioFreqMinHz = kAudioFreqMinHzDefault;  // lowest audio frequency considered default
constexpr double kAudioFreqMaxHzDefault = 10000.0;  // default upper limit (10 kHz)
constexpr double kSilenceThreshold = 5.0;     // peak magnitude below this = silence
constexpr size_t kSpectrumBands = 96;          // log-spaced bands in the published spectrum

// Result of analysing one channel: the dominant frequency (Hz) and the
// normalized loudness (0..1) of that tone.
struct ToneInfo {
    double hz = 0.0;
    double level = 0.0;
};

// Returns the dominant tone of the sample buffer. hz/level are 0.0 if the
// signal is (near) silence. Only bins in [freq_min_hz, freq_max_hz] are
// considered. level is the peak amplitude of the tone normalized to the
// full-scale range (a full-scale sine wave gives ~1.0).
//
// If spectrum_bands is non-null it is also filled with the log-spaced
// magnitude spectrum (kSpectrumBands values, freq_min_hz .. freq_max_hz), each
// on a dB scale with a -60 dBFS floor (0 dBFS -> 1.0, below -60 dBFS -> 0)
// for direct display. The spectrum is filled even for (near) silence.
ToneInfo dominant_tone(const std::vector<float>& samples, double sample_rate,
                       double freq_min_hz = kAudioFreqMinHzDefault,
                       double freq_max_hz = kAudioFreqMaxHzDefault,
                       std::vector<float>* spectrum_bands = nullptr);

// Maps a detected audio frequency (Hz) logarithmically into the 0..1 range
// of [freq_min_hz, freq_max_hz]; 0 for silence.
double audio_freq_ratio(double hz, double freq_min_hz = kAudioFreqMinHzDefault,
                         double freq_max_hz = kAudioFreqMaxHzDefault);

// Blends a generator's base waveform frequency (its `frequency` field, treated
// on the same 0..100 scale as the output period) with the audio peak-frequency
// ratio. ratio 0 keeps the generator's own tempo; ratio 1 pushes the output to
// the fastest rate (100 Hz). Returns the compressed on-wire frequency value.
inline uint8_t blend_audio_freq(uint8_t base_freq, double ratio) {
    const double base = (base_freq >= kFreqMin) ? static_cast<double>(base_freq)
                                                : static_cast<double>(kFreqMin);
    const double out_freq = base + ratio * (100.0 - base);
    const int period = std::clamp(static_cast<int>(std::lround(1000.0 / out_freq)), 10, 100);
    return compress_frequency(period);
}

// Applies a symmetric exponential moving average to a loudness sample with
// time constant smooth_ms (ms); smooth_s must persist between calls.
// smooth_ms <= 0 bypasses smoothing (the raw sample is returned).
double smooth_level(double* smooth_s, double sample, double smooth_ms, int tick_ms);

// ---------------------------------------------------------------------------
// Capture state + lifecycle.
// ---------------------------------------------------------------------------

// Shared state between the audio capture thread and the BLE main loop.
struct AudioState {
    std::atomic<bool> running{false};
    std::atomic<bool> capture_ok{false};
    std::atomic<float> left_hz{0.0};
    std::atomic<float> right_hz{0.0};
    std::atomic<float> left_level{0.0};
    std::atomic<float> right_level{0.0};
    std::atomic<int> analyses{0};
    // Log-spaced magnitude spectrum (kSpectrumBands values in 0..1) per
    // channel, refreshed together with left/right_hz and left/right_level.
    // Readers must hold spectrum_mutex.
    std::mutex spectrum_mutex;
    std::vector<float> left_spectrum;
    std::vector<float> right_spectrum;
    // Lower audio frequency considered (Hz). Updated live by GUI Freq min slider.
    std::atomic<float> freq_min_hz{static_cast<float>(kAudioFreqMinHzDefault)};
    // Upper audio frequency considered (Hz). The capture thread reads this
    // each analysis window, so it can be changed live (e.g. by the GUI's
    // Freq max slider) without restarting capture.
    std::atomic<float> freq_max_hz{static_cast<float>(kAudioFreqMaxHzDefault)};
};

// Starts the capture thread publishing into `st`. `freq_min_hz` and `freq_max_hz` seed
// st->freq_min_hz and st->freq_max_hz (see above); they can still be updated live afterwards.
// Returns false (and leaves capture_ok false) if the platform has no capture
// support or the device could not be opened.
bool start_audio_capture(AudioState* st, double freq_min_hz, double freq_max_hz);
inline bool start_audio_capture(AudioState* st, double freq_max_hz) {
    return start_audio_capture(st, kAudioFreqMinHzDefault, freq_max_hz);
}

// Stops the capture thread and joins it.
void stop_audio_capture(AudioState* st);

}  // namespace dglab
