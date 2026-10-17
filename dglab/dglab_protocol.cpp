// dglab_protocol.cpp
//
// Implementation of the shared DG-LAB protocol pieces: mode names/arg
// parsing and the per-tick waveform computation (channel_tick).

#include "dglab_protocol.h"

#include <cstdlib>

#include "dglab_audio.h"

namespace dglab {

const char* const kModeNames[8] = { "default", "breath", "waves", "strobe",
                                    "pulse",   "rainbow", "bass", "spectrum" };

int parse_mode_arg(const char* arg) {
    if (std::strcmp(arg, "breath") == 0 || std::strcmp(arg, "1") == 0) {
        return MODE_BREATH;
    }
    if (std::strcmp(arg, "waves") == 0 || std::strcmp(arg, "2") == 0) {
        return MODE_WAVES;
    }
    if (std::strcmp(arg, "strobe") == 0 || std::strcmp(arg, "3") == 0) {
        return MODE_STROBE;
    }
    if (std::strcmp(arg, "pulse") == 0 || std::strcmp(arg, "4") == 0) {
        return MODE_PULSE;
    }
    if (std::strcmp(arg, "rainbow") == 0 || std::strcmp(arg, "5") == 0) {
        return MODE_RAINBOW;
    }
    if (std::strcmp(arg, "bass") == 0 || std::strcmp(arg, "6") == 0) {
        return MODE_BASS;
    }
    if (std::strcmp(arg, "spectrum") == 0 || std::strcmp(arg, "7") == 0) {
        return MODE_SPECTRUM;
    }
    return std::atoi(arg);
}

// ---------------------------------------------------------------------------
// Per-tick waveform computation
// ---------------------------------------------------------------------------

void spectrum_slot_energies(const std::vector<float>& spectrum, double out[4]) {
    out[0] = out[1] = out[2] = out[3] = 0.0;
    if (spectrum.empty()) {
        return;
    }
    const int bands = static_cast<int>(spectrum.size());
    const int per = bands / 4;
    for (int slot = 0; slot < 4; slot++) {
        const int lo = slot * per;
        const int hi = (slot == 3) ? bands : (slot + 1) * per;
        double e = 0.0;
        for (int i = lo; i < hi; i++) {
            if (spectrum[i] > e) {
                e = spectrum[i];
            }
        }
        out[slot] = e;
    }
}

void channel_tick(WaveState& state, int mode, double hz, double level,
                  double idle_intensity, double freq_min_khz, double freq_max_khz,
                  uint8_t out_freq[4], uint8_t out_intensity[4],
                  const std::vector<float>* spectrum,
                  SpectrumState* spectrum_state,
                  double smooth_ms) {
    const double idle = std::clamp(idle_intensity, 0.0,
                                   static_cast<double>(kIntensityMax));
    const double ratio = audio_freq_ratio(hz, freq_min_khz * 1000.0, freq_max_khz * 1000.0);

    // SPECTRUM mode repurposes the four B0 slots as four band groups
    // (low..high) rather than four 25 ms sub-frames. The mode generators are
    // skipped: each slot's intensity is that group's spectrum energy (with
    // its own EMA via spectrum_state). Each slot's frequency is mapped from
    // its band group's *center* frequency — slot 0 (lowest) is the slowest
    // rate, slot 3 (highest) the fastest — so the spectrum reads as a
    // low-to-high sweep rather than four slots sharing one pitch.
    if (mode == MODE_SPECTRUM) {
        double energies[4] = {0.0, 0.0, 0.0, 0.0};
        if (spectrum != nullptr) {
            spectrum_slot_energies(*spectrum, energies);
        }
        for (int i = 0; i < 4; i++) {
            // Center of band group i in the log-spaced spectrum is at the
            // fraction (i + 0.5) / 4 of the [freq_min, freq_max] log range.
            const double group_ratio = (static_cast<double>(i) + 0.5) / 4.0;
            const uint8_t freq = blend_audio_freq(kFreqMin, group_ratio);
            double e = energies[i];
            if (spectrum_state != nullptr) {
                // Per-slot EMA; the streaming tick is 100 ms.
                e = smooth_level(&spectrum_state->slot_s[i], e, smooth_ms, 100);
            }
            const int amp = static_cast<int>(std::lround(e * kIntensityMax));
            const int intensity =
                static_cast<int>(std::clamp(amp, static_cast<int>(idle),
                                            static_cast<int>(kIntensityMax)));
            out_freq[i] = freq;
            out_intensity[i] = static_cast<uint8_t>(intensity);
        }
        return;
    }

    // The mode generator's sub-tick clock advances once per B0 slot (25 ms),
    // so it is called once per i: each of the 4 frequency/intensity slots
    // carries the pattern for its own 25 ms slice of the 100 ms tick.
    for (int i = 0; i < 4; i++) {
        CoyotePattern pat;
        switch (mode) {
            case MODE_WAVES:
                pat = coyote_mode_waves(state.waveclock, state.cyclecount);
                break;
            case MODE_STROBE:
                pat = coyote_mode_strobe(state.waveclock, state.cyclecount);
                break;
            case MODE_PULSE:
                pat = coyote_mode_pulse(state.waveclock, state.cyclecount);
                break;
            case MODE_RAINBOW:
                pat = coyote_mode_rainbow(state.waveclock, state.cyclecount);
                break;
            case MODE_BASS:
                pat = coyote_mode_bass(state.waveclock, state.cyclecount);
                break;
            case MODE_BREATH:
            case MODE_DEFAULT:
            default:
                pat = coyote_mode_breath(state.waveclock, state.cyclecount);
                break;
        }

        uint8_t freq;
        if (mode == MODE_WAVES) {
            // WAVES modulates its own frequency field, so the audio ratio is
            // blended onto a per-tick base (mirrors the original inline math).
            const double base_freq = (pat.frequency >= kFreqMin)
                                         ? static_cast<double>(pat.frequency)
                                         : static_cast<double>(kFreqMin);
            const double out_freq = base_freq + ratio * (100.0 - base_freq);
            const int period =
                std::clamp(static_cast<int>(std::lround(1000.0 / out_freq)), 10, 100);
            freq = compress_frequency(period);
        } else {
            freq = blend_audio_freq(pat.frequency, ratio);
        }
        out_freq[i] = freq;
        const int amp = static_cast<int>(std::lround(pat.amplitude * level));
        const int intensity =
            static_cast<int>(std::clamp(amp, static_cast<int>(idle),
                                        static_cast<int>(kIntensityMax)));
        out_intensity[i] = static_cast<uint8_t>(intensity);
    }
}

}  // namespace dglab
