// dglab_protocol.h
//
// DG-LAB Coyote V3 Bluetooth protocol (https://github.com/dungeonlab-open/
// dglab-bluetooth-protocol): UUIDs, command heads, waveform-mode generators,
// B0/BF command builders, and the shared runtime settings (CoyoteSettings).
//
// Both the CLI (dglab_coyote_audio) and the GUI (dglab_gui) link this:
// the CLI's command-line arguments/keystrokes and the GUI's sliders/radios
// all mutate the same CoyoteSettings, which the session reads every tick.

#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace dglab {

// 16-bit base UUID: 0000xxxx-0000-1000-8000-00805f9b34fb
constexpr const char* kServiceMain = "0000180c-0000-1000-8000-00805f9b34fb";
constexpr const char* kCharWrite = "0000150a-0000-1000-8000-00805f9b34fb";
constexpr const char* kCharNotify = "0000150b-0000-1000-8000-00805f9b34fb";
constexpr const char* kServiceBattery = "0000180a-0000-1000-8000-00805f9b34fb";
constexpr const char* kCharBattery = "00001500-0000-1000-8000-00805f9b34fb";

// Device advertisement names.
constexpr const char* kNamePrefix = "47L";  // 47L121000 = host 3.0, 47L120100 = sensor

// Command heads.
constexpr uint8_t kCmdB0 = 0xB0;  // 20-byte waveform/strength command, sent every 100 ms
constexpr uint8_t kCmdBF = 0xBF;  // 7-byte soft caps + balance parameters
constexpr uint8_t kReplyB1 = 0xB1;  // 4-byte strength reply

// Strength value interpretation modes (2 bits per channel).
constexpr uint8_t kStrengthModeNone = 0b00;
constexpr uint8_t kStrengthModeIncrease = 0b01;
constexpr uint8_t kStrengthModeDecrease = 0b10;
constexpr uint8_t kStrengthModeAbsolute = 0b11;

// Valid ranges.
constexpr uint8_t kFreqMin = 10;
constexpr uint8_t kIntensityMax = 100;
constexpr uint8_t kStrengthMax = 200;

// Waveform generator modes (consistent with coyote-modes.h).
enum WaveformMode {
    MODE_DEFAULT = 0,
    MODE_BREATH = 1,
    MODE_WAVES = 2,
    MODE_STROBE = 3,
    MODE_PULSE = 4,
    MODE_RAINBOW = 5,
    MODE_BASS = 6,
};

// Display names for the waveform modes (shared by the CLI help text and the
// GUI mode radios).
extern const char* const kModeNames[7];

// Parse a mode argument: accepts the mode name or its numeric id.
int parse_mode_arg(const char* arg);

struct CoyotePattern {
    uint8_t frequency = 10;
    uint8_t amplitude = 0;
    uint8_t pulse_length = 0;
    uint16_t pause_length = 0;
};

// Breath waveform mode generator (1100 ms cycle, 44 sub-ticks of 25 ms)
inline CoyotePattern coyote_mode_breath(uint32_t& waveclock, uint32_t&) {
    CoyotePattern out;
    if (waveclock < 8 * 4) {
        out.pulse_length = 1;
        out.pause_length = 9;
        out.frequency = 10;
        out.amplitude = static_cast<uint8_t>(std::min<uint32_t>(100, waveclock * 4));
    }
    waveclock++;
    if (waveclock > (7 + 3) * 4) {
        waveclock = 0;
    }
    return out;
}

// Waves waveform mode generator (8000 ms cycle, 320 sub-ticks of 25 ms)
inline CoyotePattern coyote_mode_waves(uint32_t& waveclock, uint32_t& cyclecount) {
    CoyotePattern out;
    constexpr uint16_t rampUpTime = 30 * 4;
    constexpr uint16_t rampDownTime = 50 * 4;
    constexpr uint16_t cycleTime = rampUpTime + rampDownTime;
    constexpr uint16_t maxAmp = 100;
    constexpr double piOverTwo = 1.57079632679489661923;  // M_PI_2

    out.pulse_length = 10;

    if (waveclock <= rampUpTime) {
        double index = static_cast<double>(waveclock) / static_cast<double>(rampUpTime);
        out.amplitude = static_cast<uint8_t>(std::floor(std::sin(piOverTwo * index) * static_cast<double>(maxAmp)));
    } else {
        double index = static_cast<double>(waveclock - rampUpTime) / static_cast<double>(rampDownTime);
        out.amplitude = static_cast<uint8_t>(std::floor(std::sin(piOverTwo * index + piOverTwo) * static_cast<double>(maxAmp)));
    }

    out.pause_length = 10 * ((cyclecount % 8) + 2);
    out.frequency = static_cast<uint8_t>(10 + (cyclecount % 8) * 3);

    waveclock++;
    if (waveclock > cycleTime) {
        waveclock = 0;
        cyclecount++;
    }
    return out;
}

// Strobe waveform mode generator (240 ms cycle, 96 sub-ticks of 25 ms).
// Holds full amplitude with a single sharp pulse and flickers the off-time
// (pause_length) through a fast ramp, so the strobe rate can be tuned by the
// caller via the audio-driven frequency. Only the on/off timing varies here,
// unlike breath/waves which modulate amplitude.
inline CoyotePattern coyote_mode_strobe(uint32_t& waveclock, uint32_t& cyclecount) {
    CoyotePattern out;
    constexpr uint16_t strobeTicks = 6 * 4;  // 6 flicker steps

    out.frequency = kFreqMin;
    out.amplitude = 100;
    out.pulse_length = 1;
    out.pause_length = 10 + static_cast<uint16_t>(cyclecount % strobeTicks) * 4;

    waveclock++;
    if (waveclock > strobeTicks) {
        waveclock = 0;
        cyclecount++;
    }
    return out;
}

// Pulse / heartbeat waveform mode generator (16 sub-ticks of 25 ms).
// Emits two sharp beats close together (lub-dub) with a short gap, then a
// longer pause, at full amplitude. Drives the on/off timing rather than the
// amplitude envelope.
inline CoyotePattern coyote_mode_pulse(uint32_t& waveclock, uint32_t&) {
    CoyotePattern out;

    out.frequency = kFreqMin;
    out.amplitude = 100;
    out.pulse_length = 1;
    if (waveclock < 2 * 4) {
        out.pause_length = 4;   // gap between the two beats
    } else if (waveclock < 4 * 4) {
        out.pause_length = 30;  // rest after the second beat
    }

    waveclock++;
    if (waveclock > 4 * 4) {
        waveclock = 0;
    }
    return out;
}

// Rainbow / color-chase waveform mode generator (32 sub-ticks of 25 ms).
// Sweeps the output frequency up and back down through a spectrum while the
// amplitude rises to full at the sweep peak and then fades. This makes the
// *rate* the primary animated axis, unlike breath/waves which modulate
// amplitude.
inline CoyotePattern coyote_mode_rainbow(uint32_t& waveclock, uint32_t& cyclecount) {
    CoyotePattern out;
    constexpr uint16_t rainbowTicks = 8 * 4;  // 8 sweep steps
    constexpr uint8_t sweepLo = 10;
    constexpr uint8_t sweepHi = 42;
    constexpr uint8_t maxAmp = 100;

    double frac = static_cast<double>(cyclecount % rainbowTicks) / static_cast<double>(rainbowTicks);
    if (frac < 0.5) {
        double t = frac / 0.5;
        out.frequency = static_cast<uint8_t>(sweepLo + t * (sweepHi - sweepLo));
    } else {
        double t = (frac - 0.5) / 0.5;
        out.frequency = static_cast<uint8_t>(sweepHi - t * (sweepHi - sweepLo));
    }
    double amp = frac < 0.5 ? (frac / 0.5) : (1.0 - frac / 0.5);
    out.amplitude = static_cast<uint8_t>(std::lround(amp * maxAmp));
    out.pulse_length = 10;
    out.pause_length = 10;

    waveclock++;
    if (waveclock > rainbowTicks) {
        waveclock = 0;
        cyclecount++;
    }
    return out;
}

// Bass / firefly waveform mode generator (16 sub-ticks of 25 ms).
// Emits mostly darkness with an occasional sparse, bright spike. The caller
// scales the amplitude (intensity) by the detected audio level so louder
// input produces brighter flashes. The generator only decides *when* a
// firefly fires (a sparse, irregular cadence).
inline CoyotePattern coyote_mode_bass(uint32_t& waveclock, uint32_t& cyclecount) {
    CoyotePattern out;
    out.frequency = kFreqMin;
    if (cyclecount % 8 < 3) {
        // Between fires: off.
        out.amplitude = 0;
        out.pulse_length = 0;
        out.pause_length = 0;
    } else {
        // Fire: a bright, wide pulse.
        out.amplitude = 100;
        out.pulse_length = 10;
        out.pause_length = 5;
    }
    waveclock++;
    if (waveclock > 4 * 4) {
        waveclock = 0;
        cyclecount++;
    }
    return out;
}

// Converts a linear "waveform frequency" input (10..1000) to the compressed
// on-wire value (10..240) as specified by the V3 protocol.
inline uint8_t compress_frequency(int input) {
    if (input >= 10 && input <= 100) {
        return static_cast<uint8_t>(input);
    }
    if (input >= 101 && input <= 600) {
        return static_cast<uint8_t>((input - 100) / 5 + 100);
    }
    if (input >= 601 && input <= 1000) {
        return static_cast<uint8_t>((input - 600) / 10 + 200);
    }
    return kFreqMin;
}

// Builds a 20-byte B0 command.
//   seq: 0..15 (non-zero requests a B1 acknowledgement of strength changes)
//   a_mode/b_mode: one of the kStrengthMode* values
struct B0Command {
    uint8_t seq = 0;
    uint8_t a_mode = kStrengthModeNone;
    uint8_t b_mode = kStrengthModeNone;
    uint8_t a_strength = 0;
    uint8_t b_strength = 0;
    uint8_t a_freq[4] = {0, 0, 0, 0};
    uint8_t a_intensity[4] = {0, 0, 0, 0};
    uint8_t b_freq[4] = {0, 0, 0, 0};
    uint8_t b_intensity[4] = {0, 0, 0, 0};

    std::vector<uint8_t> encode() const {
        std::vector<uint8_t> pkt(20, 0);
        pkt[0] = kCmdB0;
        pkt[1] = static_cast<uint8_t>(((seq & 0x0F) << 4) | (a_mode & 0x03) << 2 | (b_mode & 0x03));
        pkt[2] = a_strength;
        pkt[3] = b_strength;
        std::memcpy(&pkt[4], a_freq, 4);
        std::memcpy(&pkt[8], a_intensity, 4);
        std::memcpy(&pkt[12], b_freq, 4);
        std::memcpy(&pkt[16], b_intensity, 4);
        return pkt;
    }
};

// Builds a 7-byte BF command.
struct BFCommand {
    uint8_t a_soft_cap = kStrengthMax;
    uint8_t b_soft_cap = kStrengthMax;
    uint8_t a_freq_balance = 0;
    uint8_t b_freq_balance = 0;
    uint8_t a_intensity_balance = 0;
    uint8_t b_intensity_balance = 0;

    std::vector<uint8_t> encode() const {
        return {kCmdBF, a_soft_cap, b_soft_cap, a_freq_balance, b_freq_balance,
                a_intensity_balance, b_intensity_balance};
    }
};

// ---------------------------------------------------------------------------
// Shared runtime settings.
//
// Every value is an atomic: the CLI (key handler) and the GUI (sliders /
// radios) run on their own threads and mutate these directly, while the
// session's streaming tick reads them 10 times a second.
//
// The adjust helpers are the GUI analogues of the CLI keystrokes:
//   r/t -> gain_up/gain_down for channel A   (steps of 5)
//   o/p -> gain_up/gain_down for channel B   (steps of 5)
// ---------------------------------------------------------------------------
class CoyoteSettings {
public:
    CoyoteSettings()
        : gain_a_(5), gain_b_(5), freq_min_khz_(0.02f), freq_max_khz_(10.0),
          smooth_ms_(0.0), idle_intensity_(0.0),
          a_mode_(MODE_DEFAULT), b_mode_(MODE_DEFAULT) {}

    // Channel A (left) master strength, 0..200.
    int gain_a() const { return gain_a_.load(); }
    void set_gain_a(int v) { gain_a_.store(clamp_strength(v)); }

    // Channel B (right) master strength, 0..200.
    int gain_b() const { return gain_b_.load(); }
    void set_gain_b(int v) { gain_b_.store(clamp_strength(v)); }

    // Lowest audio frequency considered, kHz, in [0.02, 24.0).
    double freq_min_khz() const { return freq_min_khz_.load(); }
    void set_freq_min_khz(double khz) { freq_min_khz_.store(clamp_freq_khz(khz)); }

    // Highest audio frequency considered, kHz, in (0.02, 24].
    double freq_max_khz() const { return freq_max_khz_.load(); }
    void set_freq_max_khz(double khz) { freq_max_khz_.store(clamp_freq_khz(khz)); }

    // Intensity smoothing time constant, ms; 0 = off. Range [0, 10000].
    double smooth_ms() const { return smooth_ms_.load(); }
    void set_smooth_ms(double ms) { smooth_ms_.store(clamp_smooth_ms(ms)); }

    // Baseline intensity applied to silent channels, 0..100.
    double idle_intensity() const { return idle_intensity_.load(); }
    void set_idle_intensity(double v) { idle_intensity_.store(clamp_idle(v)); }

    // Waveform mode per channel (one of WaveformMode, clamped to 0..6).
    int a_mode() const { return a_mode_.load(); }
    void set_a_mode(int m) { a_mode_.store(clamp_mode(m)); }
    int b_mode() const { return b_mode_.load(); }
    void set_b_mode(int m) { b_mode_.store(clamp_mode(m)); }

    // GUI analogues of the CLI gain keystrokes (steps of 5, clamped to 0..200).
    void gain_up(int channel) {
        std::atomic<int>& g = (channel == 1 ? gain_b_ : gain_a_);
        g.store(clamp_strength(g.load() + kGainStep));
    }
    void gain_down(int channel) {
        std::atomic<int>& g = (channel == 1 ? gain_b_ : gain_a_);
        g.store(clamp_strength(g.load() - kGainStep));
    }

    static constexpr int kGainStep = 5;

private:
    static int clamp_strength(int v) {
        return v < 0 ? 0 : (v > kStrengthMax ? kStrengthMax : v);
    }
    static double clamp_freq_khz(double v) {
        if (v <= 0.02) return 0.02;
        if (v > 24.0) return 24.0;
        return v;
    }
    static double clamp_smooth_ms(double v) {
        if (v < 0.0) return 0.0;
        if (v > 10000.0) return 10000.0;
        return v;
    }
    static double clamp_idle(double v) {
        if (v < 0.0) return 0.0;
        if (v > 100.0) return 100.0;
        return v;
    }
    static int clamp_mode(int v) {
        if (v < MODE_DEFAULT) return MODE_DEFAULT;
        if (v > MODE_BASS) return MODE_BASS;
        return v;
    }

    std::atomic<int> gain_a_;
    std::atomic<int> gain_b_;
    std::atomic<float> freq_min_khz_;
    std::atomic<float> freq_max_khz_;
    std::atomic<float> smooth_ms_;
    std::atomic<float> idle_intensity_;
    std::atomic<int> a_mode_;
    std::atomic<int> b_mode_;
};

// ---------------------------------------------------------------------------
// Per-tick waveform computation, shared by the session.
// ---------------------------------------------------------------------------

// Per-channel generator state (25 ms sub-tick clock + cycle counter).
struct WaveState {
    uint32_t waveclock = 0;
    uint32_t cyclecount = 0;
};

// Computes the four (frequency, intensity) pairs sent to `channel` (0 = A /
// left, 1 = B / right) in one 100 ms tick. `state` must persist across ticks;
// `level` is the (already smoothed) 0..1 loudness of the channel's dominant
// tone and `hz` its frequency (0/0 when silent). The frequency is blended
// with the detected audio pitch; the intensity scales with loudness and is
// floored at the idle intensity.
void channel_tick(WaveState& state, int mode, double hz, double level,
                  double idle_intensity, double freq_min_khz, double freq_max_khz,
                  uint8_t out_freq[4], uint8_t out_intensity[4]);
inline void channel_tick(WaveState& state, int mode, double hz, double level,
                         double idle_intensity, double freq_max_khz,
                         uint8_t out_freq[4], uint8_t out_intensity[4]) {
    channel_tick(state, mode, hz, level, idle_intensity, 0.02, freq_max_khz,
                 out_freq, out_intensity);
}

}  // namespace dglab
