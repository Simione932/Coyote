// dglab_session_v2.h
//
// CoyoteV2Session: a shared BLE backend for the DG-LAB Coyote V2 pulse host,
// with an interface compatible to CoyoteSession (V3). The same start()/stop()
// lifecycle, status queries, and audio-driven tick are available so that the
// GUI and CLI can switch between V2 and V3 backends with minimal changes.
//
// V2-specific notes:
//   - Power is sent as a 3-byte packet on 0x955a1504 (both channels in one
//     write), not as per-channel strength fields inside a B0 command.
//   - Patterns are per-channel: 0x955a1506 drives Channel A, 0x955a1505
//     drives Channel B.
//   - No B1 strength notifications; power feedback comes from the power
//     characteristic's NOTIFY handle (0x955a1504).
//   - Battery is read from 0x955a180a/0x955a1500, same as V3.
//
// Typical usage:
//
//   dglab::CoyoteV2Settings settings;
//   dglab::AudioState audio;
//   dglab::start_audio_capture(&audio, settings.freq_max_khz() * 1000.0);
//   dglab::CoyoteV2Session session(&audio, &settings, log);
//   dglab::SessionStatus st = session.start();   // blocks until stop()
//   dglab::stop_audio_capture(&audio);

#pragma once

#include <atomic>
#include <cstdio>
#include <thread>
#include <vector>

#include <simplecble/simplecble.h>

#include "dglab_audio.h"
#include "dglab_session.h"       // for AudioState, CoyoteSettings, SessionStatus, LogFn
#include "dglab_protocol_v2.h"  // for CoyotePatternV2, encode helpers

namespace dglab {

// Re-use the same SessionStatus enum so callers can handle V2/V3 identically.
// (defined in dglab_session.h)

// ---------------------------------------------------------------------------
// Settings that mirror CoyoteSettings but are scoped for V2-specific fields.
// In the future this may be folded back into CoyoteSettings.
// ---------------------------------------------------------------------------
class CoyoteV2Settings {
public:
    CoyoteV2Settings()
        : gain_a_(20), gain_b_(20), freq_min_khz_(0.02f), freq_max_khz_(10.0),
          a_mode_(MODE_DEFAULT), b_mode_(MODE_DEFAULT) {}

    // Channel A (left) master power scale, 0..100 (scales against kPowerMaxV2).
    int gain_a() const { return gain_a_.load(); }
    void set_gain_a(int v) { gain_a_.store(clamp_gain(v)); }

    // Channel B (right) master power scale, 0..100.
    int gain_b() const { return gain_b_.load(); }
    void set_gain_b(int v) { gain_b_.store(clamp_gain(v)); }

    // Lowest audio frequency considered, kHz, in [0.02, 24.0).
    double freq_min_khz() const { return freq_min_khz_.load(); }
    void set_freq_min_khz(double khz) { freq_min_khz_.store(clamp_freq_khz(khz)); }

    // Highest audio frequency considered, kHz, in (0.02, 24].
    double freq_max_khz() const { return freq_max_khz_.load(); }
    void set_freq_max_khz(double khz) { freq_max_khz_.store(clamp_freq_khz(khz)); }

    // Waveform mode per channel (one of WaveformModeV2, clamped to 0..6).
    int a_mode() const { return a_mode_.load(); }
    void set_a_mode(int m) { a_mode_.store(clamp_mode(m)); }
    int b_mode() const { return b_mode_.load(); }
    void set_b_mode(int m) { b_mode_.store(clamp_mode(m)); }

    // GUI analogues of the CLI gain keystrokes (steps of 5, clamped to 0..100).
    void gain_up(int channel) {
        std::atomic<int>& g = (channel == 1 ? gain_b_ : gain_a_);
        g.store(clamp_gain(g.load() + kGainStep));
    }
    void gain_down(int channel) {
        std::atomic<int>& g = (channel == 1 ? gain_b_ : gain_a_);
        g.store(clamp_gain(g.load() - kGainStep));
    }

    static constexpr int kGainStep = 5;

private:
    static int clamp_gain(int v) { return v < 0 ? 0 : (v > 100 ? 100 : v); }
    static double clamp_freq_khz(double v) {
        if (v <= 0.02) return 0.02;
        if (v > 24.0) return 24.0;
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
    std::atomic<int> a_mode_;
    std::atomic<int> b_mode_;
};

// ---------------------------------------------------------------------------
// Session (compatible with CoyoteSession from dglab_session.h).
// ---------------------------------------------------------------------------
class CoyoteV2Session {
public:
    CoyoteV2Session(AudioState* audio, CoyoteV2Settings* settings,
                    LogFn log = nullptr, void* log_user = nullptr);
    ~CoyoteV2Session();

    CoyoteV2Session(const CoyoteV2Session&) = delete;
    CoyoteV2Session& operator=(const CoyoteV2Session&) = delete;

    // Runs the full lifecycle on the calling thread and blocks until stop()
    // is called from another thread or an error ends the session.
    SessionStatus start();

    // Asks the (blocking) streaming loop to exit. Safe when not running.
    void stop();

    bool running() const { return running_.load(); }

    // Status out, readable from any thread.
    bool connected() const { return connected_.load(); }
    int battery() const { return battery_.load(); }

    // Period of the B0 streaming loop in ms (the protocol rate).
    static constexpr int kPeriodMs = 100;

private:
    // simplecble callbacks (registered with `this` as user data).
    static void on_scan_found(simpleble_adapter_t, simpleble_peripheral_t peripheral,
                              void* user);
    static void on_power_notify(simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t,
                                const uint8_t* data, size_t data_length, void* user);

    void logf(const char* fmt, ...) const;
    bool setup_device(SessionStatus* status);
    bool streaming_loop();  // false if a write failed mid-stream
    void cleanup();

    AudioState* audio_;
    CoyoteV2Settings* settings_;
    LogFn log_;
    void* log_user_;

    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<int> battery_{-1};

    // SimpleBLE handles owned while the session is alive.
    simpleble_adapter_t adapter_ = nullptr;
    simpleble_peripheral_t peripheral_ = nullptr;
    std::vector<simpleble_peripheral_t> found_;
};

}  // namespace dglab
