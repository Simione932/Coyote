// dglab_session.h
//
// CoyoteSession: the shared BLE backend for the DG-LAB Coyote V3 pulse host.
// Owns the whole device lifecycle:
//
//   adapter -> scan (names starting with "47L") -> connect -> GATT layout
//   verification (0x180C/0x150A write + 0x150B notify + 0x180A/0x1500
//   battery) -> subscribe to 0x150B B1 strength notifications -> read
//   battery -> write BF (soft caps) -> stream B0 waveform commands every
//   100 ms until stop() is called or the device goes away.
//
// The streaming tick reads a shared CoyoteSettings (gains, modes, freq max,
// smoothing, idle intensity) and an AudioState (dominant tone per channel),
// so the CLI keystrokes and the GUI controls both take effect live without
// restarting the session.
//
// Typical usage:
//
//   dglab::CoyoteSettings settings;      // filled by CLI args or GUI
//   dglab::AudioState audio;
//   dglab::start_audio_capture(&audio, settings.freq_max_khz() * 1000.0);
//   dglab::CoyoteSession session(&audio, &settings, log);
//   dglab::SessionStatus st = session.start();   // blocks until stop()
//   dglab::stop_audio_capture(&audio);
//
// NOTE: The DG-LAB open protocol is licensed for personal/hobby use only.

#pragma once

#include <atomic>
#include <cstdio>
#include <thread>
#include <vector>

#include <simplecble/simplecble.h>

#include "dglab_audio.h"
#include "dglab_protocol.h"

namespace dglab {

// Final state after a start()/stop() cycle.
enum class SessionStatus {
    kStopped,        // stop() was requested (the normal CLI 'q' / GUI Quit path)
    kNoAdapter,      // no BLE adapter (or Bluetooth disabled)
    kNoDevice,       // scan finished without finding a DG-LAB device
    kConnectFailed,  // connect failed
    kGattMissing,    // expected GATT layout not found
    kStreamFailed,   // a B0 write failed mid-stream
};

// Diagnostics callback; msg is a printf-style formatted line. nullptr = quiet.
using LogFn = void (*)(const char* msg, void* user);

class CoyoteSession {
public:
    CoyoteSession(AudioState* audio, CoyoteSettings* settings,
                  LogFn log = nullptr, void* log_user = nullptr);
    ~CoyoteSession();

    CoyoteSession(const CoyoteSession&) = delete;
    CoyoteSession& operator=(const CoyoteSession&) = delete;

    // Runs the full lifecycle on the calling thread and blocks until stop()
    // is called from another thread or an error ends the session.
    SessionStatus start();

    // Asks the (blocking) streaming loop to exit. Safe when not running.
    void stop();

    bool running() const { return running_.load(); }

    // Status out, readable from any thread.
    bool connected() const { return connected_.load(); }
    int battery() const { return battery_.load(); }
    int b1_count() const { return b1_count_.load(); }
    int b1_ack_count() const { return b1_ack_count_.load(); }
    int last_b1_seq() const { return last_b1_seq_.load(); }
    uint8_t last_b1_a() const { return last_b1_a_.load(); }
    uint8_t last_b1_b() const { return last_b1_b_.load(); }

    // Period of the B0 streaming loop in ms (the protocol rate).
    static constexpr int kPeriodMs = 100;

private:
    // simplecble callbacks (registered with `this` as user data).
    static void on_scan_found(simpleble_adapter_t, simpleble_peripheral_t peripheral,
                              void* user);
    static void on_notify(simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t,
                          const uint8_t* data, size_t data_length, void* user);

    void logf(const char* fmt, ...) const;
    bool setup_device(SessionStatus* status);
    bool streaming_loop();  // false if a B0 write failed mid-stream
    void cleanup();

    AudioState* audio_;
    CoyoteSettings* settings_;
    LogFn log_;
    void* log_user_;

    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<int> battery_{-1};

    // B1 strength-notification accounting.
    std::atomic<int> b1_count_{0};
    std::atomic<int> b1_ack_count_{0};
    std::atomic<int> last_b1_seq_{-1};
    std::atomic<uint8_t> last_b1_a_{0};
    std::atomic<uint8_t> last_b1_b_{0};

    // Owned while the session is alive.
    simpleble_adapter_t adapter_ = nullptr;
    simpleble_peripheral_t peripheral_ = nullptr;
    std::vector<simpleble_peripheral_t> found_;
};

}  // namespace dglab
