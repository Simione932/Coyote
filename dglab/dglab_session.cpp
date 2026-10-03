// dglab_session.cpp
//
// Implementation of CoyoteSession: the shared BLE + streaming backend for the
// DG-LAB Coyote V3 pulse host.

#include "dglab_session.h"

#include <algorithm>
#include <cstdarg>
#include <cstring>
#include <cstdlib>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

namespace dglab {

namespace {

simpleble_uuid_t make_uuid(const char* value) {
    simpleble_uuid_t uuid;
    std::memset(&uuid, 0, sizeof(uuid));
    std::strncpy(uuid.value, value, SIMPLEBLE_UUID_STR_LEN - 1);
    return uuid;
}

bool uuid_is(const simpleble_uuid_t* uuid, const char* expected) {
    for (size_t i = 0; i < SIMPLEBLE_UUID_STR_LEN - 1; i++) {
        char a = uuid->value[i];
        char b = expected[i];
        if (a >= 'A' && a <= 'Z') {
            a = static_cast<char>(a - 'A' + 'a');
        }
        if (b >= 'A' && b <= 'Z') {
            b = static_cast<char>(b - 'A' + 'a');
        }
        if (a != b) {
            return false;
        }
    }
    return true;
}

simpleble_err_t write_characteristic(simpleble_peripheral_t peripheral, simpleble_uuid_t service,
                                     simpleble_uuid_t characteristic, const uint8_t* data,
                                     size_t data_length) {
    simpleble_err_t err = simpleble_peripheral_write_request(peripheral, service, characteristic, data, data_length);
    if (err != SIMPLEBLE_SUCCESS) {
        err = simpleble_peripheral_write_command(peripheral, service, characteristic, data, data_length);
    }
    return err;
}

void msleep(int ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    usleep(ms * 1000);
#endif
}

}  // namespace

// --- simplecble callbacks ---------------------------------------------------

void CoyoteSession::on_scan_found(simpleble_adapter_t, simpleble_peripheral_t peripheral,
                                  void* user) {
    auto* self = static_cast<CoyoteSession*>(user);
    char* name = simpleble_peripheral_identifier(peripheral);
    char* address = simpleble_peripheral_address(peripheral);
    int16_t rssi = simpleble_peripheral_rssi(peripheral);
    if (name != nullptr && std::strncmp(name, kNamePrefix, 3) == 0) {
        self->logf("  [DG-LAB] %s [%s] rssi=%d", name, address ? address : "?", rssi);
        self->found_.push_back(peripheral);
    } else {
        // Not a DG-LAB device; release the handle.
        simpleble_peripheral_release_handle(peripheral);
    }
    simpleble_free(name);
    simpleble_free(address);
}

void CoyoteSession::on_notify(simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t,
                              const uint8_t* data, size_t data_length, void* user) {
    auto* self = static_cast<CoyoteSession*>(user);
    if (data == nullptr || data_length == 0) {
        return;
    }
    if (data[0] == kReplyB1 && data_length >= 4) {
        self->b1_count_++;
        self->last_b1_seq_.store(data[1]);
        self->last_b1_a_.store(data[2]);
        self->last_b1_b_.store(data[3]);
        if (data[1] != 0) {
            self->b1_ack_count_++;
        }
    }
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

CoyoteSession::CoyoteSession(AudioState* audio, CoyoteSettings* settings,
                             LogFn log, void* log_user)
    : audio_(audio), settings_(settings), log_(log), log_user_(log_user) {}

CoyoteSession::~CoyoteSession() {
    stop();
    cleanup();
}

void CoyoteSession::logf(const char* fmt, ...) const {
    if (log_ == nullptr) {
        return;
    }
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    log_(buf, log_user_);
}

SessionStatus CoyoteSession::start() {
    if (running_.load()) {
        return SessionStatus::kStopped;
    }
    running_.store(true);
    stop_requested_.store(false);
    connected_.store(false);
    battery_.store(-1);
    b1_count_.store(0);
    b1_ack_count_.store(0);
    last_b1_seq_.store(-1);
    last_b1_a_.store(0);
    last_b1_b_.store(0);
    found_.clear();

    // --- 0. Adapter -------------------------------------------------------
    if (!simpleble_adapter_is_bluetooth_enabled()) {
        logf("Bluetooth is not enabled.");
        running_.store(false);
        return SessionStatus::kNoAdapter;
    }

    size_t adapter_count = simpleble_adapter_get_count();
    if (adapter_count == 0) {
        logf("No BLE adapter found.");
        running_.store(false);
        return SessionStatus::kNoAdapter;
    }

    adapter_ = simpleble_adapter_get_handle(0);
    if (adapter_ == nullptr) {
        logf("Failed to get adapter handle.");
        running_.store(false);
        return SessionStatus::kNoAdapter;
    }

    char* adapter_id = simpleble_adapter_identifier(adapter_);
    logf("Using adapter: %s (SimpleBLE %s)", adapter_id ? adapter_id : "?",
         simpleble_get_version());
    simpleble_free(adapter_id);

    SessionStatus status = SessionStatus::kStopped;
    {
        // --- 1. Scan -------------------------------------------------------
        logf("Scanning for DG-LAB devices (10 s)...");
        simpleble_adapter_set_callback_on_scan_found(
            adapter_, on_scan_found, this);
        simpleble_adapter_scan_for(adapter_, 10000);
    }

    if (found_.empty()) {
        logf("No DG-LAB device found. Make sure the Coyote host is powered on.");
        running_.store(false);
        cleanup();
        return SessionStatus::kNoDevice;
    }

    if (found_.size() > 1) {
        logf("Multiple DG-LAB devices found; using the first one.");
    }

    peripheral_ = found_[0];
    char* name = simpleble_peripheral_identifier(peripheral_);
    char* address = simpleble_peripheral_address(peripheral_);
    logf("Connecting to %s [%s]...", name ? name : "?", address ? address : "?");
    simpleble_free(name);
    simpleble_free(address);

    // --- 2. Connect + GATT + stream --------------------------------------
    if (!setup_device(&status)) {
        running_.store(false);
        cleanup();
        return status;
    }

    // --- 3. Stream until stop() or error ---------------------------------
    bool stream_ok = streaming_loop();

    // Give the device a moment to deliver any final B1 replies.
    msleep(500);

    running_.store(false);
    connected_.store(false);
    logf("Audio analyses: %d | B1 notifications: %d | B1 acks (seq!=0): %d",
         audio_ ? audio_->analyses.load() : 0, b1_count_.load(), b1_ack_count_.load());
    if (b1_count_.load() > 0) {
        logf("Last B1: seq=%d A=%u B=%u", last_b1_seq_.load(),
             last_b1_a_.load(), last_b1_b_.load());
    }
    if (battery_.load() >= 0) {
        logf("Battery: %d%%", battery_.load());
    }

    // --- 4. Tidy up the device connection --------------------------------
    cleanup();
    return stream_ok ? status : SessionStatus::kStreamFailed;
}

void CoyoteSession::stop() {
    stop_requested_.store(true);
    if (adapter_ != nullptr) {
        simpleble_adapter_scan_stop(adapter_);
    }
}

// ---------------------------------------------------------------------------
// Device setup (scan already done in start)
// ---------------------------------------------------------------------------

bool CoyoteSession::setup_device(SessionStatus* status) {
    // --- 3. Connect -------------------------------------------------------
    if (simpleble_peripheral_connect(peripheral_) != SIMPLEBLE_SUCCESS) {
        logf("Failed to connect.");
        *status = SessionStatus::kConnectFailed;
        return false;
    }

    bool connected = false;
    simpleble_peripheral_is_connected(peripheral_, &connected);
    logf("Connected: %s", connected ? "yes" : "no");
    if (!connected) {
        *status = SessionStatus::kConnectFailed;
        return false;
    }
    connected_.store(true);

    // --- 4. Verify GATT layout -------------------------------------------
    simpleble_uuid_t svc_main = make_uuid(kServiceMain);
    simpleble_uuid_t svc_battery = make_uuid(kServiceBattery);

    bool have_write = false;
    bool have_notify = false;
    bool have_battery = false;

    for (size_t i = 0; i < simpleble_peripheral_services_count(peripheral_); i++) {
        simpleble_service_t service;
        if (simpleble_peripheral_services_get(peripheral_, i, &service) != SIMPLEBLE_SUCCESS) {
            continue;
        }
        logf("  service %s (%zu characteristics)", service.uuid.value,
             service.characteristic_count);

        for (size_t j = 0; j < service.characteristic_count; j++) {
            const simpleble_characteristic_t& c = service.characteristics[j];
            logf("    char %s read=%d write_req=%d write_cmd=%d notify=%d", c.uuid.value,
                 c.can_read, c.can_write_request, c.can_write_command, c.can_notify);
            if (uuid_is(&c.uuid, kCharWrite)) {
                have_write = true;
            } else if (uuid_is(&c.uuid, kCharNotify)) {
                have_notify = true;
            } else if (uuid_is(&c.uuid, kCharBattery)) {
                have_battery = true;
            }
        }
    }

    if (!have_write || !have_notify) {
        logf("FAIL: expected DG-LAB GATT layout (0x180C/0x150A + 0x150B) not found.");
        simpleble_peripheral_disconnect(peripheral_);
        *status = SessionStatus::kGattMissing;
        return false;
    }
    logf("GATT layout OK (write=yes, notify=yes, battery=%s)", have_battery ? "yes" : "no");

    // --- 5. Subscribe to strength notifications --------------------------
    simpleble_uuid_t char_write = make_uuid(kCharWrite);
    simpleble_uuid_t char_notify = make_uuid(kCharNotify);
    simpleble_uuid_t char_battery = make_uuid(kCharBattery);

    if (simpleble_peripheral_notify(peripheral_, svc_main, char_notify, on_notify, this) !=
        SIMPLEBLE_SUCCESS) {
        logf("WARN: failed to subscribe to 0x150B notifications.");
    } else {
        logf("Subscribed to 0x150B notifications.");
    }

    // --- 6. Read battery --------------------------------------------------
    if (have_battery) {
        uint8_t* data = nullptr;
        size_t data_length = 0;
        if (simpleble_peripheral_read(peripheral_, svc_battery, char_battery, &data,
                                      &data_length) == SIMPLEBLE_SUCCESS &&
            data != nullptr && data_length >= 1) {
            battery_.store(data[0]);
            logf("Battery level: %d%%", data[0]);
            simpleble_free(data);
        } else {
            logf("WARN: battery read failed.");
        }
    }

    // --- 7. Write BF (soft caps + balance parameters) ---------------------
    // Must be re-sent after every reconnect; it has no reply.
    BFCommand bf;
    bf.a_soft_cap = kStrengthMax;
    bf.b_soft_cap = kStrengthMax;

    std::vector<uint8_t> bf_pkt = bf.encode();
    if (write_characteristic(peripheral_, svc_main, char_write, bf_pkt.data(),
                             bf_pkt.size()) != SIMPLEBLE_SUCCESS) {
        logf("FAIL: BF write failed.");
        simpleble_peripheral_disconnect(peripheral_);
        *status = SessionStatus::kStreamFailed;
        return false;
    }
    logf("BF write OK.");

    *status = SessionStatus::kStopped;
    return true;
}

// ---------------------------------------------------------------------------
// Streaming loop
// ---------------------------------------------------------------------------

bool CoyoteSession::streaming_loop() {
    if (audio_ == nullptr || settings_ == nullptr) {
        return true;
    }

    logf("Streaming audio-reactive B0 commands (gains A=%d B=%d, smoothing=%s, mode A=%d B=%d)",
         settings_->gain_a(), settings_->gain_b(),
         settings_->smooth_ms() > 0.0 ? "on" : "off",
         settings_->a_mode(), settings_->b_mode());

    // Intensity smoothing state, persisted across ticks; 0 = no smoothing.
    double llevel_s = 0.0;
    double rlevel_s = 0.0;

    WaveState state_a;
    WaveState state_b;

    const int period_ms = kPeriodMs;

    simpleble_uuid_t svc_main = make_uuid(kServiceMain);
    simpleble_uuid_t char_write = make_uuid(kCharWrite);

    int t = 0;
    while (running_.load() && !stop_requested_.load()) {
        const double lhz = audio_->left_hz.load();
        const double rhz = audio_->right_hz.load();
        const double smooth_ms = settings_->smooth_ms();
        const double idle = settings_->idle_intensity();
        const double freq_min_khz = settings_->freq_min_khz();
        const double freq_max_khz = settings_->freq_max_khz();

        // Keep the capture thread's analysis limits in sync with the settings
        // (the GUI's Freq min/max sliders change them live).
        audio_->freq_min_hz.store(static_cast<float>(freq_min_khz * 1000.0));
        audio_->freq_max_hz.store(static_cast<float>(freq_max_khz * 1000.0));

        const double llevel = smooth_level(&llevel_s, audio_->left_level.load(), smooth_ms, period_ms);
        const double rlevel = smooth_level(&rlevel_s, audio_->right_level.load(), smooth_ms, period_ms);

        B0Command b0;
        uint8_t freq_a[4];
        uint8_t intens_a[4];
        uint8_t freq_b[4];
        uint8_t intens_b[4];
        channel_tick(state_a, settings_->a_mode(), lhz, llevel, idle, freq_min_khz, freq_max_khz,
                     freq_a, intens_a);
        channel_tick(state_b, settings_->b_mode(), rhz, rlevel, idle, freq_min_khz, freq_max_khz,
                     freq_b, intens_b);
        std::memcpy(b0.a_freq, freq_a, 4);
        std::memcpy(b0.a_intensity, intens_a, 4);
        std::memcpy(b0.b_freq, freq_b, 4);
        std::memcpy(b0.b_intensity, intens_b, 4);

        b0.a_strength = static_cast<uint8_t>(std::clamp(settings_->gain_a(), 0, static_cast<int>(kStrengthMax)));
        b0.b_strength = static_cast<uint8_t>(std::clamp(settings_->gain_b(), 0, static_cast<int>(kStrengthMax)));
        b0.a_mode = kStrengthModeAbsolute;
        b0.b_mode = kStrengthModeAbsolute;

        // Request acknowledgement periodically (every 5th packet).
        b0.seq = (t % 5 == 0) ? 1 : 0;

        std::vector<uint8_t> pkt = b0.encode();
        simpleble_err_t err = write_characteristic(peripheral_, svc_main,
                                                   char_write, pkt.data(), pkt.size());
        if (err != SIMPLEBLE_SUCCESS) {
            logf("FAIL: B0 write failed at tick %d.", t);
            return false;
        }

        t++;
        msleep(period_ms);
    }

    return true;
}

void CoyoteSession::cleanup() {
    if (adapter_ != nullptr) {
        simpleble_adapter_set_callback_on_scan_found(adapter_, nullptr, nullptr);
    }
    simpleble_peripheral_t p_active = peripheral_;
    peripheral_ = nullptr;

    if (p_active != nullptr) {
        simpleble_uuid_t svc_main = make_uuid(kServiceMain);
        simpleble_uuid_t char_notify = make_uuid(kCharNotify);
        simpleble_peripheral_unsubscribe(p_active, svc_main, char_notify);
        simpleble_peripheral_disconnect(p_active);
        simpleble_peripheral_release_handle(p_active);
    }
    for (auto p : found_) {
        if (p == p_active) {
            continue;
        }
        simpleble_peripheral_release_handle(p);
    }
    found_.clear();
    if (adapter_ != nullptr) {
        simpleble_adapter_release_handle(adapter_);
        adapter_ = nullptr;
    }
}

}  // namespace dglab
