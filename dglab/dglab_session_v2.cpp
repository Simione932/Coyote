// dglab_session_v2.cpp
//
// Implementation of CoyoteV2Session: the shared BLE + streaming backend for
// the DG-LAB Coyote V2 pulse host. Mirrors CoyoteSession's interface while
// using the V2 GATT layout (power + pattern A/B writes instead of B0).

#include "dglab_session_v2.h"

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
    simpleble_err_t err = simpleble_peripheral_write_request(peripheral, service, characteristic,
                                                             data, data_length);
    if (err != SIMPLEBLE_SUCCESS) {
        err = simpleble_peripheral_write_command(peripheral, service, characteristic,
                                                 data, data_length);
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

void CoyoteV2Session::on_scan_found(simpleble_adapter_t, simpleble_peripheral_t peripheral,
                                    void* user) {
    auto* self = static_cast<CoyoteV2Session*>(user);
    char* name = simpleble_peripheral_identifier(peripheral);
    char* address = simpleble_peripheral_address(peripheral);
    int16_t rssi = simpleble_peripheral_rssi(peripheral);

    bool matches = false;
    if (name != nullptr && std::strncmp(name, kNamePrefixV2, 3) == 0) {
        matches = true;
    }
    if (!matches) {
        size_t mfg_count = simpleble_peripheral_manufacturer_data_count(peripheral);
        for (size_t i = 0; i < mfg_count; i++) {
            simpleble_manufacturer_data_t mfg;
            if (simpleble_peripheral_manufacturer_data_get(peripheral, i, &mfg) ==
                SIMPLEBLE_SUCCESS) {
                if (mfg.manufacturer_id == kMfgIdV2 ||
                    (mfg.data_length >= 2 && mfg.data[0] == 0x96 && mfg.data[1] == 0x19)) {
                    matches = true;
                    break;
                }
            }
        }
    }

    if (matches) {
        self->logf("  [DG-LAB V2] %s [%s] rssi=%d", name ? name : "?",
                   address ? address : "?", rssi);
        self->found_.push_back(peripheral);
    } else {
        simpleble_peripheral_release_handle(peripheral);
    }
    simpleble_free(name);
    simpleble_free(address);
}

void CoyoteV2Session::on_power_notify(simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t,
                                      const uint8_t* data, size_t data_length, void* user) {
    auto* self = static_cast<CoyoteV2Session*>(user);
    if (data == nullptr || data_length < 3) return;
    // V2 power notifications are the same format as the written packet.
    uint16_t power_a = 0, power_b = 0;
    parse_power_v2(data, data_length, power_a, power_b);
    self->logf("  [Power notify] A=%u B=%u", power_a, power_b);
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

CoyoteV2Session::CoyoteV2Session(AudioState* audio, CoyoteV2Settings* settings,
                                 LogFn log, void* log_user)
    : audio_(audio), settings_(settings), log_(log), log_user_(log_user) {}

CoyoteV2Session::~CoyoteV2Session() {
    stop();
    cleanup();
}

void CoyoteV2Session::logf(const char* fmt, ...) const {
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

SessionStatus CoyoteV2Session::start() {
    if (running_.load()) {
        return SessionStatus::kStopped;
    }
    running_.store(true);
    stop_requested_.store(false);
    connected_.store(false);
    battery_.store(-1);
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
        logf("Scanning for DG-LAB Coyote V2 devices (10 s)...");
        simpleble_adapter_set_callback_on_scan_found(adapter_, on_scan_found, this);
        simpleble_adapter_scan_for(adapter_, 10000);
    }

    if (found_.empty()) {
        logf("No DG-LAB Coyote V2 device found (looking for '47L' / 0x1996).");
        running_.store(false);
        cleanup();
        return SessionStatus::kNoDevice;
    }

    if (found_.size() > 1) {
        logf("Multiple DG-LAB V2 devices found; using the first one.");
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

    msleep(500);

    running_.store(false);
    connected_.store(false);
    logf("Audio analyses: %d", audio_ ? audio_->analyses.load() : 0);
    if (battery_.load() >= 0) {
        logf("Battery: %d%%", battery_.load());
    }

    // --- 4. Tidy up the device connection --------------------------------
    cleanup();
    return stream_ok ? status : SessionStatus::kStreamFailed;
}

void CoyoteV2Session::stop() {
    stop_requested_.store(true);
    if (adapter_ != nullptr) {
        simpleble_adapter_scan_stop(adapter_);
    }
}

// ---------------------------------------------------------------------------
// Device setup (scan already done in start)
// ---------------------------------------------------------------------------

bool CoyoteV2Session::setup_device(SessionStatus* status) {
    // --- 1. Connect -------------------------------------------------------
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

    // --- 2. Verify GATT layout -------------------------------------------
    simpleble_uuid_t svc_main = make_uuid(kServiceMainV2);
    simpleble_uuid_t svc_battery = make_uuid(kServiceBatteryV2);

    bool have_power = false;
    bool have_pattern_a = false;
    bool have_pattern_b = false;
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
            if (uuid_is(&c.uuid, kCharPowerV2)) {
                have_power = true;
            } else if (uuid_is(&c.uuid, kCharPatternA)) {
                have_pattern_a = true;
            } else if (uuid_is(&c.uuid, kCharPatternB)) {
                have_pattern_b = true;
            } else if (uuid_is(&c.uuid, kCharBatteryV2)) {
                have_battery = true;
            }
        }
    }

    if (!have_power || !have_pattern_a || !have_pattern_b) {
        logf("FAIL: expected Coyote V2 GATT layout (0x955a180b service) not found.");
        simpleble_peripheral_disconnect(peripheral_);
        *status = SessionStatus::kGattMissing;
        return false;
    }
    logf("GATT layout OK (power=%s, patternA=%s, patternB=%s, battery=%s)",
         have_power ? "yes" : "no", have_pattern_a ? "yes" : "no",
         have_pattern_b ? "yes" : "no", have_battery ? "yes" : "no");

    // --- 3. Subscribe to power notifications -----------------------------
    simpleble_uuid_t char_power = make_uuid(kCharPowerV2);
    simpleble_uuid_t char_battery = make_uuid(kCharBatteryV2);

    if (simpleble_peripheral_notify(peripheral_, svc_main, char_power, on_power_notify, this) !=
        SIMPLEBLE_SUCCESS) {
        logf("WARN: failed to subscribe to power notifications.");
    } else {
        logf("Subscribed to power notifications.");
    }

    // --- 4. Read battery --------------------------------------------------
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

    *status = SessionStatus::kStopped;
    return true;
}

// ---------------------------------------------------------------------------
// Streaming loop
// ---------------------------------------------------------------------------

bool CoyoteV2Session::streaming_loop() {
    if (audio_ == nullptr || settings_ == nullptr) {
        return true;
    }

    logf("Streaming audio-reactive V2 commands (gains A=%d B=%d, mode A=%d B=%d)",
         settings_->gain_a(), settings_->gain_b(),
         settings_->a_mode(), settings_->b_mode());

    WaveState state_a;
    WaveState state_b;

    const int period_ms = kPeriodMs;
    int tick = 0;

    simpleble_uuid_t svc_main = make_uuid(kServiceMainV2);
    simpleble_uuid_t char_power = make_uuid(kCharPowerV2);
    simpleble_uuid_t char_pattern_a = make_uuid(kCharPatternA);
    simpleble_uuid_t char_pattern_b = make_uuid(kCharPatternB);

    while (running_.load() && !stop_requested_.load()) {
        const double lhz = audio_->left_hz.load();
        const double rhz = audio_->right_hz.load();
        const double lvl_a = audio_->left_level.load();
        const double lvl_b = audio_->right_level.load();

        audio_->freq_min_hz.store(static_cast<float>(settings_->freq_min_khz() * 1000.0));
        audio_->freq_max_hz.store(static_cast<float>(settings_->freq_max_khz() * 1000.0));

        // Calculate power for each channel (0..kPowerMaxV2 range).
        // Master gain slider (0..100) sets overall channel power.
        uint16_t power_a = static_cast<uint16_t>(
            std::clamp((settings_->gain_a() / 100.0) * kPowerMaxV2, 0.0,
                       static_cast<double>(kPowerMaxV2)));
        uint16_t power_b = static_cast<uint16_t>(
            std::clamp((settings_->gain_b() / 100.0) * kPowerMaxV2, 0.0,
                       static_cast<double>(kPowerMaxV2)));

        std::vector<uint8_t> pwr_pkt = encode_power_v2(power_a, power_b);
        if (write_characteristic(peripheral_, svc_main, char_power, pwr_pkt.data(),
                                 pwr_pkt.size()) != SIMPLEBLE_SUCCESS) {
            logf("FAIL: power write failed at tick %d.", tick);
            return false;
        }

        // Generate pattern for channel A.
        CoyotePatternV2 pat_a;
        switch (settings_->a_mode()) {
            case MODE_BREATH:
                pat_a = coyote_mode_breath_v2(state_a.waveclock, state_a.cyclecount);
                break;
            case MODE_WAVES:
                pat_a = coyote_mode_waves_v2(state_a.waveclock, state_a.cyclecount);
                break;
            case MODE_STROBE:
                pat_a = coyote_mode_strobe_v2(state_a.waveclock, state_a.cyclecount);
                break;
            case MODE_PULSE:
                pat_a = coyote_mode_pulse_v2(state_a.waveclock, state_a.cyclecount);
                break;
            case MODE_RAINBOW:
                pat_a = coyote_mode_rainbow_v2(state_a.waveclock, state_a.cyclecount);
                break;
            case MODE_BASS:
                pat_a = coyote_mode_bass_v2(state_a.waveclock, state_a.cyclecount);
                break;
            default:
                pat_a = coyote_mode_breath_v2(state_a.waveclock, state_a.cyclecount);
                break;
        }
        const double amp_scale_a = (lvl_a > 0.0) ? lvl_a : 0.1;
        pat_a.amplitude = static_cast<uint8_t>(
            std::clamp(amp_scale_a * (pat_a.amplitude / 100.0) * 31.0, 0.0, 31.0));
        pat_a.pulse_length = static_cast<uint8_t>(
            std::clamp(static_cast<double>(pat_a.pulse_length), 1.0, 31.0));
        {
            const double total_cycle =
                static_cast<double>(pat_a.pulse_length + pat_a.pause_length);
            const double base_freq =
                (total_cycle > 0.0) ? (1000.0 / total_cycle) : 10.0;
            const double ratio_a = audio_freq_ratio(lhz, settings_->freq_min_khz() * 1000.0,
                                                    settings_->freq_max_khz() * 1000.0);
            const double out_freq_a = base_freq + ratio_a * (100.0 - base_freq);
            pat_a.pause_length = static_cast<uint16_t>(
                std::clamp(1000.0 / out_freq_a - pat_a.pulse_length, 10.0, 1023.0));
        }
        {
            std::vector<uint8_t> pkt = encode_pattern_v2(pat_a);
            if (write_characteristic(peripheral_, svc_main, char_pattern_a, pkt.data(),
                                     pkt.size()) != SIMPLEBLE_SUCCESS) {
                logf("FAIL: pattern A write failed at tick %d.", tick);
                return false;
            }
        }

        // Generate pattern for channel B.
        CoyotePatternV2 pat_b;
        switch (settings_->b_mode()) {
            case MODE_BREATH:
                pat_b = coyote_mode_breath_v2(state_b.waveclock, state_b.cyclecount);
                break;
            case MODE_WAVES:
                pat_b = coyote_mode_waves_v2(state_b.waveclock, state_b.cyclecount);
                break;
            case MODE_STROBE:
                pat_b = coyote_mode_strobe_v2(state_b.waveclock, state_b.cyclecount);
                break;
            case MODE_PULSE:
                pat_b = coyote_mode_pulse_v2(state_b.waveclock, state_b.cyclecount);
                break;
            case MODE_RAINBOW:
                pat_b = coyote_mode_rainbow_v2(state_b.waveclock, state_b.cyclecount);
                break;
            case MODE_BASS:
                pat_b = coyote_mode_bass_v2(state_b.waveclock, state_b.cyclecount);
                break;
            default:
                pat_b = coyote_mode_breath_v2(state_b.waveclock, state_b.cyclecount);
                break;
        }
        const double amp_scale_b = (lvl_b > 0.0) ? lvl_b : 0.1;
        pat_b.amplitude = static_cast<uint8_t>(
            std::clamp(amp_scale_b * (pat_b.amplitude / 100.0) * 31.0, 0.0, 31.0));
        pat_b.pulse_length = static_cast<uint8_t>(
            std::clamp(static_cast<double>(pat_b.pulse_length), 1.0, 31.0));
        {
            const double total_cycle =
                static_cast<double>(pat_b.pulse_length + pat_b.pause_length);
            const double base_freq =
                (total_cycle > 0.0) ? (1000.0 / total_cycle) : 10.0;
            const double ratio_b = audio_freq_ratio(rhz, settings_->freq_min_khz() * 1000.0,
                                                    settings_->freq_max_khz() * 1000.0);
            const double out_freq_b = base_freq + ratio_b * (100.0 - base_freq);
            pat_b.pause_length = static_cast<uint16_t>(
                std::clamp(1000.0 / out_freq_b - pat_b.pulse_length, 10.0, 1023.0));
        }
        {
            std::vector<uint8_t> pkt = encode_pattern_v2(pat_b);
            if (write_characteristic(peripheral_, svc_main, char_pattern_b, pkt.data(),
                                     pkt.size()) != SIMPLEBLE_SUCCESS) {
                logf("FAIL: pattern B write failed at tick %d.", tick);
                return false;
            }
        }

        if (tick % 20 == 0) {
            logf("[Tick %d] Audio L: %.1f Hz (lvl %.2f) -> PwrA=%u | Audio R: %.1f Hz "
                 "(lvl %.2f) -> PwrB=%u",
                 tick, lhz, lvl_a, power_a, rhz, lvl_b, power_b);
        }

        tick++;
        msleep(period_ms);
    }

    return true;
}

void CoyoteV2Session::cleanup() {
    if (adapter_ != nullptr) {
        simpleble_adapter_set_callback_on_scan_found(adapter_, nullptr, nullptr);
    }
    simpleble_peripheral_t p_active = peripheral_;
    peripheral_ = nullptr;

    if (p_active != nullptr) {
        simpleble_uuid_t svc_main = make_uuid(kServiceMainV2);
        simpleble_uuid_t char_power = make_uuid(kCharPowerV2);
        simpleble_peripheral_unsubscribe(p_active, svc_main, char_power);
        simpleble_peripheral_disconnect(p_active);
        simpleble_peripheral_release_handle(p_active);
    }
    for (auto p : found_) {
        if (p == p_active) continue;
        simpleble_peripheral_release_handle(p);
    }
    found_.clear();
    if (adapter_ != nullptr) {
        simpleble_adapter_release_handle(adapter_);
        adapter_ = nullptr;
    }
}

}  // namespace dglab
