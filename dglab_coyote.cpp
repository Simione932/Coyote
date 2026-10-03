// dglab_coyote.cpp
//
// C++ test application that exercises the SimpleBLE C API (simplecble)
// against a DG-LAB Coyote (郊狼) pulse host running the V3 Bluetooth
// protocol (https://github.com/dungeonlab-open/dglab-bluetooth-protocol).
//
// What it does:
//   1. Scans for DG-LAB devices (names starting with "47L").
//   2. Connects to the selected device.
//   3. Verifies the expected GATT layout:
//        service 0x180C / write char 0x150A (WRITE)
//        service 0x180C / notify char 0x150B (NOTIFY)
//        service 0x180A / battery char 0x1500 (READ/NOTIFY)
//   4. Subscribes to 0x150B notifications and parses B1 strength replies.
//   5. Reads the battery level from 0x180A/0x1500.
//   6. Writes the BF command (soft caps + balance parameters).
//   7. Streams B0 waveform commands every 100 ms, including one absolute
//      strength change on channel A with a sequence number, and waits for
//      the matching B1 acknowledgement.
//   8. Disconnects and releases all handles.
//
// Usage: dglab_coyote [a_freq_ms] [b_freq_ms]
//   a_freq_ms  channel A waveform frequency in ms, 10..1000 (default: ramp)
//   b_freq_ms  channel B waveform frequency in ms, 10..1000 (default: inverse of A)
//
// NOTE: The DG-LAB open protocol is licensed for personal/hobby use only.

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include <simplecble/simplecble.h>

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

// ---------------------------------------------------------------------------
// DG-LAB Coyote V3 protocol constants & waveform modes (from coyote-modes.h)
// ---------------------------------------------------------------------------

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
constexpr uint8_t kFreqMax = 240;
constexpr uint8_t kIntensityMax = 100;
constexpr uint8_t kStrengthMax = 200;

// Waveform generator modes (consistent with coyote-modes.h)
enum WaveformMode {
    MODE_DEFAULT = 0,
    MODE_BREATH = 1,
    MODE_WAVES = 2,
};

struct CoyotePattern {
    uint8_t frequency = 10;
    uint8_t amplitude = 0;
    uint8_t pulse_length = 0;
    uint16_t pause_length = 0;
};

inline CoyotePattern coyote_mode_nothing(uint32_t&, uint32_t&) {
    return {};
}

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
    constexpr double piOverTwo = M_PI_2;

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

}  // namespace dglab

// ---------------------------------------------------------------------------
// Test application
// ---------------------------------------------------------------------------

namespace {

struct Context {
    simpleble_adapter_t adapter = nullptr;
    simpleble_peripheral_t peripheral = nullptr;
    std::vector<simpleble_peripheral_t> found;
    int b1_count = 0;
    int b1_ack_count = 0;
    int last_b1_seq = -1;
    uint8_t last_b1_a = 0;
    uint8_t last_b1_b = 0;
    int battery = -1;
};

Context g_ctx;

void msleep(int ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    usleep(ms * 1000);
#endif
}

void print_hex(const char* label, const uint8_t* data, size_t len) {
    printf("%s", label);
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

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

void on_scan_found(simpleble_adapter_t, simpleble_peripheral_t peripheral, void*) {
    char* name = simpleble_peripheral_identifier(peripheral);
    char* address = simpleble_peripheral_address(peripheral);
    int16_t rssi = simpleble_peripheral_rssi(peripheral);

    if (name != nullptr && std::strncmp(name, dglab::kNamePrefix, 3) == 0) {
        printf("  [DG-LAB] %s [%s] rssi=%d\n", name, address ? address : "?", rssi);
        g_ctx.found.push_back(peripheral);
    } else {
        // Not a DG-LAB device; release the handle.
        simpleble_peripheral_release_handle(peripheral);
    }

    simpleble_free(name);
    simpleble_free(address);
}

void on_notify(simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t, const uint8_t* data,
               size_t data_length, void*) {
    if (data == nullptr || data_length == 0) {
        return;
    }

    if (data[0] == dglab::kReplyB1 && data_length >= 4) {
        g_ctx.b1_count++;
        g_ctx.last_b1_seq = data[1];
        g_ctx.last_b1_a = data[2];
        g_ctx.last_b1_b = data[3];
        printf("  [B1] seq=%d A_strength=%u B_strength=%u\n", data[1], data[2], data[3]);
        if (data[1] != 0) {
            g_ctx.b1_ack_count++;
        }
    } else {
        print_hex("  [notify] ", data, data_length);
    }
}

// a_freq_ms / b_freq_ms: linear waveform frequency per channel in ms (10..1000).
// 0 means "use the default pattern" (A: ramp, B: inverse of A).
int run(int a_freq_ms, int b_freq_ms) {
    atexit([]() {
        for (auto p : g_ctx.found) {
            if (p == g_ctx.peripheral) {
                g_ctx.peripheral = nullptr;
            }
            simpleble_peripheral_release_handle(p);
        }
        g_ctx.found.clear();
        if (g_ctx.peripheral != nullptr) {
            simpleble_peripheral_release_handle(g_ctx.peripheral);
            g_ctx.peripheral = nullptr;
        }
        if (g_ctx.adapter != nullptr) {
            simpleble_adapter_release_handle(g_ctx.adapter);
            g_ctx.adapter = nullptr;
        }
    });

    // -- 1. Adapter ---------------------------------------------------------
    if (!simpleble_adapter_is_bluetooth_enabled()) {
        printf("Bluetooth is not enabled.\n");
        return 1;
    }

    size_t adapter_count = simpleble_adapter_get_count();
    if (adapter_count == 0) {
        printf("No BLE adapter found.\n");
        return 1;
    }

    g_ctx.adapter = simpleble_adapter_get_handle(0);
    if (g_ctx.adapter == nullptr) {
        printf("Failed to get adapter handle.\n");
        return 1;
    }

    char* adapter_id = simpleble_adapter_identifier(g_ctx.adapter);
    printf("Using adapter: %s (SimpleBLE %s)\n", adapter_id ? adapter_id : "?",
           simpleble_get_version());
    simpleble_free(adapter_id);

    // -- 2. Scan ------------------------------------------------------------
    printf("Scanning for DG-LAB devices (10 s)...\n");
    simpleble_adapter_set_callback_on_scan_found(g_ctx.adapter, on_scan_found, nullptr);
    simpleble_adapter_scan_for(g_ctx.adapter, 10000);

    if (g_ctx.found.empty()) {
        printf("No DG-LAB device found. Make sure the Coyote host is powered on.\n");
        return 1;
    }

    if (g_ctx.found.size() > 1) {
        printf("Multiple DG-LAB devices found; using the first one.\n");
    }

    g_ctx.peripheral = g_ctx.found[0];
    char* name = simpleble_peripheral_identifier(g_ctx.peripheral);
    char* address = simpleble_peripheral_address(g_ctx.peripheral);
    printf("Connecting to %s [%s]...\n", name ? name : "?", address ? address : "?");
    simpleble_free(name);
    simpleble_free(address);

    // -- 3. Connect ---------------------------------------------------------
    if (simpleble_peripheral_connect(g_ctx.peripheral) != SIMPLEBLE_SUCCESS) {
        printf("Failed to connect.\n");
        return 1;
    }

    bool connected = false;
    simpleble_peripheral_is_connected(g_ctx.peripheral, &connected);
    printf("Connected: %s\n", connected ? "yes" : "no");
    if (!connected) {
        return 1;
    }

    // -- 4. Verify GATT layout ---------------------------------------------
    simpleble_uuid_t svc_main = make_uuid(dglab::kServiceMain);
    simpleble_uuid_t svc_battery = make_uuid(dglab::kServiceBattery);

    bool have_write = false;
    bool have_notify = false;
    bool have_battery = false;
    bool write_can_command = false;

    for (size_t i = 0; i < simpleble_peripheral_services_count(g_ctx.peripheral); i++) {
        simpleble_service_t service;
        if (simpleble_peripheral_services_get(g_ctx.peripheral, i, &service) != SIMPLEBLE_SUCCESS) {
            continue;
        }

        printf("  service %s (%zu characteristics)\n", service.uuid.value, service.characteristic_count);

        for (size_t j = 0; j < service.characteristic_count; j++) {
            const simpleble_characteristic_t& c = service.characteristics[j];
            printf("    char %s read=%d write_req=%d write_cmd=%d notify=%d\n", c.uuid.value,
                   c.can_read, c.can_write_request, c.can_write_command, c.can_notify);

            if (uuid_is(&c.uuid, dglab::kCharWrite)) {
                have_write = true;
                write_can_command = c.can_write_command;
            } else if (uuid_is(&c.uuid, dglab::kCharNotify)) {
                have_notify = true;
            } else if (uuid_is(&c.uuid, dglab::kCharBattery)) {
                have_battery = true;
            }
        }
    }

    if (!have_write || !have_notify) {
        printf("FAIL: expected DG-LAB GATT layout (0x180C/0x150A + 0x150B) not found.\n");
        simpleble_peripheral_disconnect(g_ctx.peripheral);
        return 1;
    }
    printf("GATT layout OK (write=%s, notify=%s, battery=%s)\n", have_write ? "yes" : "no",
           have_notify ? "yes" : "no", have_battery ? "yes" : "no");

    // -- 5. Subscribe to strength notifications ----------------------------
    simpleble_uuid_t char_write = make_uuid(dglab::kCharWrite);
    simpleble_uuid_t char_notify = make_uuid(dglab::kCharNotify);
    simpleble_uuid_t char_battery = make_uuid(dglab::kCharBattery);

    if (simpleble_peripheral_notify(g_ctx.peripheral, svc_main, char_notify, on_notify, nullptr) !=
        SIMPLEBLE_SUCCESS) {
        printf("WARN: failed to subscribe to 0x150B notifications.\n");
    } else {
        printf("Subscribed to 0x150B notifications.\n");
    }

    // -- 6. Read battery ----------------------------------------------------
    if (have_battery) {
        uint8_t* data = nullptr;
        size_t data_length = 0;
        if (simpleble_peripheral_read(g_ctx.peripheral, svc_battery, char_battery, &data,
                                      &data_length) == SIMPLEBLE_SUCCESS &&
            data != nullptr && data_length >= 1) {
            g_ctx.battery = data[0];
            printf("Battery level: %d%%\n", data[0]);
            simpleble_free(data);
        } else {
            printf("WARN: battery read failed.\n");
        }
    }

    // -- 7. Write BF (soft caps + balance parameters) -----------------------
    // Must be re-sent after every reconnect; it has no reply.
    dglab::BFCommand bf;
    bf.a_soft_cap = 100;  // keep the test conservative
    bf.b_soft_cap = 100;
    bf.a_freq_balance = 0;
    bf.b_freq_balance = 0;
    bf.a_intensity_balance = 0;
    bf.b_intensity_balance = 0;

    std::vector<uint8_t> bf_pkt = bf.encode();
    print_hex("BF write: ", bf_pkt.data(), bf_pkt.size());
    if (write_characteristic(g_ctx.peripheral, svc_main, char_write, bf_pkt.data(),
                             bf_pkt.size()) != SIMPLEBLE_SUCCESS) {
        printf("FAIL: BF write failed.\n");
        simpleble_peripheral_disconnect(g_ctx.peripheral);
        return 1;
    }
    printf("BF write OK.\n");

    // -- 8. Stream B0 waveform commands for 10 s ----------------------------
    // Test pattern: generates waveforms using coyote-modes.h pattern functions
    // (coyote_mode_breath / coyote_mode_waves) or default frequency ramps.
    // Every 5th packet sets channel A strength absolutely to 20 with
    // sequence number 1 and expects a B1 acknowledgement.
    const int duration_ms = 10000;
    const int period_ms = 100;
    const int ticks = duration_ms / period_ms;

    uint32_t waveclock_a = 0;
    uint32_t cyclecount_a = 0;
    uint32_t waveclock_b = 0;
    uint32_t cyclecount_b = 0;

    printf("Streaming B0 commands for %d s (Ctrl+C to abort)...\n", duration_ms / 1000);

    for (int t = 0; t < ticks; t++) {
        dglab::B0Command b0;

        for (int i = 0; i < 4; i++) {
            // Channel A waveform generation from coyote-modes.h
            dglab::CoyotePattern pat_a;
            if (a_freq_ms == dglab::MODE_BREATH) {
                pat_a = dglab::coyote_mode_breath(waveclock_a, cyclecount_a);
            } else if (a_freq_ms == dglab::MODE_WAVES) {
                pat_a = dglab::coyote_mode_waves(waveclock_a, cyclecount_a);
            } else {
                int linear = (a_freq_ms >= 10 && a_freq_ms <= 1000) ? a_freq_ms : (100 + (t % 10) * 100);
                pat_a.frequency = dglab::compress_frequency(linear);
                pat_a.amplitude = static_cast<uint8_t>(100 * t / ticks);
            }
            b0.a_freq[i] = (pat_a.frequency >= dglab::kFreqMin) ? pat_a.frequency : dglab::kFreqMin;
            b0.a_intensity[i] = pat_a.amplitude;

            // Channel B waveform generation from coyote-modes.h
            dglab::CoyotePattern pat_b;
            if (b_freq_ms == dglab::MODE_BREATH) {
                pat_b = dglab::coyote_mode_breath(waveclock_b, cyclecount_b);
            } else if (b_freq_ms == dglab::MODE_WAVES) {
                pat_b = dglab::coyote_mode_waves(waveclock_b, cyclecount_b);
            } else {
                int linear = (b_freq_ms >= 10 && b_freq_ms <= 1000) ? b_freq_ms : ((a_freq_ms >= 10 && a_freq_ms <= 1000) ? a_freq_ms : (100 + (t % 10) * 100));
                pat_b.frequency = dglab::compress_frequency(linear);
                pat_b.amplitude = static_cast<uint8_t>(dglab::kIntensityMax - (100 * t / ticks));
            }
            b0.b_freq[i] = (pat_b.frequency >= dglab::kFreqMin) ? pat_b.frequency : dglab::kFreqMin;
            b0.b_intensity[i] = pat_b.amplitude;
        }

        b0.a_strength = 20;
        b0.b_strength = 20;
        b0.a_mode = dglab::kStrengthModeAbsolute;
        b0.b_mode = dglab::kStrengthModeAbsolute;

        if (t % 5 == 0) {
            // Request acknowledgement periodically.
            b0.seq = 1;
        } else {
            b0.seq = 0;
        }

        std::vector<uint8_t> pkt = b0.encode();
        if (t % 10 == 0) {
            print_hex("B0 write: ", pkt.data(), pkt.size());
        }

        simpleble_err_t err = write_characteristic(g_ctx.peripheral, svc_main,
                                                 char_write, pkt.data(), pkt.size());
        if (err != SIMPLEBLE_SUCCESS) {
            printf("FAIL: B0 write failed at tick %d.\n", t);
            break;
        }

        msleep(period_ms);
    }

    // Give the device a moment to deliver any final B1 replies.
    msleep(500);

    // -- 9. Results ---------------------------------------------------------
    printf("\n--- Test summary ---\n");
    printf("B1 notifications received: %d\n", g_ctx.b1_count);
    printf("B1 acknowledgements (seq != 0): %d\n", g_ctx.b1_ack_count);
    if (g_ctx.b1_count > 0) {
        printf("Last B1: seq=%d A=%u B=%u\n", g_ctx.last_b1_seq, g_ctx.last_b1_a, g_ctx.last_b1_b);
    }
    if (g_ctx.battery >= 0) {
        printf("Battery: %d%%\n", g_ctx.battery);
    }

    bool pass = have_write && have_notify && g_ctx.b1_ack_count > 0;
    printf("Result: %s\n", pass ? "PASS" : "FAIL");

    // -- 10. Cleanup --------------------------------------------------------
    simpleble_peripheral_unsubscribe(g_ctx.peripheral, svc_main, char_notify);
    simpleble_peripheral_disconnect(g_ctx.peripheral);
    return pass ? 0 : 1;
}

}  // namespace

inline int parse_mode_arg(const char* arg) {
    if (std::strcmp(arg, "breath") == 0 || std::strcmp(arg, "1") == 0) {
        return dglab::MODE_BREATH;
    }
    if (std::strcmp(arg, "waves") == 0 || std::strcmp(arg, "2") == 0) {
        return dglab::MODE_WAVES;
    }
    return std::atoi(arg);
}

int main(int argc, char** argv) {
    int a_freq_ms = 0;
    int b_freq_ms = 0;

    if (argc > 1) {
        a_freq_ms = parse_mode_arg(argv[1]);
    }
    if (argc > 2) {
        b_freq_ms = parse_mode_arg(argv[2]);
    }

    return run(a_freq_ms, b_freq_ms);
}
