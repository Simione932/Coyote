// dglab_pawprint.cpp
//
// C++ test application that exercises the SimpleBLE C API (simplecble)
// against a DG-LAB PawPrint (爪印) wireless button sensor running the
// V1.1 Bluetooth protocol
// (https://github.com/dungeonlab-open/dglab-bluetooth-protocol).
//
// What it does:
//   1. Scans for a PawPrint sensor (name 47L120300 = V1.1).
//   2. Connects and verifies the GATT layout:
//        service 0x180C / write char 0x150A (WRITE)
//        service 0x180C / notify char 0x150B (NOTIFY)
//   3. Subscribes to 0x150B notifications and parses 51/5A/5B/5C/D0/F1
//      messages.
//   4. Immediately writes the 50 command in "no trigger" mode (required
//      right after connect, otherwise the device may drop the link).
//   5. Waits for the 51 message (device type + battery).
//   6. Exercises the 70 command: solid green, blue/white blink, stop.
//   7. Writes 5F (parameter reset).
//   8. Switches to D0 physical-data mode and streams the 100 ms D0
//      telemetry (button state, acceleration, XYZ angles, external
//      voltage) for a few seconds. Press the button during this phase
//      to see the pressed state change.
//   9. Restores "no trigger" mode, disconnects, releases all handles.
//
// NOTE: The DG-LAB open protocol is licensed for personal/hobby use only.

#include <cinttypes>
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

// ---------------------------------------------------------------------------
// DG-LAB PawPrint V1.1 protocol constants
// ---------------------------------------------------------------------------

namespace dglab {

// 16-bit base UUID: 0000xxxx-0000-1000-8000-00805f9b34fb
constexpr const char* kServiceMain = "0000180c-0000-1000-8000-00805f9b34fb";
constexpr const char* kCharWrite = "0000150a-0000-1000-8000-00805f9b34fb";
constexpr const char* kCharNotify = "0000150b-0000-1000-8000-00805f9b34fb";

// Device advertisement names.
constexpr const char* kNameV11 = "47L120300";  // PawPrint V1.1
constexpr const char* kNameV10 = "47L120100";  // PawPrint V1.0 (needs firmware update)

// Command heads.
constexpr uint8_t kCmd50 = 0x50;  // 17-byte trigger mode configuration
constexpr uint8_t kCmd5F = 0x5F;  // reset trigger parameter to 0
constexpr uint8_t kCmd60 = 0x60;  // start XYZ angle auto-detection
constexpr uint8_t kCmd70 = 0x70;  // shoulder light on / blink

// 50 command trigger modes.
constexpr uint8_t kModeNone = 0x00;
constexpr uint8_t kModeD0 = 0xD0;  // physical data direct mode (100 ms stream)

// Message heads (received on 0x150B).
constexpr uint8_t kMsg51 = 0x51;  // color + device type + battery
constexpr uint8_t kMsg5A = 0x5A;  // trigger event
constexpr uint8_t kMsg5B = 0x5B;  // trigger cancelled
constexpr uint8_t kMsg5C = 0x5C;  // parameter value change
constexpr uint8_t kMsgD0 = 0xD0;  // physical data (every 100 ms in D0 mode)
constexpr uint8_t kMsgF1 = 0xF1;  // auto-detection result

// Device type reported in 51 messages.
constexpr uint8_t kDeviceTypeV11 = 0x03;

// Shoulder light colors (0x00..0x07).
constexpr uint8_t kColorOff = 0x00;
constexpr uint8_t kColorYellow = 0x01;
constexpr uint8_t kColorRed = 0x02;
constexpr uint8_t kColorPurple = 0x03;
constexpr uint8_t kColorBlue = 0x04;
constexpr uint8_t kColorCyan = 0x05;
constexpr uint8_t kColorGreen = 0x06;
constexpr uint8_t kColorWhite = 0x07;

// Blink speeds for the 70 command.
constexpr uint8_t kBlinkSlow = 0x01;
constexpr uint8_t kBlinkFast = 0x02;
constexpr uint8_t kBlinkStop = 0x03;

// Builds a 17-byte 50 command: 0x50 + indicator color + mode + 14B settings.
struct Cmd50 {
    uint8_t indicator_color = kColorOff;
    uint8_t mode = kModeNone;
    uint8_t settings[14] = {0};

    std::vector<uint8_t> encode() const {
        std::vector<uint8_t> pkt(17, 0);
        pkt[0] = kCmd50;
        pkt[1] = indicator_color;
        pkt[2] = mode;
        std::memcpy(&pkt[3], settings, 14);
        return pkt;
    }
};

// Builds a 2-byte 70 command: solid shoulder light.
inline std::vector<uint8_t> cmd70_solid(uint8_t color) { return {kCmd70, color}; }

// Builds a 4-byte 70 command: blinking shoulder light.
inline std::vector<uint8_t> cmd70_blink(uint8_t color1, uint8_t color2, uint8_t speed) {
    return {kCmd70, color1, color2, speed};
}

}  // namespace dglab

// ---------------------------------------------------------------------------
// Test application
// ---------------------------------------------------------------------------

namespace {

struct Context {
    simpleble_adapter_t adapter = nullptr;
    simpleble_peripheral_t peripheral = nullptr;
    std::vector<simpleble_peripheral_t> found;
    int msg51_count = 0;
    int msg5a_count = 0;
    int msg5b_count = 0;
    int msg5c_count = 0;
    int msgd0_count = 0;
    int msgf1_count = 0;
    int device_type = -1;
    int battery = -1;
    int pressed_ticks = 0;
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

void on_scan_found(simpleble_adapter_t, simpleble_peripheral_t peripheral, void*) {
    char* name = simpleble_peripheral_identifier(peripheral);
    char* address = simpleble_peripheral_address(peripheral);
    int16_t rssi = simpleble_peripheral_rssi(peripheral);

    if (name != nullptr &&
        (std::strcmp(name, dglab::kNameV11) == 0 || std::strcmp(name, dglab::kNameV10) == 0)) {
        printf("  [PawPrint] %s [%s] rssi=%d\n", name, address ? address : "?", rssi);
        g_ctx.found.push_back(peripheral);
    } else {
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

    switch (data[0]) {
        case dglab::kMsg51:
            if (data_length >= 4) {
                g_ctx.msg51_count++;
                g_ctx.device_type = data[2];
                g_ctx.battery = data[3];
                printf("  [51] color=%u type=%u battery=%u%%\n", data[1], data[2], data[3]);
            }
            break;
        case dglab::kMsg5A:
            if (data_length >= 4) {
                g_ctx.msg5a_count++;
                printf("  [5A] trigger event id=%u param=%u\n", data[2], data[3]);
            }
            break;
        case dglab::kMsg5B:
            if (data_length >= 3) {
                g_ctx.msg5b_count++;
                printf("  [5B] trigger cancelled, event id=%u\n", data[2]);
            }
            break;
        case dglab::kMsg5C:
            if (data_length >= 4) {
                g_ctx.msg5c_count++;
                printf("  [5C] param change, event id=%u param=%u\n", data[2], data[3]);
            }
            break;
        case dglab::kMsgD0:
            if (data_length >= 9) {
                g_ctx.msgd0_count++;
                bool pressed = data[3] != 0x00;
                if (pressed) {
                    g_ctx.pressed_ticks++;
                }
                // Print every 10th sample (1 s) plus any pressed sample.
                if (pressed || data[2] % 10 == 0) {
                    printf("  [D0] seq=%u pressed=%s accel=%u X=%u Y=%u Z=%u extV=%u\n", data[2],
                           pressed ? "yes" : "no", data[4], data[5], data[6], data[7], data[8]);
                }
            }
            break;
        case dglab::kMsgF1:
            g_ctx.msgf1_count++;
            print_hex("  [F1] auto-detect: ", data, data_length);
            break;
        default:
            print_hex("  [notify] ", data, data_length);
            break;
    }
}

int run() {
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
    // Put the sensor in pairing mode first: power on, hold the button until
    // the shoulder lights flash white/blue alternately.
    printf("Scanning for PawPrint sensor (10 s)...\n");
    simpleble_adapter_set_callback_on_scan_found(g_ctx.adapter, on_scan_found, nullptr);
    simpleble_adapter_scan_for(g_ctx.adapter, 10000);

    if (g_ctx.found.empty()) {
        printf("No PawPrint found. Hold the button after power-on until the\n"
               "shoulder lights flash white/blue, then retry.\n");
        return 1;
    }

    g_ctx.peripheral = g_ctx.found[0];
    char* name = simpleble_peripheral_identifier(g_ctx.peripheral);
    char* address = simpleble_peripheral_address(g_ctx.peripheral);
    printf("Connecting to %s [%s]...\n", name ? name : "?", address ? address : "?");

    if (name != nullptr && std::strcmp(name, dglab::kNameV10) == 0) {
        printf("WARN: device reports as V1.0 (47L120100). Update its firmware via the\n"
               "DG-LAB 3.0 app (Settings -> Accessory firmware) for V1.1 features.\n");
    }

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
    simpleble_uuid_t char_write = make_uuid(dglab::kCharWrite);
    simpleble_uuid_t char_notify = make_uuid(dglab::kCharNotify);

    bool have_write = false;
    bool have_notify = false;

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
            } else if (uuid_is(&c.uuid, dglab::kCharNotify)) {
                have_notify = true;
            }
        }
    }

    if (!have_write || !have_notify) {
        printf("FAIL: expected DG-LAB GATT layout (0x180C/0x150A + 0x150B) not found.\n");
        simpleble_peripheral_disconnect(g_ctx.peripheral);
        return 1;
    }
    printf("GATT layout OK.\n");

    // -- 5. Subscribe to notifications --------------------------------------
    if (simpleble_peripheral_notify(g_ctx.peripheral, svc_main, char_notify, on_notify, nullptr) !=
        SIMPLEBLE_SUCCESS) {
        printf("FAIL: could not subscribe to 0x150B notifications.\n");
        simpleble_peripheral_disconnect(g_ctx.peripheral);
        return 1;
    }
    printf("Subscribed to 0x150B notifications.\n");

    auto write_cmd = [&](const std::vector<uint8_t>& pkt, const char* label) -> bool {
        print_hex(label, pkt.data(), pkt.size());
        simpleble_err_t err = simpleble_peripheral_write_request(g_ctx.peripheral, svc_main, char_write,
                                                                 pkt.data(), pkt.size());
        if (err != SIMPLEBLE_SUCCESS) {
            err = simpleble_peripheral_write_command(g_ctx.peripheral, svc_main, char_write,
                                                     pkt.data(), pkt.size());
        }
        if (err != SIMPLEBLE_SUCCESS) {
            printf("FAIL: %s write failed.\n", label);
            return false;
        }
        return true;
    };

    // -- 6. Immediately write 50 (no trigger mode) --------------------------
    // Required right after connect, otherwise the device may drop the link.
    dglab::Cmd50 cmd50_none;
    cmd50_none.indicator_color = dglab::kColorOff;
    cmd50_none.mode = dglab::kModeNone;
    if (!write_cmd(cmd50_none.encode(), "50 write (no trigger): ")) {
        simpleble_peripheral_disconnect(g_ctx.peripheral);
        return 1;
    }
    printf("50 write OK.\n");

    // -- 7. Wait for the 51 message (device type + battery) -----------------
    for (int i = 0; i < 50 && g_ctx.msg51_count == 0; i++) {
        msleep(100);
    }
    if (g_ctx.msg51_count > 0) {
        printf("51 message OK (type=%d, battery=%d%%)\n", g_ctx.device_type, g_ctx.battery);
    } else {
        printf("WARN: no 51 message received.\n");
    }

    // -- 8. Exercise the 70 shoulder light command --------------------------
    if (!write_cmd(dglab::cmd70_solid(dglab::kColorGreen), "70 write (solid green): ")) {
        simpleble_peripheral_disconnect(g_ctx.peripheral);
        return 1;
    }
    msleep(2000);

    if (!write_cmd(dglab::cmd70_blink(dglab::kColorBlue, dglab::kColorWhite, dglab::kBlinkSlow),
                   "70 write (blue/white blink): ")) {
        simpleble_peripheral_disconnect(g_ctx.peripheral);
        return 1;
    }
    msleep(2000);

    if (!write_cmd(dglab::cmd70_blink(dglab::kColorBlue, dglab::kColorWhite, dglab::kBlinkStop),
                   "70 write (blink stop): ")) {
        simpleble_peripheral_disconnect(g_ctx.peripheral);
        return 1;
    }

    // -- 9. Reset the trigger parameter (5F) --------------------------------
    if (!write_cmd({dglab::kCmd5F}, "5F write (param reset): ")) {
        simpleble_peripheral_disconnect(g_ctx.peripheral);
        return 1;
    }
    printf("5F write OK.\n");

    // -- 10. D0 physical data mode ------------------------------------------
    // Streams color + button state + acceleration + XYZ angles + external
    // voltage every 100 ms. Press the button during this phase.
    dglab::Cmd50 cmd50_d0;
    cmd50_d0.indicator_color = dglab::kColorOff;
    cmd50_d0.mode = dglab::kModeD0;
    if (!write_cmd(cmd50_d0.encode(), "50 write (D0 mode): ")) {
        simpleble_peripheral_disconnect(g_ctx.peripheral);
        return 1;
    }

    const int d0_duration_ms = 10000;
    printf("Streaming D0 telemetry for %d s - press the button now...\n", d0_duration_ms / 1000);
    for (int i = 0; i < d0_duration_ms / 100; i++) {
        msleep(100);
    }

    // -- 11. Restore no-trigger mode and disconnect -------------------------
    if (!write_cmd(cmd50_none.encode(), "50 write (restore no trigger): ")) {
        // Non-fatal: the device will be powered down or reconfigured anyway.
    }

    simpleble_peripheral_unsubscribe(g_ctx.peripheral, svc_main, char_notify);
    simpleble_peripheral_disconnect(g_ctx.peripheral);

    // -- 12. Results ---------------------------------------------------------
    printf("\n--- Test summary ---\n");
    printf("51 messages: %d (type=%d, battery=%d%%)\n", g_ctx.msg51_count, g_ctx.device_type,
           g_ctx.battery);
    printf("5A trigger events: %d\n", g_ctx.msg5a_count);
    printf("5B trigger cancels: %d\n", g_ctx.msg5b_count);
    printf("5C param changes: %d\n", g_ctx.msg5c_count);
    printf("D0 samples: %d (pressed in %d samples)\n", g_ctx.msgd0_count, g_ctx.pressed_ticks);
    printf("F1 auto-detect results: %d\n", g_ctx.msgf1_count);

    bool pass = have_write && have_notify && g_ctx.msg51_count > 0 && g_ctx.msgd0_count > 0;
    printf("Result: %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

}  // namespace

int main() { return run(); }
