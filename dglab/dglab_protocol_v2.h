// dglab_protocol_v2.h
//
// DG-LAB Coyote V2 protocol constants and helpers.
// V2 uses a different GATT layout from V3: the main service is 955a180b-...
// and power / pattern A / pattern B are separate characteristics on that
// service. Waveform patterns are expressed as a single CoyotePattern per
// tick (amplitude + pulse_length + pause_length) rather than four per-slot
// (freq, intensity) pairs used by V3.
//
// Both the CLI test app (dglab_coyote2_audio) and the future GUI session
// (CoyoteV2Session) share this header.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace dglab {

// ---------------------------------------------------------------------------
// V2 GATT layout (955a... base UUID)
// ---------------------------------------------------------------------------
constexpr const char* kServiceMainV2 = "955a180b-0fe2-f5aa-a094-84b8d4f3e8ad";
constexpr const char* kCharPowerV2   = "955a1504-0fe2-f5aa-a094-84b8d4f3e8ad";
constexpr const char* kCharPatternA  = "955a1506-0fe2-f5aa-a094-84b8d4f3e8ad";
constexpr const char* kCharPatternB  = "955a1505-0fe2-f5aa-a094-84b8d4f3e8ad";

constexpr const char* kServiceBatteryV2 = "955a180a-0fe2-f5aa-a094-84b8d4f3e8ad";
constexpr const char* kCharBatteryV2    = "955a1500-0fe2-f5aa-a094-84b8d4f3e8ad";

// Maximum power value on wire (0..2000)
constexpr uint16_t kPowerMaxV2 = 2000;

// Device advertisement names (shared with V3's kNamePrefix for scan).
// V2 devices may also carry manufacturer ID 0x1996 (DG-LAB).
extern const char* kNamePrefixV2;
extern uint16_t kMfgIdV2;

// ---------------------------------------------------------------------------
// V2 waveform pattern. Maps one 100 ms tick to a single pulse-shape param.
//   amplitude   0..31   (on-wire 5-bit)
//   pulse_length 0..31  (on-wire 5-bit, ms-equivalent)
//   pause_length  0..1023 (on-wire 10-bit, ms-equivalent)
// ---------------------------------------------------------------------------
struct CoyotePatternV2 {
    uint8_t amplitude = 0;       // 0..31
    uint16_t pause_length = 0;   // 0..1023
    uint8_t pulse_length = 0;    // 0..31
};

// Parse a mode argument: accepts the mode name or its numeric id.
int parse_mode_arg_v2(const char* arg);

// ---------------------------------------------------------------------------
// Waveform generators (return CoyotePatternV2; amplitude is 0..100 internal,
// scaled to 0..31 by the session before encode).
// ---------------------------------------------------------------------------
inline CoyotePatternV2 coyote_mode_breath_v2(uint32_t& waveclock, uint32_t&) {
    CoyotePatternV2 out;
    if (waveclock < 8 * 4) {
        out.pulse_length = 1;
        out.pause_length = 9;
        out.amplitude = static_cast<uint8_t>(std::min<uint32_t>(100, waveclock * 4));
    }
    waveclock += 4;
    if (waveclock > (7 + 3) * 4) {
        waveclock = 0;
    }
    return out;
}

inline CoyotePatternV2 coyote_mode_waves_v2(uint32_t& waveclock, uint32_t& cyclecount) {
    CoyotePatternV2 out;
    constexpr uint16_t rampUpTime = 30 * 4;
    constexpr uint16_t rampDownTime = 50 * 4;
    constexpr uint16_t cycleTime = rampUpTime + rampDownTime;
    constexpr uint16_t maxAmp = 100;
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif
    constexpr double piOverTwo = M_PI_2;

    out.pulse_length = 10;

    if (waveclock <= rampUpTime) {
        double index = static_cast<double>(waveclock) / static_cast<double>(rampUpTime);
        out.amplitude = static_cast<uint8_t>(
            std::floor(std::sin(piOverTwo * index) * static_cast<double>(maxAmp)));
    } else {
        double index = static_cast<double>(waveclock - rampUpTime) /
                       static_cast<double>(rampDownTime);
        out.amplitude = static_cast<uint8_t>(
            std::floor(std::sin(piOverTwo * index + piOverTwo) *
                       static_cast<double>(maxAmp)));
    }

    out.pause_length = 10 * ((cyclecount % 8) + 2);
    waveclock += 4;
    if (waveclock > cycleTime) {
        waveclock = 0;
        cyclecount++;
    }
    return out;
}

inline CoyotePatternV2 coyote_mode_strobe_v2(uint32_t& waveclock, uint32_t& cyclecount) {
    CoyotePatternV2 out;
    constexpr uint16_t strobeTicks = 6 * 4;

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

inline CoyotePatternV2 coyote_mode_pulse_v2(uint32_t& waveclock, uint32_t&) {
    CoyotePatternV2 out;
    out.amplitude = 100;
    out.pulse_length = 1;
    if (waveclock < 2 * 4) {
        out.pause_length = 4;
    } else if (waveclock < 4 * 4) {
        out.pause_length = 30;
    }
    waveclock++;
    if (waveclock > 4 * 4) {
        waveclock = 0;
    }
    return out;
}

inline CoyotePatternV2 coyote_mode_rainbow_v2(uint32_t& waveclock, uint32_t& cyclecount) {
    CoyotePatternV2 out;
    constexpr uint16_t rainbowTicks = 8 * 4;
    constexpr uint8_t maxAmp = 100;

    double frac = static_cast<double>(cyclecount % rainbowTicks) /
                  static_cast<double>(rainbowTicks);
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

inline CoyotePatternV2 coyote_mode_bass_v2(uint32_t& waveclock, uint32_t& cyclecount) {
    CoyotePatternV2 out;
    if (cyclecount % 8 < 3) {
        out.amplitude = 0;
        out.pulse_length = 0;
        out.pause_length = 0;
    } else {
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

// ---------------------------------------------------------------------------
// Packets
// ---------------------------------------------------------------------------

// Encode 11-bit channel A and 11-bit channel B power levels into 3 bytes.
// Layout: flipFirstAndThirdByte(zero(2) ~ uint(11).as("powerB") ~ uint(11).as("powerA"))
std::vector<uint8_t> encode_power_v2(uint16_t power_a, uint16_t power_b);

// Parse a 3-byte power notification payload into channel A and channel B.
void parse_power_v2(const uint8_t* data, size_t len, uint16_t& power_a, uint16_t& power_b);

// Encode Coyote V2 pattern into 3 bytes.
// Layout: flipFirstAndThirdByte(zero(4) ~ uint(5).as("az") ~ uint(10).as("ay") ~ uint(5).as("ax"))
std::vector<uint8_t> encode_pattern_v2(const CoyotePatternV2& p);

}  // namespace dglab
