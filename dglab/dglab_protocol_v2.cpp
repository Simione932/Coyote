// dglab_protocol_v2.cpp
//
// Implementation of the Coyote V2 protocol helpers declared in
// dglab_protocol_v2.h. These are shared between the legacy test app
// (dglab_coyote2_audio) and the future compatible session backend.

#include "dglab_protocol.h"
#include "dglab_protocol_v2.h"

#include <algorithm>
#include <cstdlib>

namespace dglab {

const char* kNamePrefixV2 = "47L";
uint16_t kMfgIdV2 = 0x1996;

int parse_mode_arg_v2(const char* arg) {
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
    return std::atoi(arg);
}

std::vector<uint8_t> encode_power_v2(uint16_t power_a, uint16_t power_b) {
    power_a = std::min<uint16_t>(power_a, kPowerMaxV2);
    power_b = std::min<uint16_t>(power_b, kPowerMaxV2);

    std::vector<uint8_t> buf(3, 0);
    buf[2] = static_cast<uint8_t>((power_a & 0x7E0) >> 5);
    buf[1] = static_cast<uint8_t>(((power_a & 0x1F) << 3) | ((power_b & 0x700) >> 8));
    buf[0] = static_cast<uint8_t>(power_b & 0xFF);
    return buf;
}

void parse_power_v2(const uint8_t* data, size_t len, uint16_t& power_a, uint16_t& power_b) {
    if (data == nullptr || len < 3) return;
    power_a = static_cast<uint16_t>((data[2] * 256 + data[1]) >> 3);
    power_b = static_cast<uint16_t>(((data[1] * 256) + data[0]) & 0x7FF);
}

std::vector<uint8_t> encode_pattern_v2(const CoyotePatternV2& p) {
    std::vector<uint8_t> buf(3, 0);
    uint8_t az = p.amplitude & 0x1F;
    uint16_t ay = p.pause_length & 0x3FF;
    uint8_t ax = p.pulse_length & 0x1F;

    buf[2] = static_cast<uint8_t>((az >> 1) & 0x0F);
    buf[1] = static_cast<uint8_t>(((az & 0x01) << 7) | ((ay >> 3) & 0x7F));
    buf[0] = static_cast<uint8_t>((ax & 0x1F) | ((ay & 0x07) << 5));
    return buf;
}

}  // namespace dglab
