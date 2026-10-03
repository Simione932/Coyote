// dglab_coyote2_audio.cpp
//
// C++ test application that exercises the SimpleBLE C API (simplecble)
// against a DG-LAB Coyote 2 (郊狼 2.0 / ESTIM01) pulse host running the V2
// Bluetooth protocol (consistent with https://github.com/OpenDGLab/OpenDGLab-Connect/blob/master/src/services/DGLab.js),
// with waveform patterns and power driven by the audio input stream.
//
// What it does:
//   1. Opens the audio input device in stereo ('BlackHole 2ch' on macOS,
//      or WASAPI loopback capture on Windows).
//   2. Every 100 ms it computes the frequency spectrum (Hann window +
//      8192-point FFT) of the left and right channels independently and
//      finds the dominant audio frequency in each (20 Hz .. 10 kHz default,
//      upper limit settable via [freq_max_khz]).
//   3. Scans for DG-LAB Coyote V2 devices (name starting with or containing
//      "ESTIM01" or manufacturer ID 0x1996), connects and verifies GATT:
//        service 0x955a180b-...
//          power char   0x955a1504-... (WRITE / NOTIFY)
//          pattern A    0x955a1506-... (WRITE)
//          pattern B    0x955a1505-... (WRITE)
//        service 0x955a180a-...
//          battery char 0x955a1500-... (READ / NOTIFY)
//   4. Subscribes to power notifications on 0x955a1504.
//   5. Reads the battery level from 0x955a180a/0x955a1500.
//   6. Streams Coyote V2 power (3-byte bitfield) and pattern (3-byte bitfield
//      per channel) commands every 100 ms:
//        - Left channel drives Channel A power and pulse timing.
//        - Right channel drives Channel B power and pulse timing.
//   7. Runs continuously until the 'q' key is pressed, then disconnects
//      and releases all handles.
//
// Usage: dglab_coyote2_audio [a_gain] [b_gain] [freq_max_khz] [a_mode] [b_mode]
//   a_gain      channel A (left)  initial intensity scale, 0..100 (default: 20)
//   b_gain      channel B (right) initial intensity scale, 0..100 (default: 20)
//   freq_max_khz highest audio frequency considered, kHz (default: 10)
//   a_mode      channel A mode: default (0)=waves, breath (1), waves (2), strobe (3), pulse (4), rainbow (5), or bass (6)
//   b_mode      channel B mode: default (0)=waves, breath (1), waves (2), strobe (3), pulse (4), rainbow (5), or bass (6)
//
// Keys (while running):
//   q   quit
//   h   show help
//   r/t channel A gain up/down (steps of 5)
//   o/p channel B gain up/down (steps of 5)
//
// NOTE: The DG-LAB open protocol is licensed for personal/hobby use only.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#include <conio.h>
#include <mmdeviceapi.h>
#include <audioclient.h>
#elif defined(__APPLE__)
#include <CoreFoundation/CoreFoundation.h>
#include <AudioToolbox/AudioToolbox.h>
#include <termios.h>
#include <mutex>
#else
#include <termios.h>
#include <sys/select.h>
#include <unistd.h>
#endif

#include <simplecble/simplecble.h>

// ---------------------------------------------------------------------------
// DG-LAB Coyote V2 protocol constants (955a... base UUIDs)
// ---------------------------------------------------------------------------

namespace dglab2 {

// Base UUID: 955aXXXX-0fe2-f5aa-a094-84b8d4f3e8ad
constexpr const char* kServiceMain = "955a180b-0fe2-f5aa-a094-84b8d4f3e8ad";
constexpr const char* kCharPower = "955a1504-0fe2-f5aa-a094-84b8d4f3e8ad";
constexpr const char* kCharPatternA = "955a1506-0fe2-f5aa-a094-84b8d4f3e8ad";
constexpr const char* kCharPatternB = "955a1505-0fe2-f5aa-a094-84b8d4f3e8ad";

constexpr const char* kServiceBattery = "955a180a-0fe2-f5aa-a094-84b8d4f3e8ad";
constexpr const char* kCharBattery = "955a1500-0fe2-f5aa-a094-84b8d4f3e8ad";

// Maximum power value on wire (0..2000)
constexpr uint16_t kPowerMax = 2000;

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

// Waveform pattern parameters for Coyote 2
struct CoyotePattern {
    uint8_t amplitude = 0;      // 0..31 (amplitude intensity)
    uint16_t pause_length = 0;  // 0..1023 ms (pause length between pulses)
    uint8_t pulse_length = 0;   // 0..31 ms (pulse width)
};

// Waveform generator modes (consistent with coyote-modes.h)
enum WaveformMode {
    MODE_DEFAULT = 0,
    MODE_BREATH = 1,
    MODE_WAVES = 2,
    MODE_STROBE = 3,
    MODE_PULSE = 4,
    MODE_RAINBOW = 5,
    MODE_BASS = 6,
};

// Breath waveform mode generator (1100 ms cycle)
inline CoyotePattern coyote_mode_breath(uint32_t& waveclock, uint32_t&) {
    CoyotePattern out;
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

// Waves waveform mode generator (8000 ms cycle)
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

    waveclock += 4;
    if (waveclock > cycleTime) {
        waveclock = 0;
        cyclecount++;
    }
    return out;
}

// Strobe waveform mode generator (240 ms cycle, 6 flicker steps).
// Holds full amplitude with a single sharp pulse and flickers the off-time
// (pause_length) through a fast ramp, so the strobe rate can be tuned by the
// caller via the audio-driven frequency. Only the on/off timing varies here,
// unlike breath/waves which modulate amplitude.
inline CoyotePattern coyote_mode_strobe(uint32_t& waveclock, uint32_t& cyclecount) {
    CoyotePattern out;
    constexpr uint16_t strobeTicks = 6 * 4;  // 6 flicker steps

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

// Rainbow / color-chase waveform mode generator (8 sweep steps).
// Sweeps the output frequency up and back down through a spectrum while the
// amplitude rises to full at the sweep peak and then fades. This makes the
// *rate* the primary animated axis, unlike breath/waves which modulate
// amplitude.
inline CoyotePattern coyote_mode_rainbow(uint32_t& waveclock, uint32_t& cyclecount) {
    CoyotePattern out;
    constexpr uint16_t rainbowTicks = 8 * 4;  // 8 sweep steps
    constexpr uint8_t maxAmp = 100;

    double frac = static_cast<double>(cyclecount % rainbowTicks) / static_cast<double>(rainbowTicks);
    double amp = frac < 0.5 ? (frac / 0.5) : (1.0 - frac / 0.5);
    out.amplitude = static_cast<uint8_t>(std::lround(amp * maxAmp));
    out.pulse_length = 10;
    out.pause_length = 10;  // overridden by the audio-blended timing in the caller

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

// Encodes 11-bit channel A and 11-bit channel B power levels into 3 bytes.
// Layout: flipFirstAndThirdByte(zero(2) ~ uint(11).as("powerB") ~ uint(11).as("powerA"))
inline std::vector<uint8_t> encode_power(uint16_t power_a, uint16_t power_b) {
    power_a = std::min<uint16_t>(power_a, kPowerMax);
    power_b = std::min<uint16_t>(power_b, kPowerMax);

    std::vector<uint8_t> buf(3, 0);
    buf[2] = static_cast<uint8_t>((power_a & 0x7E0) >> 5);
    buf[1] = static_cast<uint8_t>(((power_a & 0x1F) << 3) | ((power_b & 0x700) >> 8));
    buf[0] = static_cast<uint8_t>(power_b & 0xFF);
    return buf;
}

// Parses a 3-byte power notification payload into channel A and channel B power levels.
inline void parse_power(const uint8_t* data, size_t len, uint16_t& power_a, uint16_t& power_b) {
    if (data != nullptr && len >= 3) {
        power_a = static_cast<uint16_t>((data[2] * 256 + data[1]) >> 3);
        power_b = static_cast<uint16_t>(((data[1] * 256) + data[0]) & 0x7FF);
    }
}

// Encodes Coyote 2 pattern into 3 bytes.
// Layout: flipFirstAndThirdByte(zero(4) ~ uint(5).as("az") ~ uint(10).as("ay") ~ uint(5).as("ax"))
//   az = amplitude (0..31)
//   ay = pause_length (0..1023)
//   ax = pulse_length (0..31)
inline std::vector<uint8_t> encode_pattern(const CoyotePattern& p) {
    std::vector<uint8_t> buf(3, 0);
    uint8_t az = p.amplitude & 0x1F;
    uint16_t ay = p.pause_length & 0x3FF;
    uint8_t ax = p.pulse_length & 0x1F;

    buf[2] = static_cast<uint8_t>((az >> 1) & 0x0F);
    buf[1] = static_cast<uint8_t>(((az & 0x01) << 7) | ((ay >> 3) & 0x7F));
    buf[0] = static_cast<uint8_t>((ax & 0x1F) | ((ay & 0x07) << 5));
    return buf;
}

}  // namespace dglab2

// ---------------------------------------------------------------------------
// Audio analysis (WASAPI loopback + FFT)
// ---------------------------------------------------------------------------

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kAnalysisWindowS = 0.1;      // 100 ms audio window
constexpr size_t kFftSize = 8192;             // FFT size
constexpr double kAudioFreqMinHz = 20.0;      // lowest audio frequency considered
double g_audio_freq_max_hz = 10000.0;
constexpr double kSilenceThreshold = 5.0;

void fft_inplace(std::vector<std::complex<double>>& a) {
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; i++) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            std::swap(a[i], a[j]);
        }
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        const double ang = -2.0 * kPi / static_cast<double>(len);
        const std::complex<double> wlen(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> w(1.0, 0.0);
            for (size_t k = 0; k < len / 2; k++) {
                const std::complex<double> u = a[i + k];
                const std::complex<double> v = a[i + k + len / 2] * w;
                a[i + k] = u + v;
                a[i + k + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
}

struct ToneInfo {
    double hz = 0.0;
    double level = 0.0;
};

ToneInfo dominant_tone(const std::vector<float>& samples, double sample_rate) {
    ToneInfo result;
    if (samples.size() < 256) {
        return result;
    }

    const size_t n = kFftSize;
    std::vector<std::complex<double>> a(n, std::complex<double>(0.0, 0.0));
    const size_t count = samples.size();
    for (size_t i = 0; i < count; i++) {
        const double w = 0.5 * (1.0 - std::cos(2.0 * kPi * static_cast<double>(i) /
                                               static_cast<double>(count - 1)));  // Hann
        a[i] = std::complex<double>(static_cast<double>(samples[i]) * w, 0.0);
    }
    fft_inplace(a);

    size_t lo = static_cast<size_t>(kAudioFreqMinHz * n / sample_rate);
    size_t hi = static_cast<size_t>(g_audio_freq_max_hz * n / sample_rate);
    if (hi >= n / 2) {
        hi = n / 2 - 1;
    }
    if (lo >= hi) {
        return result;
    }

    size_t peak_bin = lo;
    double peak_mag = 0.0;
    for (size_t i = lo; i <= hi; i++) {
        const double mag = std::abs(a[i]);
        if (mag > peak_mag) {
            peak_mag = mag;
            peak_bin = i;
        }
    }

    if (peak_mag < kSilenceThreshold) {
        return result;
    }
    result.hz = static_cast<double>(peak_bin) * sample_rate / static_cast<double>(n);
    result.level = std::clamp(peak_mag / (0.5 * static_cast<double>(count)), 0.0, 1.0);
    return result;
}

double audio_freq_ratio(double hz) {
    if (hz <= 0.0) {
        return 0.0;
    }
    const double f = std::clamp(hz, kAudioFreqMinHz, g_audio_freq_max_hz);
    const double ratio = (std::log10(f) - std::log10(kAudioFreqMinHz)) /
                         (std::log10(g_audio_freq_max_hz) - std::log10(kAudioFreqMinHz));
    return std::clamp(ratio, 0.0, 1.0);
}

struct AudioState {
    std::atomic<bool> running{false};
    std::atomic<bool> capture_ok{false};
    std::atomic<double> left_hz{0.0};
    std::atomic<double> right_hz{0.0};
    std::atomic<double> left_level{0.0};
    std::atomic<double> right_level{0.0};
    std::atomic<int> analyses{0};
};

#ifdef _WIN32

void audio_capture_thread(AudioState* st) {
    CoInitializeEx(nullptr, COINIT_MULTITHREADED);

    IMMDeviceEnumerator* enumerator = nullptr;
    IMMDevice* device = nullptr;
    IAudioClient* client = nullptr;
    IAudioCaptureClient* capture = nullptr;
    WAVEFORMATEX* mix_format = nullptr;

    HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL,
                                  __uuidof(IMMDeviceEnumerator),
                                  reinterpret_cast<void**>(&enumerator));
    if (SUCCEEDED(hr)) {
        hr = enumerator->GetDefaultAudioEndpoint(eRender, eConsole, &device);
    }
    if (SUCCEEDED(hr)) {
        hr = device->Activate(__uuidof(IAudioClient), 0, nullptr,
                              reinterpret_cast<void**>(&client));
    }
    if (SUCCEEDED(hr)) {
        hr = client->GetMixFormat(&mix_format);
    }
    if (SUCCEEDED(hr)) {
        hr = client->Initialize(AUDCLNT_SHAREMODE_SHARED, AUDCLNT_STREAMFLAGS_LOOPBACK, 0, 0,
                                mix_format, nullptr);
    }
    if (SUCCEEDED(hr)) {
        hr = client->GetService(__uuidof(IAudioCaptureClient),
                                reinterpret_cast<void**>(&capture));
    }
    if (SUCCEEDED(hr)) {
        hr = client->Start();
    }

    double sample_rate = 0.0;
    int channels = 0;
    bool is_float = false;
    size_t window_samples = 0;
    std::vector<float> left;
    std::vector<float> right;

    if (FAILED(hr) || mix_format == nullptr) {
        printf("WARN: could not open default audio output device (HRESULT 0x%08lX)\n",
               static_cast<unsigned long>(hr));
        st->capture_ok = false;
        while (st->running) {
            Sleep(100);
        }
        goto cleanup;
    }

    st->capture_ok = true;
    sample_rate = static_cast<double>(mix_format->nSamplesPerSec);
    channels = mix_format->nChannels;
    is_float = (mix_format->wFormatTag == WAVE_FORMAT_IEEE_FLOAT);
    window_samples = static_cast<size_t>(sample_rate * kAnalysisWindowS);
    printf("Audio capture: %s, %u Hz, %d channels\n",
           is_float ? "float32" : "pcm16", mix_format->nSamplesPerSec, channels);

    left.reserve(window_samples * 2);
    right.reserve(window_samples * 2);

    while (st->running) {
        BYTE* data = nullptr;
        UINT32 frames = 0;
        DWORD flags = 0;
        HRESULT ghr = capture->GetBuffer(&data, &frames, &flags, nullptr, nullptr);
        if (SUCCEEDED(ghr) && data != nullptr && frames > 0) {
            const size_t bytes_per_sample =
                (mix_format->wBitsPerSample / 8) * static_cast<size_t>(channels);
            for (UINT32 f = 0; f < frames; f++) {
                const uint8_t* p = data + static_cast<size_t>(f) * bytes_per_sample;
                float l = 0.0f;
                float r = 0.0f;
                if (is_float) {
                    const float* pf = reinterpret_cast<const float*>(p);
                    l = pf[0];
                    r = (channels > 1) ? pf[1] : pf[0];
                } else if (mix_format->wBitsPerSample == 16) {
                    const int16_t* pi = reinterpret_cast<const int16_t*>(p);
                    l = static_cast<float>(pi[0]) / 32768.0f;
                    r = (channels > 1) ? static_cast<float>(pi[1]) / 32768.0f : l;
                }
                left.push_back(l);
                right.push_back(r);
            }
            capture->ReleaseBuffer(frames);
        }

        if (left.size() >= window_samples) {
            const ToneInfo lt = dominant_tone(left, sample_rate);
            const ToneInfo rt = dominant_tone(right, sample_rate);
            st->left_hz = lt.hz;
            st->right_hz = rt.hz;
            st->left_level = lt.level;
            st->right_level = rt.level;
            st->analyses++;
            left.clear();
            right.clear();
        }
        Sleep(10);
    }

    client->Stop();

cleanup:
    if (capture != nullptr) capture->Release();
    if (client != nullptr) client->Release();
    if (device != nullptr) device->Release();
    if (enumerator != nullptr) enumerator->Release();
    if (mix_format != nullptr) CoTaskMemFree(mix_format);
    CoUninitialize();
}

#elif defined(__APPLE__)

struct MacCapture {
    AudioUnit unit = nullptr;
    std::mutex mu;
    std::vector<float> left;
    std::vector<float> right;
    double sample_rate = 44100.0;
};

OSStatus mac_input_callback(void* in_ref, AudioUnitRenderActionFlags* io_flags,
                            const AudioTimeStamp* time_stamp, UInt32,
                            UInt32 num_frames, AudioBufferList*) {
    auto* cap = static_cast<MacCapture*>(in_ref);
    if (num_frames == 0) {
        return noErr;
    }

    std::vector<float> buffer(static_cast<size_t>(num_frames) * 2, 0.0f);
    AudioBufferList buf_list;
    buf_list.mNumberBuffers = 1;
    buf_list.mBuffers[0].mNumberChannels = 2;
    buf_list.mBuffers[0].mDataByteSize = static_cast<UInt32>(buffer.size() * sizeof(float));
    buf_list.mBuffers[0].mData = buffer.data();

    OSStatus os = AudioUnitRender(cap->unit, io_flags, time_stamp, 1, num_frames, &buf_list);
    if (os != noErr) {
        return os;
    }

    std::lock_guard<std::mutex> lock(cap->mu);
    for (UInt32 f = 0; f < num_frames; f++) {
        cap->left.push_back(buffer[static_cast<size_t>(f) * 2 + 0]);
        cap->right.push_back(buffer[static_cast<size_t>(f) * 2 + 1]);
    }
    return noErr;
}

void audio_capture_thread(AudioState* st) {
    MacCapture cap;

    AudioObjectPropertyAddress addr = {
        kAudioHardwarePropertyDevices,
        kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMain
    };
    UInt32 size = 0;
    OSStatus os = AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &addr, 0, nullptr, &size);

    AudioObjectID target_id = kAudioObjectUnknown;
    const std::string target_device_name = "BlackHole 2ch";

    if (os == noErr && size > 0) {
        const size_t device_count = size / sizeof(AudioObjectID);
        std::vector<AudioObjectID> devices(device_count);
        os = AudioObjectGetPropertyData(kAudioObjectSystemObject, &addr, 0, nullptr, &size, devices.data());
        if (os == noErr) {
            for (AudioObjectID id : devices) {
                CFStringRef cf_name = nullptr;
                UInt32 name_size = sizeof(cf_name);
                AudioObjectPropertyAddress name_addr = {
                    kAudioObjectPropertyName,
                    kAudioObjectPropertyScopeGlobal,
                    kAudioObjectPropertyElementMain
                };
                if (AudioObjectGetPropertyData(id, &name_addr, 0, nullptr, &name_size, &cf_name) == noErr && cf_name != nullptr) {
                    char buf[256] = {0};
                    if (CFStringGetCString(cf_name, buf, sizeof(buf), kCFStringEncodingUTF8)) {
                        if (std::string(buf).find(target_device_name) != std::string::npos) {
                            target_id = id;
                        }
                    }
                    CFRelease(cf_name);
                }
                if (target_id != kAudioObjectUnknown) {
                    break;
                }
            }
        }
    }

    if (target_id == kAudioObjectUnknown) {
        printf("WARN: could not find audio input device '%s'\n", target_device_name.c_str());
        st->capture_ok = false;
        while (st->running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return;
    }

    Float64 sample_rate = 44100.0;
    size = sizeof(sample_rate);
    AudioObjectPropertyAddress sr_addr = {
        kAudioDevicePropertyNominalSampleRate,
        kAudioObjectPropertyScopeInput,
        kAudioObjectPropertyElementMain
    };
    if (AudioObjectGetPropertyData(target_id, &sr_addr, 0, nullptr, &size, &sample_rate) != noErr) {
        sr_addr.mScope = kAudioObjectPropertyScopeGlobal;
        AudioObjectGetPropertyData(target_id, &sr_addr, 0, nullptr, &size, &sample_rate);
    }
    cap.sample_rate = sample_rate;

    AudioComponentDescription cd = {};
    cd.componentType = kAudioUnitType_Output;
    cd.componentSubType = kAudioUnitSubType_HALOutput;
    cd.componentManufacturer = kAudioUnitManufacturer_Apple;
    AudioComponent comp = AudioComponentFindNext(nullptr, &cd);
    if (comp != nullptr) {
        os = AudioComponentInstanceNew(comp, &cap.unit);
    } else {
        os = -10868;
    }

    if (os == noErr) {
        UInt32 enable_input = 1;
        os = AudioUnitSetProperty(cap.unit, kAudioOutputUnitProperty_EnableIO,
                                  kAudioUnitScope_Input, 1, &enable_input, sizeof(enable_input));
    }
    if (os == noErr) {
        UInt32 disable_output = 0;
        os = AudioUnitSetProperty(cap.unit, kAudioOutputUnitProperty_EnableIO,
                                  kAudioUnitScope_Output, 0, &disable_output, sizeof(disable_output));
    }
    if (os == noErr) {
        os = AudioUnitSetProperty(cap.unit, kAudioOutputUnitProperty_CurrentDevice,
                                  kAudioUnitScope_Global, 0, &target_id, sizeof(target_id));
    }
    if (os == noErr) {
        AudioStreamBasicDescription fmt = {};
        fmt.mSampleRate = sample_rate;
        fmt.mFormatID = kAudioFormatLinearPCM;
        fmt.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
        fmt.mBytesPerPacket = sizeof(float) * 2;
        fmt.mFramesPerPacket = 1;
        fmt.mBytesPerFrame = sizeof(float) * 2;
        fmt.mChannelsPerFrame = 2;
        fmt.mBitsPerChannel = 32;
        os = AudioUnitSetProperty(cap.unit, kAudioUnitProperty_StreamFormat,
                                  kAudioUnitScope_Output, 1, &fmt, sizeof(fmt));
    }
    if (os == noErr) {
        AURenderCallbackStruct cb = {mac_input_callback, &cap};
        os = AudioUnitSetProperty(cap.unit, kAudioOutputUnitProperty_SetInputCallback,
                                  kAudioUnitScope_Global, 0, &cb, sizeof(cb));
    }
    if (os == noErr) {
        os = AudioUnitInitialize(cap.unit);
    }
    if (os == noErr) {
        os = AudioOutputUnitStart(cap.unit);
    }

    if (os != noErr) {
        printf("WARN: could not open audio device '%s' (OSStatus %d)\n", target_device_name.c_str(), static_cast<int>(os));
        st->capture_ok = false;
        while (st->running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (cap.unit != nullptr) {
            AudioUnitUninitialize(cap.unit);
            AudioComponentInstanceDispose(cap.unit);
        }
        return;
    }

    st->capture_ok = true;
    const size_t window_samples = static_cast<size_t>(cap.sample_rate * kAnalysisWindowS);
    printf("Audio capture: float32, %.0f Hz, '%s' input\n",
           cap.sample_rate, target_device_name.c_str());

    std::vector<float> left;
    std::vector<float> right;
    left.reserve(window_samples * 2);
    right.reserve(window_samples * 2);

    while (st->running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::vector<float> lchunk;
        std::vector<float> rchunk;
        {
            std::lock_guard<std::mutex> lock(cap.mu);
            lchunk.swap(cap.left);
            rchunk.swap(cap.right);
        }
        left.insert(left.end(), lchunk.begin(), lchunk.end());
        right.insert(right.end(), rchunk.begin(), rchunk.end());

        if (left.size() >= window_samples) {
            const ToneInfo lt = dominant_tone(left, cap.sample_rate);
            const ToneInfo rt = dominant_tone(right, cap.sample_rate);
            st->left_hz = lt.hz;
            st->right_hz = rt.hz;
            st->left_level = lt.level;
            st->right_level = rt.level;
            st->analyses++;
            left.clear();
            right.clear();
        }
    }

    AudioOutputUnitStop(cap.unit);
    AudioUnitUninitialize(cap.unit);
    AudioComponentInstanceDispose(cap.unit);
}

#else

void audio_capture_thread(AudioState* st) {
    printf("WARN: audio capture not supported on this platform.\n");
    st->capture_ok = false;
    while (st->running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

#endif  // _WIN32 / __APPLE__

}  // namespace

// ---------------------------------------------------------------------------
// Test application
// ---------------------------------------------------------------------------

namespace {

#ifndef _WIN32
struct TerminalRaw {
    bool changed = false;
    termios orig = {};
    TerminalRaw() {
        if (tcgetattr(STDIN_FILENO, &orig) == 0) {
            changed = true;
            termios raw = orig;
            raw.c_lflag &= ~(tcflag_t)(ICANON | ECHO);
            raw.c_iflag |= (tcflag_t)(ICRNL | INLCR);
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        }
    }
    ~TerminalRaw() {
        if (changed) {
            tcsetattr(STDIN_FILENO, TCSANOW, &orig);
        }
    }
};
#endif

struct Context {
    simpleble_adapter_t adapter = nullptr;
    simpleble_peripheral_t peripheral = nullptr;
    std::vector<simpleble_peripheral_t> found;
    uint16_t current_power_a = 0;
    uint16_t current_power_b = 0;
    int battery = -1;
};

Context g_ctx;
AudioState g_audio;

void msleep(int ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    usleep(ms * 1000);
#endif
}

bool process_keys(int* gain_a, int* gain_b) {
    const int step = 5;

    auto get_key = []() -> char {
#ifdef _WIN32
        if (_kbhit()) {
            return static_cast<char>(_getch());
        }
#else
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        timeval tv = {0, 0};
        if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
            char c = 0;
            if (read(STDIN_FILENO, &c, 1) == 1) {
                return c;
            }
        }
#endif
        return 0;
    };

    for (;;) {
        const char c = get_key();
        if (c == 0) {
            return false;
        }
        switch (c) {
            case 'q':
            case 'Q':
                return true;
            case 'h':
            case 'H':
                printf("Keys: q quit | h help | r/t channel A gain up/down | "
                       "o/p channel B gain up/down (steps of 5, 0..100)\n");
                printf("Modes A/B: default(0)=waves breath(1) waves(2) strobe(3) pulse(4) "
                       "rainbow(5) bass(6)\n");
                printf("Current gains: A=%d B=%d\n", *gain_a, *gain_b);
                break;
            case 'r':
            case 'R':
                *gain_a = std::clamp(*gain_a + step, 0, 100);
                printf("Channel A gain: %d\n", *gain_a);
                break;
            case 't':
            case 'T':
                *gain_a = std::clamp(*gain_a - step, 0, 100);
                printf("Channel A gain: %d\n", *gain_a);
                break;
            case 'o':
            case 'O':
                *gain_b = std::clamp(*gain_b + step, 0, 100);
                printf("Channel B gain: %d\n", *gain_b);
                break;
            case 'p':
            case 'P':
                *gain_b = std::clamp(*gain_b - step, 0, 100);
                printf("Channel B gain: %d\n", *gain_b);
                break;
            default:
                break;
        }
    }
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
        if (a >= 'A' && a <= 'Z') a = static_cast<char>(a - 'A' + 'a');
        if (b >= 'A' && b <= 'Z') b = static_cast<char>(b - 'A' + 'a');
        if (a != b) return false;
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

    bool matches_coyote2 = false;
    if (name != nullptr && (std::strstr(name, "ESTIM01") != nullptr || std::strstr(name, "D-LAB") != nullptr)) {
        matches_coyote2 = true;
    }

    // Also check manufacturer data for DG-LAB manufacturer ID 0x1996 (0x96, 0x19)
    if (!matches_coyote2) {
        size_t mfg_count = simpleble_peripheral_manufacturer_data_count(peripheral);
        for (size_t i = 0; i < mfg_count; i++) {
            simpleble_manufacturer_data_t mfg;
            if (simpleble_peripheral_manufacturer_data_get(peripheral, i, &mfg) == SIMPLEBLE_SUCCESS) {
                if (mfg.manufacturer_id == 0x1996 ||
                    (mfg.data_length >= 2 && mfg.data[0] == 0x96 && mfg.data[1] == 0x19)) {
                    matches_coyote2 = true;
                    break;
                }
            }
        }
    }

    if (matches_coyote2) {
        printf("  [DG-LAB Coyote V2] %s [%s] rssi=%d\n", name ? name : "?", address ? address : "?", rssi);
        g_ctx.found.push_back(peripheral);
    } else {
        simpleble_peripheral_release_handle(peripheral);
    }

    simpleble_free(name);
    simpleble_free(address);
}

void on_power_notify(simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t, const uint8_t* data,
                        size_t data_length, void*) {
    if (data == nullptr || data_length < 3) return;
    dglab2::parse_power(data, data_length, g_ctx.current_power_a, g_ctx.current_power_b);
    printf("  [Power notify] A=%u B=%u\n", g_ctx.current_power_a, g_ctx.current_power_b);
}

int run(int gain_a, int gain_b, int a_mode, int b_mode) {
    atexit([]() {
        g_audio.running = false;
        for (auto p : g_ctx.found) {
            if (p == g_ctx.peripheral) g_ctx.peripheral = nullptr;
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

    // -- 1. Start Audio Capture Thread ---------------------------------------
    g_audio.running = true;
    std::thread audio_thread(audio_capture_thread, &g_audio);

    // -- 2. Adapter ---------------------------------------------------------
    if (!simpleble_adapter_is_bluetooth_enabled()) {
        printf("Bluetooth is not enabled.\n");
        g_audio.running = false;
        if (audio_thread.joinable()) audio_thread.join();
        return 1;
    }

    size_t adapter_count = simpleble_adapter_get_count();
    if (adapter_count == 0) {
        printf("No BLE adapter found.\n");
        g_audio.running = false;
        if (audio_thread.joinable()) audio_thread.join();
        return 1;
    }

    g_ctx.adapter = simpleble_adapter_get_handle(0);
    if (g_ctx.adapter == nullptr) {
        printf("Failed to get adapter handle.\n");
        g_audio.running = false;
        if (audio_thread.joinable()) audio_thread.join();
        return 1;
    }

    char* adapter_id = simpleble_adapter_identifier(g_ctx.adapter);
    printf("Using adapter: %s (SimpleBLE %s)\n", adapter_id ? adapter_id : "?",
           simpleble_get_version());
    simpleble_free(adapter_id);

    // -- 3. Scan ------------------------------------------------------------
    printf("Scanning for DG-LAB Coyote V2 devices (10 s)...\n");
    simpleble_adapter_set_callback_on_scan_found(g_ctx.adapter, on_scan_found, nullptr);
    simpleble_adapter_scan_for(g_ctx.adapter, 10000);

    if (g_ctx.found.empty()) {
        printf("No DG-LAB Coyote V2 device found (looking for 'ESTIM01' / 0x1996).\n");
        g_audio.running = false;
        if (audio_thread.joinable()) audio_thread.join();
        return 1;
    }

    g_ctx.peripheral = g_ctx.found[0];
    char* name = simpleble_peripheral_identifier(g_ctx.peripheral);
    char* address = simpleble_peripheral_address(g_ctx.peripheral);
    printf("Connecting to %s [%s]...\n", name ? name : "?", address ? address : "?");
    simpleble_free(name);
    simpleble_free(address);

    // -- 4. Connect ---------------------------------------------------------
    if (simpleble_peripheral_connect(g_ctx.peripheral) != SIMPLEBLE_SUCCESS) {
        printf("Failed to connect.\n");
        g_audio.running = false;
        if (audio_thread.joinable()) audio_thread.join();
        return 1;
    }

    bool connected = false;
    simpleble_peripheral_is_connected(g_ctx.peripheral, &connected);
    printf("Connected: %s\n", connected ? "yes" : "no");
    if (!connected) {
        g_audio.running = false;
        if (audio_thread.joinable()) audio_thread.join();
        return 1;
    }

    // -- 5. Verify GATT layout ---------------------------------------------
    simpleble_uuid_t svc_main = make_uuid(dglab2::kServiceMain);
    simpleble_uuid_t svc_battery = make_uuid(dglab2::kServiceBattery);

    bool have_power = false;
    bool have_pattern_a = false;
    bool have_pattern_b = false;
    bool have_battery = false;

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

            if (uuid_is(&c.uuid, dglab2::kCharPower)) {
                have_power = true;
            } else if (uuid_is(&c.uuid, dglab2::kCharPatternA)) {
                have_pattern_a = true;
            } else if (uuid_is(&c.uuid, dglab2::kCharPatternB)) {
                have_pattern_b = true;
            } else if (uuid_is(&c.uuid, dglab2::kCharBattery)) {
                have_battery = true;
            }
        }
    }

    if (!have_power || !have_pattern_a || !have_pattern_b) {
        printf("FAIL: expected Coyote V2 GATT layout (0x955a180b service) not found.\n");
        simpleble_peripheral_disconnect(g_ctx.peripheral);
        g_audio.running = false;
        if (audio_thread.joinable()) audio_thread.join();
        return 1;
    }
    printf("GATT layout OK (power=%s, patternA=%s, patternB=%s, battery=%s)\n",
           have_power ? "yes" : "no", have_pattern_a ? "yes" : "no",
           have_pattern_b ? "yes" : "no", have_battery ? "yes" : "no");

    // -- 6. Subscribe to power notifications --------------------------------
    simpleble_uuid_t char_power = make_uuid(dglab2::kCharPower);
    simpleble_uuid_t char_pattern_a = make_uuid(dglab2::kCharPatternA);
    simpleble_uuid_t char_pattern_b = make_uuid(dglab2::kCharPatternB);
    simpleble_uuid_t char_battery = make_uuid(dglab2::kCharBattery);

    if (simpleble_peripheral_notify(g_ctx.peripheral, svc_main, char_power, on_power_notify, nullptr) != SIMPLEBLE_SUCCESS) {
        printf("WARN: failed to subscribe to power notifications.\n");
    } else {
        printf("Subscribed to power notifications.\n");
    }

    // -- 7. Read battery ----------------------------------------------------
    if (have_battery) {
        uint8_t* data = nullptr;
        size_t data_length = 0;
        if (simpleble_peripheral_read(g_ctx.peripheral, svc_battery, char_battery, &data, &data_length) == SIMPLEBLE_SUCCESS &&
            data != nullptr && data_length >= 1) {
            g_ctx.battery = data[0];
            printf("Battery level: %d%%\n", data[0]);
            simpleble_free(data);
        } else {
            printf("WARN: battery read failed.\n");
        }
    }

    // -- 8. Stream Audio-Driven Coyote 2 Patterns ---------------------------
    printf("\nStreaming audio-driven Coyote 2 patterns every 100 ms.\n");
    printf("Press 'q' to quit, 'h' for help.\n\n");

#ifndef _WIN32
    TerminalRaw raw_term;
#endif

    const int period_ms = 100;
    int tick = 0;

    uint32_t waveclock_a = 0;
    uint32_t cyclecount_a = 0;
    uint32_t waveclock_b = 0;
    uint32_t cyclecount_b = 0;

    while (g_audio.running) {
        if (process_keys(&gain_a, &gain_b)) {
            printf("Quitting on user request.\n");
            break;
        }

        double hz_a = g_audio.left_hz.load();
        double hz_b = g_audio.right_hz.load();
        double lvl_a = g_audio.left_level.load();
        double lvl_b = g_audio.right_level.load();

        // Calculate power for each channel (0..2000 range)
        uint16_t power_a = static_cast<uint16_t>(std::clamp(lvl_a * (gain_a / 100.0) * dglab2::kPowerMax, 0.0, static_cast<double>(dglab2::kPowerMax)));
        uint16_t power_b = static_cast<uint16_t>(std::clamp(lvl_b * (gain_b / 100.0) * dglab2::kPowerMax, 0.0, static_cast<double>(dglab2::kPowerMax)));

        // Send power update packet
        std::vector<uint8_t> pwr_pkt = dglab2::encode_power(power_a, power_b);
        write_characteristic(g_ctx.peripheral, svc_main, char_power, pwr_pkt.data(), pwr_pkt.size());

        // Calculate pattern parameters for channel A
        dglab2::CoyotePattern pat_a;
        if (a_mode == dglab2::MODE_BREATH) {
            dglab2::CoyotePattern gen_a = dglab2::coyote_mode_breath(waveclock_a, cyclecount_a);
            pat_a.amplitude = static_cast<uint8_t>(std::clamp(lvl_a * (gen_a.amplitude / 100.0) * 100.0, 0.0, 100.0));
            pat_a.pulse_length = static_cast<uint8_t>(std::clamp(static_cast<double>(gen_a.pulse_length), 1.0, 31.0));
            const double total_cycle = static_cast<double>(gen_a.pulse_length + gen_a.pause_length);
            const double base_freq = (total_cycle > 0.0) ? (1000.0 / total_cycle) : 10.0;
            const double ratio_a = audio_freq_ratio(hz_a);
            const double out_freq_a = base_freq + ratio_a * (100.0 - base_freq);
            pat_a.pause_length = static_cast<uint16_t>(std::clamp(1000.0 / out_freq_a - pat_a.pulse_length, 10.0, 1023.0));
            std::vector<uint8_t> pat_a_pkt = dglab2::encode_pattern(pat_a);
            write_characteristic(g_ctx.peripheral, svc_main, char_pattern_a, pat_a_pkt.data(), pat_a_pkt.size());
        } else if (a_mode == dglab2::MODE_WAVES || a_mode == dglab2::MODE_DEFAULT) {
            dglab2::CoyotePattern gen_a = dglab2::coyote_mode_waves(waveclock_a, cyclecount_a);
            const double total_cycle = static_cast<double>(gen_a.pulse_length + gen_a.pause_length);
            const double base_freq = (total_cycle > 0.0) ? (1000.0 / total_cycle) : 10.0;
            const double ratio_a = audio_freq_ratio(hz_a);
            const double out_freq_a = base_freq + ratio_a * (100.0 - base_freq);
            pat_a.amplitude = static_cast<uint8_t>(std::clamp(lvl_a * (gen_a.amplitude / 100.0) * 100.0, 0.0, 100.0));
            pat_a.pulse_length = static_cast<uint8_t>(std::clamp(static_cast<double>(gen_a.pulse_length), 1.0, 31.0));
            pat_a.pause_length = static_cast<uint16_t>(std::clamp(1000.0 / out_freq_a - pat_a.pulse_length, 10.0, 1023.0));
            std::vector<uint8_t> pat_a_pkt = dglab2::encode_pattern(pat_a);
            write_characteristic(g_ctx.peripheral, svc_main, char_pattern_a, pat_a_pkt.data(), pat_a_pkt.size());
        } else if (a_mode == dglab2::MODE_STROBE) {
            dglab2::CoyotePattern gen_a = dglab2::coyote_mode_strobe(waveclock_a, cyclecount_a);
            const double total_cycle = static_cast<double>(gen_a.pulse_length + gen_a.pause_length);
            const double base_freq = (total_cycle > 0.0) ? (1000.0 / total_cycle) : 10.0;
            const double ratio_a = audio_freq_ratio(hz_a);
            const double out_freq_a = base_freq + ratio_a * (100.0 - base_freq);
            pat_a.amplitude = static_cast<uint8_t>(std::clamp(lvl_a * (gen_a.amplitude / 100.0) * 100.0, 0.0, 100.0));
            pat_a.pulse_length = static_cast<uint8_t>(std::clamp(static_cast<double>(gen_a.pulse_length), 1.0, 31.0));
            pat_a.pause_length = static_cast<uint16_t>(std::clamp(1000.0 / out_freq_a - pat_a.pulse_length, 10.0, 1023.0));
            std::vector<uint8_t> pat_a_pkt = dglab2::encode_pattern(pat_a);
            write_characteristic(g_ctx.peripheral, svc_main, char_pattern_a, pat_a_pkt.data(), pat_a_pkt.size());
        } else if (a_mode == dglab2::MODE_PULSE) {
            dglab2::CoyotePattern gen_a = dglab2::coyote_mode_pulse(waveclock_a, cyclecount_a);
            const double total_cycle = static_cast<double>(gen_a.pulse_length + gen_a.pause_length);
            const double base_freq = (total_cycle > 0.0) ? (1000.0 / total_cycle) : 10.0;
            const double ratio_a = audio_freq_ratio(hz_a);
            const double out_freq_a = base_freq + ratio_a * (100.0 - base_freq);
            pat_a.amplitude = static_cast<uint8_t>(std::clamp(lvl_a * (gen_a.amplitude / 100.0) * 100.0, 0.0, 100.0));
            pat_a.pulse_length = static_cast<uint8_t>(std::clamp(static_cast<double>(gen_a.pulse_length), 1.0, 31.0));
            pat_a.pause_length = static_cast<uint16_t>(std::clamp(1000.0 / out_freq_a - pat_a.pulse_length, 10.0, 1023.0));
            std::vector<uint8_t> pat_a_pkt = dglab2::encode_pattern(pat_a);
            write_characteristic(g_ctx.peripheral, svc_main, char_pattern_a, pat_a_pkt.data(), pat_a_pkt.size());
        } else if (a_mode == dglab2::MODE_RAINBOW) {
            dglab2::CoyotePattern gen_a = dglab2::coyote_mode_rainbow(waveclock_a, cyclecount_a);
            const double total_cycle = static_cast<double>(gen_a.pulse_length + gen_a.pause_length);
            const double base_freq = (total_cycle > 0.0) ? (1000.0 / total_cycle) : 10.0;
            const double ratio_a = audio_freq_ratio(hz_a);
            const double out_freq_a = base_freq + ratio_a * (100.0 - base_freq);
            pat_a.amplitude = static_cast<uint8_t>(std::clamp(lvl_a * (gen_a.amplitude / 100.0) * 100.0, 0.0, 100.0));
            pat_a.pulse_length = static_cast<uint8_t>(std::clamp(static_cast<double>(gen_a.pulse_length), 1.0, 31.0));
            pat_a.pause_length = static_cast<uint16_t>(std::clamp(1000.0 / out_freq_a - pat_a.pulse_length, 10.0, 1023.0));
            std::vector<uint8_t> pat_a_pkt = dglab2::encode_pattern(pat_a);
            write_characteristic(g_ctx.peripheral, svc_main, char_pattern_a, pat_a_pkt.data(), pat_a_pkt.size());
        } else if (a_mode == dglab2::MODE_BASS) {
            dglab2::CoyotePattern gen_a = dglab2::coyote_mode_bass(waveclock_a, cyclecount_a);
            const double total_cycle = static_cast<double>(gen_a.pulse_length + gen_a.pause_length);
            const double base_freq = (total_cycle > 0.0) ? (1000.0 / total_cycle) : 10.0;
            const double ratio_a = audio_freq_ratio(hz_a);
            const double out_freq_a = base_freq + ratio_a * (100.0 - base_freq);
            pat_a.amplitude = static_cast<uint8_t>(std::clamp(lvl_a * (gen_a.amplitude / 100.0) * 100.0, 0.0, 100.0));
            pat_a.pulse_length = static_cast<uint8_t>(std::clamp(static_cast<double>(gen_a.pulse_length), 1.0, 31.0));
            pat_a.pause_length = static_cast<uint16_t>(std::clamp(1000.0 / out_freq_a - pat_a.pulse_length, 10.0, 1023.0));
            std::vector<uint8_t> pat_a_pkt = dglab2::encode_pattern(pat_a);
            write_characteristic(g_ctx.peripheral, svc_main, char_pattern_a, pat_a_pkt.data(), pat_a_pkt.size());
        }
        // Calculate pattern parameters for channel B
        dglab2::CoyotePattern pat_b;
        if (b_mode == dglab2::MODE_BREATH) {
            dglab2::CoyotePattern gen_b = dglab2::coyote_mode_breath(waveclock_b, cyclecount_b);
            pat_b.amplitude = static_cast<uint8_t>(std::clamp(lvl_b * (gen_b.amplitude / 100.0) * 100.0, 0.0, 100.0));
            pat_b.pulse_length = static_cast<uint8_t>(std::clamp(static_cast<double>(gen_b.pulse_length), 1.0, 31.0));
            const double total_cycle = static_cast<double>(gen_b.pulse_length + gen_b.pause_length);
            const double base_freq = (total_cycle > 0.0) ? (1000.0 / total_cycle) : 10.0;
            const double ratio_b = audio_freq_ratio(hz_b);
            const double out_freq_b = base_freq + ratio_b * (100.0 - base_freq);
            pat_b.pause_length = static_cast<uint16_t>(std::clamp(1000.0 / out_freq_b - pat_b.pulse_length, 10.0, 1023.0));
            std::vector<uint8_t> pat_b_pkt = dglab2::encode_pattern(pat_b);
            write_characteristic(g_ctx.peripheral, svc_main, char_pattern_b, pat_b_pkt.data(), pat_b_pkt.size());
        } else if (b_mode == dglab2::MODE_WAVES || b_mode == dglab2::MODE_DEFAULT) {
            dglab2::CoyotePattern gen_b = dglab2::coyote_mode_waves(waveclock_b, cyclecount_b);
            const double total_cycle = static_cast<double>(gen_b.pulse_length + gen_b.pause_length);
            const double base_freq = (total_cycle > 0.0) ? (1000.0 / total_cycle) : 10.0;
            const double ratio_b = audio_freq_ratio(hz_b);
            const double out_freq_b = base_freq + ratio_b * (100.0 - base_freq);
            pat_b.amplitude = static_cast<uint8_t>(std::clamp(lvl_b * (gen_b.amplitude / 100.0) * 100.0, 0.0, 100.0));
            pat_b.pulse_length = static_cast<uint8_t>(std::clamp(static_cast<double>(gen_b.pulse_length), 1.0, 31.0));
            pat_b.pause_length = static_cast<uint16_t>(std::clamp(1000.0 / out_freq_b - pat_b.pulse_length, 10.0, 1023.0));
            std::vector<uint8_t> pat_b_pkt = dglab2::encode_pattern(pat_b);
            write_characteristic(g_ctx.peripheral, svc_main, char_pattern_b, pat_b_pkt.data(), pat_b_pkt.size());
        } else if (b_mode == dglab2::MODE_STROBE) {
            dglab2::CoyotePattern gen_b = dglab2::coyote_mode_strobe(waveclock_b, cyclecount_b);
            const double total_cycle = static_cast<double>(gen_b.pulse_length + gen_b.pause_length);
            const double base_freq = (total_cycle > 0.0) ? (1000.0 / total_cycle) : 10.0;
            const double ratio_b = audio_freq_ratio(hz_b);
            const double out_freq_b = base_freq + ratio_b * (100.0 - base_freq);
            pat_b.amplitude = static_cast<uint8_t>(std::clamp(lvl_b * (gen_b.amplitude / 100.0) * 100.0, 0.0, 100.0));
            pat_b.pulse_length = static_cast<uint8_t>(std::clamp(static_cast<double>(gen_b.pulse_length), 1.0, 31.0));
            pat_b.pause_length = static_cast<uint16_t>(std::clamp(1000.0 / out_freq_b - pat_b.pulse_length, 10.0, 1023.0));
            std::vector<uint8_t> pat_b_pkt = dglab2::encode_pattern(pat_b);
            write_characteristic(g_ctx.peripheral, svc_main, char_pattern_b, pat_b_pkt.data(), pat_b_pkt.size());
        } else if (b_mode == dglab2::MODE_PULSE) {
            dglab2::CoyotePattern gen_b = dglab2::coyote_mode_pulse(waveclock_b, cyclecount_b);
            const double total_cycle = static_cast<double>(gen_b.pulse_length + gen_b.pause_length);
            const double base_freq = (total_cycle > 0.0) ? (1000.0 / total_cycle) : 10.0;
            const double ratio_b = audio_freq_ratio(hz_b);
            const double out_freq_b = base_freq + ratio_b * (100.0 - base_freq);
            pat_b.amplitude = static_cast<uint8_t>(std::clamp(lvl_b * (gen_b.amplitude / 100.0) * 100.0, 0.0, 100.0));
            pat_b.pulse_length = static_cast<uint8_t>(std::clamp(static_cast<double>(gen_b.pulse_length), 1.0, 31.0));
            pat_b.pause_length = static_cast<uint16_t>(std::clamp(1000.0 / out_freq_b - pat_b.pulse_length, 10.0, 1023.0));
            std::vector<uint8_t> pat_b_pkt = dglab2::encode_pattern(pat_b);
            write_characteristic(g_ctx.peripheral, svc_main, char_pattern_b, pat_b_pkt.data(), pat_b_pkt.size());
        } else if (b_mode == dglab2::MODE_RAINBOW) {
            dglab2::CoyotePattern gen_b = dglab2::coyote_mode_rainbow(waveclock_b, cyclecount_b);
            const double total_cycle = static_cast<double>(gen_b.pulse_length + gen_b.pause_length);
            const double base_freq = (total_cycle > 0.0) ? (1000.0 / total_cycle) : 10.0;
            const double ratio_b = audio_freq_ratio(hz_b);
            const double out_freq_b = base_freq + ratio_b * (100.0 - base_freq);
            pat_b.amplitude = static_cast<uint8_t>(std::clamp(lvl_b * (gen_b.amplitude / 100.0) * 100.0, 0.0, 100.0));
            pat_b.pulse_length = static_cast<uint8_t>(std::clamp(static_cast<double>(gen_b.pulse_length), 1.0, 31.0));
            pat_b.pause_length = static_cast<uint16_t>(std::clamp(1000.0 / out_freq_b - pat_b.pulse_length, 10.0, 1023.0));
            std::vector<uint8_t> pat_b_pkt = dglab2::encode_pattern(pat_b);
            write_characteristic(g_ctx.peripheral, svc_main, char_pattern_b, pat_b_pkt.data(), pat_b_pkt.size());
        } else if (b_mode == dglab2::MODE_BASS) {
            dglab2::CoyotePattern gen_b = dglab2::coyote_mode_bass(waveclock_b, cyclecount_b);
            const double total_cycle = static_cast<double>(gen_b.pulse_length + gen_b.pause_length);
            const double base_freq = (total_cycle > 0.0) ? (1000.0 / total_cycle) : 10.0;
            const double ratio_b = audio_freq_ratio(hz_b);
            const double out_freq_b = base_freq + ratio_b * (100.0 - base_freq);
            pat_b.amplitude = static_cast<uint8_t>(std::clamp(lvl_b * (gen_b.amplitude / 100.0) * 100.0, 0.0, 100.0));
            pat_b.pulse_length = static_cast<uint8_t>(std::clamp(static_cast<double>(gen_b.pulse_length), 1.0, 31.0));
            pat_b.pause_length = static_cast<uint16_t>(std::clamp(1000.0 / out_freq_b - pat_b.pulse_length, 10.0, 1023.0));
            std::vector<uint8_t> pat_b_pkt = dglab2::encode_pattern(pat_b);
            write_characteristic(g_ctx.peripheral, svc_main, char_pattern_b, pat_b_pkt.data(), pat_b_pkt.size());
        }


        if (tick % 20 == 0) {
            printf("[Tick %d] Audio L: %.1f Hz (lvl %.2f) -> PwrA=%u | Audio R: %.1f Hz (lvl %.2f) -> PwrB=%u\n",
                   tick, hz_a, lvl_a, power_a, hz_b, lvl_b, power_b);
        }

        tick++;
        msleep(period_ms);
    }

    g_audio.running = false;
    if (audio_thread.joinable()) audio_thread.join();

    // -- 9. Cleanup --------------------------------------------------------
    simpleble_peripheral_unsubscribe(g_ctx.peripheral, svc_main, char_power);
    simpleble_peripheral_disconnect(g_ctx.peripheral);
    printf("Disconnected.\n");
    return 0;
}

}  // namespace

inline int parse_mode_arg(const char* arg) {
    if (std::strcmp(arg, "breath") == 0 || std::strcmp(arg, "1") == 0) {
        return dglab2::MODE_BREATH;
    }
    if (std::strcmp(arg, "waves") == 0 || std::strcmp(arg, "2") == 0) {
        return dglab2::MODE_WAVES;
    }
    if (std::strcmp(arg, "strobe") == 0 || std::strcmp(arg, "3") == 0) {
        return dglab2::MODE_STROBE;
    }
    if (std::strcmp(arg, "pulse") == 0 || std::strcmp(arg, "4") == 0) {
        return dglab2::MODE_PULSE;
    }
    if (std::strcmp(arg, "rainbow") == 0 || std::strcmp(arg, "5") == 0) {
        return dglab2::MODE_RAINBOW;
    }
    if (std::strcmp(arg, "bass") == 0 || std::strcmp(arg, "6") == 0) {
        return dglab2::MODE_BASS;
    }
    return std::atoi(arg);
}

int main(int argc, char** argv) {
    int gain_a = 20;
    int gain_b = 20;

    if (argc > 1) {
        gain_a = std::atoi(argv[1]);
        if (gain_a < 0 || gain_a > 100) {
            printf("Usage: %s [a_gain] [b_gain] [freq_max_khz] [a_mode] [b_mode]\n", argv[0]);
            printf("  a_gain      channel A (left)  intensity scale, 0..100 (default: 20)\n");
            printf("  b_gain      channel B (right) intensity scale, 0..100 (default: 20)\n");
            printf("  freq_max_khz highest audio frequency considered, kHz (default: 10)\n");
            printf("  a_mode      channel A mode: default(0)=waves breath(1) waves(2) strobe(3) pulse(4) rainbow(5) bass(6)\n");
            printf("  b_mode      channel B mode: default(0)=waves breath(1) waves(2) strobe(3) pulse(4) rainbow(5) bass(6)\n");
            return 1;
        }
    }
    if (argc > 2) {
        gain_b = std::atoi(argv[2]);
        if (gain_b < 0 || gain_b > 100) {
            printf("Usage: %s [a_gain] [b_gain] [freq_max_khz] [a_mode] [b_mode]\n", argv[0]);
            return 1;
        }
    }
    if (argc > 3) {
        double max_khz = std::atof(argv[3]);
        if (max_khz >= 0.1 && max_khz <= 20.0) {
            g_audio_freq_max_hz = max_khz * 1000.0;
        }
    }

    int a_mode = dglab2::MODE_DEFAULT;
    if (argc > 4) {
        a_mode = parse_mode_arg(argv[4]);
    }
    int b_mode = dglab2::MODE_DEFAULT;
    if (argc > 5) {
        b_mode = parse_mode_arg(argv[5]);
    }

    return run(gain_a, gain_b, a_mode, b_mode);
}
