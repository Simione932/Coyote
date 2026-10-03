// dglab_audio_backend_macos.mm
//
// macOS audio capture backend: opens the 'BlackHole 2ch' audio device via a
// CoreAudio AUHAL input (monitor) unit. Every 100 ms of audio it computes the
// dominant frequency of each channel and publishes it in AudioState.

#include "dglab_audio_backend.h"

#include <CoreFoundation/CoreFoundation.h>
#include <AudioToolbox/AudioToolbox.h>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

namespace dglab_audio {

namespace {

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

}  // namespace

void audio_capture_thread(dglab::AudioState* st, double) {
    using dglab::dominant_tone;
    MacCapture cap;

    // 1. Find the 'BlackHole 2ch' audio device
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
        printf("WARN: could not find audio input device '%s'; "
               "using 10 Hz on both channels.\n", target_device_name.c_str());
        st->capture_ok = false;
        while (st->running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return;
    }

    // 2. Query device nominal sample rate
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

    // 3. Create HALOutput AudioUnit
    AudioComponentDescription cd = {};
    cd.componentType = kAudioUnitType_Output;
    cd.componentSubType = kAudioUnitSubType_HALOutput;
    cd.componentManufacturer = kAudioUnitManufacturer_Apple;
    AudioComponent comp = AudioComponentFindNext(nullptr, &cd);
    if (comp != nullptr) {
        os = AudioComponentInstanceNew(comp, &cap.unit);
    } else {
        os = -10868;  // kAudioUnitErr_CannotFindInput
    }

    // Enable input (bus 1), disable output (bus 0)
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

    // Set device ID
    if (os == noErr) {
        os = AudioUnitSetProperty(cap.unit, kAudioOutputUnitProperty_CurrentDevice,
                                  kAudioUnitScope_Global, 0, &target_id, sizeof(target_id));
    }

    // Set stream format (2-channel float32) on Bus 1 output scope
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

    // Set input callback
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
        printf("WARN: could not open audio device '%s' (OSStatus %d); "
               "using 10 Hz on both channels.\n", target_device_name.c_str(), static_cast<int>(os));
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
    const size_t window_samples = static_cast<size_t>(cap.sample_rate * dglab::kAnalysisWindowS);
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
            const double freq_min_hz = st->freq_min_hz.load();
            const double freq_max_hz = st->freq_max_hz.load();
            std::vector<float> lspec, rspec;
            const dglab::ToneInfo lt = dominant_tone(left, cap.sample_rate, freq_min_hz, freq_max_hz, &lspec);
            const dglab::ToneInfo rt = dominant_tone(right, cap.sample_rate, freq_min_hz, freq_max_hz, &rspec);
            st->left_hz = static_cast<float>(lt.hz);
            st->right_hz = static_cast<float>(rt.hz);
            st->left_level = static_cast<float>(lt.level);
            st->right_level = static_cast<float>(rt.level);
            {
                std::lock_guard<std::mutex> lock(st->spectrum_mutex);
                st->left_spectrum.swap(lspec);
                st->right_spectrum.swap(rspec);
            }
            st->analyses++;
            left.clear();
            right.clear();
        }
    }

    AudioOutputUnitStop(cap.unit);
    AudioUnitUninitialize(cap.unit);
    AudioComponentInstanceDispose(cap.unit);
}

}  // namespace dglab_audio
