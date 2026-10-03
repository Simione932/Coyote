// dglab_audio_backend_linux.cpp
//
// Linux audio capture backend: PortAudio tap on the default output device's
// monitor stream (PulseAudio / PipeWire), stereo. Every 100 ms of audio it
// computes the dominant frequency of each channel and publishes it in
// AudioState.

#include "dglab_audio_backend.h"

#include <portaudio.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace dglab_audio {

void audio_capture_thread(dglab::AudioState* st, double) {
    using dglab::dominant_tone;

    PaError err = Pa_Initialize();

    // Find a PulseAudio/PipeWire monitor source (a device named 'monitor of
    // <sink>' or containing 'monitor'), preferring the monitor of the default output device.
    int device_index = -1;
    std::string default_out_name;
    if (err == paNoError) {
        const PaDeviceIndex default_out = Pa_GetDefaultOutputDevice();
        if (default_out >= 0) {
            const PaDeviceInfo* di = Pa_GetDeviceInfo(default_out);
            if (di != nullptr && di->name != nullptr) {
                default_out_name = di->name;
            }
        }

        const PaDeviceIndex device_count = Pa_GetDeviceCount();
        for (PaDeviceIndex i = 0; i < device_count; i++) {
            const PaDeviceInfo* di = Pa_GetDeviceInfo(i);
            if (di == nullptr || di->name == nullptr || di->maxInputChannels < 2) {
                continue;
            }
            const bool is_monitor = (strstr(di->name, "monitor") != nullptr || strstr(di->name, "Monitor") != nullptr);
            if (is_monitor && !default_out_name.empty() && strstr(di->name, default_out_name.c_str()) != nullptr) {
                device_index = i;
                break;
            }
            if (device_index == -1 && is_monitor) {
                device_index = i;
            }
        }

        if (device_index == -1) {
            device_index = Pa_GetDefaultInputDevice();
        }
    }

    PaStream* stream = nullptr;
    double sample_rate = 0.0;
    const unsigned long frames_per_buffer = 1024;

    if (err == paNoError && device_index >= 0) {
        const PaDeviceInfo* di = Pa_GetDeviceInfo(device_index);
        if (di != nullptr) {
            sample_rate = di->defaultSampleRate;
            PaStreamParameters inputParams{};
            inputParams.device = device_index;
            inputParams.channelCount = 2;
            inputParams.sampleFormat = paFloat32;
            inputParams.suggestedLatency = di->defaultLowInputLatency;
            inputParams.hostApiSpecificStreamInfo = nullptr;

            err = Pa_OpenStream(&stream,
                                &inputParams,
                                nullptr,  // output parameters
                                sample_rate,
                                frames_per_buffer,
                                paNoFlag,
                                nullptr,  // streamCallback (blocking read)
                                nullptr); // userData

            if (err == paNoError && stream != nullptr) {
                err = Pa_StartStream(stream);
            }
        }
    }

    if (err != paNoError || stream == nullptr) {
        printf("WARN: could not find or open a PulseAudio/PipeWire monitor stream "
               "(PortAudio error: %s); using 10 Hz on both channels.\n",
               Pa_GetErrorText(err));
        st->capture_ok = false;
        while (st->running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (stream != nullptr) {
            Pa_CloseStream(stream);
        }
        if (err == paNoError) {
            Pa_Terminate();
        }
        return;
    }

    st->capture_ok = true;
    const size_t window_samples = static_cast<size_t>(sample_rate * dglab::kAnalysisWindowS);
    const PaDeviceInfo* dev_info = Pa_GetDeviceInfo(device_index);
    printf("Audio capture: float32, %.0f Hz, '%s' input\n", sample_rate,
           dev_info ? dev_info->name : "unknown");

    std::vector<float> left;
    std::vector<float> right;
    left.reserve(window_samples * 2);
    right.reserve(window_samples * 2);

    std::vector<float> buf(frames_per_buffer * 2);

    while (st->running) {
        PaError r = Pa_ReadStream(stream, buf.data(), frames_per_buffer);
        if (r != paNoError && r != paInputOverflow) {
            printf("WARN: Pa_ReadStream returned error: %s\n", Pa_GetErrorText(r));
            break;
        }

        for (unsigned long f = 0; f < frames_per_buffer; f++) {
            left.push_back(buf[f * 2 + 0]);
            right.push_back(buf[f * 2 + 1]);
        }

        if (left.size() >= window_samples) {
            const double freq_min_hz = st->freq_min_hz.load();
            const double freq_max_hz = st->freq_max_hz.load();
            std::vector<float> lspec, rspec;
            const dglab::ToneInfo lt = dominant_tone(left, sample_rate, freq_min_hz, freq_max_hz, &lspec);
            const dglab::ToneInfo rt = dominant_tone(right, sample_rate, freq_min_hz, freq_max_hz, &rspec);
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

    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
}

}  // namespace dglab_audio
