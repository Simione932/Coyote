// dglab_audio_backend_win.cpp
//
// Windows audio capture backend: WASAPI loopback on the default render device,
// stereo. Every 100 ms of audio it computes the dominant frequency of each
// channel and publishes it in AudioState.

#include "dglab_audio_backend.h"

#include <Windows.h>
#include <mmdeviceapi.h>
#include <audioclient.h>
#include <cstdio>

namespace dglab_audio {

void audio_capture_thread(dglab::AudioState* st, double) {
    using dglab::dominant_tone;

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
        printf("WARN: could not open the default audio output device (HRESULT 0x%08lX); "
               "using 10 Hz on both channels.\n", static_cast<unsigned long>(hr));
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
    window_samples = static_cast<size_t>(sample_rate * dglab::kAnalysisWindowS);
    printf("Audio capture: %s, %u Hz, %d channels, %zu samples per 100 ms window\n",
           is_float ? "float32" : "pcm16", mix_format->nSamplesPerSec, channels, window_samples);

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
        Sleep(10);
    }

    client->Stop();

cleanup:
    if (capture != nullptr) {
        capture->Release();
    }
    if (client != nullptr) {
        client->Release();
    }
    if (device != nullptr) {
        device->Release();
    }
    if (enumerator != nullptr) {
        enumerator->Release();
    }
    if (mix_format != nullptr) {
        CoTaskMemFree(mix_format);
    }
    CoUninitialize();
}

}  // namespace dglab_audio
