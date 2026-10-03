// dglab_audio_backend_none.cpp
//
// No-op audio capture backend for platforms without WASAPI/CoreAudio support.

#include "dglab_audio_backend.h"

#include <chrono>
#include <cstdio>
#include <thread>

namespace dglab_audio {

void audio_capture_thread(dglab::AudioState* st, double) {
    printf("WARN: audio capture is not supported on this platform; "
           "using 10 Hz on both channels.\n");
    st->capture_ok = false;
    while (st->running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

}  // namespace dglab_audio
