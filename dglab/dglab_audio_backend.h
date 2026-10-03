// dglab_audio_backend.h
//
// Platform capture thread, implemented in the per-platform backend files:
//   dglab_audio_backend_win.cpp   WASAPI loopback (Windows)
//   dglab_audio_backend_macos.mm  BlackHole 2ch AUHAL monitor (macOS)
//   dglab_audio_backend_linux.cpp PortAudio monitor tap (Linux)
//   dglab_audio_backend_none.cpp  no-op stub (other platforms)
//
// Runs until st->running is cleared by stop_audio_capture(), publishing the
// dominant tone of each channel into st every 100 ms of audio.

#pragma once

#include "dglab_audio.h"

namespace dglab_audio {

// Runs on the capture thread (see dglab_audio.cpp start_audio_capture).
// Sets st->capture_ok on success or on the failure paths before returning.
void audio_capture_thread(dglab::AudioState* st, double freq_max_hz);

}  // namespace dglab_audio
