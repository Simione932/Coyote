// dglab_coyote_audio.cpp
//
// C++ CLI test application that exercises the shared dglab library
// (dglab_protocol / dglab_audio / dglab_session) against a DG-LAB Coyote
// (郊狼) pulse host running the V3 Bluetooth protocol
// (https://github.com/dungeonlab-open/dglab-bluetooth-protocol), with the
// waveform frequencies driven by the audio input stream.
//
// What it does:
//   1. Opens the audio input device in stereo ('BlackHole 2ch' on macOS,
//      or WASAPI loopback capture on Windows).
//   2. Every 100 ms it computes the frequency spectrum (Hann window +
//      8192-point FFT) of the left and right channels independently and
//      finds the dominant audio frequency in each (20 Hz .. freq_max_khz,
//      default 10 kHz, upper limit settable via the [freq_max_khz] command
//      line parameter).
//   3. Maps each dominant frequency logarithmically into the Coyote
//      waveform period range:
//         20 Hz   -> 100 ms period (10 Hz output, the minimum allowed)
//         10 kHz  ->  10 ms period (100 Hz output)
//      Silence maps to the minimum output frequency (10 Hz / 100 ms).
//   4. Scans for DG-LAB devices (names starting with "47L"), connects and
//      verifies the expected GATT layout.
//   5. Subscribes to 0x150B notifications and parses B1 strength replies.
//   6. Reads the battery level from 0x180A/0x1500.
//   7. Writes the BF command (soft caps + balance parameters).
//   8. Streams B0 waveform commands every 100 ms: channel A (1) uses the
//      left audio dominant frequency, channel B (2) the right one. The
//      intensity of each channel is proportional to the loudness of the
//      loudest tone on that channel, scaled by the a_gain / b_gain command
//      line parameters. Every 5th packet sets the strength absolutely to 20
//      with a sequence number and expects a B1 acknowledgement.
//   9. Runs continuously until the 'q' key is pressed, then disconnects
//      and releases all handles.
//
// Everything except command-line parsing and keystroke handling now lives in
// the shared dglab library: the command-line parameters below seed a shared
// CoyoteSettings, and the keystrokes mutate the same struct the session reads
// every tick (the GUI drives those same fields from its sliders/radios).
//
// Usage: dglab_coyote_audio [--v2] [a_gain] [b_gain] [freq_max_khz] [smooth_ms]
// [a_mode] [b_mode] [idle_intensity]
//   --v2      drive a DG-LAB Coyote V2 (郊狼 2.0 / ESTIM01) instead of V3
//   a_gain      channel A (left)  initial intensity scale, 0..200 (default: 5)
//   b_gain      channel B (right) initial intensity scale, 0..200 (default: 5)
//   freq_max_khz highest audio frequency considered, kHz (default: 10)
//   smooth_ms   intensity smoothing time constant in ms, 0 = off (default: 0)
//   a_mode      channel A mode: default (0), breath (1), waves (2), strobe (3),
//   pulse (4), rainbow (5), or bass (6) (default: 0) b_mode      channel B
//   mode: default (0), breath (1), waves (2), strobe (3), pulse (4), rainbow
//   (5), or bass (6) (default: 0) idle_intensity baseline intensity applied to
//   silent channels (no audio detected), 0..100 (default: 0)
//
// Keys (while running):
//   q   quit
//   h   show this help
//   r/t channel A gain up/down (steps of 5)
//   o/p channel B gain up/down (steps of 5)
//
// NOTE: The DG-LAB open protocol is licensed for personal/hobby use only.
// NOTE: Audio capture uses WASAPI loopback on Windows, and 'BlackHole 2ch'
// input on macOS.

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#ifdef _WIN32
#include <Windows.h>
#include <conio.h>
#else
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#endif

#include "dglab_audio.h"
#include "dglab_protocol.h"
#include "dglab_session.h"

#include "dglab_protocol_v2.h"
#include "dglab_session_v2.h"

namespace {

static std::atomic<bool> g_exit_requested{false};

static void signal_handler(int) { g_exit_requested.store(true); }

#ifndef _WIN32
// Puts stdin in raw (non-canonical) mode so a key press is delivered
// immediately without waiting for Return, and restores the previous
// termios settings on destruction.
struct TerminalRaw {
  bool changed = false;
  termios orig = {};
  TerminalRaw() {
    if (tcgetattr(STDIN_FILENO, &orig) == 0) {
      changed = true;
      orig.c_lflag &= ~(tcflag_t)(ICANON | ECHO);
      orig.c_iflag |= (tcflag_t)(ICRNL | INLCR);
      tcsetattr(STDIN_FILENO, TCSANOW, &orig);
    }
  }
  ~TerminalRaw() {
    if (changed) {
      tcsetattr(STDIN_FILENO, TCSANOW, &orig);
    }
  }
};
#endif

// Session log lines go to the terminal (the session may be on another thread).
void session_log(const char *msg, void *) {
  printf("%s\n", msg);
  fflush(stdout);
}

// Non-blocking key handling. Returns true if the user requested exit ('q').
// 'r'/'t' raise/lower the channel A gain, 'o'/'p' the channel B gain via the
// shared CoyoteSettings (steps of 5, clamped to 0..200).
bool process_keys(dglab::CoyoteSettings *settings) {
  if (g_exit_requested.load()) {
    return true;
  }
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
             "o/p channel B gain up/down (steps of 5, 0..200)\n");
      printf("Modes A/B: default(0) breath(1) waves(2) strobe(3) pulse(4) "
             "rainbow(5) bass(6)\n");
      printf("Current gains: A=%d B=%d\n", settings->gain_a(),
             settings->gain_b());
      break;
    case 'r':
    case 'R':
      settings->gain_up(0);
      printf("Channel A gain: %d\n", settings->gain_a());
      break;
    case 't':
    case 'T':
      settings->gain_down(0);
      printf("Channel A gain: %d\n", settings->gain_a());
      break;
    case 'o':
    case 'O':
      settings->gain_up(1);
      printf("Channel B gain: %d\n", settings->gain_b());
      break;
    case 'p':
    case 'P':
      settings->gain_down(1);
      printf("Channel B gain: %d\n", settings->gain_b());
      break;
    default:
      break;
    }
  }
}

// Overload for CoyoteV2Settings (gains 0..100).
bool process_keys(dglab::CoyoteV2Settings *settings) {
  if (g_exit_requested.load()) {
    return true;
  }
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
      printf("Modes A/B: default(0) breath(1) waves(2) strobe(3) pulse(4) "
             "rainbow(5) bass(6)\n");
      printf("Current gains: A=%d B=%d\n", settings->gain_a(),
             settings->gain_b());
      break;
    case 'r':
    case 'R':
      settings->gain_up(0);
      printf("Channel A gain: %d\n", settings->gain_a());
      break;
    case 't':
    case 'T':
      settings->gain_down(0);
      printf("Channel A gain: %d\n", settings->gain_a());
      break;
    case 'o':
    case 'O':
      settings->gain_up(1);
      printf("Channel B gain: %d\n", settings->gain_b());
      break;
    case 'p':
    case 'P':
      settings->gain_down(1);
      printf("Channel B gain: %d\n", settings->gain_b());
      break;
    default:
      break;
    }
  }
}

#ifdef _WIN32
inline void msleep_or_10ms() { Sleep(10); }
#else
inline void msleep_or_10ms() { usleep(10 * 1000); }
#endif

int run(dglab::CoyoteSettings *settings, dglab::AudioState *audio) {
  g_exit_requested.store(false);
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // -- 0. Start the audio capture thread ---------------------------------
  if (!dglab::start_audio_capture(audio, settings->freq_max_khz() * 1000.0)) {
    printf("WARN: audio capture is not available; using 10 Hz on both "
           "channels.\n");
  }

  // -- 1..8. Session: adapter, scan, connect, GATT, notifications,
  //          battery, BF write, B0 streaming. Runs on its own thread so
  //          the main thread can keep handling keys; every tick reads the
  //          shared settings, so 'r/t'/'o/p' take effect immediately.
  dglab::CoyoteSession session(audio, settings, session_log, nullptr);
  std::atomic<dglab::SessionStatus> status{dglab::SessionStatus::kStopped};
  std::thread session_thread([&]() { status = session.start(); });

#ifdef _WIN32
  // Windows _getch() is already character-oriented; no termios change needed.
#else
  TerminalRaw raw_terminal; // keys register immediately, no Return needed
#endif

  printf("Keys: q quit | h help | r/t channel A gain up/down | o/p channel B "
         "gain up/down\n");

  // -- 9. Wait for quit (or the session ending on its own), then clean up.
  while (!process_keys(settings) && session.running()) {
    msleep_or_10ms();
  }
  session.stop();
  session_thread.join();

  dglab::stop_audio_capture(audio);

  bool pass = status != dglab::SessionStatus::kNoAdapter &&
              status != dglab::SessionStatus::kNoDevice &&
              status != dglab::SessionStatus::kConnectFailed &&
              status != dglab::SessionStatus::kGattMissing &&
              session.b1_ack_count() > 0;
  printf("Result: %s\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}

int run_v2(dglab::CoyoteV2Settings *settings, dglab::AudioState *audio) {
  g_exit_requested.store(false);
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // -- 0. Start the audio capture thread ---------------------------------
  if (!dglab::start_audio_capture(audio, settings->freq_max_khz() * 1000.0)) {
    printf("WARN: audio capture is not available; using 10 Hz on both "
           "channels.\n");
  }

  // -- 1..8. Session: adapter, scan, connect, GATT, notifications,
  //          battery, power/pattern streaming. Runs on its own thread so
  //          the main thread can keep handling keys; every tick reads the
  //          shared settings, so 'r/t'/'o/p' take effect immediately.
  dglab::CoyoteV2Session session(audio, settings, session_log, nullptr);
  std::atomic<dglab::SessionStatus> status{dglab::SessionStatus::kStopped};
  std::thread session_thread([&]() { status = session.start(); });

#ifdef _WIN32
  // Windows _getch() is already character-oriented; no termios change needed.
#else
  TerminalRaw raw_terminal; // keys register immediately, no Return needed
#endif

  printf("Keys: q quit | h help | r/t channel A gain up/down | o/p channel B "
         "gain up/down\n");

  // -- 9. Wait for quit (or the session ending on its own), then clean up.
  while (!process_keys(settings) && session.running()) {
    msleep_or_10ms();
  }
  session.stop();
  session_thread.join();

  dglab::stop_audio_capture(audio);

  bool pass = status != dglab::SessionStatus::kNoAdapter &&
              status != dglab::SessionStatus::kNoDevice &&
              status != dglab::SessionStatus::kConnectFailed &&
              status != dglab::SessionStatus::kGattMissing;
  printf("Result: %s\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}

} // namespace

int main(int argc, char **argv) {
  bool use_v2 = false;
  if (argc > 1 && std::strcmp(argv[1], "--v2") == 0) {
    use_v2 = true;
  }

  dglab::CoyoteSettings settings;

  if (argc > 1) {
    int g = std::atoi(argv[1]);
    if (g < 0 || g > 200) {
      printf("Usage: %s [--v2] [a_gain] [b_gain] [freq_max_khz] [smooth_ms] "
             "[a_mode] [b_mode] [idle_intensity]\n",
             argv[0]);
      printf("  --v2      drive a DG-LAB Coyote V2 (郊狼 2.0 / ESTIM01) "
             "instead of V3\n");
      printf("  a_gain      channel A (left)  intensity scale, 0..200 "
             "(default: 5)\n");
      printf("  b_gain      channel B (right) intensity scale, 0..200 "
             "(default: 5)\n");
      printf("  freq_max_khz highest audio frequency considered, kHz (default: "
             "10)\n");
      printf("  smooth_ms   intensity smoothing time constant, ms, 0 = off "
             "(default: 0)\n");
      printf(
          "  a_mode      channel A mode: default (0), breath (1), waves (2), "
          "strobe (3), pulse (4), rainbow (5), or bass (6) (default: 0)\n");
      printf(
          "  b_mode      channel B mode: default (0), breath (1), waves (2), "
          "strobe (3), pulse (4), rainbow (5), or bass (6) (default: 0)\n");
      printf("  idle_intensity baseline intensity for silent channels, 0..100 "
             "(default: 0)\n");
      printf("Runs continuously; press q to quit.\n");
      return 1;
    }
    settings.set_gain_a(g);
  }
  if (argc > 2) {
    int g = std::atoi(argv[2]);
    if (g < 0 || g > 200) {
      printf("b_gain must be in 0..200\n");
      return 1;
    }
    settings.set_gain_b(g);
  }

  if (argc > 3) {
    char *end = nullptr;
    double f = std::strtod(argv[3], &end);
    if (end == argv[3] || f <= dglab::kAudioFreqMinHz / 1000.0 || f > 24.0) {
      printf("freq_max_khz must be a float in (0.02, 24] kHz\n");
      return 1;
    }
    settings.set_freq_max_khz(f);
  }

  if (argc > 4) {
    char *end = nullptr;
    double s = std::strtod(argv[4], &end);
    if (end == argv[4] || s < 0.0 || s > 10000.0) {
      printf("smooth_ms must be a float in [0, 10000] ms\n");
      return 1;
    }
    settings.set_smooth_ms(s);
  }

  if (argc > 5) {
    settings.set_a_mode(dglab::parse_mode_arg(argv[5]));
  }
  if (argc > 6) {
    settings.set_b_mode(dglab::parse_mode_arg(argv[6]));
  }
  if (argc > 7) {
    char *end = nullptr;
    double i = std::strtod(argv[7], &end);
    if (end == argv[7] || i < 0.0 || i > 100.0) {
      printf("idle_intensity must be a float in [0, 100]\n");
      return 1;
    }
    settings.set_idle_intensity(i);
  }

  dglab::AudioState audio;
  if (use_v2) {
    dglab::CoyoteV2Settings v2_settings;
    v2_settings.set_gain_a(settings.gain_a());
    v2_settings.set_gain_b(settings.gain_b());
    v2_settings.set_freq_max_khz(settings.freq_max_khz());
    v2_settings.set_a_mode(settings.a_mode());
    v2_settings.set_b_mode(settings.b_mode());
    return run_v2(&v2_settings, &audio);
  }
  return run(&settings, &audio);
}
