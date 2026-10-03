// dglab_gui.cpp
//
// GUI for the DG-LAB Coyote audio-reactive light show.
//
// Standalone microui + fenster + renderer stack (C++ `Fenster` wrapper),
// rendering into the 800x600 framebuffer. It is the full live app: audio
// capture and the BLE streaming session come from the shared dglab library
// (dglab_audio / dglab_session), and every control is bound to the shared
// CoyoteSettings the session reads on every 100 ms tick — the GUI analogues
// of the CLI's command-line parameters and keystrokes:
//
//   Gain slider            <-> a_gain / b_gain (step 5, the CLI r/t, o/p step)
//   Mode radios            <-> a_mode / b_mode
//   Freq max slider        <-> freq_max_khz (live: capture re-reads it each
//   window) Smooth slider          <-> smooth_ms Idle intensity slider  <->
//   idle_intensity Scan button            <-> rescan + reconnect Quit button
//   <-> 'q'
//
// The level meters, Hz readouts and the visualizer show the live AudioState.
//
// Build: see CMakeLists.txt (target dglab_gui).

#include "fenster.h"
#include "microui.h"
#include "renderer.h"

#include "dglab_audio.h"
#include "dglab_protocol.h"
#include "dglab_session.h"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

// ---------------------------------------------------------------------------
// Shared live state: the same struct the CLI's keystrokes mutate, and the
// session's streaming tick reads every 100 ms.
// ---------------------------------------------------------------------------
static dglab::CoyoteSettings g_settings;
static dglab::AudioState g_audio;
static dglab::CoyoteSession *g_session = nullptr;

static std::atomic<bool> g_quit{false};
static std::atomic<bool> g_rescan{false};

static const double kAccentA[3] = {66, 205, 235}; // cyan    (channel A / left)
static const double kAccentB[3] = {255, 105,
                                   205}; // magenta (channel B / right)

static double g_t = 0.0; // seconds

// Session log lines go to stdout (the session runs on its own thread).
static void session_log(const char *msg, void *) {
  printf("%s\n", msg);
  fflush(stdout);
}

// ---------------------------------------------------------------------------
// Small drawing helpers
// ---------------------------------------------------------------------------
static inline mu_Color col_ac(double a, double r, double g, double b) {
  return mu_color(static_cast<unsigned char>(r), static_cast<unsigned char>(g),
                  static_cast<unsigned char>(b),
                  static_cast<unsigned char>(a * 255));
}

// text at an explicit position with an explicit color
static void draw_text_at(mu_Context *ctx, const char *text, int x, int y,
                         mu_Color c) {
  mu_draw_text(ctx, ctx->style->font, text, -1, mu_vec2(x, y), c);
}

// left-aligned label inside a rect (normal text color)
static void draw_label(mu_Context *ctx, const char *text, mu_Rect r) {
  mu_draw_control_text(ctx, text, r, MU_COLOR_TEXT, 0);
}

// horizontal level meter, filled left-to-right up to `level`. The level is
// a linear amplitude (full-scale = 1.0); the fill is drawn on a -60..0 dBFS
// scale so typical levels occupy most of the bar (display only -- the engine
// and session still see the linear level).
static void draw_level_meter(mu_Context *ctx, mu_Rect r, double level,
                             double r_, double g, double b) {
  mu_draw_rect(ctx, r, col_ac(1, 30, 33, 42));
  double v = (level > 0.0) ? (20.0 * std::log10(level) + 60.0) / 60.0 : 0.0;
  double fill = mu_clamp(v, 0.0, 1.0) * (r.w - 4);
  mu_draw_rect(ctx, mu_rect(r.x + 2, r.y + 2, fill, r.h - 4),
               col_ac(1, r_, g, b));
}

// radio-group of modes laid out in a single row; selects on click
static void draw_mode_radios(mu_Context *ctx, int *mode,
                             const char *const names[], int n, double r_,
                             double g, double b, int height) {
  // Give every radio an equal cell. Cell width is a tunable const so we can
  // adjust the spacing empirically without recomputing from the body width.
  const int radio_cell = 61;
  int widths[6];
  for (int i = 0; i < n; i++)
    widths[i] = radio_cell;
  mu_layout_row(ctx, n, widths, height);
  for (int i = 0; i < n; i++) {
    mu_Rect r = mu_layout_next(ctx);
    int cx = r.x + r.w / 2, cy = r.y + r.h / 2;
    int rad = r.h / 2;
    bool sel = (*mode == i);
    mu_draw_rect(ctx, mu_rect(cx - rad, cy - rad, rad * 2, rad * 2),
                 col_ac(sel ? 1 : 0.35, r_, g, b));
    if (sel) {
      mu_draw_rect(ctx, mu_rect(cx - 3, cy - 3, 6, 6), col_ac(1, r_, g, b));
    }
    mu_draw_control_text(ctx, names[i], r, MU_COLOR_TEXT, MU_OPT_ALIGNCENTER);
    if (mu_mouse_over(ctx, r) && ctx->mouse_pressed == MU_MOUSE_LEFT) {
      *mode = i;
    }
  }
}

// ---------------------------------------------------------------------------
// One channel panel. Sliders/radios read their value from the shared
// CoyoteSettings at the top of the frame and write it back after the user
// has interacted, so changes reach the session on the next 100 ms tick.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// One channel panel. Gain slider, mode radios, level meter, and frequency.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// One channel panel (Channel A / Left, Channel B / Right)
// ---------------------------------------------------------------------------
static void draw_channel(mu_Context *ctx, const char *name, int channel,
                         double level, double hz, double r_, double g,
                         double b) {
  mu_push_id(ctx, name, strlen(name));
  mu_layout_width(ctx, 392);

  // Accent header bar
  {
    int header_widths[1] = {-1};
    mu_layout_row(ctx, 1, header_widths, 26);
    mu_Rect header_r = mu_layout_next(ctx);
    mu_draw_rect(ctx, header_r, col_ac(1, r_, g, b));
    draw_text_at(ctx, name, header_r.x + 8, header_r.y + 5,
                 col_ac(1, 255, 255, 255));
  }

  // Gain slider
  {
    float gain = (channel == 0 ? g_settings.gain_a() : g_settings.gain_b());
    int widths[2] = {65, -1};
    mu_layout_row(ctx, 2, widths, 38);
    draw_label(ctx, "Gain", mu_layout_next(ctx));
    mu_push_id(ctx, "gain", 4);
    if (mu_slider_ex(ctx, &gain, 0, 200, 5, "%.0f", 0) & MU_RES_CHANGE) {
      if (channel == 0) {
        g_settings.set_gain_a(static_cast<int>(gain));
      } else {
        g_settings.set_gain_b(static_cast<int>(gain));
      }
    }
    mu_pop_id(ctx);
  }

  // Mode radios
  {
    int mode = (channel == 0 ? g_settings.a_mode() : g_settings.b_mode());
    if (mode > dglab::MODE_DEFAULT)
      --mode;
    draw_mode_radios(ctx, &mode, dglab::kModeNames + 1, 6, r_, g, b, 54);
    if (channel == 0) {
      g_settings.set_a_mode(mode + 1);
    } else {
      g_settings.set_b_mode(mode + 1);
    }
  }

  // Level meter + detected frequency readout
  int meter_widths[1] = {-1};
  mu_layout_row(ctx, 1, meter_widths, 38);
  mu_Rect meter = mu_layout_next(ctx);
  draw_level_meter(ctx, meter, level, r_, g, b);
  char buf[32];
  snprintf(buf, sizeof(buf), "%.0f Hz", hz);
  draw_text_at(ctx, buf, meter.x + meter.w - 60, meter.y + meter.h / 2 - 8,
               col_ac(0.9, r_, g, b));

  mu_pop_id(ctx);
}

// ---------------------------------------------------------------------------
// Dedicated Global Audio Settings Panel (shared engine parameters)
// ---------------------------------------------------------------------------
static void draw_global_settings(mu_Context *ctx) {
  mu_push_id(ctx, "global_settings", 15);
  mu_layout_width(ctx, 792);

  // Accent header bar
  {
    int header_widths[1] = {-1};
    mu_layout_row(ctx, 1, header_widths, 26);
    mu_Rect header_r = mu_layout_next(ctx);
    mu_draw_rect(ctx, header_r, col_ac(1, 78, 170, 212));
    draw_text_at(ctx, "GLOBAL AUDIO SETTINGS", header_r.x + 8, header_r.y + 5,
                 col_ac(1, 255, 255, 255));
  }

  // Freq min / Freq max
  {
    float min_khz = static_cast<float>(g_settings.freq_min_khz());
    float max_khz = static_cast<float>(g_settings.freq_max_khz());
    int widths[2] = {110, -1};
    mu_layout_row(ctx, 2, widths, 38);
    draw_label(ctx, "Freq min", mu_layout_next(ctx));
    mu_push_id(ctx, "freq_min", 8);
    if (mu_slider_ex(ctx, &min_khz, 0.02, max_khz, 0.01, "%.2f kHz", 0) &
        MU_RES_CHANGE) {
      g_settings.set_freq_min_khz(min_khz);
    }
    mu_pop_id(ctx);

    mu_layout_row(ctx, 2, widths, 38);
    draw_label(ctx, "Freq max", mu_layout_next(ctx));
    mu_push_id(ctx, "freq_max", 8);
    if (mu_slider_ex(ctx, &max_khz, min_khz, 10.0, 0.01, "%.2f kHz", 0) &
        MU_RES_CHANGE) {
      g_settings.set_freq_max_khz(max_khz);
    }
    mu_pop_id(ctx);
  }

  // Smooth
  {
    float ms = static_cast<float>(g_settings.smooth_ms());
    int widths[2] = {110, -1};
    mu_layout_row(ctx, 2, widths, 38);
    draw_label(ctx, "Smooth", mu_layout_next(ctx));
    mu_push_id(ctx, "smooth", 6);
    if (mu_slider_ex(ctx, &ms, 0, 10000, 10, "%.0f ms", 0) & MU_RES_CHANGE) {
      g_settings.set_smooth_ms(ms);
    }
    mu_pop_id(ctx);
  }

  // Idle intensity
  {
    float idle = static_cast<float>(g_settings.idle_intensity());
    int widths[2] = {110, -1};
    mu_layout_row(ctx, 2, widths, 38);
    draw_label(ctx, "Idle intensity", mu_layout_next(ctx));
    mu_push_id(ctx, "idle", 4);
    if (mu_slider_ex(ctx, &idle, 0, 100, 1, "%.0f%%", 0) & MU_RES_CHANGE) {
      g_settings.set_idle_intensity(idle);
    }
    mu_pop_id(ctx);
  }

  mu_pop_id(ctx);
}

// Side-by-side FFT waterfalls (Channel A on left, Channel B on right).
// Each analysis tick (10 Hz) the capture thread publishes a log-spaced band
// spectrum per channel; each channel keeps a scroll buffer of those rows and
// renders them oldest (top) to newest (bottom), x-axis log-spaced 20 Hz ->
// freq_max, colored background -> channel accent by band energy.
struct Waterfall {
  std::deque<std::vector<float>> rows; // rows.back() = newest
  int last_analyses = 0;
};
static Waterfall g_wf_a;
static Waterfall g_wf_b;

static void draw_waterfall(mu_Context *ctx, mu_Rect box, const char *title,
                           const std::vector<float> *spectrum, int analyses,
                           Waterfall *wf, double ar, double ag, double ab) {
  const int W = box.w - 4;
  const int H = box.h - 22 - 8; // leave room for the title and the box border
  const mu_Rect plot{box.x + 2, box.y + 22, W, H};

  mu_draw_rect(ctx, box, col_ac(1, 20, 22, 28));
  mu_draw_box(ctx, box, col_ac(1, 58, 64, 84));
  draw_text_at(ctx, title, box.x + 10, box.y + 8, col_ac(1, ar, ag, ab));

  // New analysis since last frame? Stretch its bands into one pixel-wide row.
  if (spectrum != nullptr && spectrum->size() == dglab::kSpectrumBands &&
      wf->last_analyses != analyses) {
    std::vector<float> row(W, 0.0f);
    const int nBands = static_cast<int>(spectrum->size());
    for (int x = 0; x < W; x++) {
      int band = static_cast<int>(static_cast<double>(x) * nBands /
                                  static_cast<double>(W));
      if (band >= nBands) {
        band = nBands - 1;
      }
      row[x] = (*spectrum)[band];
    }
    wf->rows.push_back(std::move(row));
    while (wf->rows.size() > H) {
      wf->rows.pop_front();
    }
    wf->last_analyses = analyses;
  }

  // Quantize energy into a few levels and merge same-level runs into
  // single rect strips (the renderer is a software rect filler).
  const int Q = 12;
  const auto quant = [](float v) {
    int q = static_cast<int>(v * Q);
    return q < 0 ? 0 : (q >= Q ? Q - 1 : q);
  };
  size_t row_i = 0;
  for (auto it = wf->rows.begin(); it != wf->rows.end(); ++it, ++row_i) {
    const int y = plot.y + static_cast<int>(row_i);
    int x = 0;
    while (x < W) {
      const int q = quant((*it)[x]);
      int x1 = x + 1;
      while (x1 < W && quant((*it)[x1]) == q) {
        x1++;
      }
      if (q > 0) {
        const double t = static_cast<double>(q) / static_cast<double>(Q - 1);
        const int cr = static_cast<int>(20 + (ar - 20) * t);
        const int cg = static_cast<int>(22 + (ag - 22) * t);
        const int cb = static_cast<int>(28 + (ab - 28) * t);
        mu_draw_rect(ctx, mu_rect(plot.x + x, y, x1 - x, 1),
                     col_ac(1, cr, cg, cb));
      }
      x = x1;
    }
  }
}

static void draw_visualizer(mu_Context *ctx) {
  const int boxW = 392;
  const int boxH = 156;
  const int boxY = 434;
  const int boxA_X = 4;
  const int boxB_X = 404;

  // Snapshot both channel spectra atomically (the capture thread is the
  // writer).
  std::vector<float> spec_a, spec_b;
  int analyses = 0;
  {
    std::lock_guard<std::mutex> lock(g_audio.spectrum_mutex);
    spec_a = g_audio.left_spectrum;
    spec_b = g_audio.right_spectrum;
    analyses = g_audio.analyses.load();
  }

  draw_waterfall(ctx, mu_rect(boxA_X, boxY, boxW, boxH), "CHANNEL A WATERFALL",
                 spec_a.empty() ? nullptr : &spec_a, analyses, &g_wf_a,
                 kAccentA[0], kAccentA[1], kAccentA[2]);
  draw_waterfall(ctx, mu_rect(boxB_X, boxY, boxW, boxH), "CHANNEL B WATERFALL",
                 spec_b.empty() ? nullptr : &spec_b, analyses, &g_wf_b,
                 kAccentB[0], kAccentB[1], kAccentB[2]);
}

// ---------------------------------------------------------------------------
// Frame construction
// ---------------------------------------------------------------------------
static void build_frame(mu_Context *ctx) {
  mu_begin(ctx);
  mu_layout_width(ctx, 800);
  mu_layout_height(ctx, 600);
  mu_begin_window_ex(ctx, "Coyote", mu_rect(0, 0, 800, 600),
                     MU_OPT_NOFRAME | MU_OPT_NOTITLE | MU_OPT_NOCLOSE |
                         MU_OPT_NORESIZE);

  // ---- 1. Single Top Status Bar (height 30) ----
  int status_widths[4] = {320, 170, 140, 140};
  mu_layout_row(ctx, 4, status_widths, 30);

  char st[64];
  if (g_session->connected()) {
    snprintf(st, sizeof(st), "Status: Coyote connected");
  } else if (g_session->running()) {
    snprintf(st, sizeof(st), "Status: Scanning for Coyote...");
  } else {
    snprintf(st, sizeof(st), "Status: No device found");
  }

  mu_draw_control_text(ctx, st, mu_layout_next(ctx), MU_COLOR_TEXT, 0);

  char bat[32];
  const int battery = g_session->battery();
  snprintf(bat, sizeof(bat), battery >= 0 ? "Battery: %d%%" : "Battery: --",
           battery);
  mu_draw_control_text(ctx, bat, mu_layout_next(ctx), MU_COLOR_TEXT, 0);

  if (mu_button(ctx, "Scan")) {
    g_rescan = true;
  }
  if (mu_button(ctx, "Quit")) {
    g_quit = true;
  }

  // ---- 2. Channel Columns ----
  int column_widths[2] = {395, 405};
  mu_layout_row(ctx, 2, column_widths, 0);

  mu_layout_begin_column(ctx);
  draw_channel(ctx, "CHANNEL A", 0, g_audio.left_level.load(),
               g_audio.left_hz.load(), kAccentA[0], kAccentA[1], kAccentA[2]);
  mu_layout_end_column(ctx);

  mu_layout_begin_column(ctx);
  draw_channel(ctx, "CHANNEL B", 1, g_audio.right_level.load(),
               g_audio.right_hz.load(), kAccentB[0], kAccentB[1], kAccentB[2]);
  mu_layout_end_column(ctx);

  // ---- 3. Global Audio Settings ----
  draw_global_settings(ctx);

  // ---- 4. FFT Waterfalls ----
  draw_visualizer(ctx);

  mu_end_window(ctx);
  mu_end(ctx);
}

// ---------------------------------------------------------------------------
// Text / render callbacks
// ---------------------------------------------------------------------------
static int text_width(mu_Font, const char *text, int len) {
  if (len == -1)
    len = (int)strlen(text);
  return r_get_text_width(text, len);
}
static int text_height(mu_Font) { return r_get_text_height(); }

static std::thread g_session_thread;

// Runs the (re)start of the session on a fresh thread. Only called when the
// previous start() has returned (g_session->running() == false).
static void session_thread_start() {
  if (g_session_thread.joinable()) {
    g_session_thread.join();
  }
  g_session_thread = std::thread([]() {
    if (g_session) {
      (void)g_session->start();
    }
  });
}

int main() {
  setvbuf(stdout, NULL, _IOLBF, 0);
  r_init();

  mu_Context *ctx = new mu_Context;
  mu_init(ctx);
  ctx->text_width = text_width;
  ctx->text_height = text_height;

  // Dark theme.
  ctx->style->colors[MU_COLOR_TEXT] = mu_color(210, 216, 228, 255);
  ctx->style->colors[MU_COLOR_BORDER] = mu_color(58, 64, 84, 255);
  ctx->style->colors[MU_COLOR_WINDOWBG] = mu_color(18, 20, 26, 255);
  ctx->style->colors[MU_COLOR_TITLEBG] = mu_color(30, 34, 46, 255);
  ctx->style->colors[MU_COLOR_TITLETEXT] = mu_color(235, 240, 250, 255);
  ctx->style->colors[MU_COLOR_PANELBG] = mu_color(25, 28, 37, 255);
  ctx->style->colors[MU_COLOR_BUTTON] = mu_color(40, 45, 60, 255);
  ctx->style->colors[MU_COLOR_BUTTONHOVER] = mu_color(62, 70, 95, 255);
  ctx->style->colors[MU_COLOR_BUTTONFOCUS] = mu_color(80, 92, 140, 255);
  // Dark input field background (replaces former solid cyan fill).
  ctx->style->colors[MU_COLOR_BASE] = mu_color(32, 37, 50, 255);

  // -- Live backend: audio capture + BLE streaming session ---------------
  if (!dglab::start_audio_capture(&g_audio, g_settings.freq_min_khz() * 1000.0,
                                  g_settings.freq_max_khz() * 1000.0)) {
    printf("WARN: audio capture is not available; level meters will read 0.\n");
  }
  g_session =
      new dglab::CoyoteSession(&g_audio, &g_settings, session_log, nullptr);
  session_thread_start();

  int mousex = 0, mousey = 0;
  for (;;) {
    // --- input ---
    if (g_quit || r_window_closed())
      break;
    if (r_mouse_moved(&mousex, &mousey)) {
      mu_input_mousemove(ctx, mousex, mousey);
    }
    if (r_mouse_down()) {
      mu_input_mousedown(ctx, mousex, mousey, MU_MOUSE_LEFT);
    } else if (r_mouse_up()) {
      mu_input_mouseup(ctx, mousex, mousey, MU_MOUSE_LEFT);
    }

    // --- rescan: wait for the previous start() to end, then restart ---
    if (g_rescan && !g_session->running()) {
      g_rescan = false;
      session_thread_start();
    }

    g_t = static_cast<double>(r_get_time()) / 1000.0;

    // --- build + render ---
    build_frame(ctx);
    r_clear(mu_color(18, 20, 26, 255));
    mu_Command *cmd = NULL;
    while (mu_next_command(ctx, &cmd)) {
      switch (cmd->type) {
      case MU_COMMAND_TEXT:
        r_draw_text(cmd->text.str, cmd->text.pos, cmd->text.color);
        break;
      case MU_COMMAND_RECT:
        r_draw_rect(cmd->rect.rect, cmd->rect.color);
        break;
      case MU_COMMAND_ICON:
        r_draw_icon(cmd->icon.id, cmd->icon.rect, cmd->icon.color);
        break;
      case MU_COMMAND_CLIP:
        r_set_clip_rect(cmd->clip.rect);
        break;
      }
    }
    r_present(); // flush + fenster event loop (blocks until window closed)
  }

  // -- Shutdown: stop the session, stop the capture, free the UI ---------
  if (g_session) {
    g_session->stop();
  }
  if (g_session_thread.joinable()) {
    g_session_thread.join();
  }
  delete g_session;
  g_session = nullptr;
  dglab::stop_audio_capture(&g_audio);
  delete ctx;
  r_exit();
  return 0;
}
