/*
 * ============================================================================
 * DIY FLIR-STYLE THERMAL CAMERA — v16
 * ============================================================================
 * MCU      : DFRobot FireBeetle 2 ESP32-S3-U (DFR0975-U), N16R8
 * Display  : DFRobot Fermion 3.5" 480x320 ILI9488 + GT911 touch (DFR0669)
 * Camera   : OV3660 over DVP (replaced OV2640), QVGA RGB565 (with digital crop-zoom)
 * Thermal  : Melexis MLX90640 32x24, on dedicated Wire1 @ 1 MHz
 *
 * Init order (every step is load-bearing — don't reorder casually):
 *   1. Wire.begin(1, 2) @ 100 kHz — shared bus for AXP / GT911 / SCCB.
 *   2. AXP313A enable camera power, then 500 ms.
 *   3. esp_camera_init BEFORE lcd.init (LCD touches Wire internals).
 *      Camera shares Wire's I2C driver via pin_sccb_*=-1, sccb_i2c_port=0.
 *   4. GT911 manual reset pulse: INT low 11 ms → input → 50 ms (selects 0x5D).
 *   5. lcd.init, then apply the saved landscape/portrait orientation.
 *   6. Wire1.begin(11, 14) @ 1 MHz, MLX init.
 *
 * Pinout — verified, do not change without re-verifying GDI ribbon:
 *   GDI    : SCLK=17 MOSI=15 MISO=16 DC=3 RST=38 CS=18 BL=21
 *            TOUCH SDA=1 SCL=2 INT=13   (TCS=GPIO12 is on the ribbon — avoid!)
 *   CAMERA : XCLK=45 SIOD=1 SIOC=2 D0..D7=39,40,41,4,7,8,46,48
 *            VSYNC=6 HREF=42 PCLK=5
 *   MLX    : SDA=11 SCL=14   (Wire1; NOT 12 — that's GDI's TCS line)
 *   BUTTON : GPIO10, INPUT_PULLUP, NO to GND
 *
 * MLX90640_I2C_Driver.cpp must be patched: every "Wire." → "Wire1.".
 *
 * Byte-order rules (this was THE bug in v8 — semantics flipped):
 *   LovyanGFX setSwapBytes:
 *     true  → buffer is treated as rgb565_t  (native uint16_t, R in high bits)
 *     false → buffer is treated as swap565_t (byte-swapped / wire format)
 *   OV2640 RGB565 frame buffer is wire format (byte0 = R+G_hi).
 *   palette_native[] is native uint16_t for UI drawing. Render chunk_buf is
 *   written in wire-format (swap565) so pushImage() can use the DMA fast path
 *   with setSwapBytes(false).
 *   So:
 *     pushImage(camera_buf)   -> setSwapBytes(false)
 *     pushImage(chunk_buf)    -> setSwapBytes(false)
 *     drawFastHLine(palette)  -> palette_native[] / normal colour APIs
 *   v8 had ALL THREE inverted, which is why everything was green/cyan/purple.
 *
 * Camera tearing mitigation:
 *   fb_count = 3, grab_mode = LATEST, XCLK = 16 MHz, snapshot to our own
 *   PSRAM buffer immediately and esp_camera_fb_return() before rendering, so
 *   the camera DMA never has to wait on the LCD push.
 *
 * Known unsolved: SD card on shared SPI kills the LCD. SD storage is disabled;
 * use the freeze export or browser portal instead.
 *
 * Build: ESP32 Arduino core 2.x, "DFRobot FireBeetle 2 ESP32-S3", USB CDC On
 *        Boot, 16MB Flash, partition 16M (3MB APP / 9.9MB FATFS), OPI PSRAM.
 * Libs : LovyanGFX, DFRobot_AXP313A, Melexis MLX90640_API (with Wire1 patch).
 * ============================================================================
 */

#define LGFX_USE_V1
#include <SPI.h>
#include <LovyanGFX.hpp>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "esp_camera.h"
#include "esp_err.h"
#include "img_converters.h"
#include "DFRobot_AXP313A.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include <algorithm>     // std::nth_element for AUTO-range percentile clipping

// Smooth display interpolation. The upstream frame is kept edge-aware; this
// just avoids turning every MLX cell into an obvious display block.
#define USE_BILINEAR_THERMAL 1

// Experimental: reinitialize the camera as native JPEG while the browser
// portal is active. Kept off by default until verified on the actual unit.
#ifndef STREAM_CAMERA_NATIVE_JPEG
#define STREAM_CAMERA_NATIVE_JPEG 0
#endif
#ifndef STREAM_CAMERA_CHUNKED_JPEG
#define STREAM_CAMERA_CHUNKED_JPEG 0
#endif

// ---------------------------------------------------------------- Pins ----
#define TFT_SCLK 17
#define TFT_MOSI 15
#define TFT_MISO 16
#define TFT_DC    3
#define TFT_RST  38
#define TFT_CS   18
#define TFT_BL   21
#define TOUCH_SDA 1
#define TOUCH_SCL 2
#define TOUCH_INT 13
#define BUTTON_PIN 10
#define MLX_SDA  11
#define MLX_SCL  14

// LCD SPI write clock. 60 MHz is an overclock for many ILI9488 boards, but
// this is a contained performance test knob: set back to 40000000UL if the
// display shows speckles, wrong colours, or intermittent blanking.
// ILI9488 published spec is 20 MHz max SPI write; in practice 40 MHz is the
// known-safe sweet spot, 60 MHz produced random-colored full-width lines
// (bit-flips on the wire). 50 MHz is a midpoint we're testing — if you see
// any random-color lines reappear, drop back to 40 MHz; if it's clean, you
// can push higher (55, then back toward 60) to find this panel's actual
// ceiling.
static constexpr uint32_t TFT_WRITE_FREQ_HZ = 50000000UL;

#define XCLK_GPIO_NUM 45
#define SIOD_GPIO_NUM  1
#define SIOC_GPIO_NUM  2
#define Y9_GPIO_NUM   48
#define Y8_GPIO_NUM   46
#define Y7_GPIO_NUM    8
#define Y6_GPIO_NUM    7
#define Y5_GPIO_NUM    4
#define Y4_GPIO_NUM   41
#define Y3_GPIO_NUM   40
#define Y2_GPIO_NUM   39
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 42
#define PCLK_GPIO_NUM  5

// ----------------------------------------------------------- LovyanGFX ----
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9488 _panel;
  lgfx::Bus_SPI _bus;
  lgfx::Light_PWM _light;
public:
  LGFX(void) {
    { auto cfg = _bus.config();
      cfg.spi_host=SPI2_HOST; cfg.spi_mode=0;
      cfg.freq_write=TFT_WRITE_FREQ_HZ; cfg.freq_read=16000000;
      cfg.spi_3wire=false; cfg.use_lock=true;
      cfg.dma_channel=SPI_DMA_CH_AUTO;
      cfg.pin_sclk=TFT_SCLK; cfg.pin_mosi=TFT_MOSI;
      cfg.pin_miso=TFT_MISO; cfg.pin_dc=TFT_DC;
      _bus.config(cfg); _panel.setBus(&_bus);
    }
    { auto cfg=_panel.config();
      cfg.pin_cs=TFT_CS; cfg.pin_rst=TFT_RST; cfg.pin_busy=-1;
      cfg.panel_width=320; cfg.panel_height=480;
      cfg.bus_shared=true;
      _panel.config(cfg);
    }
    { auto cfg=_light.config();
      cfg.pin_bl=TFT_BL; cfg.freq=12000; cfg.pwm_channel=7;
      _light.config(cfg); _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};

LGFX lcd;
DFRobot_AXP313A axp;

// ---------------------------------------------------------------- Layout --
static int IMG_X = 24;
static int IMG_Y = 40;
static constexpr int IMG_W = 320;
static constexpr int IMG_H = 240;
static constexpr int CROSS_CX = IMG_W / 2;
static constexpr int CROSS_CY = IMG_H / 2;
static constexpr int CROSS_R  = 10;

static inline bool isCrosshairPixel(int x, int y) {
  bool in_x = (x >= CROSS_CX - CROSS_R && x <= CROSS_CX + CROSS_R);
  bool in_y = (y >= CROSS_CY - CROSS_R && y <= CROSS_CY + CROSS_R);
  return (y == CROSS_CY && in_x) ||
         (x == CROSS_CX && in_y) ||
         ((y == CROSS_CY - CROSS_R || y == CROSS_CY + CROSS_R) && in_x) ||
         ((x == CROSS_CX - CROSS_R || x == CROSS_CX + CROSS_R) && in_y);
}

// ---------------------------------------------------------------- MLX ----
const byte MLX_ADDR = 0x33;
#define TA_SHIFT 8
paramsMLX90640 mlx_params;
// Quality-first acquisition. 0x07/64 Hz is right at the 1 MHz I2C read-time
// limit for an 832-word MLX RAM dump, so a new subpage can land while the old
// one is still being read. That creates incoherent chess noise that no palette
// or smoothing pass can fix. 0x05 = 16 subpages/s = 8 complete frames/s.
static constexpr uint8_t  MLX_REFRESH_RATE_CODE = 0x05;
static constexpr uint8_t  MLX_ADC_RESOLUTION_CODE = 0x03; // 19-bit, lowest sensor noise
static constexpr uint32_t MLX_POLL_MS = 3;               // cheap data-ready check cadence
// Filtering knobs. Conservative tuning for "see fingers vs palm clearly":
//
// Temporal IIR — was alpha=0.16 / fast=0.82 / delta=1.0 °C. That made
// sub-1 °C changes (which is the ENTIRE useful skin signal) take ~6 frames
// to register. Now alpha=0.45 / delta=0.4: skin variations cross the "fast"
// threshold quickly and small changes still update meaningfully each frame.
static constexpr float MLX_TEMP_ALPHA_STILL = 0.45f;     // small changes still respond fast
static constexpr float MLX_TEMP_ALPHA_FAST  = 0.85f;     // real motion: catch up almost instantly
static constexpr float MLX_TEMP_FAST_DELTA_C = 0.40f;    // engage fast mode at half-a-degree

// Spatial — was 0.70 °C, which is wider than the typical temperature
// difference *within* a hand. So fingers (33 °C) and palm (32.5 °C) were
// being averaged together → fingers vanished. 0.0 disables the spatial pass
// entirely (neighbours only count if exactly equal, which essentially never).
// Try 0.20 if cell-level noise is too pronounced — that smooths within-noise
// neighbours but preserves real edges between fingers and palm.
static constexpr float MLX_SPATIAL_EDGE_C = 0.0f;        // 0 = disabled

// Chess subpages can have a small relative bias from CP/TGC noise and timing.
// Correct only the global checkerboard offset; do not blur local edges.
static constexpr bool  MLX_BALANCE_CHESS_SUBPAGES = true;
static constexpr float MLX_CHESS_BALANCE_MAX_C = 1.50f;  // ignore implausible scene-driven medians

// Auto-range — was using raw min/max which means one outlier cell stretches
// the whole palette. Switch to percentile clipping (see updateAutoDisplayRange).
static constexpr float MLX_AUTO_MIN_SPAN_C = 2.50f;      // prevent auto-range from magnifying noise
static constexpr float MLX_RANGE_ALPHA_EXPAND = 0.70f;   // new hot/cold enters range fast
static constexpr float MLX_RANGE_ALPHA_SHRINK = 0.30f;   // contraction speed (was 0.18, too slow)
// Trim the hottest/coldest pixels when computing AUTO range — kills outliers
// (one hot bulb in the frame) so most of the palette is spent on the bulk
// of the scene where you actually want to see contrast.
static constexpr int   MLX_AUTO_TRIM_LO_PCT = 5;         // ignore coldest 5% of pixels
static constexpr int   MLX_AUTO_TRIM_HI_PCT = 5;         // ignore hottest 5% of pixels

// mlx_temps is the chess-mode accumulator: each MLX90640_CalculateTo() call
// only writes the half of the array matching the current subpage (the other
// half retains the previous-subpage values from the prior MLX subpage). Rendering
// straight from this buffer makes 1-2-cell-wide features (fingers!) flicker
// in and out, because they often land entirely on cells of one subpage.
//
// mlx_temps_full is the rendering buffer: we only promote mlx_temps into it
// *after* both subpages have been collected this cycle. Render code reads
// from here so it always sees a complete frame. This is exactly what the
// old TFT_eSPI sketch did with `for (byte x=0; x<2; x++)` reading both
// subpages back-to-back before rendering. v15 lost that and v16 brings it
// back.
float mlx_temps[768];          // chess-pattern accumulator
float mlx_temps_full[768];     // last complete frame — what render code reads
float mlx_publish[768];        // repaired/ranged frame staged before publishing
float mlx_filtered[768];       // adaptive temporal denoise state
float mlx_spatial[768];        // edge-aware spatial denoise scratch
bool  mlx_filter_valid = false;
float mlx_range_lo = 20.0f, mlx_range_hi = 30.0f;
bool  mlx_range_valid = false;
uint8_t mlx_subpage_mask = 0;  // bit0=subpage0 seen, bit1=subpage1 seen
bool   mlx_full_ready = false; // becomes true on first complete frame
portMUX_TYPE mlx_mux = portMUX_INITIALIZER_UNLOCKED;

#define MLX_READ_FAIL  0
#define MLX_READ_HALF  1
#define MLX_READ_FULL  2
#define MLX_FRAME_NOT_READY -7
#define MLX_FRAME_STALE     -8

float t_min = 20.0f, t_max = 30.0f, t_center = 25.0f;
float t_range_min = 20.0f, t_range_max = 30.0f;
uint32_t last_mlx_ms = 0;
volatile uint32_t thermal_frame_seq = 0; // increments after mlx_temps_full is repaired/ranged
TaskHandle_t mlx_task_handle = nullptr;
extern volatile bool freeze_mode;

volatile uint32_t mlx_diag_not_ready = 0;
volatile uint32_t mlx_diag_i2c_fail = 0;
volatile uint32_t mlx_diag_stale = 0;
volatile uint32_t mlx_diag_ok_subpages = 0;
volatile uint32_t mlx_diag_duplicate_subpages = 0;
volatile uint32_t mlx_diag_full_frames = 0;
volatile uint32_t mlx_diag_read_us_last = 0;
volatile uint32_t mlx_diag_read_us_max = 0;
volatile float mlx_diag_chess_bias_last = 0.0f;

// Forward declarations for the bits of the State section that analyzeMLX
// needs to consult. Full definitions live further down (~line 670). Kept
// here because Arduino doesn't auto-generate forward decls for variables
// or enums, only for free functions.
enum RangeMode { RANGE_AUTO=0, RANGE_AUTO2, RANGE_SKIN, RANGE_BODY,
                 RANGE_WARM, RANGE_HOT, RANGE_VHOT, RANGE_MANUAL, RANGE_COUNT };
extern volatile RangeMode range_mode;
static constexpr float AUTO2_AMBIENT_DELTA = 1.5f;
static constexpr int   AUTO2_MIN_SUBJECT_PIXELS = 8;
static constexpr float AUTO2_PAD_C = 0.3f;

bool initMLX() {
  Wire1.begin(MLX_SDA, MLX_SCL);
  Wire1.setClock(1000000UL);
  Wire1.beginTransmission(MLX_ADDR);
  if (Wire1.endTransmission() != 0) return false;
  uint16_t ee[832];
  if (MLX90640_DumpEE(MLX_ADDR, ee) != 0) return false;
  if (MLX90640_ExtractParameters(ee, &mlx_params) != 0) return false;
  MLX90640_I2CWrite(MLX_ADDR, 0x800D, 6401);
  MLX90640_SetResolution(MLX_ADDR, MLX_ADC_RESOLUTION_CODE);
  MLX90640_SetRefreshRate(MLX_ADDR, MLX_REFRESH_RATE_CODE);
  MLX90640_SetChessMode(MLX_ADDR);
  return true;
}

// Reads ONE chess subpage. Returns:
//   MLX_READ_FAIL — sensor I2C failure
//   MLX_READ_HALF — no complete frame yet (no ready subpage, or one stale half)
//   MLX_READ_FULL — both subpages have been collected this cycle; analyzeMLX()
//                   can now repair/range and publish a new render frame.
//
// The stock Melexis MLX90640_GetFrameData() loop is conservative: if another
// subpage becomes ready while it is reading the current one, it discards the
// just-read RAM and tries again up to five times. At high refresh rates the
// read itself can take about a subpage period, so that policy can burn most of
// the frame budget chasing "newer" data. For display use, one coherent subpage
// is better than five discarded attempts.
// Despite the historical "Fast" name, this now rejects RAM reads that overlap
// a new subpage. Coherent data matters more than chasing the max refresh rate.
int readMLXFrameDataFast(uint16_t *frameData) {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    uint16_t statusRegister;
    uint16_t statusAfter;
    uint16_t controlRegister1;
    int error = MLX90640_I2CRead(MLX_ADDR, 0x8000, 1, &statusRegister);
    if (error != 0) {
      mlx_diag_i2c_fail++;
      return error;
    }
    if (!(statusRegister & 0x0008)) {
      mlx_diag_not_ready++;
      return MLX_FRAME_NOT_READY;
    }

    uint32_t read_start_us = micros();
    error = MLX90640_I2CWrite(MLX_ADDR, 0x8000, 0x0030);
    if (error == -1) {
      mlx_diag_i2c_fail++;
      return error; // Match Melexis: ignore verify mismatch (-2).
    }

    error = MLX90640_I2CRead(MLX_ADDR, 0x0400, 832, frameData);
    if (error != 0) {
      mlx_diag_i2c_fail++;
      return error;
    }

    error = MLX90640_I2CRead(MLX_ADDR, 0x8000, 1, &statusAfter);
    if (error != 0) {
      mlx_diag_i2c_fail++;
      return error;
    }

    uint32_t read_us = micros() - read_start_us;
    mlx_diag_read_us_last = read_us;
    if (read_us > mlx_diag_read_us_max) mlx_diag_read_us_max = read_us;

    // A new subpage completed during the RAM burst. Discard this read rather
    // than calculating temperatures from a possibly mixed old/new RAM image.
    if (statusAfter & 0x0008) {
      mlx_diag_stale++;
      continue;
    }

    error = MLX90640_I2CRead(MLX_ADDR, 0x800D, 1, &controlRegister1);
    if (error != 0) {
      mlx_diag_i2c_fail++;
      return error;
    }

    frameData[832] = controlRegister1;
    frameData[833] = statusRegister & 0x0001;
    mlx_diag_ok_subpages++;
    return frameData[833];
  }

  return MLX_FRAME_STALE;
}

int readMLXSubpage() {
  uint16_t raw[834];
  int frame_status = readMLXFrameDataFast(raw);
  if (frame_status == MLX_FRAME_NOT_READY) return MLX_READ_HALF;
  if (frame_status == MLX_FRAME_STALE) {
    mlx_subpage_mask = 0;
    return MLX_READ_HALF;
  }
  if (frame_status < 0) {
    mlx_subpage_mask = 0;
    return MLX_READ_FAIL;
  }
  int sp = raw[833] & 0x01;          // which subpage this frame is — Melexis stores it at index 833
  float Ta = MLX90640_GetTa(raw, &mlx_params);
  float tr = Ta - TA_SHIFT;
  MLX90640_CalculateTo(raw, &mlx_params, 0.95f, tr, mlx_temps);
  if (mlx_subpage_mask & (uint8_t)(1 << sp)) mlx_diag_duplicate_subpages++;
  mlx_subpage_mask |= (uint8_t)(1 << sp);
  if ((mlx_subpage_mask & 0x03) == 0x03) {
    mlx_subpage_mask = 0;
    mlx_diag_full_frames++;
    return MLX_READ_FULL;
  }
  return MLX_READ_HALF;
}

bool mlxDataReady() {
  uint16_t statusRegister;
  return MLX90640_I2CRead(MLX_ADDR, 0x8000, 1, &statusRegister) == 0 &&
         (statusRegister & 0x0008);
}

static inline float absf_fast(float v) {
  return v < 0.0f ? -v : v;
}

static inline float median4(float a, float b, float c, float d) {
  if (a > b) { float t = a; a = b; b = t; }
  if (c > d) { float t = c; c = d; d = t; }
  if (a > c) { float t = a; a = c; c = t; }
  if (b > d) { float t = b; b = d; d = t; }
  return 0.5f * ((b < c) ? b : c) + 0.5f * ((b > c) ? b : c);
}

void correctOneBadPixel(uint16_t pixel, float *t) {
  if (pixel >= 768 || pixel == 0xFFFF) return;

  int row = pixel / 32;
  int col = pixel & 31;
  float corrected;

  // Chess mode neighbours of the same subpage live on the diagonals. This is
  // the important bit for small features: don't smear a bad cell horizontally
  // into the opposite chess subpage.
  if (row == 0) {
    if (col == 0) corrected = t[pixel + 33];
    else if (col == 31) corrected = t[pixel + 31];
    else corrected = 0.5f * (t[pixel + 31] + t[pixel + 33]);
  } else if (row == 23) {
    if (col == 0) corrected = t[pixel - 31];
    else if (col == 31) corrected = t[pixel - 33];
    else corrected = 0.5f * (t[pixel - 33] + t[pixel - 31]);
  } else if (col == 0) {
    corrected = 0.5f * (t[pixel - 31] + t[pixel + 33]);
  } else if (col == 31) {
    corrected = 0.5f * (t[pixel - 33] + t[pixel + 31]);
  } else {
    corrected = median4(t[pixel - 33], t[pixel - 31],
                        t[pixel + 31], t[pixel + 33]);
  }

  if (corrected >= -40.0f && corrected <= 300.0f) t[pixel] = corrected;
}

void correctEEPROMBadPixels(float *t) {
  for (int i = 0; i < 5; i++) {
    correctOneBadPixel(mlx_params.brokenPixels[i], t);
    correctOneBadPixel(mlx_params.outlierPixels[i], t);
  }
}

int countPixelList(const uint16_t *pixels) {
  int count = 0;
  for (int i = 0; i < 5; i++) {
    if (pixels[i] != 0xFFFF && pixels[i] < 768) count++;
  }
  return count;
}

void balanceChessSubpages(float *t) {
  if (!MLX_BALANCE_CHESS_SUBPAGES) return;

  static float parity0[384];
  static float parity1[384];
  int n0 = 0, n1 = 0;

  for (int row = 0; row < 24; row++) {
    for (int col = 0; col < 32; col++) {
      int idx = row * 32 + col;
      float v = t[idx];
      if (!(v >= -40.0f && v <= 300.0f)) continue;
      if ((row ^ col) & 1) parity1[n1++] = v;
      else                 parity0[n0++] = v;
    }
  }

  if (n0 < 64 || n1 < 64) return;

  std::nth_element(parity0, parity0 + n0 / 2, parity0 + n0);
  std::nth_element(parity1, parity1 + n1 / 2, parity1 + n1);
  float bias = parity0[n0 / 2] - parity1[n1 / 2];
  mlx_diag_chess_bias_last = bias;

  if (absf_fast(bias) > MLX_CHESS_BALANCE_MAX_C) return;

  float half = 0.5f * bias;
  for (int row = 0; row < 24; row++) {
    for (int col = 0; col < 32; col++) {
      int idx = row * 32 + col;
      if ((row ^ col) & 1) t[idx] += half;
      else                 t[idx] -= half;
    }
  }
}

void denoiseMLXFrame(float *t) {
  if (!mlx_filter_valid) {
    memcpy(mlx_filtered, t, sizeof(mlx_filtered));
    mlx_filter_valid = true;
  } else {
    for (int i = 0; i < 768; i++) {
      float prev = mlx_filtered[i];
      float delta = t[i] - prev;
      float ad = absf_fast(delta);
      float alpha = MLX_TEMP_ALPHA_STILL;
      if (ad >= MLX_TEMP_FAST_DELTA_C) {
        alpha = MLX_TEMP_ALPHA_FAST;
      } else {
        float ramp = ad / MLX_TEMP_FAST_DELTA_C;
        alpha = MLX_TEMP_ALPHA_STILL +
                (MLX_TEMP_ALPHA_FAST - MLX_TEMP_ALPHA_STILL) * ramp;
      }
      float filtered = prev + delta * alpha;
      mlx_filtered[i] = filtered;
      t[i] = filtered;
    }
  }

  // Light edge-aware spatial pass. Similar neighbours average out cell noise;
  // strong thermal edges are left alone so fingers/tools don't smear badly.
  for (int row = 0; row < 24; row++) {
    for (int col = 0; col < 32; col++) {
      int idx = row * 32 + col;
      float c = t[idx];
      float acc = c * 4.0f;
      float weight = 4.0f;

      if (row > 0) {
        float n = t[idx - 32];
        if (absf_fast(n - c) <= MLX_SPATIAL_EDGE_C) { acc += n; weight += 1.0f; }
      }
      if (row < 23) {
        float n = t[idx + 32];
        if (absf_fast(n - c) <= MLX_SPATIAL_EDGE_C) { acc += n; weight += 1.0f; }
      }
      if (col > 0) {
        float n = t[idx - 1];
        if (absf_fast(n - c) <= MLX_SPATIAL_EDGE_C) { acc += n; weight += 1.0f; }
      }
      if (col < 31) {
        float n = t[idx + 1];
        if (absf_fast(n - c) <= MLX_SPATIAL_EDGE_C) { acc += n; weight += 1.0f; }
      }
      mlx_spatial[idx] = acc / weight;
    }
  }
  memcpy(t, mlx_spatial, sizeof(mlx_spatial));
}

void updateAutoDisplayRange(float mn, float mx, float &lo, float &hi) {
  float target_lo = mn;
  float target_hi = mx;
  float span = target_hi - target_lo;
  if (span < MLX_AUTO_MIN_SPAN_C) {
    float mid = 0.5f * (target_lo + target_hi);
    target_lo = mid - 0.5f * MLX_AUTO_MIN_SPAN_C;
    target_hi = mid + 0.5f * MLX_AUTO_MIN_SPAN_C;
  }

  if (!mlx_range_valid) {
    mlx_range_lo = target_lo;
    mlx_range_hi = target_hi;
    mlx_range_valid = true;
  } else {
    float lo_alpha = (target_lo < mlx_range_lo) ? MLX_RANGE_ALPHA_EXPAND : MLX_RANGE_ALPHA_SHRINK;
    float hi_alpha = (target_hi > mlx_range_hi) ? MLX_RANGE_ALPHA_EXPAND : MLX_RANGE_ALPHA_SHRINK;
    mlx_range_lo += (target_lo - mlx_range_lo) * lo_alpha;
    mlx_range_hi += (target_hi - mlx_range_hi) * hi_alpha;
  }

  lo = mlx_range_lo;
  hi = mlx_range_hi;
}

// Repair bad pixels, range-find, recompute centre temperature, then publish.
// Operates on a staged copy of the complete chess accumulator. Borrowed from
// the old TFT_eSPI sketch: any cell whose temperature is impossible (<-40 °C
// or >300 °C) is replaced with its left neighbour, which kills isolated dead
// pixels far better than just clipping them.
void analyzeMLX() {
  memcpy(mlx_publish, mlx_temps, sizeof(mlx_publish));
  float *t = mlx_publish;

  correctEEPROMBadPixels(t);
  balanceChessSubpages(t);

  // Two known-bad cells on this specific MLX unit.
  t[1*32 + 21] = 0.5f * (t[1*32 + 20] + t[1*32 + 22]);
  t[4*32 + 30] = 0.5f * (t[4*32 + 29] + t[4*32 + 31]);

  // Generic dead-pixel substitution from neighbour. Sweeps L→R so a run of
  // bad pixels still gets reasonable values from the last good cell.
  // CRITICAL: also catches NaN. The Melexis math can produce NaN when a
  // pixel is saturated or a calibration term divides by zero, and NaN
  // compares FALSE against everything, so a plain `t[i] < -40 || t[i] > 300`
  // would silently let it through. A poisoned NaN cell then crashed
  // buildThermalPaletteFrame because (int)(NaN*255+0.5f) saturates to
  // INT_MAX and the palette lookup ran off the end of the array. Use
  // `!(t[i] >= -40 && t[i] <= 300)` because that condition is TRUE for NaN.
  if (!(t[0] >= -40 && t[0] <= 300)) t[0] = t[1];
  for (int i = 1; i < 768; i++) {
    if (!(t[i] >= -40 && t[i] <= 300)) t[i] = t[i-1];
  }

  denoiseMLXFrame(t);

  // Compute true min/max for the side-palette labels — those should reflect
  // actual scene extremes — AND the trimmed min/max that AUTO range uses.
  // The trimmed pair excludes the coldest MLX_AUTO_TRIM_LO_PCT% and hottest
  // MLX_AUTO_TRIM_HI_PCT% of pixels so a single bright outlier (lamp, hot
  // mug) doesn't stretch the palette and crush the rest of the scene.
  static float scratch[768];
  memcpy(scratch, t, sizeof(scratch));
  const int trim_lo_idx = (768 * MLX_AUTO_TRIM_LO_PCT) / 100;
  const int trim_hi_idx = 768 - 1 - (768 * MLX_AUTO_TRIM_HI_PCT) / 100;
  // nth_element partial-sorts in O(n) — far cheaper than a full sort.
  std::nth_element(scratch, scratch + trim_lo_idx, scratch + 768);
  float mn_trimmed = scratch[trim_lo_idx];
  std::nth_element(scratch + trim_lo_idx + 1,
                   scratch + trim_hi_idx,
                   scratch + 768);
  float mx_trimmed = scratch[trim_hi_idx];

  // True min/max for the labels (still useful to know if scene has hot/cold
  // outliers). Doing this from the partially-sorted scratch is free since
  // we know the smallest and largest live in the unsorted ends.
  float mn = scratch[0], mx = scratch[767];
  for (int i = 1; i < trim_lo_idx; i++) if (scratch[i] < mn) mn = scratch[i];
  for (int i = trim_hi_idx + 1; i < 767; i++) if (scratch[i] > mx) mx = scratch[i];

  // Pick the AUTO target. AUTO2 is "subject-focused": find the median
  // (an estimate of ambient/room), throw away pixels close to ambient,
  // and use only the leftover "interesting" pixels' min/max as the target.
  // If too few pixels qualify (uniform scene), fall back to plain AUTO.
  float auto_target_lo = mn_trimmed;
  float auto_target_hi = mx_trimmed;
  if (range_mode == RANGE_AUTO2) {
    // Median lives at the partition we already established with the trimmed
    // partial sorts — but to be precise, do one more nth_element at index 384.
    std::nth_element(scratch, scratch + 384, scratch + 768);
    float median = scratch[384];
    float sig_lo = 1000.0f, sig_hi = -1000.0f;
    int sig_count = 0;
    for (int i = 0; i < 768; i++) {
      float dt = t[i] - median;
      if (dt < 0) dt = -dt;
      if (dt >= AUTO2_AMBIENT_DELTA) {
        if (t[i] < sig_lo) sig_lo = t[i];
        if (t[i] > sig_hi) sig_hi = t[i];
        sig_count++;
      }
    }
    if (sig_count >= AUTO2_MIN_SUBJECT_PIXELS) {
      // Pad the range slightly so the most extreme subject pixel doesn't
      // sit exactly at the palette edge — gives a smoother visual.
      auto_target_lo = sig_lo - AUTO2_PAD_C;
      auto_target_hi = sig_hi + AUTO2_PAD_C;
    }
    // else: fall back to mn_trimmed/mx_trimmed already in auto_target_*.
  }

  float center = (t[11*32 + 15] + t[11*32 + 16] +
                  t[12*32 + 15] + t[12*32 + 16]) * 0.25f;
  float range_lo, range_hi;
  updateAutoDisplayRange(auto_target_lo, auto_target_hi, range_lo, range_hi);

  portENTER_CRITICAL(&mlx_mux);
  memcpy(mlx_temps_full, mlx_publish, sizeof(mlx_temps_full));
  t_min = mn; t_max = mx; t_center = center;
  t_range_min = range_lo; t_range_max = range_hi;
  mlx_full_ready = true;
  thermal_frame_seq++;
  portEXIT_CRITICAL(&mlx_mux);
}

void mlxTask(void *arg) {
  (void)arg;
  for (;;) {
    if (!freeze_mode) {
      if (readMLXSubpage() == MLX_READ_FULL) analyzeMLX();
      vTaskDelay(pdMS_TO_TICKS(MLX_POLL_MS));
    } else {
      vTaskDelay(pdMS_TO_TICKS(25));
    }
  }
}

// ---------------------------------------------------------------- Camera --
extern volatile int8_t cam_brightness;
uint8_t *cam_snapshot = nullptr;   // PSRAM, our copy of the latest frame
size_t   cam_snapshot_len = 0;
bool     cam_have_frame = false;
bool     cam_ok = false;           // false => camera disconnected or init failed
bool     camera_driver_active = false;
bool     stream_native_jpeg_active = false;

enum CameraFailStage : uint8_t {
  CAM_STAGE_NOT_STARTED = 0,
  CAM_STAGE_OK,
  CAM_STAGE_NO_PSRAM,
  CAM_STAGE_SNAPSHOT_ALLOC_FAIL,
  CAM_STAGE_INIT_FAIL,
  CAM_STAGE_NO_SENSOR,
  CAM_STAGE_NO_FRAME,
  CAM_STAGE_FRAME_LEN_MISMATCH
};

CameraFailStage cam_fail_stage = CAM_STAGE_NOT_STARTED;
esp_err_t cam_last_err = ESP_OK;
int cam_sensor_pid = -1;
uint32_t cam_init_attempts = 0;
uint32_t cam_frame_fail_count = 0;
uint32_t cam_frame_len_mismatch_count = 0;
uint32_t cam_last_frame_log_ms = 0;
size_t cam_last_frame_len = 0;
bool cam_psram_found = false;
uint32_t cam_psram_size = 0;
uint32_t cam_psram_free = 0;

const char *cameraStageName(uint8_t stage) {
  switch (stage) {
    case CAM_STAGE_OK: return "OK";
    case CAM_STAGE_NO_PSRAM: return "NO_PSRAM";
    case CAM_STAGE_SNAPSHOT_ALLOC_FAIL: return "SNAPSHOT_ALLOC_FAIL";
    case CAM_STAGE_INIT_FAIL: return "ESP_CAMERA_INIT_FAIL";
    case CAM_STAGE_NO_SENSOR: return "NO_SENSOR";
    case CAM_STAGE_NO_FRAME: return "NO_FRAME";
    case CAM_STAGE_FRAME_LEN_MISMATCH: return "FRAME_LEN_MISMATCH";
    default: return "NOT_STARTED";
  }
}

const char *cameraDisplayMessage() {
  switch (cam_fail_stage) {
    case CAM_STAGE_NO_PSRAM: return "NO PSRAM";
    case CAM_STAGE_SNAPSHOT_ALLOC_FAIL: return "PSRAM ALLOC FAIL";
    case CAM_STAGE_INIT_FAIL: return "CAM INIT FAIL";
    case CAM_STAGE_NO_SENSOR: return "NO SENSOR";
    case CAM_STAGE_FRAME_LEN_MISMATCH: return "FRAME BAD LEN";
    case CAM_STAGE_NO_FRAME: return "NO FRAME";
    default: return "NO CAMERA";
  }
}

const char *cameraErrName() {
  return cam_last_err == ESP_OK ? "OK" : esp_err_to_name(cam_last_err);
}

void updateCameraMemoryDiagnostics() {
  cam_psram_found = psramFound();
  cam_psram_size = ESP.getPsramSize();
  cam_psram_free = ESP.getFreePsram();
}

bool ensureCameraSnapshotBuffer() {
  cam_snapshot_len = IMG_W * IMG_H * 2;
  updateCameraMemoryDiagnostics();
  if (!cam_psram_found) {
    cam_fail_stage = CAM_STAGE_NO_PSRAM;
    Serial.printf("Camera: PSRAM not found; expected %u-byte snapshot buffer\n",
                  (unsigned)cam_snapshot_len);
    return false;
  }
  if (!cam_snapshot) {
    cam_snapshot = (uint8_t*)heap_caps_malloc(cam_snapshot_len, MALLOC_CAP_SPIRAM);
    if (!cam_snapshot) {
      updateCameraMemoryDiagnostics();
      cam_fail_stage = CAM_STAGE_SNAPSHOT_ALLOC_FAIL;
      Serial.printf("Camera: PSRAM snapshot allocation failed (%u bytes), free PSRAM=%lu\n",
                    (unsigned)cam_snapshot_len,
                    (unsigned long)cam_psram_free);
      return false;
    }
    memset(cam_snapshot, 0, cam_snapshot_len);
  }
  return true;
}

void applyCameraSensorDefaults() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
  } else {
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
  }
  s->set_colorbar(s, 0);
  s->set_brightness(s, (int)cam_brightness);
}

bool initCameraFormat(pixformat_t format, framesize_t frame_size,
                      uint8_t fb_count, camera_grab_mode_t grab_mode) {
  cam_init_attempts++;
  cam_ok = false;
  cam_have_frame = false;
  cam_last_err = ESP_OK;
  cam_sensor_pid = -1;
  cam_last_frame_len = 0;
  updateCameraMemoryDiagnostics();
  Serial.printf("Camera init attempt %lu: PSRAM found=%d size=%lu free=%lu format=%d frame=%d fb=%u\n",
                (unsigned long)cam_init_attempts,
                cam_psram_found ? 1 : 0,
                (unsigned long)cam_psram_size,
                (unsigned long)cam_psram_free,
                (int)format,
                (int)frame_size,
                (unsigned)fb_count);
  if (!ensureCameraSnapshotBuffer()) return false;
  // Always allocate the snapshot buffer first — render paths use it to draw
  // fallback content (e.g. "NO CAMERA") when the sensor isn't available.
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0=Y2_GPIO_NUM; cfg.pin_d1=Y3_GPIO_NUM;
  cfg.pin_d2=Y4_GPIO_NUM; cfg.pin_d3=Y5_GPIO_NUM;
  cfg.pin_d4=Y6_GPIO_NUM; cfg.pin_d5=Y7_GPIO_NUM;
  cfg.pin_d6=Y8_GPIO_NUM; cfg.pin_d7=Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM;
  cfg.pin_pclk = PCLK_GPIO_NUM;
  cfg.pin_vsync= VSYNC_GPIO_NUM;
  cfg.pin_href = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = -1;            // share Wire (esp-camera issue #741)
  cfg.pin_sccb_scl = -1;
  cfg.sccb_i2c_port = 0;
  cfg.pin_pwdn = -1;
  cfg.pin_reset= -1;
  cfg.xclk_freq_hz = 16000000;
  cfg.pixel_format = format;
  cfg.frame_size   = frame_size;
  cfg.fb_count     = fb_count;
  cfg.fb_location  = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode    = grab_mode;
  cam_last_err = esp_camera_init(&cfg);
  if (cam_last_err != ESP_OK) {
    cam_fail_stage = CAM_STAGE_INIT_FAIL;
    Serial.printf("Camera init failed: 0x%X %s\n",
                  (unsigned)cam_last_err, cameraErrName());
    return false;
  }
  camera_driver_active = true;

  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    cam_fail_stage = CAM_STAGE_NO_SENSOR;
    Serial.println("Camera init failed: sensor pointer is null after esp_camera_init");
    esp_camera_deinit();
    camera_driver_active = false;
    return false;
  }
  cam_sensor_pid = s->id.PID;

  // Stay close to sensor defaults — every extra filter we turn on (dcw,
  // lenc, bpc, wpc, aec2) softens the image. The "raw feed" look the user
  // prefers comes from the sensor's tuned defaults; we only override
  // orientation and the test pattern toggle.
  // OV3660 note: this sensor is physically mounted upside-down relative to
  // where the OV2640 sat, so vflip must be inverted. We detect the sensor
  // type at runtime via PID so the sketch works with either module.
  applyCameraSensorDefaults();

  cam_ok = true;
  cam_fail_stage = CAM_STAGE_OK;
  updateCameraMemoryDiagnostics();
  Serial.printf("Camera init OK: PID=0x%04X PSRAM free=%lu snapshot=%u\n",
                cam_sensor_pid,
                (unsigned long)cam_psram_free,
                (unsigned)cam_snapshot_len);
  return true;
}

bool initCamera() {
  return initCameraFormat(PIXFORMAT_RGB565, FRAMESIZE_QVGA, 3,
                          CAMERA_GRAB_LATEST);
}

void deinitCameraDriver() {
  if (!camera_driver_active) return;
  esp_camera_deinit();
  camera_driver_active = false;
  cam_ok = false;
  cam_have_frame = false;
}

bool enterStreamCameraMode() {
  stream_native_jpeg_active = false;
#if STREAM_CAMERA_NATIVE_JPEG
  if (!cam_ok && !camera_driver_active) return true;
  deinitCameraDriver();
  delay(50);
  if (initCameraFormat(PIXFORMAT_JPEG, FRAMESIZE_QVGA, 3,
                       CAMERA_GRAB_LATEST)) {
    stream_native_jpeg_active = true;
    Serial.println("Stream camera: native JPEG mode");
    return true;
  }
  Serial.println("Stream camera: native JPEG init failed, restoring RGB565");
  deinitCameraDriver();
  delay(50);
  if (!initCamera()) return false;
#endif
  return true;
}

void exitStreamCameraMode() {
#if STREAM_CAMERA_NATIVE_JPEG
  if (!stream_native_jpeg_active) return;
  deinitCameraDriver();
  delay(50);
  if (!initCamera()) {
    Serial.println("Stream camera: RGB565 restore failed");
  }
#endif
  stream_native_jpeg_active = false;
}

// Pull latest frame, copy to our PSRAM buffer, return camera fb immediately.
// Returns true if a new frame was captured. No-op if camera missing.
bool grabCamera() {
  if (!cam_ok || stream_native_jpeg_active) return false;
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    cam_frame_fail_count++;
    cam_fail_stage = CAM_STAGE_NO_FRAME;
    uint32_t now = millis();
    if (now - cam_last_frame_log_ms > 2000) {
      cam_last_frame_log_ms = now;
      Serial.printf("Camera frame unavailable: failures=%lu\n",
                    (unsigned long)cam_frame_fail_count);
    }
    return false;
  }
  cam_last_frame_len = fb->len;
  if (fb->len == cam_snapshot_len) {
    memcpy(cam_snapshot, fb->buf, cam_snapshot_len);
    cam_have_frame = true;
    cam_fail_stage = CAM_STAGE_OK;
  } else {
    cam_frame_len_mismatch_count++;
    cam_fail_stage = CAM_STAGE_FRAME_LEN_MISMATCH;
    uint32_t now = millis();
    if (now - cam_last_frame_log_ms > 2000) {
      cam_last_frame_log_ms = now;
      Serial.printf("Camera frame length mismatch: got=%u expected=%u count=%lu\n",
                    (unsigned)fb->len,
                    (unsigned)cam_snapshot_len,
                    (unsigned long)cam_frame_len_mismatch_count);
    }
  }
  esp_camera_fb_return(fb);
  return cam_have_frame;
}

// ---------------------------------------------------------------- Palette
// Ironbow (the v8 palette — restored verbatim, since the green/cyan tiles
// in v8 came from setSwapBytes being inverted, not from the colour ramp).
//   0%  → black           (cold)
//   ~17% → blue/violet
//   ~33% → magenta/red
//   ~50% → red
//   ~67% → orange
//   ~83% → yellow
//  100% → white            (hot)
// No green component appears in isolation; greens only show up combined
// with red as orange/yellow.
uint16_t palette_native[256];
uint16_t palette_wire[256];
uint8_t  palR[256], palG[256], palB[256];

static inline uint8_t clamp8(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }
static inline uint16_t swap565(uint16_t v) { return (uint16_t)((v >> 8) | (v << 8)); }

void buildPalette() {
  for (int i = 0; i < 256; i++) {
    int j = (i * 180) / 255;            // remap into 0..180 segments
    int R, G, B;
    if (j < 30) {                       // black → blue
      R = 0;                       G = 0;                              B = 20  + (120 * j) / 30;
    } else if (j < 60) {                // blue → red
      R = (120 * (j - 30)) / 30;   G = 0;                              B = 140 - (60  * (j - 30)) / 30;
    } else if (j < 90) {                // red → red
      R = 120 + (135 * (j - 60)) / 30;  G = 0;                         B = 80  - (70  * (j - 60)) / 30;
    } else if (j < 120) {               // red → red-orange
      R = 255;                     G = (60 * (j - 90)) / 30;           B = 10  - (10  * (j - 90)) / 30;
    } else if (j < 150) {               // orange → yellow
      R = 255;                     G = 60 + (175 * (j - 120)) / 30;    B = 0;
    } else {                            // yellow → white-ish
      R = 255;                     G = 235 + (20 * (j - 150)) / 30;    B = (255 * (j - 150)) / 30;
    }
    palR[i] = clamp8(R); palG[i] = clamp8(G); palB[i] = clamp8(B);
    palette_native[i] = ((palR[i] & 0xF8) << 8) |
                        ((palG[i] & 0xFC) << 3) |
                        ( palB[i] >> 3);
    palette_wire[i] = swap565(palette_native[i]);
  }
}

uint8_t tempToPaletteIndex(float t, float lo, float hi) {
  float f = (t - lo) / ((hi - lo) + 0.001f);
  if (!(f >= 0.0f)) f = 0.0f;
  else if (f > 1.0f) f = 1.0f;
  int idx = (int)(f * 255.0f + 0.5f);
  if (idx < 0) idx = 0; else if (idx > 255) idx = 255;
  return (uint8_t)idx;
}

// ---------------------------------------------------------------- State ---
enum DisplayMode { MODE_TINT=0, MODE_THERMAL_ONLY, MODE_CAMERA_ONLY, MODE_COUNT };
const char *MODE_NAMES[MODE_COUNT] = {"TINT", "THERM", "CAM"};
const uint16_t MODE_BG[MODE_COUNT] = {TFT_GREEN, TFT_ORANGE, TFT_BLUE};

// Range presets.
//
// AUTO  : trimmed min/max of all 768 pixels — full scene mapped to palette.
//         Good when you don't know what's in frame. Background dominates the
//         palette when the subject is small.
// AUTO2 : "subject-focused auto" — finds the median (ambient/room) and
//         throws everything within ±AUTO2_AMBIENT_DELTA °C of it AWAY,
//         then maps the palette only to the remaining "interesting"
//         pixels. So a hand against a wall: room-temp wall ignored,
//         palette stretched across the temperature range *of the hand*.
//         Falls back to AUTO if nothing in frame is significantly
//         hotter or colder than ambient.
// SKIN  : 30.5 - 35.5 °C — exposed skin only. Tightest palette.
// BODY  : 28 - 37 °C — clothed body separation, palms + forearms.
// WARM  : 30 - 50 °C — mugs, electronics, sun-warm surfaces.
// HOT   : 40 - 90 °C — stovetops, irons.
// VHOT  : 60 - 140 °C — flame, exhaust manifolds.
// MANUAL: user drags LO/HI sliders directly (RANGE tab on the side panel).
//         Persisted across reboots.
// RangeMode enum + AUTO2 constants are forward-declared near the top of the
// file (right under the MLX block) because analyzeMLX needs them before this
// point. Only the *arrays* live here.
const char *RANGE_NAMES[RANGE_COUNT] = {"AUTO", "AUT2", "SKIN", "BODY",
                                        "WARM", "HOT", "VHOT", "MAN"};
static constexpr float RANGE_LO[RANGE_COUNT] = {0.0f,  0.0f, 30.5f, 28.0f,
                                                30.0f, 40.0f, 60.0f, 0.0f};
static constexpr float RANGE_HI[RANGE_COUNT] = {0.0f,  0.0f, 35.5f, 37.0f,
                                                50.0f, 90.0f, 140.0f, 0.0f};

static constexpr int PARALLAX_MIN = -90;
static constexpr int PARALLAX_MAX =  90;

volatile int parallax_x = 0;
volatile int parallax_y = 0;
// zoom_pct is a digital crop factor applied ONLY to the camera image now.
// The thermal grid is fixed full-screen and never moves or scales. 100% =
// full sensor frame, no crop. 200% = read a centred 160×120 region and
// stretch it. Default is 124%, which crops the camera FOV (~68°) down to
// roughly the MLX FOV (~55°) so the two views naturally align without
// fiddling. Adjust ± from there to fine-tune. Range 100..250.
volatile int zoom_pct   = 124;
volatile uint8_t tint_pct = 70;
volatile DisplayMode display_mode = MODE_TINT;
volatile RangeMode range_mode = RANGE_AUTO;
volatile bool freeze_mode = false;
volatile bool stream_mode = false;
volatile bool ui_portrait = false;
volatile uint8_t stream_view_mode = 0;  // 0=overlay, 1=camera, 2=thermal
volatile bool stream_hud_enabled = true;
volatile bool stream_crosshair_enabled = true;

// MANUAL-mode lo/hi setpoints. Used only when range_mode == RANGE_MANUAL.
// On entering MANUAL we seed these from whatever range was active so the
// transition isn't a jarring jump. Persisted across reboots.
volatile float manual_lo = 22.0f;
volatile float manual_hi = 38.0f;
// Side-panel tab — swaps which sliders the panel shows:
//   PANEL_TAB_POS:   X / Y / Zoom sliders (camera-vs-thermal alignment)
//   PANEL_TAB_RANGE: LO / HI sliders (only useful when range_mode == MANUAL,
//                    but visible/draggable in any mode so you can preview)
enum PanelTab { PANEL_TAB_POS = 0, PANEL_TAB_RANGE = 1, PANEL_TAB_COUNT };
volatile uint8_t panel_tab = PANEL_TAB_POS;
// Slider domain for MANUAL — covers room temp through hot-ish electronics.
// If you need to range outside this, use a fixed preset.
static constexpr float MANUAL_T_MIN = 5.0f;
static constexpr float MANUAL_T_MAX = 60.0f;

// Camera brightness: -2..+2, applied via the OV2640/OV3660 hardware register
// (sensor's set_brightness call), so it costs zero CPU per pixel — the change
// happens once over SCCB and the sensor's analog front-end does the rest.
// 0 = neutral, +2 = brightest. Persisted in NVS.
volatile int8_t cam_brightness = 0;
bool             brightness_apply_pending = false;

// Lens FOVs. MLX90640 standard variant: 55°×35°. OV2640 with the stock
// FireBeetle DVP lens: ~68°×51°. From these we derive how big each thermal
// cell *should* be in camera-pixel space so the two views actually align.
static constexpr float MLX_HFOV_DEG = 55.0f;
static constexpr float MLX_VFOV_DEG = 35.0f;
static constexpr float CAM_HFOV_DEG = 68.0f;
static constexpr float CAM_VFOV_DEG = 51.0f;
// Default cell size in camera pixels at zoom = 100%. Outside this rectangle,
// no thermal data exists — the camera image stays untouched.
static constexpr float BASE_CELL_PX_X = (MLX_HFOV_DEG / 32.0f) /
                                        (CAM_HFOV_DEG / (float)IMG_W);   // ≈ 8.09
static constexpr float BASE_CELL_PX_Y = (MLX_VFOV_DEG / 24.0f) /
                                        (CAM_VFOV_DEG / (float)IMG_H);   // ≈ 6.86

// ---------------------------------------------------------- Persistence --
// Settings (parallax, zoom, tint, mode, range) are kept in NVS via the
// Preferences library — flash-backed and wear-levelled. We auto-save 3 s
// after the last change so the user never has to press a "save" button:
// twiddle the controls, stop, and a moment later the values commit. A brief
// "SAV" badge in the top bar confirms the write.
Preferences prefs;
bool     settings_dirty = false;
uint32_t settings_dirty_ms = 0;
uint32_t save_indicator_until_ms = 0;
static constexpr uint32_t SETTINGS_DEBOUNCE_MS = 3000;

static inline void markDirty() {
  settings_dirty = true;
  settings_dirty_ms = millis();
}

void loadSettings() {
  // Namespace bumped to flir2 in v14 — semantics of zoom_pct changed (now a
  // camera-crop factor, default 124%). Old "flir" entries from v10..v13 are
  // intentionally ignored so users get the new default on first v14 boot.
  prefs.begin("flir2", true);                  // read-only
  parallax_x   = prefs.getInt  ("px",   0);
  parallax_y   = prefs.getInt  ("py",   0);
  zoom_pct     = prefs.getInt  ("zoom", 124);
  tint_pct     = prefs.getUChar("tint", 70);
  uint8_t m    = prefs.getUChar("mode", MODE_TINT);
  uint8_t r    = prefs.getUChar("rng",  RANGE_AUTO);
  cam_brightness = (int8_t)prefs.getChar("brt", 0);
  manual_lo    = prefs.getFloat("mlo", 22.0f);
  manual_hi    = prefs.getFloat("mhi", 38.0f);
  uint8_t pt   = prefs.getUChar("ptab", PANEL_TAB_POS);
  ui_portrait  = prefs.getBool ("port", false);
  prefs.end();
  // Defensive clamps in case ranges have moved across firmware versions.
  if (parallax_x < PARALLAX_MIN) parallax_x = PARALLAX_MIN; else if (parallax_x > PARALLAX_MAX) parallax_x = PARALLAX_MAX;
  if (parallax_y < PARALLAX_MIN) parallax_y = PARALLAX_MIN; else if (parallax_y > PARALLAX_MAX) parallax_y = PARALLAX_MAX;
  if (zoom_pct   < 100) zoom_pct   = 100; else if (zoom_pct  > 250) zoom_pct  = 250;
  if (tint_pct   > 100) tint_pct   = 100;
  if (cam_brightness < -2) cam_brightness = -2; else if (cam_brightness > 2) cam_brightness = 2;
  if (manual_lo  < MANUAL_T_MIN) manual_lo = MANUAL_T_MIN;
  if (manual_lo  > MANUAL_T_MAX) manual_lo = MANUAL_T_MAX;
  if (manual_hi  < MANUAL_T_MIN) manual_hi = MANUAL_T_MIN;
  if (manual_hi  > MANUAL_T_MAX) manual_hi = MANUAL_T_MAX;
  if (manual_hi - manual_lo < 1.0f) manual_hi = manual_lo + 1.0f;
  display_mode = (m < MODE_COUNT)  ? (DisplayMode)m : MODE_TINT;
  range_mode   = (r < RANGE_COUNT) ? (RangeMode)r   : RANGE_AUTO;
  panel_tab    = (pt < PANEL_TAB_COUNT) ? (uint8_t)pt : (uint8_t)PANEL_TAB_POS;
  brightness_apply_pending = true;        // push into sensor on first frame
  Serial.printf("Loaded: PX=%+d,%+d Z=%d%% Tint=%d%% Brt=%+d Mode=%s Rng=%s Man=[%.1f,%.1f]\n",
                parallax_x, parallax_y, zoom_pct, tint_pct, cam_brightness,
                MODE_NAMES[display_mode], RANGE_NAMES[range_mode],
                manual_lo, manual_hi);
}

void saveSettings() {
  prefs.begin("flir2", false);                 // read-write
  prefs.putInt  ("px",   parallax_x);
  prefs.putInt  ("py",   parallax_y);
  prefs.putInt  ("zoom", zoom_pct);
  prefs.putUChar("tint", tint_pct);
  prefs.putUChar("mode", (uint8_t)display_mode);
  prefs.putUChar("rng",  (uint8_t)range_mode);
  prefs.putChar ("brt",  (int8_t)cam_brightness);
  prefs.putFloat("mlo",  manual_lo);
  prefs.putFloat("mhi",  manual_hi);
  prefs.putUChar("ptab", panel_tab);
  prefs.putBool ("port", ui_portrait);
  prefs.end();
  save_indicator_until_ms = millis() + 1500;
  Serial.printf("Saved: PX=%+d,%+d Z=%d%% Tint=%d%% Brt=%+d Mode=%s Rng=%s Man=[%.1f,%.1f]\n",
                parallax_x, parallax_y, zoom_pct, tint_pct, cam_brightness,
                MODE_NAMES[display_mode], RANGE_NAMES[range_mode],
                manual_lo, manual_hi);
}

// Apply brightness to the OV2640/OV3660 via SCCB. set_brightness() is one
// I2C write — far cheaper than a software multiplier per pixel — and we
// only call it when the value actually changes.
void applyCameraBrightness() {
  if (!cam_ok) return;
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  s->set_brightness(s, (int)cam_brightness);
  Serial.printf("Cam brightness → %+d\n", cam_brightness);
}

// ---------------------------------------------------------- Battery ------
// Reads via on-board voltage divider on a configurable ADC pin. The DFRobot
// FireBeetle 2 ESP32-S3 doesn't have a documented battery sense pin in this
// firmware's pin map (most ADCs are claimed by camera/LCD), so set this to
// the GPIO you've wired your divider to. Leave at -1 to disable the readout.
#define BATTERY_ADC_PIN -1            // e.g. 6 if you free up VSYNC, or wire externally
#define BATTERY_DIVIDER 2.0f          // (R_top + R_bot)/R_bot — typical 1:1 divider
#define BATTERY_VREF    3.3f
#define BATTERY_VMIN    3.30f         // 0%   — protect under-voltage cutoff
#define BATTERY_VMAX    4.20f         // 100% — full charge

float    battery_v   = 0.0f;
int      battery_pct = -1;            // -1 = "no reading yet"
uint32_t last_bat_ms = 0;

void readBattery() {
#if BATTERY_ADC_PIN >= 0
  // Average a few samples — ADC on ESP32-S3 has a noticeable ~LSB jitter.
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(BATTERY_ADC_PIN);
  float v_adc = (sum / 8.0f) / 4095.0f * BATTERY_VREF;
  battery_v = v_adc * BATTERY_DIVIDER;
  int pct = (int)((battery_v - BATTERY_VMIN) * 100.0f / (BATTERY_VMAX - BATTERY_VMIN));
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  battery_pct = pct;
#else
  // Pin unset — leave battery_pct at -1 so the UI hides the indicator.
  battery_pct = -1;
#endif
}

void getThermalRange(float &lo, float &hi) {
  // MANUAL: user-set values — bypass the IIR-tracked range entirely.
  if (range_mode == RANGE_MANUAL) {
    float ml = manual_lo, mh = manual_hi;
    // Keep at least a 1 °C span so the palette never collapses to a point.
    if (mh - ml < 1.0f) mh = ml + 1.0f;
    lo = ml; hi = mh;
    return;
  }
  // AUTO and AUTO2 both pipe through analyzeMLX → IIR-smoothed t_range_min/max.
  // The "smart subject" math for AUTO2 lives in analyzeMLX, this just reads.
  if (range_mode == RANGE_AUTO || range_mode == RANGE_AUTO2) {
    portENTER_CRITICAL(&mlx_mux);
    lo = t_range_min; hi = t_range_max;
    portEXIT_CRITICAL(&mlx_mux);
    return;
  }
  // Fixed preset.
  lo = RANGE_LO[range_mode];
  hi = RANGE_HI[range_mode];
}

// ---------------------------------------------------------------- GT911 --
// LovyanGFX's touch wrapper was unreliable on this combo. Poll directly.
bool gt911_ok = false;
uint8_t gt911_addr = 0x5D;

bool gt911_init_manual() {
  pinMode(TOUCH_INT, OUTPUT);
  digitalWrite(TOUCH_INT, LOW);
  delay(11);
  pinMode(TOUCH_INT, INPUT);
  delay(50);
  for (uint8_t addr : {(uint8_t)0x5D, (uint8_t)0x14}) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      gt911_addr = addr; gt911_ok = true;
      Serial.printf("GT911: ACK at 0x%02X\n", addr);
      return true;
    }
  }
  return false;
}

bool gt911_read_touch(uint16_t *tx, uint16_t *ty) {
  if (!gt911_ok) return false;
  Wire.beginTransmission(gt911_addr);
  Wire.write(0x81); Wire.write(0x4E);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(gt911_addr, (uint8_t)1);
  if (!Wire.available()) return false;
  uint8_t status = Wire.read();
  if (!(status & 0x80) || (status & 0x0F) == 0) {
    Wire.beginTransmission(gt911_addr);
    Wire.write(0x81); Wire.write(0x4E); Wire.write(0x00);
    Wire.endTransmission();
    return false;
  }
  Wire.beginTransmission(gt911_addr);
  Wire.write(0x81); Wire.write(0x50);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(gt911_addr, (uint8_t)4);
  if (Wire.available() < 4) return false;
  uint16_t rx = Wire.read() | (Wire.read() << 8);
  uint16_t ry = Wire.read() | (Wire.read() << 8);
  Wire.beginTransmission(gt911_addr);
  Wire.write(0x81); Wire.write(0x4E); Wire.write(0x00);
  Wire.endTransmission();
  // GT911 native = 320x480 portrait. Rotation 1 remaps to landscape.
  if (ui_portrait) {
    *tx = rx;
    *ty = ry;
  } else {
    *tx = ry;
    *ty = 319 - rx;
  }
  return true;
}

// ---------------------------------------------------------------- UI -----
struct TouchButton {
  int x, y, w, h;
  const char *label;
  uint16_t color;
  void (*action)();
};

bool adjust_unlocked = false;
uint32_t adjust_unlock_until_ms = 0;
static constexpr uint32_t ADJUST_UNLOCK_MS = 12000;
static int PANEL_X = 348;
static int PANEL_W = 128;
static int PANEL_SLIDER_X = 352;
static int PANEL_SLIDER_W = 120;
static int PANEL_SLIDER_H = 20;
static constexpr int PANEL_NUDGE_W  = 22;
static constexpr int PANEL_NUDGE_GAP = 3;
static constexpr uint32_t NUDGE_REPEAT_MS = 130;
static constexpr int PARALLAX_NUDGE_PX = 1;
static constexpr int ZOOM_NUDGE_PCT = 1;
static constexpr int TINT_NUDGE_PCT = 1;
static constexpr float MANUAL_NUDGE_C = 0.1f;
static constexpr float MANUAL_MIN_SPAN_C = 1.0f;
static int TAB_ROW_Y     = 58;       // tabs sit between top buttons and X slider
static int TAB_ROW_H     = 22;
static int X_SLIDER_Y    = 94;
static int Y_SLIDER_Y    = 136;
static int Z_SLIDER_Y    = 178;
static int TINT_SLIDER_Y = 232;
// In the RANGE tab the slider lineup becomes LO / HI / (empty). The third
// slot stays at Z_SLIDER_Y — we just clear it instead of drawing a slider.
// LO and HI reuse the X / Y slider rows so the visual layout stays consistent.

void btnToggleAdjust() {
  adjust_unlocked = !adjust_unlocked;
  adjust_unlock_until_ms = adjust_unlocked ? millis() + ADJUST_UNLOCK_MS : 0;
}
void btnCycleMode()  { display_mode = (DisplayMode)((display_mode + 1) % MODE_COUNT);  markDirty(); }
void btnReset()      { if (adjust_unlocked) { parallax_x = 0; parallax_y = 0; zoom_pct = 100; markDirty(); } }
void btnCycleRange() {
  range_mode = (RangeMode)((range_mode + 1) % RANGE_COUNT);
  // When the user lands on MANUAL, jump the panel into the RANGE tab so the
  // sliders they need are immediately in front of them. Conversely, leaving
  // MANUAL doesn't auto-switch back to POS — they might still want to nudge
  // LO/HI to compare modes.
  if (range_mode == RANGE_MANUAL) panel_tab = PANEL_TAB_RANGE;
  markDirty();
}
void btnTabPos()   { panel_tab = PANEL_TAB_POS;   markDirty(); }
void btnTabRange() { panel_tab = PANEL_TAB_RANGE; markDirty(); }

void drawStaticUI();
void layoutUi();

void applyScreenOrientation() {
  lcd.setRotation(ui_portrait ? 0 : 1);
  layoutUi();
}

void redrawFullUi() {
  lcd.fillScreen(TFT_BLACK);
  drawStaticUI();
}

void btnToggleOrientation() {
  ui_portrait = !ui_portrait;
  markDirty();
  applyScreenOrientation();
  redrawFullUi();
}

// Tab-button widths split the panel into two roughly-equal halves with a
// 4-px gap between them.
static int TAB_W = 62;
static int SLIDER_X = 352;
static int SLIDER_Y = 232;
static int SLIDER_W = 120;
static int SLIDER_H = 20;
static int BRT_BAR_X     = 46;
static int BRT_BAR_Y     = 290;
static int BRT_CELL_W    = 50;
static int BRT_CELL_H    = 24;
static constexpr int BRT_NUM_CELLS = 5;
static int BRT_CLEAR_X   = 40;
static int BRT_CLEAR_R   = 336;
static int PALETTE_X = 5;
static int PALETTE_Y_TOP = 24;
static int PALETTE_Y_BOTTOM = 280;
static int PALETTE_W = 12;
static int TMAX_LABEL_X = 2;
static int TMAX_LABEL_Y = 10;
static int TMIN_LABEL_X = 2;
static int TMIN_LABEL_Y = 290;
static int TOPBAR_CLEAR_X = 40;
static int TOPBAR_CLEAR_W = 440;
static int TB_TMAX  =   2;
static int TB_CTR   =  44;
static int TB_FPS   = 116;
static int TB_ZOOM  = 162;
static int TB_PARA  = 200;
static int TB_MODE  = 256;
static int TB_MODE_W = 42;
static int TB_RNG   = 302;
static int TB_RNG_W = 36;
static int TB_STAT  = 342;
static int TB_STAT_W = 24;
static int TB_BAT   = 412;
static int READOUT_X = 378;
static int READOUT_Y = 158;
static int READOUT_W = 98;
static int READOUT_H = 10;

TouchButton buttons[] = {
  {0, 0, 0, 0, "ADJ",  TFT_DARKCYAN,  btnToggleAdjust },
  {0, 0, 0, 0, "MODE", TFT_DARKGREEN, btnCycleMode    },
  {0, 0, 0, 0, "RNG",  TFT_PURPLE,    btnCycleRange   },
  {0, 0, 0, 0, "ROT",  TFT_DARKGREY,  btnToggleOrientation },
  // Tab row.
  {0, 0, 0, 0, "POS",   TFT_DARKGREY, btnTabPos   },
  {0, 0, 0, 0, "RANGE", TFT_DARKGREY, btnTabRange },
  // Reset.
  {0, 0, 0, 0, "RST", TFT_MAROON, btnReset        },
};
const int NUM_BUTTONS = sizeof(buttons) / sizeof(buttons[0]);

void layoutUi() {
  if (ui_portrait) {
    IMG_X = 0;
    IMG_Y = 24;
    PANEL_X = 4;
    PANEL_W = 312;
    PANEL_SLIDER_X = 8;
    PANEL_SLIDER_W = 304;
    PANEL_SLIDER_H = 16;
    TAB_ROW_Y = 296;
    TAB_ROW_H = 20;
    X_SLIDER_Y = 330;
    Y_SLIDER_Y = 362;
    Z_SLIDER_Y = 394;
    TINT_SLIDER_Y = 426;
    TAB_W = (PANEL_W - 4) / 2;
    SLIDER_X = PANEL_SLIDER_X;
    SLIDER_Y = TINT_SLIDER_Y;
    SLIDER_W = PANEL_SLIDER_W;
    SLIDER_H = PANEL_SLIDER_H;
    BRT_BAR_X = 32;
    BRT_BAR_Y = 456;
    BRT_CELL_W = 42;
    BRT_CELL_H = 18;
    BRT_CLEAR_X = 0;
    BRT_CLEAR_R = 320;
    PALETTE_X = 0;
    PALETTE_Y_TOP = IMG_Y;
    PALETTE_Y_BOTTOM = IMG_Y + IMG_H - 1;
    PALETTE_W = 0;
    TMAX_LABEL_X = 2;
    TMAX_LABEL_Y = 4;
    TMIN_LABEL_X = 2;
    TMIN_LABEL_Y = -1;
    TOPBAR_CLEAR_X = 40;
    TOPBAR_CLEAR_W = 280;
    TB_TMAX = TMAX_LABEL_X;
    TB_CTR = 40;
    TB_FPS = 108;
    TB_ZOOM = 150;
    TB_PARA = 184;
    TB_MODE = 236;
    TB_MODE_W = 34;
    TB_RNG = 272;
    TB_RNG_W = 30;
    TB_STAT = -1;
    TB_STAT_W = 0;
    TB_BAT = -1;
    READOUT_X = 0;
    READOUT_Y = 0;
    READOUT_W = 0;
    READOUT_H = 0;

    const int gap = 4;
    int bw = (320 - 2 * PANEL_X - gap * 4) / 5;
    int by = 268;
    const int top_ids[] = {0, 1, 2, 3, 6};
    for (int i = 0; i < 5; i++) {
      TouchButton &b = buttons[top_ids[i]];
      b.x = PANEL_X + i * (bw + gap);
      b.y = by;
      b.w = bw;
      b.h = 24;
    }
    buttons[4].x = PANEL_X;
    buttons[4].y = TAB_ROW_Y;
    buttons[4].w = TAB_W;
    buttons[4].h = TAB_ROW_H;
    buttons[5].x = PANEL_X + TAB_W + 4;
    buttons[5].y = TAB_ROW_Y;
    buttons[5].w = TAB_W;
    buttons[5].h = TAB_ROW_H;
  } else {
    IMG_X = 24;
    IMG_Y = 40;
    PANEL_X = IMG_X + IMG_W + 4;
    PANEL_W = 480 - PANEL_X - 4;
    PANEL_SLIDER_X = PANEL_X + 4;
    PANEL_SLIDER_W = PANEL_W - 8;
    PANEL_SLIDER_H = 20;
    TAB_ROW_Y = 58;
    TAB_ROW_H = 22;
    X_SLIDER_Y = 94;
    Y_SLIDER_Y = 136;
    Z_SLIDER_Y = 178;
    TINT_SLIDER_Y = 232;
    TAB_W = (PANEL_W - 4) / 2;
    SLIDER_X = PANEL_SLIDER_X;
    SLIDER_Y = TINT_SLIDER_Y;
    SLIDER_W = PANEL_SLIDER_W;
    SLIDER_H = PANEL_SLIDER_H;
    BRT_BAR_X = 46;
    BRT_BAR_Y = 290;
    BRT_CELL_W = 50;
    BRT_CELL_H = 24;
    BRT_CLEAR_X = 40;
    BRT_CLEAR_R = BRT_BAR_X + BRT_NUM_CELLS * BRT_CELL_W + 40;
    PALETTE_X = 5;
    PALETTE_Y_TOP = 24;
    PALETTE_Y_BOTTOM = 280;
    PALETTE_W = 12;
    TMAX_LABEL_X = 2;
    TMAX_LABEL_Y = 10;
    TMIN_LABEL_X = 2;
    TMIN_LABEL_Y = 290;
    TOPBAR_CLEAR_X = 40;
    TOPBAR_CLEAR_W = 440;
    TB_TMAX = 2;
    TB_CTR = 44;
    TB_FPS = 116;
    TB_ZOOM = 162;
    TB_PARA = 200;
    TB_MODE = 256;
    TB_MODE_W = 42;
    TB_RNG = 302;
    TB_RNG_W = 36;
    TB_STAT = 342;
    TB_STAT_W = 24;
    TB_BAT = 412;
    READOUT_X = 378;
    READOUT_Y = 158;
    READOUT_W = 98;
    READOUT_H = 10;

    const int gap = 3;
    int bw = (PANEL_W - gap * 3) / 4;
    for (int i = 0; i < 4; i++) {
      buttons[i].x = PANEL_X + i * (bw + gap);
      buttons[i].y = 22;
      buttons[i].w = bw;
      buttons[i].h = 28;
    }
    buttons[4].x = PANEL_X;
    buttons[4].y = TAB_ROW_Y;
    buttons[4].w = TAB_W;
    buttons[4].h = TAB_ROW_H;
    buttons[5].x = PANEL_X + TAB_W + 4;
    buttons[5].y = TAB_ROW_Y;
    buttons[5].w = TAB_W;
    buttons[5].h = TAB_ROW_H;
    buttons[6].x = PANEL_X;
    buttons[6].y = 286;
    buttons[6].w = PANEL_W;
    buttons[6].h = 30;
  }
}

void drawButtons() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    auto &b = buttons[i];
    // Background: ADJ flips between green (unlocked) and dark-cyan (locked).
    // Tab buttons highlight blue when active. Everything else keeps its
    // assigned color regardless of state. Previously RST went TFT_DARKGREY
    // when locked, which rendered as nearly-black on this panel and made
    // the button look invisible — now it stays maroon and we indicate
    // "locked" with a dim text/outline instead.
    uint16_t bg = b.color;
    if (b.action == btnToggleAdjust) bg = adjust_unlocked ? TFT_GREEN : TFT_DARKCYAN;
    bool tab_active = (b.action == btnTabPos   && panel_tab == PANEL_TAB_POS) ||
                      (b.action == btnTabRange && panel_tab == PANEL_TAB_RANGE);
    if (b.action == btnTabPos || b.action == btnTabRange) {
      bg = tab_active ? TFT_BLUE : TFT_DARKGREY;
    }
    bool dim = (b.action == btnReset && !adjust_unlocked);
    uint16_t outline = dim ? TFT_LIGHTGREY : TFT_WHITE;
    uint16_t fg      = dim ? TFT_LIGHTGREY
                           : ((b.action == btnToggleAdjust && adjust_unlocked) ||
                              tab_active
                                ? TFT_WHITE : TFT_WHITE);
    lcd.fillRoundRect(b.x, b.y, b.w, b.h, 4, bg);
    lcd.drawRoundRect(b.x, b.y, b.w, b.h, 4, outline);
    lcd.setTextColor(fg, bg);
    lcd.setTextSize(1);
    lcd.setCursor(b.x + (b.w - (int)strlen(b.label) * 6) / 2, b.y + (b.h - 8) / 2);
    lcd.print(b.label);
  }
}

static inline int sliderTrackX(int x) {
  return x + PANEL_NUDGE_W + PANEL_NUDGE_GAP;
}

static inline int sliderTrackW(int w) {
  return w - 2 * (PANEL_NUDGE_W + PANEL_NUDGE_GAP);
}

static inline bool sliderRowHit(uint16_t tx, uint16_t ty,
                                int x, int y, int w, int h) {
  return tx >= x && tx < x + w && ty >= y && ty < y + h;
}

static inline int sliderNudgeDir(uint16_t tx, int x, int w) {
  if (tx < x + PANEL_NUDGE_W) return -1;
  if (tx >= x + w - PANEL_NUDGE_W) return 1;
  return 0;
}

static inline bool sliderTrackHit(uint16_t tx, int x, int w) {
  int tx0 = sliderTrackX(x);
  int tw = sliderTrackW(w);
  return tx >= tx0 && tx < tx0 + tw;
}

static inline int sliderPctFromTrack(uint16_t tx, int x, int w) {
  int tx0 = sliderTrackX(x);
  int tw = sliderTrackW(w);
  if (tw < 2) return 0;
  return constrain((int)(tx - tx0) * 100 / (tw - 1), 0, 100);
}

void drawNudgeButton(int x, int y, int w, int h,
                     const char *label, bool enabled) {
  uint16_t bg = enabled ? TFT_DARKGREY : 0x4208;
  uint16_t fg = enabled ? TFT_WHITE : TFT_LIGHTGREY;
  lcd.fillRoundRect(x, y, w, h, 3, bg);
  lcd.drawRoundRect(x, y, w, h, 3, TFT_WHITE);
  lcd.setTextSize(1);
  lcd.setTextColor(fg, bg);
  lcd.setCursor(x + (w - (int)strlen(label) * 6) / 2, y + (h - 8) / 2);
  lcd.print(label);
}

void drawHSlider(int x, int y, int w, int h, int pct,
                 const char *label, const char *value,
                 uint16_t fill, bool enabled) {
  if (pct < 0) pct = 0; else if (pct > 100) pct = 100;
  uint16_t bg = enabled ? TFT_DARKGREY : 0x4208;
  uint16_t fg = enabled ? TFT_WHITE : TFT_LIGHTGREY;
  int track_x = sliderTrackX(x);
  int track_w = sliderTrackW(w);
  lcd.setTextSize(1);
  lcd.setTextColor(fg, TFT_BLACK);
  lcd.fillRect(x, y - 13, w, h + 17, TFT_BLACK);
  lcd.setCursor(x, y - 11); lcd.print(label);
  int tw = (int)strlen(value) * 6;
  lcd.setCursor(x + w - tw, y - 11); lcd.print(value);
  drawNudgeButton(x, y, PANEL_NUDGE_W, h, "-", enabled);
  drawNudgeButton(x + w - PANEL_NUDGE_W, y, PANEL_NUDGE_W, h, "+", enabled);
  lcd.fillRect(track_x, y, track_w, h, bg);
  lcd.drawRect(track_x, y, track_w, h, TFT_WHITE);
  int fill_w = (pct * (track_w - 4)) / 100;
  if (fill_w > 0) lcd.fillRect(track_x + 2, y + 2, fill_w, h - 4, enabled ? fill : TFT_DARKGREY);
  int knob_x = track_x + 2 + fill_w;
  if (knob_x > track_x + track_w - 4) knob_x = track_x + track_w - 4;
  lcd.fillRect(knob_x - 2, y - 2, 5, h + 4, enabled ? TFT_WHITE : TFT_LIGHTGREY);
}

void drawSlider() {
  char buf[12];
  snprintf(buf, sizeof(buf), "%d%%", tint_pct);
  drawHSlider(SLIDER_X, SLIDER_Y, SLIDER_W, SLIDER_H,
              tint_pct, "TINT", buf, TFT_CYAN, true);
}

// Convert a manual_lo/hi temperature in [MANUAL_T_MIN..MANUAL_T_MAX] into the
// 0..100 percentage the slider expects.
static inline int manualTempToPct(float t) {
  if (t <= MANUAL_T_MIN) return 0;
  if (t >= MANUAL_T_MAX) return 100;
  return (int)((t - MANUAL_T_MIN) * 100.0f / (MANUAL_T_MAX - MANUAL_T_MIN));
}

void setManualLoValue(float t) {
  if (t > manual_hi - MANUAL_MIN_SPAN_C) t = manual_hi - MANUAL_MIN_SPAN_C;
  if (t < MANUAL_T_MIN) t = MANUAL_T_MIN;
  if (t > MANUAL_T_MAX) t = MANUAL_T_MAX;
  if (manual_lo != t) {
    manual_lo = t;
    markDirty();
  }
}

void setManualHiValue(float t) {
  if (t < manual_lo + MANUAL_MIN_SPAN_C) t = manual_lo + MANUAL_MIN_SPAN_C;
  if (t < MANUAL_T_MIN) t = MANUAL_T_MIN;
  if (t > MANUAL_T_MAX) t = MANUAL_T_MAX;
  if (manual_hi != t) {
    manual_hi = t;
    markDirty();
  }
}

void drawAdjustSliders() {
  char buf[18];
  if (panel_tab == PANEL_TAB_POS) {
    // Position / zoom sliders. Drag-enable depends on adjust_unlocked.
    bool enabled = adjust_unlocked;
    snprintf(buf, sizeof(buf), "%+d", parallax_x);
    int xpct = (parallax_x - PARALLAX_MIN) * 100 / (PARALLAX_MAX - PARALLAX_MIN);
    drawHSlider(PANEL_SLIDER_X, X_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H,
                xpct, "X", buf, TFT_NAVY, enabled);

    snprintf(buf, sizeof(buf), "%+d", parallax_y);
    int ypct = (parallax_y - PARALLAX_MIN) * 100 / (PARALLAX_MAX - PARALLAX_MIN);
    drawHSlider(PANEL_SLIDER_X, Y_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H,
                ypct, "Y", buf, TFT_NAVY, enabled);

    snprintf(buf, sizeof(buf), "%d%%", zoom_pct);
    int zpct = (zoom_pct - 100) * 100 / 150;
    drawHSlider(PANEL_SLIDER_X, Z_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H,
                zpct, "ZOOM", buf, TFT_DARKCYAN, enabled);
  } else {
    // RANGE tab — LO and HI sliders (only have effect when range_mode == MAN
    // but you can preview values in any mode). The third slider slot is
    // unused on this tab; clear that strip so leftover Z slider pixels from
    // the previous tab don't linger.
    bool enabled = (range_mode == RANGE_MANUAL);
    snprintf(buf, sizeof(buf), "%.1fC", manual_lo);
    drawHSlider(PANEL_SLIDER_X, X_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H,
                manualTempToPct(manual_lo), "LO", buf, TFT_BLUE, enabled);

    snprintf(buf, sizeof(buf), "%.1fC", manual_hi);
    drawHSlider(PANEL_SLIDER_X, Y_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H,
                manualTempToPct(manual_hi), "HI", buf, TFT_RED, enabled);

    // Wipe the old Z-slider area + its label/value row above it.
    lcd.fillRect(PANEL_SLIDER_X, Z_SLIDER_Y - 13,
                 PANEL_SLIDER_W, PANEL_SLIDER_H + 17, TFT_BLACK);
    // Hint text in that empty slot — keeps the layout from looking broken.
    if (!enabled) {
      lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
      lcd.setTextSize(1);
      lcd.setCursor(PANEL_SLIDER_X, Z_SLIDER_Y);
      lcd.print("Cycle to MAN");
      lcd.setCursor(PANEL_SLIDER_X, Z_SLIDER_Y + 10);
      lcd.print("to drag.");
    }
  }
}

void drawBrightnessSlider() {
  // Clear only the brightness band — leave x<40 alone (that's where the
  // side-palette t_min label lives) and stop before the bottom-right reset
  // button.
  lcd.fillRect(BRT_CLEAR_X, BRT_BAR_Y - 4,
               BRT_CLEAR_R - BRT_CLEAR_X, BRT_CELL_H + 8, TFT_BLACK);
  lcd.setTextSize(1);

  for (int i = 0; i < BRT_NUM_CELLS; i++) {
    int v   = i - 2;                                  // -2..+2
    int x   = BRT_BAR_X + i * BRT_CELL_W;
    bool on = (v == cam_brightness);
    uint16_t bg = on ? TFT_CYAN     : TFT_DARKGREY;
    uint16_t fg = on ? TFT_BLACK    : TFT_WHITE;
    lcd.fillRect(x, BRT_BAR_Y, BRT_CELL_W - 4, BRT_CELL_H, bg);
    lcd.drawRect(x, BRT_BAR_Y, BRT_CELL_W - 4, BRT_CELL_H, TFT_WHITE);
    lcd.setTextColor(fg, bg);
    char lbl[5];
    snprintf(lbl, sizeof(lbl), "%+d", v);
    lcd.setCursor(x + (BRT_CELL_W - 4)/2 - 6, BRT_BAR_Y + BRT_CELL_H/2 - 4);
    lcd.print(lbl);
  }
  // "BRT" label to the RIGHT of the cells, so it doesn't collide with the
  // side-palette t_min readout on the left.
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setCursor(BRT_BAR_X + BRT_NUM_CELLS * BRT_CELL_W + 4,
                BRT_BAR_Y + BRT_CELL_H/2 - 4);
  lcd.print("BRT");
}

uint32_t last_touch_ms = 0;
uint32_t last_nudge_ms = 0;

bool nudgeReady(uint32_t now) {
  if (last_nudge_ms && now - last_nudge_ms < NUDGE_REPEAT_MS) return false;
  last_nudge_ms = now;
  return true;
}

void handleTouch() {
  uint16_t tx, ty;
  if (!gt911_read_touch(&tx, &ty)) return;
  uint32_t now = millis();

  if (adjust_unlocked && now > adjust_unlock_until_ms) {
    adjust_unlocked = false;
    drawButtons();
    drawAdjustSliders();
  }

  // Tint slider — drag-able, no rate limit so it tracks finger movement.
  if (sliderRowHit(tx, ty, SLIDER_X, SLIDER_Y, SLIDER_W, SLIDER_H)) {
    int dir = sliderNudgeDir(tx, SLIDER_X, SLIDER_W);
    if (dir) {
      if (nudgeReady(now)) {
        int v = constrain((int)tint_pct + dir * TINT_NUDGE_PCT, 0, 100);
        if (v != tint_pct) { tint_pct = (uint8_t)v; markDirty(); }
        drawSlider();
      }
      return;
    }
    if (sliderTrackHit(tx, SLIDER_X, SLIDER_W)) {
      uint8_t new_pct = (uint8_t)sliderPctFromTrack(tx, SLIDER_X, SLIDER_W);
      if (new_pct != tint_pct) { tint_pct = new_pct; markDirty(); }
      drawSlider();
    }
    return;
  }

  // Side-panel slider drags. Behaviour depends on which tab is active.
  // POS tab → X / Y / Zoom (gated by adjust_unlocked, same as before).
  // RANGE tab → LO / HI in °C (gated by range_mode == MANUAL — the sliders
  // are visible in other range modes for preview, but only mutable in MAN).
  if (tx >= PANEL_SLIDER_X && tx < PANEL_SLIDER_X + PANEL_SLIDER_W) {
    int dir = sliderNudgeDir(tx, PANEL_SLIDER_X, PANEL_SLIDER_W);
    bool track_hit = sliderTrackHit(tx, PANEL_SLIDER_X, PANEL_SLIDER_W);

    if (panel_tab == PANEL_TAB_POS) {
      bool hit_x = sliderRowHit(tx, ty, PANEL_SLIDER_X, X_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H);
      bool hit_y = sliderRowHit(tx, ty, PANEL_SLIDER_X, Y_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H);
      bool hit_z = sliderRowHit(tx, ty, PANEL_SLIDER_X, Z_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H);
      if (hit_x || hit_y || hit_z) {
        if (adjust_unlocked) {
          adjust_unlock_until_ms = now + ADJUST_UNLOCK_MS;
          if (hit_x && dir && nudgeReady(now)) {
            int v = constrain((int)parallax_x + dir * PARALLAX_NUDGE_PX, PARALLAX_MIN, PARALLAX_MAX);
            if (v != parallax_x) { parallax_x = v; markDirty(); }
          } else if (hit_x && track_hit) {
            int pct = sliderPctFromTrack(tx, PANEL_SLIDER_X, PANEL_SLIDER_W);
            int v = PARALLAX_MIN + pct * (PARALLAX_MAX - PARALLAX_MIN) / 100;
            if (v != parallax_x) { parallax_x = v; markDirty(); }
          } else if (hit_y && dir && nudgeReady(now)) {
            int v = constrain((int)parallax_y + dir * PARALLAX_NUDGE_PX, PARALLAX_MIN, PARALLAX_MAX);
            if (v != parallax_y) { parallax_y = v; markDirty(); }
          } else if (hit_y && track_hit) {
            int pct = sliderPctFromTrack(tx, PANEL_SLIDER_X, PANEL_SLIDER_W);
            int v = PARALLAX_MIN + pct * (PARALLAX_MAX - PARALLAX_MIN) / 100;
            if (v != parallax_y) { parallax_y = v; markDirty(); }
          } else if (hit_z && dir && nudgeReady(now)) {
            int v = constrain((int)zoom_pct + dir * ZOOM_NUDGE_PCT, 100, 250);
            if (v != zoom_pct) { zoom_pct = v; markDirty(); }
          } else if (hit_z && track_hit) {
            int pct = sliderPctFromTrack(tx, PANEL_SLIDER_X, PANEL_SLIDER_W);
            int v = 100 + pct * 150 / 100;
            if (v != zoom_pct) { zoom_pct = v; markDirty(); }
          }
          drawAdjustSliders();
        }
        return;
      }
    } else if (panel_tab == PANEL_TAB_RANGE && range_mode == RANGE_MANUAL) {
      // Convert pct → temperature on the MANUAL_T_MIN..MAX scale.
      bool hit_lo = sliderRowHit(tx, ty, PANEL_SLIDER_X, X_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H);
      bool hit_hi = sliderRowHit(tx, ty, PANEL_SLIDER_X, Y_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H);
      if (hit_lo) {
        // LO can't go above HI (clamped to HI - 1 to keep at least 1 °C span).
        if (dir && nudgeReady(now)) {
          setManualLoValue(manual_lo + dir * MANUAL_NUDGE_C);
        } else if (track_hit) {
          int pct = sliderPctFromTrack(tx, PANEL_SLIDER_X, PANEL_SLIDER_W);
          setManualLoValue(MANUAL_T_MIN + pct * (MANUAL_T_MAX - MANUAL_T_MIN) / 100.0f);
        }
        drawAdjustSliders();
        return;
      }
      if (hit_hi) {
        if (dir && nudgeReady(now)) {
          setManualHiValue(manual_hi + dir * MANUAL_NUDGE_C);
        } else if (track_hit) {
          int pct = sliderPctFromTrack(tx, PANEL_SLIDER_X, PANEL_SLIDER_W);
          setManualHiValue(MANUAL_T_MIN + pct * (MANUAL_T_MAX - MANUAL_T_MIN) / 100.0f);
        }
        drawAdjustSliders();
        return;
      }
    }
  }

  // Brightness cells — discrete, rate-limited so a tap doesn't fire multiple
  // cells when the finger lingers across a boundary.
  if (now - last_touch_ms >= 250 &&
      ty >= BRT_BAR_Y && ty < BRT_BAR_Y + BRT_CELL_H &&
      tx >= BRT_BAR_X && tx < BRT_BAR_X + BRT_NUM_CELLS * BRT_CELL_W) {
    int cell = (tx - BRT_BAR_X) / BRT_CELL_W;        // 0..4
    if (cell < 0) cell = 0; else if (cell >= BRT_NUM_CELLS) cell = BRT_NUM_CELLS - 1;
    int8_t new_brt = (int8_t)(cell - 2);             // -2..+2
    if (new_brt != cam_brightness) {
      cam_brightness = new_brt;
      brightness_apply_pending = true;
      markDirty();
      drawBrightnessSlider();
    }
    last_touch_ms = now;
    return;
  }
  if (now - last_touch_ms < 350) return;
  for (int i = 0; i < NUM_BUTTONS; i++) {
    auto &b = buttons[i];
    if (tx >= b.x && tx < b.x + b.w && ty >= b.y && ty < b.y + b.h) {
      b.action(); last_touch_ms = now;
      lcd.fillRoundRect(b.x, b.y, b.w, b.h, 4, TFT_WHITE);
      delay(40); drawButtons(); drawAdjustSliders();
      return;
    }
  }
}

// Short release = freeze/share toggle. Holding for 3 s toggles the browser
// stream portal and suppresses the short-press event on release.
// 15 ms debounce on transitions.
//
// Return code is plain int rather than an enum because the Arduino IDE
// auto-generates forward declarations at the top of the .ino file, before
// any user-defined enum exists — using `int` dodges that trap.
#define BTN_NONE  0
#define BTN_SHORT 1
#define BTN_LONG  2
static constexpr uint32_t BTN_LONG_MS = 3000;

struct BtnState {
  bool last = HIGH;
  bool stable = HIGH;
  uint32_t last_change_ms = 0;
  uint32_t pressed_ms = 0;
  bool long_fired = false;
} btn;

int pollButton() {
  bool raw = digitalRead(BUTTON_PIN);
  uint32_t now = millis();

  // Reset the debounce timer if the raw pin state is fluctuating
  if (raw != btn.last) { 
    btn.last_change_ms = now; 
    btn.last = raw; 
  }

  // If the pin has been stable for 15ms and it's a new state
  if (now - btn.last_change_ms >= 15 && btn.stable != btn.last) {
    bool prev = btn.stable;
    btn.stable = btn.last;

    if (prev == HIGH && btn.stable == LOW) {
      btn.pressed_ms = now;
      btn.long_fired = false;
    } else if (prev == LOW && btn.stable == HIGH) {
      uint32_t held = now - btn.pressed_ms;
      if (!btn.long_fired && held < BTN_LONG_MS) return BTN_SHORT;
    }
  }

  if (btn.stable == LOW && !btn.long_fired &&
      btn.pressed_ms && now - btn.pressed_ms >= BTN_LONG_MS) {
    btn.long_fired = true;
    return BTN_LONG;
  }
  
  return BTN_NONE;
}

// ---------------------------------------------------------------- Render --
//
// Chunk buffer in DRAM (fast). The render loops write swap565/wire-format
// pixels and push with setSwapBytes(false), matching the camera fast path and
// avoiding LovyanGFX's internal byte-swap copy buffer.
//
// Per thermal frame we precompute one 8-bit palette index for each of the 768
// cells. During render we bilerp that scalar index at camera resolution, then
// look up RGB/pixel values. That is cheaper than bilerp'ing R, G and B
// separately and avoids rebuilding thermal colour data for every camera frame.
//
// Tint algorithm (additive, dark-object-friendly):
//   outC = clamp255( cam_C + palC · p / 100 )      per channel
// At p=0 you get the camera unchanged; at p=100 every channel gets the full
// palette colour added on top, capped at 255.
// v15 used a luminance-preserving blend (palC * Y / 255) which multiplied
// the thermal colour by camera brightness — so a hot finger on a dark sleeve
// became muddy, because the dark camera kills the heat tint. Additive blend
// fixes that: heat ALWAYS brightens the pixel, regardless of how dark the
// scene is. The iron-bow palette starts at near-black, so cold pixels add
// almost nothing and don't blue-tint the camera image.
// ============================================================================
// Chunked LCD output buffer — we render N rows at a time and push them in
// a single pushImage() so the per-call SPI/DMA setup overhead (CASET/PASET/
// RAMWR command sequence, DMA descriptor build, lock acquire) is paid once
// per chunk instead of once per row. With CHUNK_H=16 the overhead drops 16×
// and the actual data transfer dominates as it should. 10 KB DRAM cost.
//
// Sizing rationale: bigger chunks reduce overhead but eat DRAM. 16 rows at
// 320×2 bytes = 10 KB fits easily and gets us within ~10% of the maximum
// theoretical SPI throughput on this panel.
#define CHUNK_H 16
DRAM_ATTR uint16_t chunk_buf[CHUNK_H * IMG_W];

uint8_t therm_idx[768];
uint8_t therm_idx_prev[768];
uint8_t therm_idx_target[768];
float   therm_build_src[768];
bool     therm_idx_valid = false;
uint32_t therm_idx_seq = 0;
float    therm_idx_lo = 0.0f, therm_idx_hi = 0.0f;
uint32_t therm_idx_transition_ms = 0;
static constexpr uint32_t THERMAL_BLEND_MS = 35;

void updateThermalBlend(uint32_t now) {
  if (!therm_idx_valid) return;

  uint32_t elapsed = now - therm_idx_transition_ms;
  if (elapsed >= THERMAL_BLEND_MS) {
    memcpy(therm_idx, therm_idx_target, sizeof(therm_idx));
    return;
  }

  uint16_t alpha = (uint16_t)(elapsed * 255UL / THERMAL_BLEND_MS);
  for (int i = 0; i < 768; i++) {
    therm_idx[i] = (uint8_t)(((uint16_t)therm_idx_prev[i] * (255 - alpha) +
                              (uint16_t)therm_idx_target[i] * alpha + 127) / 255);
  }
}

// Build per-cell palette indices. Pre-mirrored in X so the render loop can
// bilerp without index gymnastics. (The MLX is mounted such that its X axis
// is reversed relative to the camera; flipping here once per frame is
// cheaper than flipping per pixel.)
void buildThermalPaletteFrame(float lo, float hi) {
  uint32_t seq = thermal_frame_seq;
  uint32_t now = millis();
  bool same_target = therm_idx_valid &&
                     therm_idx_seq == seq &&
                     therm_idx_lo == lo &&
                     therm_idx_hi == hi;

  if (same_target) {
    updateThermalBlend(now);
    return;
  }

  if (therm_idx_valid) updateThermalBlend(now);

  portENTER_CRITICAL(&mlx_mux);
  seq = thermal_frame_seq;
  memcpy(therm_build_src, mlx_temps_full, sizeof(therm_build_src));
  portEXIT_CRITICAL(&mlx_mux);

  float denom = (hi - lo) + 0.001f;
  // Source from the snapshotted complete frame, NOT the chess-mode accumulator.
  // See comments around mlx_temps_full for why this matters.
  for (int row = 0; row < 24; row++) {
    for (int col = 0; col < 32; col++) {
      float t = therm_build_src[row * 32 + (31 - col)];
      float f = (t - lo) / denom;
      // Clip f into [0,1]. The negation pattern catches NaN too —
      // `f < 0` is FALSE for NaN, so a plain compare lets NaN through and
      // the (int) cast then saturates to INT_MAX, which indexes
      // palR[INT_MAX] and reliably crashes. `!(f >= 0)` is TRUE for NaN.
      // Defence-in-depth: analyzeMLX should never let NaN reach us here,
      // but if it ever does, this stays safe.
      if (!(f >= 0.0f)) f = 0.0f;
      else if (f > 1.0f) f = 1.0f;
      int idx = (int)(f * 255.0f + 0.5f);
      if (idx < 0) idx = 0; else if (idx > 255) idx = 255;
      int dst = row * 32 + col;
      therm_idx_target[dst] = (uint8_t)idx;
    }
  }

  if (therm_idx_valid) {
    memcpy(therm_idx_prev, therm_idx, sizeof(therm_idx_prev));
  } else {
    memcpy(therm_idx_prev, therm_idx_target, sizeof(therm_idx_prev));
    memcpy(therm_idx, therm_idx_target, sizeof(therm_idx));
  }

  therm_idx_valid = true;
  therm_idx_seq = seq;
  therm_idx_lo = lo;
  therm_idx_hi = hi;
  therm_idx_transition_ms = now;
  updateThermalBlend(now);
}

// Bilerp one byte from the 32x24 thermal cell grid.
// tx_q / ty_q are 16.16 fixed-point cell coordinates (0..32, 0..24).
static inline uint8_t bilerp_cell(const uint8_t *src,
                                  int tx0, int tx1, int ty0, int ty1,
                                  uint16_t fx, uint16_t fy)
{
  uint16_t ifx = 256 - fx, ify = 256 - fy;
  uint32_t a = src[ty0 * 32 + tx0];
  uint32_t b = src[ty0 * 32 + tx1];
  uint32_t c = src[ty1 * 32 + tx0];
  uint32_t d = src[ty1 * 32 + tx1];
  uint32_t top    = (a * ifx + b * fx) >> 8;
  uint32_t bottom = (c * ifx + d * fx) >> 8;
  return (uint8_t)((top * ify + bottom * fy) >> 8);
}

// 16.16 fixed-point map from camera-pixel offset (relative to the thermal
// centre in camera coords) → thermal cell coordinate. step_q is in cells per
// camera pixel; centre_cell shifts the origin so that delta=0 lands on the
// middle of the thermal grid (16, 12).
static inline void thermal_indices_fov(int delta, int32_t step_q,
                                       int center_cell, int max_cell,
                                       int &c0, int &c1,
                                       uint16_t &frac, bool &inside)
{
  int32_t q = (int32_t)delta * step_q + ((int32_t)center_cell << 16);
  int idx = q >> 16;
  inside = (idx >= 0 && idx <= max_cell);
  if (idx < 0)              { c0 = c1 = 0;        frac = 0; }
  else if (idx >= max_cell) { c0 = c1 = max_cell; frac = 0; }
  else                      { c0 = idx; c1 = idx + 1; frac = (q >> 8) & 0xFF; }
}

// Thermal grid fills the full 320×240 image area: 32×24 cells → 10×10 px.
static constexpr int32_t TH_STEP_X_Q = (int32_t)(((int64_t)32 << 16) / IMG_W);
static constexpr int32_t TH_STEP_Y_Q = (int32_t)(((int64_t)24 << 16) / IMG_H);
static inline void thermal_steps(int32_t &step_x_q, int32_t &step_y_q) {
  step_x_q = TH_STEP_X_Q;
  step_y_q = TH_STEP_Y_Q;
}

// Camera digital crop: zoom and parallax both affect this. At zoom = 100%
// the full 320×240 frame is shown. At zoom > 100%, a centred sub-rect of
// (IMG_W/zoom × IMG_H/zoom) is sampled and stretched. Parallax then offsets
// the centre of that crop rect — in OUTPUT-PIXEL units, so a +10 px shift
// looks the same on screen at any zoom level. Negative shift in source
// coordinates because moving the visible image RIGHT on screen requires
// sampling further LEFT in the source frame.
//
// Output via reference parameters rather than a struct so the Arduino IDE's
// auto-generated forward declaration doesn't trip over an undeclared type.
static inline void cameraCropParamsFor(int z, int px, int py,
                                       int &src_x0, int &src_y0,
                                       int32_t &step_x_q, int32_t &step_y_q)
{
  if (z < 100) z = 100;
  int cw = (IMG_W * 100 + z / 2) / z;
  int ch = (IMG_H * 100 + z / 2) / z;
  if (cw > IMG_W) cw = IMG_W;
  if (ch > IMG_H) ch = IMG_H;
  // Convert parallax (output px) → source px shift via (cw/IMG_W).
  int shift_src_x = px * cw / IMG_W;
  int shift_src_y = py * ch / IMG_H;
  src_x0 = (IMG_W - cw) / 2 - shift_src_x;
  src_y0 = (IMG_H - ch) / 2 - shift_src_y;
  step_x_q = (int32_t)(((int64_t)cw << 16) / IMG_W);
  step_y_q = (int32_t)(((int64_t)ch << 16) / IMG_H);
}

static inline void cameraCropParams(int &src_x0, int &src_y0,
                                    int32_t &step_x_q, int32_t &step_y_q)
{
  cameraCropParamsFor(zoom_pct, parallax_x, parallax_y,
                      src_x0, src_y0, step_x_q, step_y_q);
}

void renderThermalOnly();      // forward decl

void renderTinted() {
  // No camera (disconnected / init failed) — gracefully fall back so the
  // user still sees the thermal data instead of a frozen screen.
  if (!cam_ok || !cam_have_frame) { renderThermalOnly(); return; }
  float lo, hi; getThermalRange(lo, hi);
  buildThermalPaletteFrame(lo, hi);

  uint16_t *cam = (uint16_t*)cam_snapshot;
  uint8_t pct = tint_pct;        // additive blend uses pct directly

  int32_t step_x_q, step_y_q;
  thermal_steps(step_x_q, step_y_q);
  // Thermal centre is FIXED at the middle of the image — parallax/zoom
  // only adjust the camera crop, so the thermal grid never moves on screen.
  const int cx_center = IMG_W / 2;
  const int cy_center = IMG_H / 2;

  int crop_x0, crop_y0; int32_t crop_step_x_q, crop_step_y_q;
  cameraCropParams(crop_x0, crop_y0, crop_step_x_q, crop_step_y_q);

  lcd.setSwapBytes(false);
  lcd.startWrite();
  for (int y0 = 0; y0 < IMG_H; y0 += CHUNK_H) {
    int chunk_h = (y0 + CHUNK_H <= IMG_H) ? CHUNK_H : (IMG_H - y0);
    for (int dy = 0; dy < chunk_h; dy++) {
      int y = y0 + dy;
      uint16_t *row = &chunk_buf[dy * IMG_W];

      int ty0, ty1; uint16_t fy; bool y_inside;
      thermal_indices_fov(y - cy_center, step_y_q, 12, 23, ty0, ty1, fy, y_inside);

      int src_y = crop_y0 + (int)(((int64_t)y * crop_step_y_q) >> 16);
      if (src_y < 0) src_y = 0; else if (src_y >= IMG_H) src_y = IMG_H - 1;
      uint16_t *cam_row = &cam[src_y * IMG_W];

      for (int x = 0; x < IMG_W; x++) {
        int src_x = crop_x0 + (int)(((int64_t)x * crop_step_x_q) >> 16);
        if (src_x < 0) src_x = 0; else if (src_x >= IMG_W) src_x = IMG_W - 1;

        uint16_t cbe = cam_row[src_x];
        uint16_t out = cbe;

        if (y_inside && pct > 0) {
          int tx0, tx1; uint16_t fx; bool x_inside;
          thermal_indices_fov(x - cx_center, step_x_q, 16, 31, tx0, tx1, fx, x_inside);
          if (x_inside) {
#if USE_BILINEAR_THERMAL
            uint8_t pidx = bilerp_cell(therm_idx, tx0, tx1, ty0, ty1, fx, fy);
#else
            uint8_t pidx = therm_idx[ty0 * 32 + tx0];
#endif
            uint8_t pr = palR[pidx];
            uint8_t pg = palG[pidx];
            uint8_t pb = palB[pidx];

            // Wire-format swap565 -> native rgb565 for arithmetic.
            uint16_t cnat = swap565(cbe);
            uint8_t cr = ((cnat >> 11) & 0x1F) << 3;
            uint8_t cg = ((cnat >> 5)  & 0x3F) << 2;
            uint8_t cb =  (cnat        & 0x1F) << 3;

            // Additive blend — heat is added, then clamped at 255.
            uint16_t r = (uint16_t)cr + (uint16_t)pr * pct / 100;
            uint16_t g = (uint16_t)cg + (uint16_t)pg * pct / 100;
            uint16_t b = (uint16_t)cb + (uint16_t)pb * pct / 100;
            if (r > 255) r = 255;
            if (g > 255) g = 255;
            if (b > 255) b = 255;
            uint16_t onat = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
            out = swap565(onat);
          }
        }
        row[x] = isCrosshairPixel(x, y) ? TFT_WHITE : out;
      }
    }
    lcd.pushImage(IMG_X, IMG_Y + y0, IMG_W, chunk_h, chunk_buf);
  }
  lcd.endWrite();
}

void renderThermalOnly() {
  float lo, hi; getThermalRange(lo, hi);
  buildThermalPaletteFrame(lo, hi);

  int32_t step_x_q, step_y_q;
  thermal_steps(step_x_q, step_y_q);
  const int cx_center = IMG_W / 2;
  const int cy_center = IMG_H / 2;

  lcd.setSwapBytes(false);
  lcd.startWrite();
  for (int y0 = 0; y0 < IMG_H; y0 += CHUNK_H) {
    int chunk_h = (y0 + CHUNK_H <= IMG_H) ? CHUNK_H : (IMG_H - y0);
    for (int dy = 0; dy < chunk_h; dy++) {
      int y = y0 + dy;
      uint16_t *row = &chunk_buf[dy * IMG_W];

      int ty0, ty1; uint16_t fy; bool y_inside;
      thermal_indices_fov(y - cy_center, step_y_q, 12, 23, ty0, ty1, fy, y_inside);

      for (int x = 0; x < IMG_W; x++) {
        uint16_t out = 0;
        if (y_inside) {
          int tx0, tx1; uint16_t fx; bool x_inside;
          thermal_indices_fov(x - cx_center, step_x_q, 16, 31, tx0, tx1, fx, x_inside);
          if (x_inside) {
#if USE_BILINEAR_THERMAL
            uint8_t pidx = bilerp_cell(therm_idx, tx0, tx1, ty0, ty1, fx, fy);
#else
            uint8_t pidx = therm_idx[ty0 * 32 + tx0];
#endif
            out = palette_wire[pidx];
          }
        }
        row[x] = isCrosshairPixel(x, y) ? TFT_WHITE : out;
      }
    }
    lcd.pushImage(IMG_X, IMG_Y + y0, IMG_W, chunk_h, chunk_buf);
  }
  lcd.endWrite();
}

void renderCameraOnly() {
  if (!cam_have_frame) {
    // No camera — paint a placeholder so the screen doesn't look frozen.
    lcd.fillRect(IMG_X, IMG_Y, IMG_W, IMG_H, TFT_BLACK);
    lcd.setTextColor(TFT_RED, TFT_BLACK);
    lcd.setTextSize(2);
    const char *msg = cam_ok ? "WAITING FOR CAMERA" : cameraDisplayMessage();
    int tw = (int)strlen(msg) * 12;
    lcd.setCursor(IMG_X + (IMG_W - tw) / 2, IMG_Y + IMG_H / 2 - 12);
    lcd.print(msg);
    lcd.setTextSize(1);
    lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    char detail[64];
    if (cam_last_err != ESP_OK) {
      snprintf(detail, sizeof(detail), "%s 0x%X",
               cameraStageName(cam_fail_stage), (unsigned)cam_last_err);
    } else if (cam_last_frame_len && cam_last_frame_len != cam_snapshot_len) {
      snprintf(detail, sizeof(detail), "len %u expected %u",
               (unsigned)cam_last_frame_len, (unsigned)cam_snapshot_len);
    } else {
      snprintf(detail, sizeof(detail), "%s", cameraStageName(cam_fail_stage));
    }
    tw = (int)strlen(detail) * 6;
    lcd.setCursor(IMG_X + (IMG_W - tw) / 2, IMG_Y + IMG_H / 2 + 14);
    lcd.print(detail);
    lcd.setTextSize(1);
    return;
  }
  // Fast path at zoom = 100% — no resampling needed.
  if (zoom_pct == 100) {
    lcd.setSwapBytes(false);  // cam_snapshot is wire-format (swap565)
    uint16_t *cam = (uint16_t*)cam_snapshot;
    lcd.startWrite();
    for (int y0 = 0; y0 < IMG_H; y0 += CHUNK_H) {
      int chunk_h = (y0 + CHUNK_H <= IMG_H) ? CHUNK_H : (IMG_H - y0);
      for (int dy = 0; dy < chunk_h; dy++) {
        int y = y0 + dy;
        uint16_t *row = &chunk_buf[dy * IMG_W];
        uint16_t *cam_row = &cam[y * IMG_W];
        for (int x = 0; x < IMG_W; x++) {
          row[x] = isCrosshairPixel(x, y) ? TFT_WHITE : cam_row[x];
        }
      }
      lcd.pushImage(IMG_X, IMG_Y + y0, IMG_W, chunk_h, chunk_buf);
    }
    lcd.endWrite();
    return;
  }
  // Cropped zoom — read from a centred sub-rect, stretch to full output.
  uint16_t *cam = (uint16_t*)cam_snapshot;
  int crop_x0, crop_y0; int32_t crop_step_x_q, crop_step_y_q;
  cameraCropParams(crop_x0, crop_y0, crop_step_x_q, crop_step_y_q);
  lcd.setSwapBytes(false);    // chunk_buf carries wire-format swap565 in this path
  lcd.startWrite();
  for (int y0 = 0; y0 < IMG_H; y0 += CHUNK_H) {
    int chunk_h = (y0 + CHUNK_H <= IMG_H) ? CHUNK_H : (IMG_H - y0);
    for (int dy = 0; dy < chunk_h; dy++) {
      int y = y0 + dy;
      uint16_t *row = &chunk_buf[dy * IMG_W];
      int src_y = crop_y0 + (int)(((int64_t)y * crop_step_y_q) >> 16);
      if (src_y < 0) src_y = 0; else if (src_y >= IMG_H) src_y = IMG_H - 1;
      uint16_t *cam_row = &cam[src_y * IMG_W];
      for (int x = 0; x < IMG_W; x++) {
        int src_x = crop_x0 + (int)(((int64_t)x * crop_step_x_q) >> 16);
        if (src_x < 0) src_x = 0; else if (src_x >= IMG_W) src_x = IMG_W - 1;
        row[x] = isCrosshairPixel(x, y) ? TFT_WHITE : cam_row[src_x];
      }
    }
    lcd.pushImage(IMG_X, IMG_Y + y0, IMG_W, chunk_h, chunk_buf);
  }
  lcd.endWrite();
}

// ------------------------------------------------------------- Freeze Share
// Short-press freeze also opens a local WiFi AP. Connect to the SSID shown on
// the screen or printed on Serial, then browse to http://192.168.4.1/ to save
// BMP/CSV exports of the frozen RAM snapshot. This avoids SD-card/LCD bus
// issues entirely.
static constexpr uint8_t SHARE_DNS_PORT = 53;
WebServer share_server(80);
DNSServer share_dns;

uint8_t *freeze_cam_snapshot = nullptr;
float freeze_thermal_snapshot[768];
bool freeze_cam_valid = false;
bool freeze_thermal_valid = false;
bool share_ap_running = false;
bool share_routes_configured = false;
char share_ap_ssid[32] = "ThermalCam";
uint32_t freeze_capture_ms = 0;
float freeze_range_lo = 20.0f, freeze_range_hi = 30.0f;
int freeze_zoom_pct = 124;
int freeze_parallax_x = 0, freeze_parallax_y = 0;
uint8_t freeze_tint_pct = 70;
DRAM_ATTR uint8_t share_bmp_line[IMG_W * 3];
DRAM_ATTR int16_t stream_thermal_packet[768];
uint32_t stream_started_ms = 0;
uint32_t stream_last_status_ms = 0;
uint32_t stream_api_count = 0;
uint32_t stream_api_timer_ms = 0;
float stream_api_fps = 0.0f;
uint32_t stream_cam_req_count = 0;
uint32_t stream_cam_req_timer_ms = 0;
float stream_cam_req_fps = 0.0f;
uint32_t stream_thermal_req_count = 0;
uint32_t stream_thermal_req_timer_ms = 0;
float stream_thermal_req_fps = 0.0f;
uint32_t stream_jpeg_fail_count = 0;
bool stream_stop_pending = false;

static constexpr uint8_t STREAM_VIEW_OVERLAY = 0;
static constexpr uint8_t STREAM_VIEW_CAMERA  = 1;
static constexpr uint8_t STREAM_VIEW_THERMAL = 2;
static constexpr uint8_t STREAM_JPEG_QUALITY = 72;

void setStreamMode(bool enabled);
extern float current_fps, cam_fps, mlx_fps;
extern uint32_t last_ui_ms;

void noteStreamRequest(uint32_t &count, uint32_t &timer_ms, float &fps) {
  count++;
  uint32_t now = millis();
  if (!timer_ms) timer_ms = now;
  if (now - timer_ms >= 1000) {
    fps = count * 1000.0f / (now - timer_ms);
    count = 0;
    timer_ms = now;
  }
}

void resetStreamDiagnostics(uint32_t now) {
  stream_api_count = 0;
  stream_api_timer_ms = now;
  stream_api_fps = 0.0f;
  stream_cam_req_count = 0;
  stream_cam_req_timer_ms = now;
  stream_cam_req_fps = 0.0f;
  stream_thermal_req_count = 0;
  stream_thermal_req_timer_ms = now;
  stream_thermal_req_fps = 0.0f;
  stream_jpeg_fail_count = 0;
}

static const char STREAM_PORTAL_HTML[] PROGMEM = R"STREAMHTML(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Thermal Stream</title>
<style>
body{margin:0;background:#0b0d0f;color:#e8edf2;font-family:system-ui,-apple-system,Segoe UI,sans-serif}main{max-width:1180px;margin:auto;padding:12px}.top{display:flex;gap:8px;align-items:center;justify-content:space-between;flex-wrap:wrap}.badge{background:#1c252d;border:1px solid #384652;border-radius:6px;padding:5px 8px;color:#b8c7d3}.bad{background:#3a1b20;border-color:#8a2630;color:#ffd1d1}canvas{width:100%;height:auto;background:#000;image-rendering:auto}.grid{display:grid;grid-template-columns:minmax(280px,640px) 1fr;gap:12px;margin-top:10px}.panel{background:#12171c;border:1px solid #28343d;border-radius:8px;padding:10px}.row{display:grid;grid-template-columns:96px 1fr 52px;gap:8px;align-items:center;margin:8px 0}.twocol{display:grid;grid-template-columns:1fr 1fr;gap:8px}button,select,input{font:inherit}button,select{background:#20303b;color:#fff;border:1px solid #425564;border-radius:6px;padding:8px}button.primary{background:#1f7a4d}button.danger{background:#8a2630}input[type=range]{width:100%}.val{color:#9fe870;text-align:right;font-variant-numeric:tabular-nums}.hud{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-top:8px}.hud div{background:#0d1115;border:1px solid #26323b;border-radius:6px;padding:6px;color:#c9d5dd}.small{font-size:12px;color:#9aa8b3}.rec{color:#ffb4b4}.hidden{display:none}@media(max-width:760px){.grid,.twocol{grid-template-columns:1fr}.row{grid-template-columns:82px 1fr 48px}}
</style></head><body><main>
<div class="top"><h2>Thermal Stream</h2><div><span class="badge" id="ssid">AP</span> <span class="badge" id="ip">192.168.4.1</span> <span class="badge" id="connState">online</span> <span class="badge" id="recState">idle</span></div></div>
<div class="grid">
<section class="panel"><canvas id="view" width="320" height="240"></canvas><canvas id="recCanvas" width="320" height="240" class="hidden"></canvas>
<div class="hud"><div>Center <b id="tCenter">--</b>C</div><div>Scene <b id="tSpan">--</b>C</div><div>Range <b id="tRange">--</b>C</div><div>Cam <b id="camFps">--</b>fps</div><div>Cam Req <b id="camReqFps">--</b>fps</div><div>MLX <b id="mlxFps">--</b>fps</div><div>Therm Req <b id="thermalReqFps">--</b>fps</div><div>API <b id="apiFps">--</b>fps</div><div>Loop <b id="loopFps">--</b>fps</div><div>Marker <b id="markerTemp">--</b>C</div><div>Heap <b id="heap">--</b></div><div>PSRAM <b id="psram">--</b></div><div>Seq <b id="seq">--</b></div><div>Uptime <b id="uptime">--</b></div></div>
<div class="twocol" style="margin-top:10px"><button class="primary" id="recBtn">Start recording</button><button class="danger" id="stopPortal">Stop portal</button></div>
<p class="small" id="saveMsg">Recording is held in browser memory until stopped. Use short clips; if recording is not supported, use the phone screen recorder.</p></section>
<section class="panel">
<div class="row"><label>View</label><select id="viewMode"><option value="0">Overlay</option><option value="1">Camera only</option><option value="2">Thermal only</option></select><span></span></div>
<div class="row"><label>Range</label><select id="rangeMode"><option value="0">AUTO</option><option value="1">AUT2</option><option value="2">SKIN</option><option value="3">BODY</option><option value="4">WARM</option><option value="5">HOT</option><option value="6">VHOT</option><option value="7">MAN</option></select><span></span></div>
<div class="row"><label>X</label><input id="px" type="range" min="-90" max="90" step="1"><span class="val" id="pxv"></span></div>
<div class="row"><label>Y</label><input id="py" type="range" min="-90" max="90" step="1"><span class="val" id="pyv"></span></div>
<div class="row"><label>Zoom</label><input id="zoom" type="range" min="100" max="250" step="1"><span class="val" id="zoomv"></span></div>
<div class="row"><label>Tint</label><input id="tint" type="range" min="0" max="100" step="1"><span class="val" id="tintv"></span></div>
<div class="row"><label>Manual LO</label><input id="mlo" type="range" min="5" max="60" step="0.1"><span class="val" id="mlov"></span></div>
<div class="row"><label>Manual HI</label><input id="mhi" type="range" min="5" max="60" step="0.1"><span class="val" id="mhiv"></span></div>
<div class="row"><label>Brightness</label><input id="brt" type="range" min="-2" max="2" step="1"><span class="val" id="brtv"></span></div>
<div class="twocol"><button id="resetAlign">Reset alignment</button><button id="snap">Snapshot</button></div>
<p><label><input type="checkbox" id="hud" checked> Show HUD</label></p>
<p><label><input type="checkbox" id="recHud" checked> Record HUD</label></p>
<p><label><input type="checkbox" id="crosshair" checked> Crosshair</label></p>
</section></div></main>
<script>
const W=320,H=240,canvas=document.getElementById('view'),ctx=canvas.getContext('2d'),recCanvas=document.getElementById('recCanvas'),recCtx=recCanvas.getContext('2d');
const thermalCanvas=document.createElement('canvas');thermalCanvas.width=W;thermalCanvas.height=H;const thermalCtx=thermalCanvas.getContext('2d');
let S={view:0,range:0,px:0,py:0,zoom:124,tint:70,mlo:22,mhi:38,brt:0,hud:1,crosshair:1,lo:20,hi:30,tCenter:0,tMin:0,tMax:0,seq:0,camFps:0,camReqFps:0,mlxFps:0,thermalReqFps:0,apiFps:0,loopFps:0,heap:0,psram:0,portalMs:0};
let camImg=null,camUrl=null,thermal=null,dirtyThermal=true,marker=null,markerTemp=null,rec=null,chunks=[],recUrl=null,uiReady=false,running=true,stopping=false;
let camTimer=0,thermalTimer=0,stateTimer=0,camBusy=false,thermalBusy=false,stateBusy=false,camFail=0,thermalFail=0,stateFail=0,overlayCamMs=83,camFastStreak=0,recStarted=0,recTick=0,recStopResolve=null;
const activeControls=new Set();
const $=id=>document.getElementById(id);
function clamp(v,a,b){return Math.max(a,Math.min(b,v))}
function pal(i){let j=i*180/255,R,G,B;if(j<30){R=0;G=0;B=20+120*j/30}else if(j<60){R=120*(j-30)/30;G=0;B=140-60*(j-30)/30}else if(j<90){R=120+135*(j-60)/30;G=0;B=80-70*(j-60)/30}else if(j<120){R=255;G=60*(j-90)/30;B=10-10*(j-90)/30}else if(j<150){R=255;G=60+175*(j-120)/30;B=0}else{R=255;G=235+20*(j-150)/30;B=255*(j-150)/30}return[R|0,G|0,B|0]}
const PAL=Array.from({length:256},(_,i)=>pal(i));
function fetchTimeout(url,opt={},ms=1500){if(!window.AbortController)return fetch(url,opt);let c=new AbortController(),t=setTimeout(()=>c.abort(),ms);return fetch(url,Object.assign({},opt,{signal:c.signal})).finally(()=>clearTimeout(t))}
function setConn(txt,bad){let e=$('connState');e.textContent=txt;e.classList.toggle('bad',!!bad)}
function backoff(n){return Math.min(2000,250*Math.pow(2,Math.min(n,3)))}
function tempAt(x,y){if(!thermal)return null;let tx=x*32/W,ty=y*24/H,x0=clamp(Math.floor(tx),0,31),y0=clamp(Math.floor(ty),0,23),x1=clamp(x0+1,0,31),y1=clamp(y0+1,0,23),fx=tx-x0,fy=ty-y0;function t(r,c){return thermal[r*32+(31-c)]}let a=t(y0,x0),b=t(y0,x1),c=t(y1,x0),d=t(y1,x1);return (a*(1-fx)+b*fx)*(1-fy)+(c*(1-fx)+d*fx)*fy}
function rebuildThermal(){if(!thermal)return;let img=thermalCtx.createImageData(W,H),d=img.data,lo=S.lo,hi=S.hi,den=(hi-lo)||1;for(let y=0;y<H;y++){for(let x=0;x<W;x++){let t=tempAt(x,y),idx=clamp(Math.round((t-lo)/den*255),0,255),p=PAL[idx],o=(y*W+x)*4;d[o]=p[0];d[o+1]=p[1];d[o+2]=p[2];d[o+3]=255}}thermalCtx.putImageData(img,0,0);dirtyThermal=false}
function drawCamera(c){if(!camImg)return;let z=clamp(+S.zoom||100,100,250),cw=Math.round(W*100/z),ch=Math.round(H*100/z),sx=(W-cw)/2-(S.px||0)*cw/W,sy=(H-ch)/2-(S.py||0)*ch/H;sx=clamp(sx,0,W-cw);sy=clamp(sy,0,H-ch);c.drawImage(camImg,sx,sy,cw,ch,0,0,W,H)}
function drawHud(c){c.save();c.font='12px system-ui';c.fillStyle='rgba(0,0,0,.62)';c.fillRect(4,4,164,54);c.fillStyle='#fff';c.fillText(`Ctr ${fmt(S.tCenter)}C  Scene ${fmt(S.tMin)}-${fmt(S.tMax)}C`,10,20);c.fillText(`Range ${fmt(S.lo)}-${fmt(S.hi)}C  MLX ${fmt(S.mlxFps)}fps`,10,36);if(marker&&markerTemp!=null)c.fillText(`Marker ${fmt(markerTemp)}C`,10,52);c.restore()}
function drawCross(c){c.save();c.strokeStyle='#fff';c.lineWidth=1;c.beginPath();c.moveTo(W/2-10,H/2);c.lineTo(W/2+10,H/2);c.moveTo(W/2,H/2-10);c.lineTo(W/2,H/2+10);c.rect(W/2-10,H/2-10,20,20);c.stroke();c.restore()}
function drawScene(c,withHud){c.clearRect(0,0,W,H);c.fillStyle='#000';c.fillRect(0,0,W,H);let v=+S.view;if((v===0||v===1)&&camImg)drawCamera(c);if((v===0||v===2)&&thermal){if(dirtyThermal)rebuildThermal();c.save();if(v===0){c.globalCompositeOperation='lighter';c.globalAlpha=(+S.tint||0)/100}c.drawImage(thermalCanvas,0,0);c.restore()}if(S.crosshair)drawCross(c);if(marker){c.strokeStyle='#9fe870';c.beginPath();c.arc(marker.x,marker.y,6,0,Math.PI*2);c.stroke()}if(withHud)drawHud(c)}
function render(){drawScene(ctx,$('hud').checked);if(rec)drawScene(recCtx,$('recHud').checked);requestAnimationFrame(render)}
function fmt(v){return Number.isFinite(+v)?(+v).toFixed(1):'--'}
function kb(v){return v?Math.round(v/1024)+'K':'--'}
function mmss(ms){let s=Math.floor((ms||0)/1000),m=Math.floor(s/60);s%=60;return m+':'+String(s).padStart(2,'0')}
function setVal(id,v){let e=$(id);if(e&&!activeControls.has(id))e.value=v}
function setCheck(id,v){let e=$(id);if(e&&!activeControls.has(id))e.checked=!!v}
function updateLabels(){$('pxv').textContent=S.px;$('pyv').textContent=S.py;$('zoomv').textContent=S.zoom+'%';$('tintv').textContent=S.tint+'%';$('mlov').textContent=fmt(S.mlo)+'C';$('mhiv').textContent=fmt(S.mhi)+'C';$('brtv').textContent=S.brt;$('tCenter').textContent=fmt(S.tCenter);$('tSpan').textContent=`${fmt(S.tMin)}-${fmt(S.tMax)}`;$('tRange').textContent=`${fmt(S.lo)}-${fmt(S.hi)}`;$('camFps').textContent=fmt(S.camFps);$('camReqFps').textContent=fmt(S.camReqFps);$('mlxFps').textContent=fmt(S.mlxFps);$('thermalReqFps').textContent=fmt(S.thermalReqFps);$('apiFps').textContent=fmt(S.apiFps);$('loopFps').textContent=fmt(S.loopFps);$('heap').textContent=kb(S.heap);$('psram').textContent=kb(S.psram);$('seq').textContent=S.seq||'--';$('uptime').textContent=mmss(S.portalMs);$('markerTemp').textContent=markerTemp==null?'--':fmt(markerTemp)}
function syncControls(){setVal('px',S.px);setVal('py',S.py);setVal('zoom',S.zoom);setVal('tint',S.tint);setVal('mlo',S.mlo);setVal('mhi',S.mhi);setVal('brt',S.brt);setVal('rangeMode',S.range);setVal('viewMode',S.view);setCheck('hud',S.hud);setCheck('crosshair',S.crosshair);uiReady=true}
function applyState(s){Object.assign(S,s);S.lo=s.rangeLo;S.hi=s.rangeHi;syncControls();dirtyThermal=true;updateLabels();$('ssid').textContent=s.ssid||'ThermalCam';$('ip').textContent=s.ip||'192.168.4.1'}
function camTargetMs(){let v=+S.view;if(v===2)return 0;if(v===1)return 67;return overlayCamMs}
function thermalTargetMs(){return +S.view===1?500:125}
function tuneOverlay(dur){if(+S.view!==0)return;if(dur<60){camFastStreak++;if(camFastStreak>=15)overlayCamMs=67}else camFastStreak=0;if(dur>110)overlayCamMs=120;else if(dur>85)overlayCamMs=100;else if(dur<75&&overlayCamMs>83)overlayCamMs=83}
function scheduleCam(dur=0,fail=false){clearTimeout(camTimer);if(!running||+S.view===2)return;let d=fail?backoff(camFail):Math.max(0,camTargetMs()-dur);camTimer=setTimeout(getCam,d)}
function scheduleThermal(dur=0,fail=false){clearTimeout(thermalTimer);if(!running)return;let d=fail?backoff(thermalFail):Math.max(0,thermalTargetMs()-dur);thermalTimer=setTimeout(getThermal,d)}
function scheduleState(dur=0,fail=false){clearTimeout(stateTimer);if(!running)return;let d=fail?backoff(stateFail):Math.max(0,700-dur);stateTimer=setTimeout(getState,d)}
function kickStreams(){if(+S.view!==2)scheduleCam(999,false);scheduleThermal(999,false)}
async function getState(){if(stateBusy||!running)return;let t=performance.now();stateBusy=true;try{let r=await fetchTimeout('/api/state',{cache:'no-store'},1500);if(!r.ok)throw new Error(r.status);applyState(await r.json());stateFail=0;setConn('online',false)}catch(e){stateFail++;setConn('state retry',true)}finally{stateBusy=false;scheduleState(performance.now()-t,stateFail>0)}}
async function getThermal(){if(thermalBusy||!running)return;let t=performance.now();thermalBusy=true;try{let r=await fetchTimeout('/thermal.bin',{cache:'no-store'},1500);if(!r.ok)throw new Error(r.status);let b=await r.arrayBuffer();if(b.byteLength<1536)throw new Error('short thermal');let dv=new DataView(b),a=new Float32Array(768);for(let i=0;i<768;i++)a[i]=dv.getInt16(i*2,true)/100;thermal=a;S.seq=+(r.headers.get('X-Frame-Seq')||S.seq);S.lo=+(r.headers.get('X-Range-Lo')||S.lo);S.hi=+(r.headers.get('X-Range-Hi')||S.hi);dirtyThermal=true;if(marker)markerTemp=tempAt(marker.x,marker.y);thermalFail=0;setConn('online',false);updateLabels()}catch(e){thermalFail++;setConn('thermal retry',true)}finally{thermalBusy=false;scheduleThermal(performance.now()-t,thermalFail>0)}}
function loadImageUrl(url){return new Promise((resolve,reject)=>{let img=new Image();img.onload=()=>resolve(img);img.onerror=()=>reject(new Error('decode'));img.src=url})}
async function getCam(){if(camBusy||!running||+S.view===2)return;let t=performance.now();camBusy=true;try{let r=await fetchTimeout('/cam.jpg',{cache:'no-store'},1500);if(!r.ok)throw new Error('http '+r.status);let blob=await r.blob();if(!blob.size)throw new Error('empty');let url=URL.createObjectURL(blob);try{let img=await loadImageUrl(url);if(camUrl)URL.revokeObjectURL(camUrl);camUrl=url;camImg=img;camFail=0;setConn('online',false);tuneOverlay(performance.now()-t)}catch(e){URL.revokeObjectURL(url);throw e}}catch(e){camFail++;setConn(e&&e.message==='decode'?'cam decode retry':'cam retry',true)}finally{camBusy=false;scheduleCam(performance.now()-t,camFail>0)}}
let sendTimer=null,pending={};async function postControl(o){let p=new URLSearchParams(o);let r=await fetchTimeout('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p},1800);if(!r.ok)throw new Error(r.status);return r}
function flushSend(){let p=pending;pending={};postControl(p).then(()=>setConn('online',false)).catch(()=>setConn('control retry',true))}
function send(o){Object.assign(pending,o);clearTimeout(sendTimer);sendTimer=setTimeout(flushSend,90)}
['px','py','zoom','tint','mlo','mhi','brt'].forEach(id=>{let e=$(id);['pointerdown','touchstart','focus'].forEach(ev=>e.addEventListener(ev,()=>activeControls.add(id)));['pointerup','pointercancel','touchend','blur','change'].forEach(ev=>e.addEventListener(ev,()=>activeControls.delete(id)));e.addEventListener('input',ev=>{S[id]=+ev.target.value;dirtyThermal=true;updateLabels();send({[id]:ev.target.value})})});
['rangeMode','viewMode','hud','crosshair'].forEach(id=>{let e=$(id);['focus','pointerdown','touchstart'].forEach(ev=>e.addEventListener(ev,()=>activeControls.add(id)));['blur','change','pointerup','touchend'].forEach(ev=>e.addEventListener(ev,()=>activeControls.delete(id)))});
$('rangeMode').onchange=e=>{S.range=+e.target.value;send({range:S.range})};$('viewMode').onchange=e=>{S.view=+e.target.value;dirtyThermal=true;send({view:S.view});kickStreams()};$('hud').onchange=e=>{S.hud=e.target.checked?1:0;send({hud:S.hud})};$('crosshair').onchange=e=>{S.crosshair=e.target.checked?1:0;send({crosshair:S.crosshair})};$('resetAlign').onclick=()=>send({reset_alignment:1});$('stopPortal').onclick=stopPortal;$('snap').onclick=()=>{let a=document.createElement('a');a.download='thermal-frame.png';a.href=canvas.toDataURL('image/png');a.click()};
canvas.onclick=e=>{let r=canvas.getBoundingClientRect();marker={x:(e.clientX-r.left)*W/r.width,y:(e.clientY-r.top)*H/r.height};markerTemp=tempAt(marker.x,marker.y);updateLabels()};
function recType(){let types=['video/mp4','video/webm;codecs=vp9','video/webm;codecs=vp8','video/webm'];return types.find(t=>window.MediaRecorder&&MediaRecorder.isTypeSupported(t))||''}
function updateRecElapsed(){if(!rec)return;let elapsed=Date.now()-recStarted;$('recState').textContent='rec '+mmss(elapsed);if(elapsed>60000)$('saveMsg').textContent='Recording is still in browser memory. Stop before leaving this page.'}
function stopRecording(){if(!rec)return Promise.resolve();return new Promise(resolve=>{recStopResolve=resolve;rec.stop()})}
$('recBtn').onclick=async()=>{if(rec){await stopRecording();return}if(!recCanvas.captureStream||!window.MediaRecorder){$('saveMsg').textContent='Browser recording is unavailable. Use screen recording.';return}chunks=[];let stream=recCanvas.captureStream(15),type=recType();rec=new MediaRecorder(stream,type?{mimeType:type}:undefined);rec.ondataavailable=e=>{if(e.data.size)chunks.push(e.data)};rec.onstop=()=>{clearInterval(recTick);let blob=new Blob(chunks,{type:type||'video/webm'});if(recUrl)URL.revokeObjectURL(recUrl);recUrl=URL.createObjectURL(blob);let a=document.createElement('a');a.href=recUrl;a.download='thermal-stream.'+(type.includes('mp4')?'mp4':'webm');a.textContent='Download recording';$('saveMsg').replaceChildren(a);$('recBtn').textContent='Start recording';$('recState').textContent='idle';$('recState').classList.remove('rec');rec=null;if(recStopResolve){recStopResolve();recStopResolve=null}};rec.start(1000);recStarted=Date.now();recTick=setInterval(updateRecElapsed,500);$('recBtn').textContent='Stop recording';$('recState').classList.add('rec');updateRecElapsed()};
async function stopPortal(){if(stopping)return;stopping=true;$('stopPortal').disabled=true;clearTimeout(sendTimer);pending={};if(rec)await stopRecording();running=false;clearTimeout(camTimer);clearTimeout(thermalTimer);clearTimeout(stateTimer);setConn('stopping',true);try{await postControl({stop_stream:1});$('saveMsg').textContent='Portal stopping. Hold the button for 3 seconds if the browser loses connection first.'}catch(e){$('saveMsg').textContent='Stop request failed. Hold the physical button for 3 seconds to exit.';setConn('stop failed',true)}}
getState();getThermal();getCam();render();
</script></body></html>
)STREAMHTML";

// Plain constants avoid Arduino's auto-prototype pass referencing an enum type
// before its declaration.
static constexpr uint8_t SHARE_BMP_CAMERA = 0;
static constexpr uint8_t SHARE_BMP_THERMAL = 1;
static constexpr uint8_t SHARE_BMP_OVERLAY = 2;

bool ensureFreezeCameraBuffer() {
  if (freeze_cam_snapshot) return true;
  freeze_cam_snapshot = (uint8_t*)heap_caps_malloc(cam_snapshot_len, MALLOC_CAP_SPIRAM);
  if (!freeze_cam_snapshot) {
    Serial.println("Share: no PSRAM for camera snapshot");
    return false;
  }
  return true;
}

void captureFreezeShareSnapshot() {
  freeze_capture_ms = millis();
  freeze_zoom_pct = zoom_pct;
  freeze_parallax_x = parallax_x;
  freeze_parallax_y = parallax_y;
  freeze_tint_pct = tint_pct;
  getThermalRange(freeze_range_lo, freeze_range_hi);

  freeze_cam_valid = false;
  if (cam_have_frame && cam_snapshot && ensureFreezeCameraBuffer()) {
    memcpy(freeze_cam_snapshot, cam_snapshot, cam_snapshot_len);
    freeze_cam_valid = true;
  }

  portENTER_CRITICAL(&mlx_mux);
  memcpy(freeze_thermal_snapshot, mlx_temps_full, sizeof(freeze_thermal_snapshot));
  freeze_thermal_valid = mlx_full_ready;
  portEXIT_CRITICAL(&mlx_mux);
}

void releaseFreezeShareSnapshot() {
  if (freeze_cam_snapshot) {
    heap_caps_free(freeze_cam_snapshot);
    freeze_cam_snapshot = nullptr;
  }
  freeze_cam_valid = false;
  freeze_thermal_valid = false;
  freeze_capture_ms = 0;
}

static inline void bmpPut16(uint8_t *h, int off, uint16_t v) {
  h[off] = v & 0xFF; h[off + 1] = (v >> 8) & 0xFF;
}

static inline void bmpPut32(uint8_t *h, int off, uint32_t v) {
  h[off] = v & 0xFF; h[off + 1] = (v >> 8) & 0xFF;
  h[off + 2] = (v >> 16) & 0xFF; h[off + 3] = (v >> 24) & 0xFF;
}

void writeBmpHeader(WiFiClient &client) {
  uint8_t hdr[54] = {0};
  const uint32_t row_bytes = IMG_W * 3;
  const uint32_t img_size = row_bytes * IMG_H;
  hdr[0] = 'B'; hdr[1] = 'M';
  bmpPut32(hdr, 2, 54 + img_size);
  bmpPut32(hdr, 10, 54);
  bmpPut32(hdr, 14, 40);
  bmpPut32(hdr, 18, IMG_W);
  bmpPut32(hdr, 22, IMG_H);
  bmpPut16(hdr, 26, 1);
  bmpPut16(hdr, 28, 24);
  bmpPut32(hdr, 34, img_size);
  client.write(hdr, sizeof(hdr));
}

static inline void rgb565WireToRgb888(uint16_t cbe,
                                      uint8_t &r, uint8_t &g, uint8_t &b) {
  uint16_t v = swap565(cbe);
  r = ((v >> 11) & 0x1F) << 3;
  g = ((v >> 5)  & 0x3F) << 2;
  b =  (v        & 0x1F) << 3;
}

static inline uint16_t freezeCameraPixelWire(int x, int y,
                                             int crop_x0, int crop_y0,
                                             int32_t crop_step_x_q,
                                             int32_t crop_step_y_q) {
  if (!freeze_cam_valid) return 0;
  int src_y = crop_y0 + (int)(((int64_t)y * crop_step_y_q) >> 16);
  if (src_y < 0) src_y = 0; else if (src_y >= IMG_H) src_y = IMG_H - 1;
  int src_x = crop_x0 + (int)(((int64_t)x * crop_step_x_q) >> 16);
  if (src_x < 0) src_x = 0; else if (src_x >= IMG_W) src_x = IMG_W - 1;
  uint16_t *cam = (uint16_t*)freeze_cam_snapshot;
  return cam[src_y * IMG_W + src_x];
}

static inline float freezeThermalCell(int row, int display_col) {
  return freeze_thermal_snapshot[row * 32 + (31 - display_col)];
}

uint8_t freezeThermalPaletteIndexAt(int x, int y, bool &inside) {
  inside = false;
  if (!freeze_thermal_valid) return 0;

  int32_t step_x_q, step_y_q;
  thermal_steps(step_x_q, step_y_q);
  int ty0, ty1, tx0, tx1;
  uint16_t fx, fy;
  bool x_inside, y_inside;
  thermal_indices_fov(y - IMG_H / 2, step_y_q, 12, 23, ty0, ty1, fy, y_inside);
  thermal_indices_fov(x - IMG_W / 2, step_x_q, 16, 31, tx0, tx1, fx, x_inside);
  inside = x_inside && y_inside;
  if (!inside) return 0;

#if USE_BILINEAR_THERMAL
  uint16_t ifx = 256 - fx, ify = 256 - fy;
  float a = freezeThermalCell(ty0, tx0);
  float b = freezeThermalCell(ty0, tx1);
  float c = freezeThermalCell(ty1, tx0);
  float d = freezeThermalCell(ty1, tx1);
  float top = (a * ifx + b * fx) / 256.0f;
  float bottom = (c * ifx + d * fx) / 256.0f;
  float temp = (top * ify + bottom * fy) / 256.0f;
#else
  float temp = freezeThermalCell(ty0, tx0);
#endif
  return tempToPaletteIndex(temp, freeze_range_lo, freeze_range_hi);
}

void sendFreezeBmp(uint8_t mode, const char *filename) {
  if ((mode == SHARE_BMP_CAMERA && !freeze_cam_valid) ||
      (mode != SHARE_BMP_CAMERA && !freeze_thermal_valid)) {
    share_server.send(404, "text/plain", "No frozen frame available");
    return;
  }

  share_server.sendHeader("Content-Disposition",
                          String("inline; filename=\"") + filename + "\"");
  share_server.setContentLength(54 + IMG_W * IMG_H * 3);
  share_server.send(200, "image/bmp", "");
  WiFiClient client = share_server.client();
  writeBmpHeader(client);

  int crop_x0, crop_y0;
  int32_t crop_step_x_q, crop_step_y_q;
  cameraCropParamsFor(freeze_zoom_pct, freeze_parallax_x, freeze_parallax_y,
                      crop_x0, crop_y0, crop_step_x_q, crop_step_y_q);

  for (int y = IMG_H - 1; y >= 0; y--) {
    for (int x = 0; x < IMG_W; x++) {
      uint8_t r = 0, g = 0, b = 0;

      if (mode == SHARE_BMP_CAMERA || mode == SHARE_BMP_OVERLAY) {
        rgb565WireToRgb888(freezeCameraPixelWire(x, y, crop_x0, crop_y0,
                                                 crop_step_x_q, crop_step_y_q),
                           r, g, b);
      }

      if (mode == SHARE_BMP_THERMAL || mode == SHARE_BMP_OVERLAY) {
        bool inside = false;
        uint8_t pidx = freezeThermalPaletteIndexAt(x, y, inside);
        if (inside) {
          if (mode == SHARE_BMP_THERMAL || !freeze_cam_valid) {
            r = palR[pidx]; g = palG[pidx]; b = palB[pidx];
          } else {
            uint16_t rr = (uint16_t)r + (uint16_t)palR[pidx] * freeze_tint_pct / 100;
            uint16_t gg = (uint16_t)g + (uint16_t)palG[pidx] * freeze_tint_pct / 100;
            uint16_t bb = (uint16_t)b + (uint16_t)palB[pidx] * freeze_tint_pct / 100;
            r = rr > 255 ? 255 : rr;
            g = gg > 255 ? 255 : gg;
            b = bb > 255 ? 255 : bb;
          }
        }
      }

      int off = x * 3;
      share_bmp_line[off + 0] = b;
      share_bmp_line[off + 1] = g;
      share_bmp_line[off + 2] = r;
    }
    client.write(share_bmp_line, sizeof(share_bmp_line));
    delay(0);
  }
}

void handleShareIndex() {
  String html;
  html.reserve(2200);
  html += F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>Thermal Frame</title><style>body{margin:0;background:#111;color:#eee;font-family:system-ui,sans-serif}main{max-width:900px;margin:auto;padding:12px}img{width:100%;height:auto;image-rendering:auto;background:#000}.links{display:flex;flex-wrap:wrap;gap:8px;margin:10px 0 14px}a{color:#111;background:#9fe870;text-decoration:none;padding:8px 10px;border-radius:6px;font-weight:700}.meta{color:#aaa;font-size:13px;margin:8px 0 12px}.grid{display:grid;grid-template-columns:1fr;gap:12px}@media(min-width:720px){.grid{grid-template-columns:1fr 1fr}}</style></head><body><main>");
  html += F("<img src='/overlay.bmp' alt='Overlay'>");
  html += F("<div class='links'><a href='/overlay.bmp' download>Overlay BMP</a><a href='/thermal.bmp' download>Thermal BMP</a><a href='/camera.bmp' download>Camera BMP</a><a href='/thermal.csv' download>Thermal CSV</a></div>");
  html += F("<div class='meta'>AP ");
  html += share_ap_ssid;
  html += F(" | IP ");
  html += WiFi.softAPIP().toString();
  html += F(" | range ");
  html += String(freeze_range_lo, 1);
  html += F(" C to ");
  html += String(freeze_range_hi, 1);
  html += F(" C</div><div class='grid'><img src='/thermal.bmp' alt='Thermal'><img src='/camera.bmp' alt='Camera'></div>");
  html += F("</main></body></html>");
  share_server.send(200, "text/html", html);
}

void handleShareCsv() {
  if (!freeze_thermal_valid) {
    share_server.send(404, "text/plain", "No frozen thermal frame available");
    return;
  }

  String csv;
  csv.reserve(768 * 7);
  for (int row = 0; row < 24; row++) {
    for (int col = 0; col < 32; col++) {
      csv += String(freeze_thermal_snapshot[row * 32 + col], 2);
      csv += (col == 31) ? '\n' : ',';
    }
  }
  share_server.sendHeader("Content-Disposition", "attachment; filename=\"thermal.csv\"");
  share_server.send(200, "text/csv", csv);
}

void handleLiveThermalCsv() {
  float snap[768];
  portENTER_CRITICAL(&mlx_mux);
  memcpy(snap, mlx_temps_full, sizeof(snap));
  bool ok = mlx_full_ready;
  portEXIT_CRITICAL(&mlx_mux);
  if (!ok) {
    share_server.send(404, "text/plain", "No live thermal frame available");
    return;
  }

  String csv;
  csv.reserve(768 * 7);
  for (int row = 0; row < 24; row++) {
    for (int col = 0; col < 32; col++) {
      csv += String(snap[row * 32 + col], 2);
      csv += (col == 31) ? '\n' : ',';
    }
  }
  share_server.sendHeader("Cache-Control", "no-store");
  share_server.sendHeader("Content-Disposition", "attachment; filename=\"thermal-live.csv\"");
  share_server.send(200, "text/csv", csv);
}

void handleThermalCsvRoute() {
  if (stream_mode) handleLiveThermalCsv();
  else handleShareCsv();
}

void handlePortalIndex() {
  if (stream_mode) {
    share_server.sendHeader("Cache-Control", "no-store");
    share_server.send_P(200, PSTR("text/html"), STREAM_PORTAL_HTML);
  } else {
    handleShareIndex();
  }
}

size_t streamJpgChunk(void *arg, size_t index, const void *data, size_t len) {
  (void)index;
  WebServer *server = (WebServer*)arg;
  server->sendContent((const char*)data, len);
  return len;
}

void sendStreamJpegBuffer(const uint8_t *jpg, size_t jpg_len) {
  share_server.sendHeader("Cache-Control", "no-store");
  share_server.setContentLength(jpg_len);
  share_server.send(200, "image/jpeg", "");
  WiFiClient client = share_server.client();
  client.write(jpg, jpg_len);
}

void handleStreamCameraJpg() {
  if (!stream_mode) {
    share_server.send(404, "text/plain", "Stream portal is not active");
    return;
  }

  noteStreamRequest(stream_cam_req_count, stream_cam_req_timer_ms,
                    stream_cam_req_fps);

#if STREAM_CAMERA_NATIVE_JPEG
  if (stream_native_jpeg_active) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      share_server.send(404, "text/plain", "No camera frame available");
      return;
    }

    if (fb->format == PIXFORMAT_JPEG) {
      sendStreamJpegBuffer(fb->buf, fb->len);
    } else {
#if STREAM_CAMERA_CHUNKED_JPEG
      share_server.sendHeader("Cache-Control", "no-store");
      share_server.setContentLength(CONTENT_LENGTH_UNKNOWN);
      share_server.send(200, "image/jpeg", "");
      bool ok = frame2jpg_cb(fb, STREAM_JPEG_QUALITY, streamJpgChunk,
                             &share_server);
      if (!ok) {
        stream_jpeg_fail_count++;
        Serial.println("Stream camera: JPEG callback failed");
      }
#else
      uint8_t *jpg = nullptr;
      size_t jpg_len = 0;
      bool ok = frame2jpg(fb, STREAM_JPEG_QUALITY, &jpg, &jpg_len);
      if (ok && jpg && jpg_len) {
        sendStreamJpegBuffer(jpg, jpg_len);
      } else {
        stream_jpeg_fail_count++;
        share_server.send(500, "text/plain", "JPEG conversion failed");
      }
      if (jpg) free(jpg);
#endif
    }
    esp_camera_fb_return(fb);
    return;
  }
#endif

  if (!cam_have_frame || !cam_snapshot) {
    share_server.send(404, "text/plain", "No camera frame available");
    return;
  }

#if STREAM_CAMERA_CHUNKED_JPEG
  share_server.sendHeader("Cache-Control", "no-store");
  share_server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  share_server.send(200, "image/jpeg", "");
  bool ok = fmt2jpg_cb(cam_snapshot, cam_snapshot_len, IMG_W, IMG_H,
                       PIXFORMAT_RGB565, STREAM_JPEG_QUALITY,
                       streamJpgChunk, &share_server);
  if (!ok) {
    stream_jpeg_fail_count++;
    Serial.println("Stream camera: RGB565 JPEG callback failed");
  }
#else
  uint8_t *jpg = nullptr;
  size_t jpg_len = 0;
  bool ok = fmt2jpg(cam_snapshot, cam_snapshot_len, IMG_W, IMG_H,
                    PIXFORMAT_RGB565, STREAM_JPEG_QUALITY, &jpg, &jpg_len);
  if (!ok || !jpg || !jpg_len) {
    stream_jpeg_fail_count++;
    if (jpg) free(jpg);
    share_server.send(500, "text/plain", "JPEG conversion failed");
    return;
  }
  sendStreamJpegBuffer(jpg, jpg_len);
  free(jpg);
#endif
}

void handleStreamThermalBin() {
  if (!stream_mode) {
    share_server.send(404, "text/plain", "Stream portal is not active");
    return;
  }

  noteStreamRequest(stream_thermal_req_count, stream_thermal_req_timer_ms,
                    stream_thermal_req_fps);

  float snap[768];
  uint32_t seq;
  float mn, mx, center, lo, hi;
  bool ok;
  portENTER_CRITICAL(&mlx_mux);
  memcpy(snap, mlx_temps_full, sizeof(snap));
  seq = thermal_frame_seq;
  mn = t_min; mx = t_max; center = t_center;
  ok = mlx_full_ready;
  portEXIT_CRITICAL(&mlx_mux);
  getThermalRange(lo, hi);

  if (!ok) {
    share_server.send(404, "text/plain", "No thermal frame available");
    return;
  }

  for (int i = 0; i < 768; i++) {
    float v = snap[i] * 100.0f;
    if (v > 32767.0f) v = 32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    stream_thermal_packet[i] = (int16_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
  }

  share_server.sendHeader("Cache-Control", "no-store");
  share_server.sendHeader("X-Frame-Seq", String(seq));
  share_server.sendHeader("X-Temp-Center", String(center, 2));
  share_server.sendHeader("X-Temp-Min", String(mn, 2));
  share_server.sendHeader("X-Temp-Max", String(mx, 2));
  share_server.sendHeader("X-Range-Lo", String(lo, 2));
  share_server.sendHeader("X-Range-Hi", String(hi, 2));
  share_server.setContentLength(sizeof(stream_thermal_packet));
  share_server.send(200, "application/octet-stream", "");
  WiFiClient client = share_server.client();
  client.write((const uint8_t*)stream_thermal_packet, sizeof(stream_thermal_packet));
}

static inline int argIntClamped(const char *name, int current, int lo, int hi) {
  if (!share_server.hasArg(name)) return current;
  int v = share_server.arg(name).toInt();
  if (v < lo) v = lo; else if (v > hi) v = hi;
  return v;
}

static inline float argFloatClamped(const char *name, float current, float lo, float hi) {
  if (!share_server.hasArg(name)) return current;
  float v = share_server.arg(name).toFloat();
  if (v < lo) v = lo; else if (v > hi) v = hi;
  return v;
}

void handleStreamControl() {
  if (!stream_mode) {
    share_server.send(409, "text/plain", "Stream portal is not active");
    return;
  }

  bool dirty = false;
  if (share_server.hasArg("view")) {
    stream_view_mode = (uint8_t)argIntClamped("view", stream_view_mode, STREAM_VIEW_OVERLAY, STREAM_VIEW_THERMAL);
  }
  if (share_server.hasArg("range")) {
    int r = argIntClamped("range", (int)range_mode, 0, RANGE_COUNT - 1);
    if (r != (int)range_mode) { range_mode = (RangeMode)r; dirty = true; }
  }
  int px = argIntClamped("px", parallax_x, PARALLAX_MIN, PARALLAX_MAX);
  int py = argIntClamped("py", parallax_y, PARALLAX_MIN, PARALLAX_MAX);
  int z  = argIntClamped("zoom", zoom_pct, 100, 250);
  int ti = argIntClamped("tint", tint_pct, 0, 100);
  int br = argIntClamped("brt", cam_brightness, -2, 2);
  if (px != parallax_x) { parallax_x = px; dirty = true; }
  if (py != parallax_y) { parallax_y = py; dirty = true; }
  if (z  != zoom_pct)   { zoom_pct = z; dirty = true; }
  if (ti != tint_pct)   { tint_pct = (uint8_t)ti; dirty = true; }
  if (br != cam_brightness) {
    cam_brightness = (int8_t)br;
    brightness_apply_pending = true;
    dirty = true;
  }
  if (share_server.hasArg("mlo")) {
    float before = manual_lo;
    setManualLoValue(argFloatClamped("mlo", manual_lo, MANUAL_T_MIN, MANUAL_T_MAX));
    dirty = dirty || (before != manual_lo);
  }
  if (share_server.hasArg("mhi")) {
    float before = manual_hi;
    setManualHiValue(argFloatClamped("mhi", manual_hi, MANUAL_T_MIN, MANUAL_T_MAX));
    dirty = dirty || (before != manual_hi);
  }
  if (share_server.hasArg("hud")) {
    stream_hud_enabled = share_server.arg("hud").toInt() != 0;
  }
  if (share_server.hasArg("crosshair")) {
    stream_crosshair_enabled = share_server.arg("crosshair").toInt() != 0;
  }
  if (share_server.hasArg("reset_alignment")) {
    parallax_x = 0; parallax_y = 0; zoom_pct = 100; dirty = true;
  }
  if (share_server.hasArg("stop_stream")) {
    stream_stop_pending = true;
    share_server.send(200, "application/json", "{\"ok\":true,\"stopping\":true}");
    return;
  }

  if (dirty) markDirty();
  share_server.sendHeader("Cache-Control", "no-store");
  share_server.send(200, "application/json", "{\"ok\":true}");
}

void handleStreamState() {
  uint32_t now = millis();
  noteStreamRequest(stream_api_count, stream_api_timer_ms, stream_api_fps);
  if (stream_cam_req_timer_ms && now - stream_cam_req_timer_ms > 1500) {
    stream_cam_req_count = 0;
    stream_cam_req_fps = 0.0f;
    stream_cam_req_timer_ms = now;
  }
  if (stream_thermal_req_timer_ms && now - stream_thermal_req_timer_ms > 1500) {
    stream_thermal_req_count = 0;
    stream_thermal_req_fps = 0.0f;
    stream_thermal_req_timer_ms = now;
  }

  float lo, hi;
  getThermalRange(lo, hi);
  uint32_t seq;
  float mn, mx, center;
  portENTER_CRITICAL(&mlx_mux);
  seq = thermal_frame_seq;
  mn = t_min; mx = t_max; center = t_center;
  portEXIT_CRITICAL(&mlx_mux);

  String json;
  updateCameraMemoryDiagnostics();
  json.reserve(1800);
  json += F("{\"stream\":");
  json += stream_mode ? F("true") : F("false");
  json += F(",\"ssid\":\""); json += share_ap_ssid; json += F("\"");
  json += F(",\"ip\":\""); json += WiFi.softAPIP().toString(); json += F("\"");
  json += F(",\"view\":"); json += stream_view_mode;
  json += F(",\"range\":"); json += (int)range_mode;
  json += F(",\"px\":"); json += parallax_x;
  json += F(",\"py\":"); json += parallax_y;
  json += F(",\"zoom\":"); json += zoom_pct;
  json += F(",\"tint\":"); json += tint_pct;
  json += F(",\"mlo\":"); json += String(manual_lo, 1);
  json += F(",\"mhi\":"); json += String(manual_hi, 1);
  json += F(",\"brt\":"); json += (int)cam_brightness;
  json += F(",\"hud\":"); json += stream_hud_enabled ? 1 : 0;
  json += F(",\"crosshair\":"); json += stream_crosshair_enabled ? 1 : 0;
  json += F(",\"rangeLo\":"); json += String(lo, 2);
  json += F(",\"rangeHi\":"); json += String(hi, 2);
  json += F(",\"tCenter\":"); json += String(center, 2);
  json += F(",\"tMin\":"); json += String(mn, 2);
  json += F(",\"tMax\":"); json += String(mx, 2);
  json += F(",\"seq\":"); json += seq;
  json += F(",\"camFps\":"); json += String(cam_fps, 1);
  json += F(",\"camReqFps\":"); json += String(stream_cam_req_fps, 1);
  json += F(",\"mlxFps\":"); json += String(mlx_fps, 1);
  json += F(",\"thermalReqFps\":"); json += String(stream_thermal_req_fps, 1);
  json += F(",\"loopFps\":"); json += String(current_fps, 1);
  json += F(",\"apiFps\":"); json += String(stream_api_fps, 1);
  json += F(",\"portalMs\":"); json += stream_started_ms ? (now - stream_started_ms) : 0;
  json += F(",\"heap\":"); json += ESP.getFreeHeap();
  json += F(",\"psram\":"); json += ESP.getFreePsram();
  json += F(",\"nativeJpeg\":"); json += stream_native_jpeg_active ? 1 : 0;
  json += F(",\"jpegChunked\":"); json += STREAM_CAMERA_CHUNKED_JPEG ? 1 : 0;
  json += F(",\"camOk\":"); json += cam_ok ? 1 : 0;
  json += F(",\"camHaveFrame\":"); json += cam_have_frame ? 1 : 0;
  json += F(",\"camFailStage\":\""); json += cameraStageName(cam_fail_stage); json += F("\"");
  json += F(",\"camErrCode\":"); json += (int)cam_last_err;
  json += F(",\"camErrName\":\""); json += cameraErrName(); json += F("\"");
  json += F(",\"camSensorPid\":"); json += cam_sensor_pid;
  json += F(",\"camInitAttempts\":"); json += cam_init_attempts;
  json += F(",\"camLastFrameLen\":"); json += (unsigned)cam_last_frame_len;
  json += F(",\"camExpectedFrameLen\":"); json += (unsigned)cam_snapshot_len;
  json += F(",\"camFrameFailCount\":"); json += cam_frame_fail_count;
  json += F(",\"camFrameLenMismatchCount\":"); json += cam_frame_len_mismatch_count;
  json += F(",\"psramFound\":"); json += cam_psram_found ? 1 : 0;
  json += F(",\"psramSize\":"); json += cam_psram_size;
  json += F(",\"psramFree\":"); json += cam_psram_free;
  json += F(",\"jpegFailCount\":"); json += stream_jpeg_fail_count;
  json += F("}");
  share_server.sendHeader("Cache-Control", "no-store");
  share_server.send(200, "application/json", json);
}

void configureShareRoutes() {
  if (share_routes_configured) return;
  share_server.on("/", handlePortalIndex);
  share_server.on("/camera.bmp", [](){ sendFreezeBmp(SHARE_BMP_CAMERA, "camera.bmp"); });
  share_server.on("/thermal.bmp", [](){ sendFreezeBmp(SHARE_BMP_THERMAL, "thermal.bmp"); });
  share_server.on("/overlay.bmp", [](){ sendFreezeBmp(SHARE_BMP_OVERLAY, "overlay.bmp"); });
  share_server.on("/thermal.csv", handleThermalCsvRoute);
  share_server.on("/cam.jpg", HTTP_GET, handleStreamCameraJpg);
  share_server.on("/thermal.bin", HTTP_GET, handleStreamThermalBin);
  share_server.on("/api/state", HTTP_GET, handleStreamState);
  share_server.on("/api/control", HTTP_POST, handleStreamControl);
  share_server.onNotFound([](){
    share_server.sendHeader("Location", "/", true);
    share_server.send(302, "text/plain", "");
  });
  share_routes_configured = true;
}

bool startFreezeShareAP() {
  configureShareRoutes();
  uint64_t mac = ESP.getEfuseMac();
  snprintf(share_ap_ssid, sizeof(share_ap_ssid), "ThermalCam-%04X",
           (uint16_t)(mac & 0xFFFF));

  IPAddress ip(192, 168, 4, 1);
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  if (!WiFi.softAP(share_ap_ssid)) {
    share_ap_running = false;
    Serial.println("Share AP: start failed");
    return false;
  }
  share_dns.start(SHARE_DNS_PORT, "*", ip);
  share_server.begin();
  share_ap_running = true;
  Serial.printf("Share AP: connect to %s, open http://%s/\n",
                share_ap_ssid, ip.toString().c_str());
  return true;
}

void stopFreezeShareAP() {
  bool was_running = share_ap_running;
  if (share_ap_running) {
    share_server.stop();
    share_dns.stop();
  }
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  share_ap_running = false;
  if (was_running) Serial.println("Share AP: stopped");
}

void handleFreezeShareAP() {
  if (!share_ap_running) return;
  share_dns.processNextRequest();
  share_server.handleClient();
}

void drawStreamStatusScreen() {
  int x = 18;
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  lcd.setCursor(x, 28); lcd.print("STREAM PORTAL");
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setCursor(x, 72); lcd.print("Connect WiFi:");
  lcd.setCursor(x, 88); lcd.print(share_ap_ssid);
  lcd.setCursor(x, 112); lcd.print("Open:");
  lcd.setCursor(x, 128); lcd.print("http://192.168.4.1/");
  lcd.setCursor(x, 160); lcd.print("Browser preview + recording.");
  lcd.setCursor(x, 176); lcd.print("Hold button 3s to exit.");
  stream_last_status_ms = 0;
}

void drawStreamStatusDynamic() {
  uint32_t now = millis();
  if (now - stream_last_status_ms < 1000) return;
  stream_last_status_ms = now;
  int x = 18;
  int y = ui_portrait ? 224 : 206;
  lcd.fillRect(x, y, lcd.width() - 2 * x, 54, TFT_BLACK);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.setCursor(x, y);
  lcd.printf("Cam %.1f/%.1ffps  MLX %.1f/%.1ffps",
             cam_fps, stream_cam_req_fps, mlx_fps, stream_thermal_req_fps);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setCursor(x, y + 18);
  lcd.printf("Ctr %.1fC  Scene %.1f..%.1fC", t_center, t_min, t_max);
  lcd.setCursor(x, y + 36);
  lcd.printf("Heap %luK PSRAM %luK",
             (unsigned long)(ESP.getFreeHeap() / 1024),
             (unsigned long)(ESP.getFreePsram() / 1024));
}

void setStreamMode(bool enabled) {
  if (enabled == stream_mode) return;

  if (enabled) {
    if (freeze_mode) {
      stopFreezeShareAP();
      releaseFreezeShareSnapshot();
      freeze_mode = false;
    }

    stream_stop_pending = false;
    stream_view_mode = STREAM_VIEW_OVERLAY;
    if (!enterStreamCameraMode()) {
      Serial.println("Stream portal: camera mode switch failed, continuing");
    }
    stream_started_ms = millis();
    resetStreamDiagnostics(stream_started_ms);
    if (!startFreezeShareAP()) {
      stopFreezeShareAP();
      exitStreamCameraMode();
      stream_started_ms = 0;
      lcd.fillScreen(TFT_BLACK);
      lcd.setTextColor(TFT_RED, TFT_BLACK);
      lcd.setTextSize(2);
      lcd.setCursor(18, 130); lcd.print("STREAM AP FAIL");
      lcd.setTextSize(1);
      delay(1200);
      lcd.fillScreen(TFT_BLACK);
      drawStaticUI();
      last_ui_ms = millis();
      Serial.println("Stream portal: AP failed, staying in normal mode");
      return;
    }
    stream_mode = true;
    drawStreamStatusScreen();
  } else {
    stopFreezeShareAP();
    exitStreamCameraMode();
    stream_mode = false;
    stream_stop_pending = false;
    stream_started_ms = 0;
    lcd.setBrightness(255);
    lcd.fillScreen(TFT_BLACK);
    drawStaticUI();
    last_ui_ms = millis();
  }

  Serial.printf("Stream portal: %s\n", stream_mode ? "ON" : "OFF");
}

void setFreezeMode(bool enabled) {
  if (enabled == freeze_mode) return;
  if (enabled && stream_mode) return;

  if (enabled) {
    freeze_mode = true;
    captureFreezeShareSnapshot();
    if (!startFreezeShareAP()) {
      Serial.println("Freeze: AP start failed, frozen frame remains local");
    }
  } else {
    stopFreezeShareAP();
    releaseFreezeShareSnapshot();
    freeze_mode = false;
  }

  Serial.printf("Freeze: %s\n", freeze_mode ? "ON" : "OFF");
}

// ---------------------------------------------------------------- UI bits
void drawCrossbar() {
  int cx = IMG_X + IMG_W / 2, cy = IMG_Y + IMG_H / 2;
  lcd.drawFastHLine(cx - 10, cy, 21, TFT_WHITE);
  lcd.drawFastVLine(cx, cy - 10, 21, TFT_WHITE);
  lcd.drawRect(cx - 10, cy - 10, 21, 21, TFT_WHITE);
}

uint32_t fps_count = 0, fps_timer = 0, cam_count = 0;
uint32_t mlx_fps_seq_prev = 0;
float current_fps = 0, cam_fps = 0, mlx_fps = 0;
uint32_t last_ui_ms = 0;

void drawStaticUI() {
  layoutUi();
  if (PALETTE_W > 0) {
    int palette_h = PALETTE_Y_BOTTOM - PALETTE_Y_TOP + 1;
    int denom = palette_h > 1 ? palette_h - 1 : 1;
    for (int y = 0; y < palette_h; y++) {
      int palette_i = 255 - (y * 255) / denom;
      lcd.drawFastHLine(PALETTE_X, PALETTE_Y_TOP + y, PALETTE_W,
                        palette_native[palette_i]);
    }
  }
  if (ui_portrait) {
    lcd.drawFastHLine(0, IMG_Y + IMG_H, lcd.width(), TFT_DARKGREY);
  } else {
    lcd.drawFastVLine(PANEL_X - 3, 22, 296, TFT_DARKGREY);
  }
  drawButtons(); drawAdjustSliders(); drawSlider(); drawBrightnessSlider();
}

void drawDynamicUI() {
  lcd.setTextColor(TFT_WHITE, TFT_BLACK); lcd.setTextSize(1);
  char buf[40];

  // Side palette labels — t_max sits in the top-bar gap on the left,
  // t_min sits in the bottom strip next to the brightness slider.
  lcd.fillRect(TMAX_LABEL_X, TMAX_LABEL_Y, 38, 12, TFT_BLACK);
  snprintf(buf, sizeof(buf), "%4.1fC", t_max);
  lcd.setCursor(TB_TMAX, TMAX_LABEL_Y); lcd.print(buf);
  if (TMIN_LABEL_Y >= 0) {
    lcd.fillRect(TMIN_LABEL_X, TMIN_LABEL_Y, 38, 12, TFT_BLACK);
    snprintf(buf, sizeof(buf), "%4.1fC", t_min);
    lcd.setCursor(TMIN_LABEL_X, TMIN_LABEL_Y); lcd.print(buf);
  }

  // Top bar — full-width clear so battery on the right doesn't leave a gap.
  lcd.fillRect(TOPBAR_CLEAR_X, 0, TOPBAR_CLEAR_W, 18, TFT_BLACK);

  // Centre temperature, large.
  lcd.setTextSize(2);
  snprintf(buf, sizeof(buf), "%.1fC", t_center);
  lcd.setCursor(TB_CTR, 1); lcd.print(buf);

  // Smaller fields with a spacer column between each so they read clearly.
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  snprintf(buf, sizeof(buf), "%.1ffps", current_fps);
  lcd.setCursor(TB_FPS, 5); lcd.print(buf);
  lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  snprintf(buf, sizeof(buf), "Z%d%%", zoom_pct);
  lcd.setCursor(TB_ZOOM, 5); lcd.print(buf);
  snprintf(buf, sizeof(buf), "P%+d,%+d", parallax_x, parallax_y);
  lcd.setCursor(TB_PARA, 5); lcd.print(buf);

  // Mode tag.
  lcd.fillRect(TB_MODE, 2, TB_MODE_W, 14, MODE_BG[display_mode]);
  lcd.setTextColor(TFT_BLACK, MODE_BG[display_mode]);
  lcd.setCursor(TB_MODE + 3, 5); lcd.print(MODE_NAMES[display_mode]);

  // Range tag.
  lcd.fillRect(TB_RNG, 2, TB_RNG_W, 14, TFT_PURPLE);
  lcd.setTextColor(TFT_WHITE, TFT_PURPLE);
  lcd.setCursor(TB_RNG + 3, 5); lcd.print(RANGE_NAMES[range_mode]);

  // Status badge: AP implies a frozen frame is being shared.
  if (TB_STAT >= 0 && share_ap_running) {
    lcd.fillRect(TB_STAT, 2, TB_STAT_W, 14, TFT_RED);
    lcd.setTextColor(TFT_WHITE, TFT_RED);
    lcd.setCursor(TB_STAT + 6, 5); lcd.print("AP");
  } else if (TB_STAT >= 0 && freeze_mode) {
    lcd.fillRect(TB_STAT, 2, TB_STAT_W, 14, TFT_RED);
    lcd.setTextColor(TFT_WHITE, TFT_RED);
    lcd.setCursor(TB_STAT + 3, 5); lcd.print("FRZ");
  } else if (TB_STAT >= 0 && millis() < save_indicator_until_ms) {
    lcd.fillRect(TB_STAT, 2, TB_STAT_W, 14, TFT_DARKGREEN);
    lcd.setTextColor(TFT_WHITE, TFT_DARKGREEN);
    lcd.setCursor(TB_STAT + 3, 5); lcd.print("SAV");
  }

  // Battery — right-justified. Hidden if BATTERY_ADC_PIN is -1 (default).
  if (TB_BAT >= 0 && battery_pct >= 0) {
    uint16_t col = battery_pct < 20 ? TFT_RED
                 : battery_pct < 50 ? TFT_YELLOW
                                    : TFT_GREEN;
    lcd.setTextColor(col, TFT_BLACK);
    snprintf(buf, sizeof(buf), "%3d%% %.2fV", battery_pct, battery_v);
    lcd.setCursor(TB_BAT, 5); lcd.print(buf);
  }

  // Right-column readout under the X+/Y+ buttons.
  if (READOUT_W > 0 && READOUT_H > 0) {
    lcd.fillRect(READOUT_X, READOUT_Y, READOUT_W, READOUT_H, TFT_BLACK);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    if (share_ap_running) {
      snprintf(buf, sizeof(buf), "%s", share_ap_ssid);
    } else {
      snprintf(buf, sizeof(buf), "X%+d Y%+d", parallax_x, parallax_y);
    }
    lcd.setCursor(READOUT_X + 2, READOUT_Y); lcd.print(buf);
  }
}

// --------------------------------------------------------------- Storage --
// SD storage is disabled; freeze/WiFi export is the save path.
// ---------------------------------------------------------------- Setup --
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== FLIR v16 ===");
  analogReadResolution(12);
  
  // Set the pin to 11dB attenuation so it can read up to ~3.3V.
#if BATTERY_ADC_PIN >= 0
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
#endif
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("[1/8] Wire on IO1/IO2");
  Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
  Wire.setClock(100000);

  Serial.println("[2/8] AXP camera power");
  uint32_t t0 = millis();
  while (axp.begin() != 0 && millis() - t0 < 500) delay(50);
  axp.enableCameraPower(axp.eOV2640);
  delay(500);

  Serial.println("[3/8] Camera (must precede LCD init)");
  if (!initCamera()) {
    Serial.println("CAMERA: init failed — continuing without camera. "
                   "TINT/CAM modes will fall back to thermal-only.");
  }

  Serial.println("[4/8] GT911 manual reset");
  gt911_init_manual();

  Serial.println("[5/8] LCD");
  lcd.init();
  lcd.setRotation(1); // Temporary boot orientation; settings load later.
  lcd.setBrightness(255);
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_WHITE); lcd.setTextSize(2);
  lcd.drawString("Booting...", 10, 10);

  Serial.println("[6/8] MLX on Wire1");
  if (!initMLX()) {
    lcd.fillScreen(TFT_RED);
    lcd.drawString("MLX FAIL", 10, 150);
    while(1) delay(1000);
  }
  Serial.printf("MLX: refresh code=0x%02X, coherent-read validation ON\n",
                MLX_REFRESH_RATE_CODE);
  Serial.printf("MLX: ADC resolution code=%d actual=%d mode=%d\n",
                MLX_ADC_RESOLUTION_CODE,
                MLX90640_GetCurResolution(MLX_ADDR),
                MLX90640_GetCurMode(MLX_ADDR));
  Serial.printf("MLX: EEPROM broken=%d outlier=%d\n",
                countPixelList(mlx_params.brokenPixels),
                countPixelList(mlx_params.outlierPixels));
  // Block until we've seen both subpages, so mlx_temps_full is ready before
  // the first render. Bounded by a few seconds; the cap is just paranoia.
  {
    uint32_t mlx_t0 = millis();
    while (millis() - mlx_t0 < 2000) {
      if (readMLXSubpage() == MLX_READ_FULL) break;
      delay(1);
    }
    analyzeMLX();
  }
  last_mlx_ms = millis();

  Serial.println("[7/8] Palette + UI + Battery");
  buildPalette();
  loadSettings();
  applyScreenOrientation();
  if (brightness_apply_pending) { applyCameraBrightness(); brightness_apply_pending = false; }
  readBattery();
  lcd.fillScreen(TFT_BLACK);
  drawStaticUI();

  if (xTaskCreatePinnedToCore(mlxTask, "mlx", 12288, nullptr, 1,
                              &mlx_task_handle, 0) != pdPASS) {
    mlx_task_handle = nullptr;
    Serial.println("MLX task: create failed, using foreground fallback");
  } else {
    Serial.println("MLX task: running on core 0");
  }

  Serial.println("[8/8] Ready");
  fps_timer = millis();
  last_ui_ms = millis();
  Serial.printf("Status: Cam=%s Touch=%s Storage=WiFi\n",
                cam_ok   ? "OK" : "MISSING",
                gt911_ok ? "OK" : "FAIL");
  Serial.printf("Camera diag: stage=%s err=0x%X/%s psram=%d size=%lu free=%lu sensor=0x%04X frameLen=%u expected=%u\n",
                cameraStageName(cam_fail_stage),
                (unsigned)cam_last_err,
                cameraErrName(),
                cam_psram_found ? 1 : 0,
                (unsigned long)cam_psram_size,
                (unsigned long)cam_psram_free,
                cam_sensor_pid,
                (unsigned)cam_last_frame_len,
                (unsigned)cam_snapshot_len);
}

// ---------------------------------------------------------------- Loop ---
void loop() {
  int ev = pollButton();
  if (ev == BTN_LONG) {
    setStreamMode(!stream_mode);
  } else if (ev == BTN_SHORT && !stream_mode) {
    setFreezeMode(!freeze_mode);
  }
  handleFreezeShareAP();
  if (stream_stop_pending) {
    setStreamMode(false);
  }

  if (!stream_mode) handleTouch();

  uint32_t now = millis();

  // Push a pending brightness change into the sensor. One SCCB write per
  // change, not per frame — touchscreen handler sets the flag, we apply it
  // at the next loop iteration.
  if (brightness_apply_pending) {
    applyCameraBrightness();
    brightness_apply_pending = false;
  }

  // MLX normally runs on its own core so foreground rendering doesn't pause
  // for I2C reads and MLX90640_CalculateTo(). This fallback only runs if the
  // task could not be created.
  if (!mlx_task_handle && !freeze_mode && now - last_mlx_ms >= MLX_POLL_MS) {
    last_mlx_ms = now;
    if (readMLXSubpage() == MLX_READ_FULL) analyzeMLX();
  }

  // Battery roughly twice a second — the divider draws no current to speak
  // of, but ADC sampling isn't free either, and percentage doesn't change
  // that fast.
  if (now - last_bat_ms >= 500) {
    readBattery();
    last_bat_ms = now;
  }

  if (!freeze_mode) {
    if (grabCamera()) cam_count++;
  }

  if (stream_mode) {
    drawStreamStatusDynamic();
  } else {
    switch (display_mode) {
      case MODE_TINT:         renderTinted();      break;
      case MODE_THERMAL_ONLY: renderThermalOnly(); break;
      case MODE_CAMERA_ONLY:  renderCameraOnly();  break;
      default: break;
    }
  }

  if (!stream_mode && now - last_ui_ms >= 200) { drawDynamicUI(); last_ui_ms = now; }

  // Auto-save tuned settings 3 s after the last change.
  if (settings_dirty && now - settings_dirty_ms >= SETTINGS_DEBOUNCE_MS) {
    saveSettings();
    settings_dirty = false;
  }

  fps_count++;
  if (now - fps_timer >= 1000) {
    uint32_t fps_elapsed = now - fps_timer;
    uint32_t mlx_seq_now = thermal_frame_seq;
    current_fps = fps_count * 1000.0f / fps_elapsed;
    cam_fps = cam_count * 1000.0f / fps_elapsed;
    mlx_fps = (mlx_seq_now - mlx_fps_seq_prev) * 1000.0f / fps_elapsed;
    mlx_fps_seq_prev = mlx_seq_now;
    fps_count = 0; cam_count = 0; fps_timer = now;
    Serial.printf("Loop=%.1f Cam=%.1f MLX=%.1f Ctr=%.1fC [%.1f,%.1f] Mode=%s Rng=%s Tint=%d%% "
                  "Z=%d%% PX=%+d,%+d Btn=%d Frz=%d\n",
                  current_fps, cam_fps, mlx_fps, t_center, t_min, t_max,
                  MODE_NAMES[display_mode], RANGE_NAMES[range_mode], tint_pct,
                  zoom_pct, parallax_x, parallax_y,
                  digitalRead(BUTTON_PIN), freeze_mode);
    Serial.printf("MLXdiag ok=%lu full=%lu stale=%lu i2c=%lu dup=%lu nr=%lu read_us=%lu max=%lu cbias=%.2f\n",
                  (unsigned long)mlx_diag_ok_subpages,
                  (unsigned long)mlx_diag_full_frames,
                  (unsigned long)mlx_diag_stale,
                  (unsigned long)mlx_diag_i2c_fail,
                  (unsigned long)mlx_diag_duplicate_subpages,
                  (unsigned long)mlx_diag_not_ready,
                  (unsigned long)mlx_diag_read_us_last,
                  (unsigned long)mlx_diag_read_us_max,
                  mlx_diag_chess_bias_last);
  }
}
