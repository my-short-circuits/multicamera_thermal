/*
 * ============================================================================
 * DIY FLIR-style thermal camera
 * ============================================================================
 * MCU      : DFRobot FireBeetle 2 ESP32-S3-U (DFR0975-U), N16R8
 * Display  : DFRobot Fermion 3.5" 480x320 ILI9488 + GT911 touch (DFR0669)
 * Camera   : OV3660 over DVP (replaced OV2640), QVGA RGB565 (with digital crop-zoom)
 * Thermal  : Melexis MLX90640 32x24, on dedicated Wire1 @ 1 MHz
 *
 * Init order:
 *   1. Wire.begin(1, 2) @ 100 kHz - shared bus for AXP / GT911 / SCCB.
 *   2. AXP313A enable camera power, then 500 ms.
 *   3. esp_camera_init BEFORE lcd.init (LCD touches Wire internals).
 *      Camera shares Wire's I2C driver via pin_sccb_*=-1, sccb_i2c_port=0.
 *   4. GT911 manual reset pulse: INT low 11 ms, input, 50 ms (selects 0x5D).
 *   5. lcd.init in landscape orientation.
 *   6. Wire1.begin(11, 14) @ 1 MHz, MLX init.
 *
 * Pinout - verified, do not change without re-verifying GDI ribbon:
 *   GDI    : SCLK=17 MOSI=15 MISO=16 DC=3 RST=38 CS=18 BL=21
 *            TOUCH SDA=1 SCL=2 INT=13   (TCS=GPIO12 is on the ribbon; avoid)
 *   CAMERA : XCLK=45 SIOD=1 SIOC=2 D0..D7=39,40,41,4,7,8,46,48
 *            VSYNC=6 HREF=42 PCLK=5
 *   MLX    : SDA=11 SCL=14   (Wire1; not 12, which is GDI TCS)
 *   BUTTON : GPIO10, INPUT_PULLUP, NO to GND
 *
 * MLX90640_I2C_Driver.cpp must be patched: every "Wire." to "Wire1.".
 *
 * Byte-order rules:
 *   LovyanGFX setSwapBytes:
 *     true  -> buffer is treated as rgb565_t  (native uint16_t, R in high bits)
 *     false -> buffer is treated as swap565_t (byte-swapped / wire format)
 *   OV2640 RGB565 frame buffer is wire format (byte0 = R+G_hi).
 *   palette_native[] is native uint16_t for UI drawing. Render chunk_buf is
 *   written in wire-format (swap565) so pushImage() can use the DMA fast path
 *   with setSwapBytes(false).
 *   So:
 *     pushImage(camera_buf)   -> setSwapBytes(false)
 *     pushImage(chunk_buf)    -> setSwapBytes(false)
 *     drawFastHLine(palette)  -> palette_native[] / normal colour APIs
 *
 * Camera tearing mitigation:
 *   fb_count = 2, grab_mode = LATEST, XCLK = 16 MHz, snapshot to our own
 *   PSRAM buffer immediately and esp_camera_fb_return() before rendering, so
 *   the camera DMA never has to wait on the LCD push.
 *
 * SD storage is disabled; use freeze export or browser portal instead.
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
#include "esp_arduino_version.h"
#include "esp_err.h"
#include "esp_system.h"
#include "img_converters.h"
#include "DFRobot_AXP313A.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include <algorithm>     // std::nth_element for parity medians

#ifndef ESP_ARDUINO_VERSION
#define ESP_ARDUINO_VERSION 0
#endif
#ifndef ESP_ARDUINO_VERSION_VAL
#define ESP_ARDUINO_VERSION_VAL(major, minor, patch) ((major << 16) | (minor << 8) | (patch))
#endif
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
#define CAMERA_HAS_SAFE_RECONFIGURE 1
#else
#define CAMERA_HAS_SAFE_RECONFIGURE 0
#endif

// Smooth display interpolation. The upstream frame is kept edge-aware; this
// just avoids turning every MLX cell into an obvious display block.
#define USE_BILINEAR_THERMAL 1

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

// LCD SPI write clock. Camera-only artifacts have been traced to camera pixels,
// not panel writes, so keep the faster 50 MHz display path.
static constexpr uint32_t TFT_WRITE_FREQ_HZ = 50000000UL;
// Panel is mounted upside down in the enclosure; rotation 3 is the installed
// landscape orientation for the local LCD UI.
static constexpr uint8_t LCD_ROTATION = 3;

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
static constexpr int CROSS_R  = 10;

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
// Filtering keeps sub-degree hand detail responsive while catching real motion.
static constexpr float MLX_TEMP_ALPHA_STILL = 0.45f;     // small changes still respond fast
static constexpr float MLX_TEMP_ALPHA_FAST  = 0.85f;     // real motion: catch up almost instantly
static constexpr float MLX_TEMP_FAST_DELTA_C = 0.40f;    // engage fast mode at half-a-degree

// Off by default: the temporal filter is cheaper and preserves fine detail.
#define ENABLE_MLX_SPATIAL_DENOISE 0
#if ENABLE_MLX_SPATIAL_DENOISE
static constexpr float MLX_SPATIAL_EDGE_C = 0.20f;
#endif

// Chess subpages can have a small relative bias from CP/TGC noise and timing.
// Correct only the global checkerboard offset; do not blur local edges.
static constexpr bool  MLX_BALANCE_CHESS_SUBPAGES = true;
static constexpr float MLX_CHESS_BALANCE_MAX_C = 1.50f;  // ignore implausible scene-driven medians

// Auto-range uses percentile clipping so outliers do not stretch the palette.
static constexpr float MLX_AUTO_MIN_SPAN_C = 2.50f;      // prevent auto-range from magnifying noise
static constexpr float MLX_RANGE_ALPHA_EXPAND = 0.70f;   // new hot/cold enters range fast
static constexpr float MLX_RANGE_ALPHA_SHRINK = 0.30f;   // contraction speed (was 0.18, too slow)
static constexpr int   MLX_AUTO_TRIM_LO_PCT = 5;         // ignore coldest 5% of pixels
static constexpr int   MLX_AUTO_TRIM_HI_PCT = 5;         // ignore hottest 5% of pixels

// Render only after both chess subpages are collected, so narrow features do
// not flicker between mixed half-frames.
float mlx_temps[768];          // chess-pattern accumulator
float mlx_temps_full[768];     // last complete frame read by render code
float mlx_publish[768];        // repaired/ranged frame staged before publishing
float mlx_filtered[768];       // adaptive temporal denoise state
#if ENABLE_MLX_SPATIAL_DENOISE
float mlx_spatial[768];        // edge-aware spatial denoise scratch
#endif
static constexpr int MLX_RANGE_HIST_BINS = 128;
uint16_t mlx_range_hist[MLX_RANGE_HIST_BINS];
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
enum RangeMode { RANGE_AUTO=0, RANGE_INVERT, RANGE_AUTO2, RANGE_AUTO3,
                 RANGE_DETAIL, RANGE_HOT, RANGE_SKIN, RANGE_PCB,
                 RANGE_HIGH, RANGE_WIDE, RANGE_COLD, RANGE_MANUAL,
                 RANGE_COUNT };
extern volatile RangeMode range_mode;
static constexpr int   DETAIL_NARROW_TRIM_PCT = 15;
static constexpr float AUTO2_ROOM_FLOOR_DELTA_C = 0.5f;
static constexpr float AUTO3_ROOM_CEILING_DELTA_C = 0.5f;
static constexpr float HOT_AMBIENT_FLOOR_DELTA_C = 0.7f;
static constexpr float HOT_HEADROOM_MIN_C = 2.0f;
static constexpr float HOT_HEADROOM_FRACTION = 0.20f;
bool shouldThrottleMLXForScreenCameraOnly();

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
//   MLX_READ_FAIL: sensor I2C failure
//   MLX_READ_HALF: no complete frame yet
//   MLX_READ_FULL: both subpages are ready; analyzeMLX()
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

    // Discard reads that overlap a subpage rollover.
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
  int sp = raw[833] & 0x01;          // Melexis stores the subpage bit at index 833.
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

#if ENABLE_MLX_SPATIAL_DENOISE
  // Edge-aware spatial pass: average only similar neighbours.
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
#endif
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
int buildMLXRangeHistogram(const float *t, float mn, float mx) {
  memset(mlx_range_hist, 0, sizeof(mlx_range_hist));
  if (!(mx > mn)) {
    mlx_range_hist[0] = 768;
    return 768;
  }

  float scale = (float)(MLX_RANGE_HIST_BINS - 1) / (mx - mn);
  int valid = 0;
  for (int i = 0; i < 768; i++) {
    float v = t[i];
    if (!(v >= -40.0f && v <= 300.0f)) continue;
    int b = (int)((v - mn) * scale);
    if (b < 0) b = 0;
    else if (b >= MLX_RANGE_HIST_BINS) b = MLX_RANGE_HIST_BINS - 1;
    mlx_range_hist[b]++;
    valid++;
  }
  return valid;
}

float mlxRangePercentile(float mn, float mx, int valid, int pct) {
  if (valid <= 0 || !(mx > mn)) return mn;
  if (pct < 0) pct = 0;
  else if (pct > 100) pct = 100;

  int target = ((valid - 1) * pct) / 100;
  int accum = 0;
  for (int b = 0; b < MLX_RANGE_HIST_BINS; b++) {
    accum += mlx_range_hist[b];
    if (accum > target) {
      return mn + (mx - mn) * (float)b / (float)(MLX_RANGE_HIST_BINS - 1);
    }
  }
  return mx;
}

// Repair bad pixels, range-find, recompute center temperature, then publish.
// Impossible cells are replaced with a neighbor before display/export.
void analyzeMLX() {
  memcpy(mlx_publish, mlx_temps, sizeof(mlx_publish));
  float *t = mlx_publish;

  correctEEPROMBadPixels(t);
  balanceChessSubpages(t);

  // Two known-bad cells on this specific MLX unit.
  t[1*32 + 21] = 0.5f * (t[1*32 + 20] + t[1*32 + 22]);
  t[4*32 + 30] = 0.5f * (t[4*32 + 29] + t[4*32 + 31]);

  // Replace invalid values, including NaN, before palette indexing.
  if (!(t[0] >= -40 && t[0] <= 300)) t[0] = t[1];
  for (int i = 1; i < 768; i++) {
    if (!(t[i] >= -40 && t[i] <= 300)) t[i] = t[i-1];
  }

  denoiseMLXFrame(t);

  // Compute true min/max for labels and histogram-percentile range targets.
  // A fixed histogram avoids repeated nth_element passes in the MLX task; at
  // 128 bins it is more than precise enough for display range selection and
  // keeps the task inside the 8 Hz sensor cadence.
  float mn = t[0], mx = t[0];
  for (int i = 1; i < 768; i++) {
    if (t[i] < mn) mn = t[i];
    if (t[i] > mx) mx = t[i];
  }
  int hist_valid = buildMLXRangeHistogram(t, mn, mx);
  float mn_trimmed = mlxRangePercentile(mn, mx, hist_valid, MLX_AUTO_TRIM_LO_PCT);
  float mx_trimmed = mlxRangePercentile(mn, mx, hist_valid, 100 - MLX_AUTO_TRIM_HI_PCT);

  // Pick the AUTO target. AUT2 raises the floor above the median so warm
  // subjects show without painting the background. AUT3 caps the range below
  // the median and uses an inverted palette for cold subjects.
  float auto_target_lo = mn_trimmed;
  float auto_target_hi = mx_trimmed;
  float median = mlxRangePercentile(mn, mx, hist_valid, 50);
  float auto2_floor = -1000.0f;
  float auto3_ceiling = 1000.0f;
  float hot_floor = -1000.0f;
  if (range_mode == RANGE_AUTO2) {
    auto2_floor = median + AUTO2_ROOM_FLOOR_DELTA_C;
    if (auto_target_lo < auto2_floor) auto_target_lo = auto2_floor;
    if (auto_target_hi < auto_target_lo + MLX_AUTO_MIN_SPAN_C) {
      auto_target_hi = auto_target_lo + MLX_AUTO_MIN_SPAN_C;
    }
  } else if (range_mode == RANGE_AUTO3) {
    auto3_ceiling = median - AUTO3_ROOM_CEILING_DELTA_C;
    if (auto_target_hi > auto3_ceiling) auto_target_hi = auto3_ceiling;
    if (auto_target_lo > auto_target_hi - MLX_AUTO_MIN_SPAN_C) {
      auto_target_lo = auto_target_hi - MLX_AUTO_MIN_SPAN_C;
    }
  } else if (range_mode == RANGE_DETAIL) {
    auto_target_lo = mlxRangePercentile(mn, mx, hist_valid, DETAIL_NARROW_TRIM_PCT);
    auto_target_hi = mlxRangePercentile(mn, mx, hist_valid, 100 - DETAIL_NARROW_TRIM_PCT);
  } else if (range_mode == RANGE_HOT) {
    hot_floor = median + HOT_AMBIENT_FLOOR_DELTA_C;
    if (auto_target_lo < hot_floor) auto_target_lo = hot_floor;
    float headroom = (auto_target_hi - auto_target_lo) * HOT_HEADROOM_FRACTION;
    if (headroom < HOT_HEADROOM_MIN_C) headroom = HOT_HEADROOM_MIN_C;
    auto_target_hi += headroom;
    if (auto_target_hi < auto_target_lo + MLX_AUTO_MIN_SPAN_C) {
      auto_target_hi = auto_target_lo + MLX_AUTO_MIN_SPAN_C;
    }
  }

  float center = (t[11*32 + 15] + t[11*32 + 16] +
                  t[12*32 + 15] + t[12*32 + 16]) * 0.25f;
  float range_lo, range_hi;
  updateAutoDisplayRange(auto_target_lo, auto_target_hi, range_lo, range_hi);
  if (range_mode == RANGE_AUTO2 && range_lo < auto2_floor) {
    range_lo = auto2_floor;
    if (range_hi < range_lo + MLX_AUTO_MIN_SPAN_C) {
      range_hi = range_lo + MLX_AUTO_MIN_SPAN_C;
    }
    mlx_range_lo = range_lo;
    mlx_range_hi = range_hi;
  } else if (range_mode == RANGE_AUTO3 && range_hi > auto3_ceiling) {
    range_hi = auto3_ceiling;
    if (range_lo > range_hi - MLX_AUTO_MIN_SPAN_C) {
      range_lo = range_hi - MLX_AUTO_MIN_SPAN_C;
    }
    mlx_range_lo = range_lo;
    mlx_range_hi = range_hi;
  }
  if (range_mode == RANGE_HOT && range_lo < hot_floor) {
    range_lo = hot_floor;
    if (range_hi < range_lo + MLX_AUTO_MIN_SPAN_C) {
      range_hi = range_lo + MLX_AUTO_MIN_SPAN_C;
    }
    mlx_range_lo = range_lo;
    mlx_range_hi = range_hi;
  }

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
      if (shouldThrottleMLXForScreenCameraOnly()) {
        vTaskDelay(pdMS_TO_TICKS(125));
        continue;
      }
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
extern volatile uint8_t stream_jpeg_quality;
extern uint32_t stream_jpeg_fail_count;
extern volatile int8_t cam_contrast;
extern volatile int8_t cam_saturation;
extern volatile int8_t cam_sharpness;
extern volatile int8_t cam_denoise;
extern volatile bool cam_lenc;
extern volatile bool cam_raw_gma;

static constexpr uint8_t CAMERA_RGB565_FB_COUNT = 2;
static constexpr camera_grab_mode_t CAMERA_RGB565_GRAB_MODE = CAMERA_GRAB_LATEST;
static constexpr bool STREAM_NATIVE_JPEG_PREFERRED = false;

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
  s->set_contrast(s, (int)cam_contrast);
  s->set_saturation(s, (int)cam_saturation);
  s->set_sharpness(s, (int)cam_sharpness);
  s->set_denoise(s, (int)cam_denoise);
  if (s->set_aec2) s->set_aec2(s, 1);
  s->set_lenc(s, cam_lenc ? 1 : 0);
  s->set_raw_gma(s, cam_raw_gma ? 1 : 0);
  if (s->set_quality) s->set_quality(s, (int)stream_jpeg_quality);
}

bool applyCameraJpegQuality() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s || !s->set_quality) return false;
  return s->set_quality(s, (int)stream_jpeg_quality) == 0;
}

void buildCameraConfig(camera_config_t &cfg, pixformat_t format,
                       framesize_t frame_size, uint8_t jpeg_quality,
                       uint8_t fb_count, camera_grab_mode_t grab_mode) {
  cfg = {};
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
  cfg.pin_sccb_sda = -1;
  cfg.pin_sccb_scl = -1;
  cfg.sccb_i2c_port = 0;
  cfg.pin_pwdn = -1;
  cfg.pin_reset= -1;
  cfg.xclk_freq_hz = 16000000;
  cfg.pixel_format = format;
  cfg.frame_size   = frame_size;
  cfg.jpeg_quality = jpeg_quality;
  cfg.fb_count     = fb_count;
  cfg.fb_location  = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode    = grab_mode;
}

void enableCameraPsramDmaIfAvailable() {
#if CAMERA_HAS_SAFE_RECONFIGURE
  if (psramFound()) {
    esp_err_t err = esp_camera_set_psram_mode(true);
    if (err != ESP_OK) {
      Serial.printf("Camera: PSRAM DMA enable skipped: 0x%X %s\n",
                    (unsigned)err, esp_err_to_name(err));
    }
  }
#endif
}

bool finishCameraDriverStart() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    cam_fail_stage = CAM_STAGE_NO_SENSOR;
    Serial.println("Camera init failed: sensor pointer is null after esp_camera_init");
    esp_camera_deinit();
    camera_driver_active = false;
    return false;
  }
  cam_sensor_pid = s->id.PID;
  enableCameraPsramDmaIfAvailable();
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

bool prepareCameraStart(pixformat_t format, uint8_t fb_count,
                        bool require_snapshot) {
  cam_init_attempts++;
  cam_ok = false;
  cam_have_frame = false;
  cam_last_err = ESP_OK;
  cam_sensor_pid = -1;
  cam_last_frame_len = 0;
  updateCameraMemoryDiagnostics();
  Serial.printf("Camera init attempt %lu: PSRAM found=%d size=%lu free=%lu fmt=%d fb=%u\n",
                (unsigned long)cam_init_attempts,
                cam_psram_found ? 1 : 0,
                (unsigned long)cam_psram_size,
                (unsigned long)cam_psram_free,
                (int)format, (unsigned)fb_count);
  return !require_snapshot || ensureCameraSnapshotBuffer();
}

bool initCameraFormat(pixformat_t format, framesize_t frame_size,
                      uint8_t jpeg_quality, uint8_t fb_count,
                      camera_grab_mode_t grab_mode,
                      bool require_snapshot) {
  if (!prepareCameraStart(format, fb_count, require_snapshot)) return false;

  camera_config_t cfg;
  buildCameraConfig(cfg, format, frame_size, jpeg_quality, fb_count, grab_mode);
  cam_last_err = esp_camera_init(&cfg);
  if (cam_last_err != ESP_OK) {
    cam_fail_stage = CAM_STAGE_INIT_FAIL;
    Serial.printf("Camera init failed: 0x%X %s\n",
                  (unsigned)cam_last_err, cameraErrName());
    return false;
  }
  camera_driver_active = true;
  return finishCameraDriverStart();
}

bool reconfigureCameraFormat(pixformat_t format, framesize_t frame_size,
                             uint8_t jpeg_quality, uint8_t fb_count,
                             camera_grab_mode_t grab_mode,
                             bool require_snapshot) {
#if CAMERA_HAS_SAFE_RECONFIGURE
  if (!camera_driver_active) {
    return initCameraFormat(format, frame_size, jpeg_quality, fb_count,
                            grab_mode, require_snapshot);
  }
  if (!prepareCameraStart(format, fb_count, require_snapshot)) return false;

  camera_config_t cfg;
  buildCameraConfig(cfg, format, frame_size, jpeg_quality, fb_count, grab_mode);
  cam_last_err = esp_camera_reconfigure(&cfg);
  if (cam_last_err != ESP_OK) {
    cam_fail_stage = CAM_STAGE_INIT_FAIL;
    Serial.printf("Camera reconfigure failed: 0x%X %s\n",
                  (unsigned)cam_last_err, cameraErrName());
    return false;
  }
  camera_driver_active = true;
  return finishCameraDriverStart();
#else
  (void)format; (void)frame_size; (void)jpeg_quality; (void)fb_count;
  (void)grab_mode; (void)require_snapshot;
  return false;
#endif
}

bool initCamera() {
  stream_native_jpeg_active = false;
  return initCameraFormat(PIXFORMAT_RGB565, FRAMESIZE_QVGA, 12,
                          CAMERA_RGB565_FB_COUNT, CAMERA_RGB565_GRAB_MODE,
                          true);
}

bool initCameraNativeJpeg() {
  return initCameraFormat(PIXFORMAT_JPEG, FRAMESIZE_QVGA,
                          stream_jpeg_quality, 2,
                          CAMERA_GRAB_LATEST, false);
}

void deinitCameraDriver() {
  if (!camera_driver_active) return;
  esp_camera_deinit();
  camera_driver_active = false;
  cam_ok = false;
  cam_have_frame = false;
}

bool grabCamera();

bool warmCameraSnapshot(uint8_t attempts) {
  for (uint8_t i = 0; i < attempts; i++) {
    if (grabCamera()) return true;
    delay(25);
  }
  return cam_have_frame;
}

bool restoreRgb565CameraMode() {
#if CAMERA_HAS_SAFE_RECONFIGURE
  if (camera_driver_active) {
    stream_native_jpeg_active = false;
    if (reconfigureCameraFormat(PIXFORMAT_RGB565, FRAMESIZE_QVGA, 12,
                                CAMERA_RGB565_FB_COUNT,
                                CAMERA_RGB565_GRAB_MODE, true)) {
      warmCameraSnapshot(3);
      return true;
    }
  }
#endif
  stream_native_jpeg_active = false;
  deinitCameraDriver();
  delay(50);
  bool ok = initCamera();
  if (ok) warmCameraSnapshot(3);
  return ok;
}

bool validateNativeJpegCamera() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;
  bool ok = (fb->format == PIXFORMAT_JPEG && fb->len > 0);
  esp_camera_fb_return(fb);
  return ok;
}

bool switchToNativeJpegCameraMode() {
  if (!STREAM_NATIVE_JPEG_PREFERRED || !camera_driver_active || !cam_ok) {
    return false;
  }

  bool ok = reconfigureCameraFormat(PIXFORMAT_JPEG, FRAMESIZE_QVGA,
                                    stream_jpeg_quality, 2,
                                    CAMERA_GRAB_LATEST, false);
  if (!ok) {
    restoreRgb565CameraMode();
    return false;
  }
  applyCameraJpegQuality();
  if (!validateNativeJpegCamera()) {
    stream_jpeg_fail_count++;
    Serial.println("Stream camera: native JPEG self-test failed");
    restoreRgb565CameraMode();
    return false;
  }
  stream_native_jpeg_active = true;
  return true;
}

bool enterStreamCameraMode() {
  stream_native_jpeg_active = false;
  if (!camera_driver_active || !cam_ok) return false;

  if (switchToNativeJpegCameraMode()) {
    Serial.println("Stream camera: native JPEG");
    return true;
  }

  warmCameraSnapshot(2);
  Serial.println("Stream camera: RGB565 JPEG fallback");
  return false;
}

void exitStreamCameraMode() {
  if (!stream_native_jpeg_active) return;
  if (!restoreRgb565CameraMode()) {
    Serial.println("Stream camera: RGB565 restore failed");
  }
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
// Ironbow palette.
//   0% black, 17% blue/violet, 33% magenta/red, 50% red,
//   67% orange, 83% yellow, 100% white.
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
    if (j < 30) {                       // black to blue
      R = 0;                       G = 0;                              B = 20  + (120 * j) / 30;
    } else if (j < 60) {                // blue to red
      R = (120 * (j - 30)) / 30;   G = 0;                              B = 140 - (60  * (j - 30)) / 30;
    } else if (j < 90) {                // red
      R = 120 + (135 * (j - 60)) / 30;  G = 0;                         B = 80  - (70  * (j - 60)) / 30;
    } else if (j < 120) {               // red to red-orange
      R = 255;                     G = (60 * (j - 90)) / 30;           B = 10  - (10  * (j - 90)) / 30;
    } else if (j < 150) {               // orange to yellow
      R = 255;                     G = 60 + (175 * (j - 120)) / 30;    B = 0;
    } else {                            // yellow to white
      R = 255;                     G = 235 + (20 * (j - 150)) / 30;    B = (255 * (j - 150)) / 30;
    }
    palR[i] = clamp8(R); palG[i] = clamp8(G); palB[i] = clamp8(B);
    palette_native[i] = ((palR[i] & 0xF8) << 8) |
                        ((palG[i] & 0xFC) << 3) |
                        ( palB[i] >> 3);
    palette_wire[i] = swap565(palette_native[i]);
  }
}

static inline bool rangeUsesColdPalette(RangeMode mode) {
  return mode == RANGE_INVERT || mode == RANGE_AUTO3;
}

uint8_t tempToPaletteIndexForMode(float t, float lo, float hi, bool cold_palette) {
  float f = cold_palette ? (hi - t) / ((hi - lo) + 0.001f)
                         : (t - lo) / ((hi - lo) + 0.001f);
  if (!(f >= 0.0f)) f = 0.0f;
  else if (f > 1.0f) f = 1.0f;
  int idx = (int)(f * 255.0f + 0.5f);
  if (idx < 0) idx = 0; else if (idx > 255) idx = 255;
  return (uint8_t)idx;
}

uint8_t tempToPaletteIndex(float t, float lo, float hi) {
  return tempToPaletteIndexForMode(t, lo, hi, rangeUsesColdPalette(range_mode));
}

// ---------------------------------------------------------------- State ---
enum DisplayMode { MODE_TINT=0, MODE_THERMAL_ONLY, MODE_CAMERA_ONLY, MODE_COUNT };
const char *MODE_NAMES[MODE_COUNT] = {"TINT", "THERM", "CAM"};
const uint16_t MODE_BG[MODE_COUNT] = {TFT_GREEN, TFT_ORANGE, TFT_BLUE};

// Range presets.
//
// AUTO : stable percentile auto range.
// INV  : same range as AUTO, but with the palette direction inverted.
// AUT2 : warm-subject auto; raises the low end above estimated background.
// AUT3 : cold-subject auto; caps the high end below estimated background.
// DET  : tighter percentile clipping for subtle differences.
// HOT  : ambient-suppressed auto range with high-end headroom.
// SKIN : 28 - 38 C.
// PCB  : 30 - 110 C.
// HIGH : 40 - 260 C.
// WIDE : -10 - 120 C.
// COLD : -20 - 25 C.
// MAN  : user drags LO/HI sliders directly (RANGE tab on the side panel).
//        Persisted across reboots.
// RangeMode enum + auto-range constants are forward-declared near the top of the
// file (right under the MLX block) because analyzeMLX needs them before this
// point. Only the *arrays* live here.
const char *RANGE_NAMES[RANGE_COUNT] = {"AUTO", "INV", "AUT2", "AUT3", "DET",
                                        "HOT", "SKIN", "PCB", "HIGH", "WIDE",
                                        "COLD", "MAN"};
static constexpr float RANGE_LO[RANGE_COUNT] = {0.0f,  0.0f,  0.0f,  0.0f,  0.0f,
                                                0.0f, 28.0f, 30.0f, 40.0f,
                                                -10.0f, -20.0f, 0.0f};
static constexpr float RANGE_HI[RANGE_COUNT] = {0.0f,  0.0f,  0.0f,  0.0f,  0.0f,
                                                0.0f, 38.0f, 110.0f, 260.0f,
                                                120.0f, 25.0f, 0.0f};
static constexpr uint8_t RANGE_SETTINGS_VERSION = 5;

static constexpr int PARALLAX_MIN = -90;
static constexpr int PARALLAX_MAX =  90;
static constexpr int PARALLAX_SCALE = 100;
static constexpr int PARALLAX_MIN100 = PARALLAX_MIN * PARALLAX_SCALE;
static constexpr int PARALLAX_MAX100 = PARALLAX_MAX * PARALLAX_SCALE;
static constexpr int ZOOM_MIN = 100;
static constexpr int ZOOM_MAX = 250;
// Field calibration from the 20-30 cm overlay test. Distance still changes the
// X parallax term; these constants correct the fixed optical-center and FOV
// mismatch so ADV sliders are only extra offsets.
static constexpr int ALIGN_BASE_PX100 = 125;
static constexpr int ALIGN_BASE_PY100 = -670;
static constexpr int ALIGN_BASE_ZX = 126;
static constexpr int ALIGN_BASE_ZY = 126;
static constexpr uint8_t ALIGN_SETTINGS_VERSION = 6;
static constexpr int ALIGN_BASE_PX100_V5_REBASE = 845;
static constexpr int ALIGN_DEFAULT_CM = 30;
static constexpr int ALIGN_MIN_CM = 5;
static constexpr int ALIGN_MAX_CM = 500;
static constexpr int ALIGN_NUDGE_CM = 5;
// px100 = coefficient / distance_cm. Coefficient is derived from:
// 100 * 320 px * 1.160 cm / (2 * tan(55 deg / 2)).
static constexpr int ALIGN_PARALLAX_COEFF_100_CM = 35654;
static constexpr int ALIGN_ZOOM_OFFSET_MIN = -50;
static constexpr int ALIGN_ZOOM_OFFSET_MAX =  50;

struct AlignPreset {
  int16_t cm;
  const char *label;
};

static constexpr AlignPreset ALIGN_PRESETS[] = {
  {  5, "Macro 5"   },
  { 15, "Close 15"  },
  { 30, "Desk 30"   },
  {100, "Room 100"  },
  {500, "Far 500"   },
};
static constexpr int ALIGN_PRESET_COUNT = sizeof(ALIGN_PRESETS) / sizeof(ALIGN_PRESETS[0]);

volatile int parallax_x100 = 374;
volatile int parallax_y100 = 0;
volatile int zoom_x_pct = ALIGN_BASE_ZX;
volatile int zoom_y_pct = ALIGN_BASE_ZY;
volatile int align_offset_x100 = 0;
volatile int align_offset_y100 = 0;
volatile int align_zoom_x_offset = 0;
volatile int align_zoom_y_offset = 0;
volatile int align_distance_cm = ALIGN_DEFAULT_CM;
volatile uint8_t tint_pct = 70;
volatile DisplayMode display_mode = MODE_TINT;
volatile RangeMode range_mode = RANGE_AUTO;
volatile bool freeze_mode = false;
volatile bool stream_mode = false;
volatile uint8_t stream_view_mode = 0;  // 0=overlay, 1=camera, 2=thermal
volatile bool stream_hud_enabled = true;
volatile bool stream_crosshair_enabled = true;
bool screen_marker_active = false;
int screen_marker_x = IMG_W / 2;
int screen_marker_y = IMG_H / 2;
float screen_marker_temp = NAN;
uint32_t screen_marker_seq = 0;

// MANUAL-mode lo/hi setpoints. Used only when range_mode == RANGE_MANUAL.
// On entering MANUAL we seed these from whatever range was active so the
// transition isn't a jarring jump. Persisted across reboots.
volatile float manual_lo = 22.0f;
volatile float manual_hi = 38.0f;
// Side-panel tab. Numeric values keep old persisted tabs meaningful:
// 0=ALIGN, 1=ADV, 2=MAN, 3=TUNE.
enum PanelTab { PANEL_TAB_PRESET = 0, PANEL_TAB_ADVANCED, PANEL_TAB_RANGE, PANEL_TAB_TUNE, PANEL_TAB_COUNT };
volatile uint8_t panel_tab = PANEL_TAB_PRESET;
static constexpr float MANUAL_T_MIN = -20.0f;
static constexpr float MANUAL_T_MAX = 300.0f;
static constexpr float MANUAL_MIN_SPAN_C = 1.0f;

// Camera brightness is a sensor register write, not per-pixel CPU work.
// 0 = neutral, +2 = brightest. Persisted in NVS.
volatile int8_t cam_brightness = 0;
volatile int8_t cam_contrast = 0;
volatile int8_t cam_saturation = 0;
volatile int8_t cam_sharpness = 0;
volatile int8_t cam_denoise = 0;
volatile bool cam_lenc = true;
volatile bool cam_raw_gma = true;
bool             brightness_apply_pending = false;

static constexpr uint8_t LCD_BRIGHTNESS_MIN = 40;
static constexpr uint8_t LCD_BRIGHTNESS_MAX = 255;
static constexpr uint8_t LCD_BRIGHTNESS_STEP = 15;
volatile uint8_t lcd_brightness = LCD_BRIGHTNESS_MAX;

static constexpr uint8_t STREAM_JPEG_QUALITY_DEFAULT = 45;
static constexpr uint8_t STREAM_JPEG_QUALITY_MIN = 35;
static constexpr uint8_t STREAM_JPEG_QUALITY_MAX = 85;
volatile uint8_t stream_jpeg_quality = STREAM_JPEG_QUALITY_DEFAULT;

static inline uint8_t clampStreamJpegQuality(int q) {
  if (q < STREAM_JPEG_QUALITY_MIN) return STREAM_JPEG_QUALITY_MIN;
  if (q > STREAM_JPEG_QUALITY_MAX) return STREAM_JPEG_QUALITY_MAX;
  return (uint8_t)q;
}

// ---------------------------------------------------------- Persistence --
// Settings are saved to NVS after a quiet period to reduce flash writes.
Preferences prefs;
bool     settings_dirty = false;
uint32_t settings_dirty_ms = 0;
uint32_t save_indicator_until_ms = 0;
static constexpr uint32_t SETTINGS_DEBOUNCE_MS = 3000;

static inline void markDirty() {
  settings_dirty = true;
  settings_dirty_ms = millis();
}

static inline void setDirtyInt(volatile int &dst, int value) {
  if (dst != value) { dst = value; markDirty(); }
}

static inline void setDirtyU8(volatile uint8_t &dst, uint8_t value) {
  if (dst != value) { dst = value; markDirty(); }
}

static inline uint8_t clampLcdBrightness(int value) {
  if (value < LCD_BRIGHTNESS_MIN) return LCD_BRIGHTNESS_MIN;
  if (value > LCD_BRIGHTNESS_MAX) return LCD_BRIGHTNESS_MAX;
  return (uint8_t)value;
}

static inline int lcdBrightnessToSliderPct(uint8_t value) {
  uint8_t v = clampLcdBrightness(value);
  return (int)(v - LCD_BRIGHTNESS_MIN) * 100 /
         (LCD_BRIGHTNESS_MAX - LCD_BRIGHTNESS_MIN);
}

static inline uint8_t sliderPctToLcdBrightness(int pct) {
  if (pct < 0) pct = 0; else if (pct > 100) pct = 100;
  return clampLcdBrightness(LCD_BRIGHTNESS_MIN +
                            pct * (LCD_BRIGHTNESS_MAX - LCD_BRIGHTNESS_MIN) / 100);
}

static inline int lcdBrightnessDisplayPct(uint8_t value) {
  return ((int)clampLcdBrightness(value) * 100 + 127) / 255;
}

void applyLcdBrightness() {
  lcd.setBrightness(clampLcdBrightness(lcd_brightness));
}

void setLcdBrightnessValue(int value, bool dirty) {
  uint8_t next = clampLcdBrightness(value);
  if (next == lcd_brightness) return;
  lcd_brightness = next;
  applyLcdBrightness();
  if (dirty) markDirty();
}

static inline int clampParallax100(int v) {
  if (v < PARALLAX_MIN100) return PARALLAX_MIN100;
  if (v > PARALLAX_MAX100) return PARALLAX_MAX100;
  return v;
}

static inline int clampZoomPct(int v) {
  if (v < ZOOM_MIN) return ZOOM_MIN;
  if (v > ZOOM_MAX) return ZOOM_MAX;
  return v;
}

static inline int clampZoomOffsetPct(int v) {
  if (v < ALIGN_ZOOM_OFFSET_MIN) return ALIGN_ZOOM_OFFSET_MIN;
  if (v > ALIGN_ZOOM_OFFSET_MAX) return ALIGN_ZOOM_OFFSET_MAX;
  return v;
}

static inline int round100(int v) {
  return v >= 0 ? (v + 50) / 100 : (v - 50) / 100;
}

int presetIndexForCm(int cm) {
  for (int i = 0; i < ALIGN_PRESET_COUNT; i++) {
    if (ALIGN_PRESETS[i].cm == cm) return i;
  }
  return -1;
}

static inline int clampDistanceCm(int cm) {
  if (cm < ALIGN_MIN_CM) return ALIGN_MIN_CM;
  if (cm > ALIGN_MAX_CM) return ALIGN_MAX_CM;
  return cm;
}

static inline int parallaxX100ForDistance(int cm) {
  cm = clampDistanceCm(cm);
  return (ALIGN_PARALLAX_COEFF_100_CM + cm / 2) / cm;
}

static inline int baseParallaxX100ForDistance(int cm) {
  return clampParallax100(ALIGN_BASE_PX100 + parallaxX100ForDistance(cm));
}

static inline int baseParallaxY100() { return ALIGN_BASE_PY100; }
static inline int baseZoomXPct() { return ALIGN_BASE_ZX; }
static inline int baseZoomYPct() { return ALIGN_BASE_ZY; }

void recomputeAlignmentTransform() {
  int base_px100 = baseParallaxX100ForDistance(align_distance_cm);
  parallax_x100 = clampParallax100(base_px100 + align_offset_x100);
  parallax_y100 = clampParallax100(baseParallaxY100() + align_offset_y100);
  zoom_x_pct = clampZoomPct(baseZoomXPct() + align_zoom_x_offset);
  zoom_y_pct = clampZoomPct(baseZoomYPct() + align_zoom_y_offset);
}

void setAlignmentOffsets(int offx100, int offy100, int offzx, int offzy, bool dirty) {
  align_offset_x100 = clampParallax100(offx100);
  align_offset_y100 = clampParallax100(offy100);
  align_zoom_x_offset = clampZoomOffsetPct(offzx);
  align_zoom_y_offset = clampZoomOffsetPct(offzy);
  recomputeAlignmentTransform();
  if (dirty) markDirty();
}

void applyAlignmentDistanceCm(int cm, bool dirty) {
  align_distance_cm = clampDistanceCm(cm);
  recomputeAlignmentTransform();
  if (dirty) markDirty();
}

bool applyAlignmentPresetCm(int cm, bool dirty) {
  if (presetIndexForCm(cm) < 0) return false;
  applyAlignmentDistanceCm(cm, dirty);
  return true;
}

void resetAlignmentCalibrated(bool dirty) {
  align_offset_x100 = 0;
  align_offset_y100 = 0;
  align_zoom_x_offset = 0;
  align_zoom_y_offset = 0;
  applyAlignmentDistanceCm(ALIGN_DEFAULT_CM, dirty);
}

void loadSettings() {
  // Keep the flir2 namespace because older zoom/range semantics were not
  // compatible with the current alignment model.
  prefs.begin("flir2", true);                  // read-only
  align_distance_cm = prefs.getInt("dist", prefs.getInt("apcm", ALIGN_DEFAULT_CM));
  uint8_t av   = prefs.getUChar("alv", 0);
  int old_px100 = prefs.getInt("px100", baseParallaxX100ForDistance(align_distance_cm));
  int old_py100 = prefs.getInt("py100", 0);
  int old_zx = prefs.getInt("zx", ALIGN_BASE_ZX);
  int old_zy = prefs.getInt("zy", ALIGN_BASE_ZY);
  align_offset_x100 = prefs.getInt("offx100", 0);
  align_offset_y100 = prefs.getInt("offy100", 0);
  align_zoom_x_offset = prefs.getInt("offzx", 0);
  align_zoom_y_offset = prefs.getInt("offzy", 0);
  tint_pct     = prefs.getUChar("tint", 70);
  uint8_t m    = prefs.getUChar("mode", MODE_TINT);
  uint8_t r    = prefs.getUChar("rng",  RANGE_AUTO);
  uint8_t rv   = prefs.getUChar("rngv", 1);
  cam_brightness = (int8_t)prefs.getChar("brt", 0);
  cam_contrast = (int8_t)prefs.getChar("con", 0);
  cam_saturation = (int8_t)prefs.getChar("sat", 0);
  cam_sharpness = (int8_t)prefs.getChar("shp", 0);
  cam_denoise = (int8_t)prefs.getChar("den", 0);
  cam_lenc = prefs.getBool("lenc", true);
  cam_raw_gma = prefs.getBool("gma", true);
  lcd_brightness = clampLcdBrightness(prefs.getUChar("lcdbrt", LCD_BRIGHTNESS_MAX));
  stream_jpeg_quality = clampStreamJpegQuality(prefs.getUChar("jpgq", STREAM_JPEG_QUALITY_DEFAULT));
  manual_lo    = prefs.getFloat("mlo", 22.0f);
  manual_hi    = prefs.getFloat("mhi", 38.0f);
  bool has_panel_tab = prefs.isKey("ptab");
  uint8_t pt   = prefs.getUChar("ptab", PANEL_TAB_TUNE);
  prefs.end();
  bool migrated_alignment = false;
  // Defensive clamps in case ranges have moved across firmware versions.
  align_distance_cm = clampDistanceCm(align_distance_cm);
  if (av < 1) {
    align_offset_x100 = old_px100 - baseParallaxX100ForDistance(align_distance_cm);
    align_offset_y100 = old_py100 - baseParallaxY100();
    align_zoom_x_offset = old_zx - ALIGN_BASE_ZX;
    align_zoom_y_offset = old_zy - ALIGN_BASE_ZY;
  }
  if (av < 5) {
    align_offset_x100 = 0;
    align_offset_y100 = 0;
    align_zoom_x_offset = 0;
    align_zoom_y_offset = 0;
    migrated_alignment = true;
  } else if (av < ALIGN_SETTINGS_VERSION) {
    align_offset_x100 = clampParallax100(align_offset_x100 - ALIGN_BASE_PX100_V5_REBASE);
    migrated_alignment = true;
  }
  setAlignmentOffsets(align_offset_x100, align_offset_y100,
                      align_zoom_x_offset, align_zoom_y_offset, false);
  applyAlignmentDistanceCm(align_distance_cm, false);
  if (tint_pct   > 100) tint_pct   = 100;
  if (cam_brightness < -2) cam_brightness = -2; else if (cam_brightness > 2) cam_brightness = 2;
  if (cam_contrast < -2) cam_contrast = -2; else if (cam_contrast > 2) cam_contrast = 2;
  if (cam_saturation < -2) cam_saturation = -2; else if (cam_saturation > 2) cam_saturation = 2;
  if (cam_sharpness < -2) cam_sharpness = -2; else if (cam_sharpness > 2) cam_sharpness = 2;
  if (cam_denoise < 0) cam_denoise = 0; else if (cam_denoise > 1) cam_denoise = 1;
  lcd_brightness = clampLcdBrightness(lcd_brightness);
  if (manual_lo  < MANUAL_T_MIN) manual_lo = MANUAL_T_MIN;
  if (manual_lo  > MANUAL_T_MAX) manual_lo = MANUAL_T_MAX;
  if (manual_hi  < MANUAL_T_MIN) manual_hi = MANUAL_T_MIN;
  if (manual_hi  > MANUAL_T_MAX) manual_hi = MANUAL_T_MAX;
  if (manual_hi - manual_lo < MANUAL_MIN_SPAN_C) {
    if (manual_lo > MANUAL_T_MAX - MANUAL_MIN_SPAN_C) {
      manual_lo = MANUAL_T_MAX - MANUAL_MIN_SPAN_C;
    }
    manual_hi = manual_lo + MANUAL_MIN_SPAN_C;
  }
  display_mode = (m < MODE_COUNT)  ? (DisplayMode)m : MODE_TINT;
  if (rv < 2) {
    switch (r) {
      case 1: r = RANGE_AUTO2; break;  // legacy AUT2
      case 2: r = RANGE_SKIN; break;   // legacy SKIN
      case 3: r = RANGE_SKIN; break;   // legacy BODY
      case 4: r = RANGE_HOT; break;    // legacy WARM
      case 5: r = RANGE_PCB; break;    // legacy HOT
      case 6: r = RANGE_HIGH; break;   // legacy VHOT
      case 7: r = RANGE_MANUAL; break; // legacy MAN
      default: break;
    }
  } else if (rv == 2) {
    switch (r) {
      case 1: r = RANGE_DETAIL; break;
      case 2: r = RANGE_HOT; break;
      case 3: r = RANGE_SKIN; break;
      case 4: r = RANGE_PCB; break;
      case 5: r = RANGE_HIGH; break;
      case 6: r = RANGE_WIDE; break;
      case 7: r = RANGE_COLD; break;
      case 8: r = RANGE_MANUAL; break;
      default: break;
    }
  } else if (rv == 3) {
    switch (r) {
      case 1: r = RANGE_DETAIL; break;
      case 2: r = RANGE_AUTO2; break;
      case 3: r = RANGE_HOT; break;
      case 4: r = RANGE_SKIN; break;
      case 5: r = RANGE_PCB; break;
      case 6: r = RANGE_HIGH; break;
      case 7: r = RANGE_WIDE; break;
      case 8: r = RANGE_COLD; break;
      case 9: r = RANGE_MANUAL; break;
      default: break;
    }
  } else if (rv == 4 && r >= RANGE_INVERT) {
    r++;
  }
  range_mode   = (r < RANGE_COUNT) ? (RangeMode)r   : RANGE_AUTO;
  panel_tab    = (has_panel_tab && pt < PANEL_TAB_COUNT) ? (uint8_t)pt : (uint8_t)PANEL_TAB_TUNE;
  brightness_apply_pending = true;        // push into sensor on first frame
  if (migrated_alignment) markDirty();
  Serial.printf("Loaded: PX100=%+d,%+d Off=%+d,%+d Z=%d/%d%% Zoff=%+d/%+d Dist=%dcm Tint=%d%% Brt=%+d LCD=%u JPGQ=%u Mode=%s Rng=%s Man=[%.1f,%.1f]\n",
                parallax_x100, parallax_y100,
                align_offset_x100, align_offset_y100,
                zoom_x_pct, zoom_y_pct,
                align_zoom_x_offset, align_zoom_y_offset,
                align_distance_cm, tint_pct, cam_brightness,
                (unsigned)lcd_brightness,
                (unsigned)stream_jpeg_quality, MODE_NAMES[display_mode], RANGE_NAMES[range_mode],
                manual_lo, manual_hi);
}

void saveSettings() {
  prefs.begin("flir2", false);                 // read-write
  prefs.putInt  ("px",   round100(parallax_x100));
  prefs.putInt  ("py",   round100(parallax_y100));
  prefs.putInt  ("zoom", zoom_y_pct);
  prefs.putInt  ("px100", parallax_x100);
  prefs.putInt  ("py100", parallax_y100);
  prefs.putInt  ("zx", zoom_x_pct);
  prefs.putInt  ("zy", zoom_y_pct);
  prefs.putInt  ("offx100", align_offset_x100);
  prefs.putInt  ("offy100", align_offset_y100);
  prefs.putInt  ("offzx", align_zoom_x_offset);
  prefs.putInt  ("offzy", align_zoom_y_offset);
  prefs.putUChar("alv", ALIGN_SETTINGS_VERSION);
  prefs.putInt  ("apcm", align_distance_cm);
  prefs.putInt  ("dist", align_distance_cm);
  prefs.putUChar("tint", tint_pct);
  prefs.putUChar("mode", (uint8_t)display_mode);
  prefs.putUChar("rng",  (uint8_t)range_mode);
  prefs.putUChar("rngv", RANGE_SETTINGS_VERSION);
  prefs.putChar ("brt",  (int8_t)cam_brightness);
  prefs.putChar ("con",  (int8_t)cam_contrast);
  prefs.putChar ("sat",  (int8_t)cam_saturation);
  prefs.putChar ("shp",  (int8_t)cam_sharpness);
  prefs.putChar ("den",  (int8_t)cam_denoise);
  prefs.putBool ("lenc", cam_lenc);
  prefs.putBool ("gma",  cam_raw_gma);
  prefs.putUChar("lcdbrt", lcd_brightness);
  prefs.putUChar("jpgq", stream_jpeg_quality);
  prefs.putFloat("mlo",  manual_lo);
  prefs.putFloat("mhi",  manual_hi);
  prefs.putUChar("ptab", panel_tab);
  prefs.end();
  save_indicator_until_ms = millis() + 1500;
  Serial.printf("Saved: PX100=%+d,%+d Off=%+d,%+d Z=%d/%d%% Zoff=%+d/%+d Dist=%dcm Tint=%d%% Brt=%+d LCD=%u JPGQ=%u Mode=%s Rng=%s Man=[%.1f,%.1f]\n",
                parallax_x100, parallax_y100,
                align_offset_x100, align_offset_y100,
                zoom_x_pct, zoom_y_pct,
                align_zoom_x_offset, align_zoom_y_offset,
                align_distance_cm, tint_pct, cam_brightness,
                (unsigned)lcd_brightness,
                (unsigned)stream_jpeg_quality, MODE_NAMES[display_mode], RANGE_NAMES[range_mode],
                manual_lo, manual_hi);
}

// Apply camera register controls only when they change.
void applyCameraBrightness() {
  if (!cam_ok) return;
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  s->set_brightness(s, (int)cam_brightness);
  s->set_contrast(s, (int)cam_contrast);
  s->set_saturation(s, (int)cam_saturation);
  s->set_sharpness(s, (int)cam_sharpness);
  s->set_denoise(s, (int)cam_denoise);
  if (s->set_aec2) s->set_aec2(s, 1);
  s->set_lenc(s, cam_lenc ? 1 : 0);
  s->set_raw_gma(s, cam_raw_gma ? 1 : 0);
  Serial.printf("Cam regs: brt=%+d con=%+d sat=%+d sharp=%+d den=%d lenc=%d gma=%d\n",
                cam_brightness, cam_contrast, cam_saturation, cam_sharpness,
                cam_denoise, cam_lenc ? 1 : 0, cam_raw_gma ? 1 : 0);
}

void getThermalRange(float &lo, float &hi) {
  // MANUAL bypasses the IIR-tracked range.
  if (range_mode == RANGE_MANUAL) {
    float ml = manual_lo, mh = manual_hi;
    // Keep at least a 1 C span.
    if (mh - ml < 1.0f) mh = ml + 1.0f;
    lo = ml; hi = mh;
    return;
  }
  // AUTO-like modes pipe through analyzeMLX -> IIR-smoothed t_range_min/max.
  if (range_mode == RANGE_AUTO || range_mode == RANGE_INVERT ||
      range_mode == RANGE_AUTO2 || range_mode == RANGE_AUTO3 ||
      range_mode == RANGE_DETAIL || range_mode == RANGE_HOT) {
    portENTER_CRITICAL(&mlx_mux);
    lo = t_range_min; hi = t_range_max;
    portEXIT_CRITICAL(&mlx_mux);
    return;
  }
  // Fixed preset.
  lo = RANGE_LO[range_mode];
  hi = RANGE_HI[range_mode];
}

void formatRangeDisplay(char *buf, size_t len, bool compact) {
  float lo, hi;
  getThermalRange(lo, hi);
  if (compact) {
    snprintf(buf, len, "%s %.0f-%.0fC", RANGE_NAMES[range_mode], lo, hi);
  } else {
    snprintf(buf, len, "%s %.1f-%.1fC", RANGE_NAMES[range_mode], lo, hi);
  }
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
  // Map GT911 native 320x480 coordinates to LCD_ROTATION.
  *tx = 479 - ry;
  *ty = rx;
  return true;
}

// ---------------------------------------------------------------- UI -----
struct TouchButton {
  int x, y, w, h;
  const char *label;
  uint16_t color;
  void (*action)();
};

bool shouldThrottleMLXForScreenCameraOnly() {
  return display_mode == MODE_CAMERA_ONLY &&
         !stream_mode &&
         !freeze_mode &&
         panel_tab != PANEL_TAB_ADVANCED;
}

static int PANEL_X = 348;
static int PANEL_W = 128;
static int PANEL_SLIDER_X = 352;
static int PANEL_SLIDER_W = 120;
static int PANEL_SLIDER_H = 20;
static constexpr int PANEL_NUDGE_W  = 22;
static constexpr int PANEL_NUDGE_GAP = 3;
static constexpr uint32_t NUDGE_REPEAT_MS = 130;
static constexpr int TINT_NUDGE_PCT = 1;
static constexpr float MANUAL_NUDGE_C = 0.1f;
static constexpr int ALIGN_OFFSET_UI_MAX100 = 4000; // +/- 40 px
static constexpr int ALIGN_OFFSET_NUDGE100 = 25;    // 0.25 px
static constexpr int ALIGN_ZOOM_NUDGE_PCT = 1;
static int ACTION_ROW_Y  = 286;
static int ACTION_ROW_H  = 30;
static int TAB_ROW_Y     = 8;
static int TAB_ROW_H     = 28;
static int TAB_COL_W     = 62;
static int X_SLIDER_Y    = 126;
static int Y_SLIDER_Y    = 168;
static int Z_SLIDER_Y    = 210;
static int TINT_SLIDER_Y = 126;
static int ADV_X_Y       = 118;
static int ADV_Y_Y       = 148;
static int ADV_ZX_Y      = 178;
static int ADV_ZY_Y      = 208;
static int ADV_BRT_Y     = 238;
static int ADV_LCD_BRT_Y = 268;
static int ADV_RESET_Y   = 292;

void btnCycleMode()  { display_mode = (DisplayMode)((display_mode + 1) % MODE_COUNT);  markDirty(); }
void btnReset()      { resetAlignmentCalibrated(true); }
void btnCycleRange() {
  range_mode = (RangeMode)((range_mode + 1) % RANGE_COUNT);
  // Entering MANUAL opens the LO/HI controls.
  if (range_mode == RANGE_MANUAL) panel_tab = PANEL_TAB_RANGE;
  markDirty();
}
void btnTabPreset() { panel_tab = PANEL_TAB_PRESET; markDirty(); }
void btnTabAdvanced() { panel_tab = PANEL_TAB_ADVANCED; markDirty(); }
void btnTabRange() { panel_tab = PANEL_TAB_RANGE; markDirty(); }
void btnTabTune() { panel_tab = PANEL_TAB_TUNE; markDirty(); }

void drawStaticUI();
void layoutUi();

void redrawFullUi() {
  lcd.fillScreen(TFT_BLACK);
  drawStaticUI();
}

static constexpr int BRT_NUM_CELLS = 5;
static int PALETTE_X = 5;
static int PALETTE_Y_TOP = 70;
static int PALETTE_Y_BOTTOM = 260;
static int PALETTE_W = 12;
static int STATUS_BADGE_X = 1;
static int STATUS_MODE_Y = 3;
static int STATUS_RANGE_Y = 22;
static int STATUS_BADGE_W = 58;
static int STATUS_BADGE_H = 16;
static int STATUS_EVENT_X = 1;
static int STATUS_EVENT_Y = 42;
static int STATUS_EVENT_W = 22;
static int STATUS_EVENT_H = 14;
static int TEMP_CLEAR_X = 70;
static int TEMP_CLEAR_Y = 0;
static int TEMP_CLEAR_W = 270;
static int TEMP_CLEAR_H = 36;
static int FPS_X = 1;
static int FPS_Y = 304;
static int FPS_W = 78;
static int FPS_H = 12;

TouchButton buttons[] = {
  {0, 0, 0, 0, "VIEW",  TFT_DARKGREEN, btnCycleMode    },
  {0, 0, 0, 0, "RANGE", TFT_PURPLE,    btnCycleRange   },
  {0, 0, 0, 0, "ALIGN", TFT_DARKGREY,  btnTabPreset    },
  {0, 0, 0, 0, "TUNE",  TFT_DARKGREY,  btnTabTune      },
  {0, 0, 0, 0, "MAN",   TFT_DARKGREY,  btnTabRange     },
  {0, 0, 0, 0, "ADV",   TFT_DARKGREY,  btnTabAdvanced  },
};
const int NUM_BUTTONS = sizeof(buttons) / sizeof(buttons[0]);

uint16_t rangeButtonColor() {
  switch (range_mode) {
    case RANGE_INVERT:
    case RANGE_AUTO3:
    case RANGE_COLD: return TFT_BLUE;
    case RANGE_AUTO2:
    case RANGE_HOT:  return TFT_DARKGREEN;
    case RANGE_MANUAL: return TFT_MAROON;
    default: return TFT_PURPLE;
  }
}

void layoutUi() {
  IMG_X = 24;
  IMG_Y = 40;
  PANEL_X = IMG_X + IMG_W + 4;
  PANEL_W = 480 - PANEL_X - 4;
  PANEL_SLIDER_X = PANEL_X + 4;
  PANEL_SLIDER_W = PANEL_W - 8;
  PANEL_SLIDER_H = 20;
  ACTION_ROW_Y = 286;
  ACTION_ROW_H = 30;
  TAB_ROW_Y = 8;
  TAB_ROW_H = 28;
  TAB_COL_W = (PANEL_W - 4) / 2;
  X_SLIDER_Y = 126;
  Y_SLIDER_Y = 168;
  Z_SLIDER_Y = 210;
  TINT_SLIDER_Y = 126;
  ADV_X_Y = 118;
  ADV_Y_Y = 148;
  ADV_ZX_Y = 178;
  ADV_ZY_Y = 208;
  ADV_BRT_Y = 238;
  ADV_LCD_BRT_Y = 268;
  ADV_RESET_Y = 292;
  PALETTE_X = 5;
  PALETTE_Y_TOP = 70;
  PALETTE_Y_BOTTOM = 260;
  PALETTE_W = 12;
  STATUS_BADGE_X = 1;
  STATUS_MODE_Y = 3;
  STATUS_RANGE_Y = 22;
  STATUS_BADGE_W = 58;
  STATUS_BADGE_H = 16;
  STATUS_EVENT_X = 1;
  STATUS_EVENT_Y = 42;
  STATUS_EVENT_W = 22;
  STATUS_EVENT_H = 14;
  TEMP_CLEAR_X = 70;
  TEMP_CLEAR_Y = 0;
  TEMP_CLEAR_W = 270;
  TEMP_CLEAR_H = 36;
  FPS_X = 1;
  FPS_Y = 304;
  FPS_W = 78;
  FPS_H = 12;

  const int gap = 4;
  const int action_gap = 6;
  const int action_x = FPS_X + FPS_W + 6;
  int action_w = (PANEL_X - action_x - action_gap - 4) / 2;
  for (int i = 0; i < 2; i++) {
    buttons[i].x = action_x + i * (action_w + action_gap);
    buttons[i].y = ACTION_ROW_Y;
    buttons[i].w = action_w;
    buttons[i].h = ACTION_ROW_H;
  }
  for (int i = 0; i < 4; i++) {
    int row = i / 2;
    int col = i % 2;
    buttons[2 + i].x = PANEL_X + col * (TAB_COL_W + gap);
    buttons[2 + i].y = TAB_ROW_Y + row * (TAB_ROW_H + 4);
    buttons[2 + i].w = TAB_COL_W;
    buttons[2 + i].h = TAB_ROW_H;
  }
}

void drawButtons() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    auto &b = buttons[i];
    uint16_t bg = b.color;
    const char *label = b.label;
    if (b.action == btnCycleMode) bg = MODE_BG[display_mode];
    if (b.action == btnCycleRange) bg = rangeButtonColor();
    if (b.action == btnCycleMode) label = MODE_NAMES[display_mode];
    if (b.action == btnCycleRange) label = RANGE_NAMES[range_mode];
    bool tab_active = (b.action == btnTabPreset && panel_tab == PANEL_TAB_PRESET) ||
                      (b.action == btnTabAdvanced && panel_tab == PANEL_TAB_ADVANCED) ||
                      (b.action == btnTabRange && panel_tab == PANEL_TAB_RANGE) ||
                      (b.action == btnTabTune && panel_tab == PANEL_TAB_TUNE);
    if (b.action == btnTabPreset || b.action == btnTabAdvanced ||
        b.action == btnTabRange || b.action == btnTabTune) {
      bg = tab_active ? TFT_BLUE : TFT_DARKGREY;
    }
    uint16_t fg = (b.action == btnCycleMode && display_mode == MODE_TINT) ? TFT_BLACK : TFT_WHITE;
    lcd.fillRoundRect(b.x, b.y, b.w, b.h, 4, bg);
    lcd.drawRoundRect(b.x, b.y, b.w, b.h, 4, TFT_WHITE);
    lcd.setTextColor(fg, bg);
    lcd.setTextSize(1);
    lcd.setCursor(b.x + (b.w - (int)strlen(label) * 6) / 2, b.y + (b.h - 8) / 2);
    lcd.print(label);
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
  return tx >= x && tx < x + w && ty >= y - 12 && ty < y + h + 5;
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
                 uint16_t fill, bool enabled,
                 const char *left_nudge, const char *right_nudge) {
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
  drawNudgeButton(x, y, PANEL_NUDGE_W, h, left_nudge, enabled);
  drawNudgeButton(x + w - PANEL_NUDGE_W, y, PANEL_NUDGE_W, h, right_nudge, enabled);
  lcd.fillRect(track_x, y, track_w, h, bg);
  lcd.drawRect(track_x, y, track_w, h, TFT_WHITE);
  int fill_w = (pct * (track_w - 4)) / 100;
  if (fill_w > 0) lcd.fillRect(track_x + 2, y + 2, fill_w, h - 4, enabled ? fill : TFT_DARKGREY);
  int knob_x = track_x + 2 + fill_w;
  if (knob_x > track_x + track_w - 4) knob_x = track_x + track_w - 4;
  lcd.fillRect(knob_x - 2, y - 2, 5, h + 4, enabled ? TFT_WHITE : TFT_LIGHTGREY);
}

static inline bool compactSliderRowHit(uint16_t tx, uint16_t ty,
                                       int x, int y, int w, int h) {
  return tx >= x && tx < x + w && ty >= y - 8 && ty < y + h + 4;
}

void drawCompactSlider(int x, int y, int w, int h, int pct,
                       const char *label, const char *value,
                       uint16_t fill, bool enabled) {
  if (pct < 0) pct = 0; else if (pct > 100) pct = 100;
  uint16_t bg = enabled ? TFT_DARKGREY : 0x4208;
  uint16_t fg = enabled ? TFT_WHITE : TFT_LIGHTGREY;
  int track_x = sliderTrackX(x);
  int track_w = sliderTrackW(w);
  lcd.fillRect(x, y - 9, w, h + 13, TFT_BLACK);
  lcd.setTextSize(1);
  lcd.setTextColor(fg, TFT_BLACK);
  lcd.setCursor(x, y - 8); lcd.print(label);
  int tw = (int)strlen(value) * 6;
  lcd.setCursor(x + w - tw, y - 8); lcd.print(value);
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

void clearPanelContent() {
  lcd.fillRect(PANEL_X, 104, PANEL_W, 216, TFT_BLACK);
}

void drawPresetPanel() {
  char buf[24];
  snprintf(buf, sizeof(buf), "%dcm", align_distance_cm);
  int pct = (align_distance_cm - ALIGN_MIN_CM) * 100 / (ALIGN_MAX_CM - ALIGN_MIN_CM);
  clearPanelContent();
  drawHSlider(PANEL_SLIDER_X, X_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H,
              pct, "DIST", buf, TFT_GREEN, true, "-5", "+5");

  int chip_gap = 3;
  int chip_w = (PANEL_SLIDER_W - chip_gap * 2) / 3;
  int chip_h = 18;
  int chip_y = Y_SLIDER_Y - 8;
  for (int i = 0; i < ALIGN_PRESET_COUNT; i++) {
    int row = i / 3;
    int col = i % 3;
    int x = PANEL_SLIDER_X + col * (chip_w + chip_gap);
    if (row == 1) x += (chip_w + chip_gap) / 2;
    int y = chip_y + row * (chip_h + 4);
    bool active = align_distance_cm == ALIGN_PRESETS[i].cm;
    uint16_t bg = active ? TFT_DARKGREEN : TFT_DARKGREY;
    lcd.fillRoundRect(x, y, chip_w, chip_h, 4, bg);
    lcd.drawRoundRect(x, y, chip_w, chip_h, 4, TFT_WHITE);
    snprintf(buf, sizeof(buf), "%d", ALIGN_PRESETS[i].cm);
    lcd.setTextColor(TFT_WHITE, bg);
    lcd.setTextSize(1);
    lcd.setCursor(x + (chip_w - (int)strlen(buf) * 6) / 2, y + 5);
    lcd.print(buf);
  }

  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setCursor(PANEL_SLIDER_X, Z_SLIDER_Y);
  lcd.print("Plane-specific");
  snprintf(buf, sizeof(buf), "X %+.2fpx", parallax_x100 / 100.0f);
  lcd.setCursor(PANEL_SLIDER_X, Z_SLIDER_Y + 14);
  lcd.print(buf);
  snprintf(buf, sizeof(buf), "ZX/Y %d/%d%%", zoom_x_pct, zoom_y_pct);
  lcd.setCursor(PANEL_SLIDER_X, Z_SLIDER_Y + 28);
  lcd.print(buf);
}

static inline int offset100ToPct(int v) {
  if (v < -ALIGN_OFFSET_UI_MAX100) v = -ALIGN_OFFSET_UI_MAX100;
  if (v >  ALIGN_OFFSET_UI_MAX100) v =  ALIGN_OFFSET_UI_MAX100;
  return (v + ALIGN_OFFSET_UI_MAX100) * 100 / (2 * ALIGN_OFFSET_UI_MAX100);
}

static inline int pctToOffset100(int pct) {
  if (pct < 0) pct = 0; else if (pct > 100) pct = 100;
  return -ALIGN_OFFSET_UI_MAX100 + pct * (2 * ALIGN_OFFSET_UI_MAX100) / 100;
}

static inline int zoomOffsetToPct(int v) {
  v = clampZoomOffsetPct(v);
  return (v - ALIGN_ZOOM_OFFSET_MIN) * 100 /
         (ALIGN_ZOOM_OFFSET_MAX - ALIGN_ZOOM_OFFSET_MIN);
}

static inline int pctToZoomOffset(int pct) {
  if (pct < 0) pct = 0; else if (pct > 100) pct = 100;
  return ALIGN_ZOOM_OFFSET_MIN +
         pct * (ALIGN_ZOOM_OFFSET_MAX - ALIGN_ZOOM_OFFSET_MIN) / 100;
}

void drawPanelBrightness() {
  char buf[8];
  const int gap = 2;
  const int h = 18;
  int cell_w = (PANEL_SLIDER_W - gap * (BRT_NUM_CELLS - 1)) / BRT_NUM_CELLS;
  lcd.fillRect(PANEL_SLIDER_X, ADV_BRT_Y - 10, PANEL_SLIDER_W, h + 12, TFT_BLACK);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setCursor(PANEL_SLIDER_X, ADV_BRT_Y - 9);
  lcd.print("CAM");
  snprintf(buf, sizeof(buf), "%+d", cam_brightness);
  lcd.setCursor(PANEL_SLIDER_X + PANEL_SLIDER_W - (int)strlen(buf) * 6, ADV_BRT_Y - 9);
  lcd.print(buf);
  for (int i = 0; i < BRT_NUM_CELLS; i++) {
    int v = i - 2;
    int x = PANEL_SLIDER_X + i * (cell_w + gap);
    bool on = (v == cam_brightness);
    uint16_t bg = on ? TFT_CYAN : TFT_DARKGREY;
    uint16_t fg = on ? TFT_BLACK : TFT_WHITE;
    lcd.fillRoundRect(x, ADV_BRT_Y, cell_w, h, 3, bg);
    lcd.drawRoundRect(x, ADV_BRT_Y, cell_w, h, 3, TFT_WHITE);
    snprintf(buf, sizeof(buf), "%+d", v);
    lcd.setTextColor(fg, bg);
    lcd.setCursor(x + (cell_w - (int)strlen(buf) * 6) / 2, ADV_BRT_Y + 5);
    lcd.print(buf);
  }
}

void drawPanelLcdBrightness() {
  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", lcdBrightnessDisplayPct(lcd_brightness));
  drawCompactSlider(PANEL_SLIDER_X, ADV_LCD_BRT_Y, PANEL_SLIDER_W, 16,
                    lcdBrightnessToSliderPct(lcd_brightness), "LCD", buf,
                    TFT_YELLOW, true);
}

void drawAdvancedResetButton() {
  const int h = 28;
  lcd.fillRoundRect(PANEL_X, ADV_RESET_Y, PANEL_W, h, 4, TFT_MAROON);
  lcd.drawRoundRect(PANEL_X, ADV_RESET_Y, PANEL_W, h, 4, TFT_WHITE);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE, TFT_MAROON);
  const char *label = "RESET ALIGN";
  lcd.setCursor(PANEL_X + (PANEL_W - (int)strlen(label) * 6) / 2, ADV_RESET_Y + 10);
  lcd.print(label);
}

void drawAdvancedAlignmentPanel() {
  char buf[18];
  clearPanelContent();
  snprintf(buf, sizeof(buf), "%+.2fpx", align_offset_x100 / 100.0f);
  drawCompactSlider(PANEL_SLIDER_X, ADV_X_Y, PANEL_SLIDER_W, 16,
                    offset100ToPct(align_offset_x100), "X OFF", buf, TFT_CYAN, true);
  snprintf(buf, sizeof(buf), "%+.2fpx", align_offset_y100 / 100.0f);
  drawCompactSlider(PANEL_SLIDER_X, ADV_Y_Y, PANEL_SLIDER_W, 16,
                    offset100ToPct(align_offset_y100), "Y OFF", buf, TFT_CYAN, true);
  snprintf(buf, sizeof(buf), "%+d%%", align_zoom_x_offset);
  drawCompactSlider(PANEL_SLIDER_X, ADV_ZX_Y, PANEL_SLIDER_W, 16,
                    zoomOffsetToPct(align_zoom_x_offset), "ZX OFF", buf, TFT_GREEN, true);
  snprintf(buf, sizeof(buf), "%+d%%", align_zoom_y_offset);
  drawCompactSlider(PANEL_SLIDER_X, ADV_ZY_Y, PANEL_SLIDER_W, 16,
                    zoomOffsetToPct(align_zoom_y_offset), "ZY OFF", buf, TFT_GREEN, true);
  drawPanelBrightness();
  drawPanelLcdBrightness();
  drawAdvancedResetButton();
}

void drawTunePanel() {
  char buf[18];
  clearPanelContent();
  snprintf(buf, sizeof(buf), "%d%%", tint_pct);
  drawHSlider(PANEL_SLIDER_X, TINT_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H,
              tint_pct, "TINT", buf, TFT_CYAN, true, "-", "+");
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  lcd.setCursor(PANEL_SLIDER_X, Y_SLIDER_Y);
  lcd.print("CAM/LCD BRT");
  lcd.setCursor(PANEL_SLIDER_X, Y_SLIDER_Y + 14);
  lcd.print("is in ADV");
  lcd.setCursor(PANEL_SLIDER_X, Y_SLIDER_Y + 28);
  lcd.print("VIEW/RANGE modes");
}

void drawRangePanel() {
  char buf[18];
  bool enabled = (range_mode == RANGE_MANUAL);
  clearPanelContent();
  snprintf(buf, sizeof(buf), "%.1fC", manual_lo);
  drawHSlider(PANEL_SLIDER_X, X_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H,
              manualTempToPct(manual_lo), "LO", buf, TFT_BLUE, enabled, "-", "+");
  snprintf(buf, sizeof(buf), "%.1fC", manual_hi);
  drawHSlider(PANEL_SLIDER_X, Y_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H,
              manualTempToPct(manual_hi), "HI", buf, TFT_RED, enabled, "-", "+");
  if (!enabled) {
    lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    lcd.setTextSize(1);
    lcd.setCursor(PANEL_SLIDER_X, Z_SLIDER_Y);
    lcd.print("Use RANGE until");
    lcd.setCursor(PANEL_SLIDER_X, Z_SLIDER_Y + 14);
    lcd.print("MAN is active.");
  }
}

void drawAdjustSliders() {
  if (panel_tab == PANEL_TAB_PRESET) {
    drawPresetPanel();
  } else if (panel_tab == PANEL_TAB_ADVANCED) {
    drawAdvancedAlignmentPanel();
  } else if (panel_tab == PANEL_TAB_TUNE) {
    drawTunePanel();
  } else {
    drawRangePanel();
  }
}

uint32_t last_touch_ms = 0;
uint32_t last_nudge_ms = 0;

bool sampleThermalAtImagePoint(int x, int y, float *out);
void setScreenMarkerFromTouch(uint16_t tx, uint16_t ty);
void updateScreenMarkerTemp();

bool nudgeReady(uint32_t now) {
  if (last_nudge_ms && now - last_nudge_ms < NUDGE_REPEAT_MS) return false;
  last_nudge_ms = now;
  return true;
}

bool handlePresetTouch(uint16_t tx, uint16_t ty) {
  int chip_gap = 3;
  int chip_w = (PANEL_SLIDER_W - chip_gap * 2) / 3;
  int chip_h = 18;
  int chip_y = Y_SLIDER_Y - 8;
  for (int i = 0; i < ALIGN_PRESET_COUNT; i++) {
    int row = i / 3;
    int col = i % 3;
    int x = PANEL_SLIDER_X + col * (chip_w + chip_gap);
    if (row == 1) x += (chip_w + chip_gap) / 2;
    int y = chip_y + row * (chip_h + 4);
    if (tx >= x && tx < x + chip_w && ty >= y && ty < y + chip_h) {
      applyAlignmentPresetCm(ALIGN_PRESETS[i].cm, true);
      drawAdjustSliders();
      return true;
    }
  }
  if (!sliderRowHit(tx, ty, PANEL_SLIDER_X, X_SLIDER_Y,
                    PANEL_SLIDER_W, PANEL_SLIDER_H)) {
    return false;
  }
  int dir = sliderNudgeDir(tx, PANEL_SLIDER_X, PANEL_SLIDER_W);
  if (dir && nudgeReady(millis())) {
    applyAlignmentDistanceCm(align_distance_cm + dir * ALIGN_NUDGE_CM, true);
  } else if (sliderTrackHit(tx, PANEL_SLIDER_X, PANEL_SLIDER_W)) {
    int pct = sliderPctFromTrack(tx, PANEL_SLIDER_X, PANEL_SLIDER_W);
    int cm = ALIGN_MIN_CM + pct * (ALIGN_MAX_CM - ALIGN_MIN_CM) / 100;
    applyAlignmentDistanceCm(cm, true);
  }
  drawAdjustSliders();
  return true;
}

bool handleTuneTouch(uint16_t tx, uint16_t ty) {
  if (!sliderRowHit(tx, ty, PANEL_SLIDER_X, TINT_SLIDER_Y,
                    PANEL_SLIDER_W, PANEL_SLIDER_H)) {
    return false;
  }
  int dir = sliderNudgeDir(tx, PANEL_SLIDER_X, PANEL_SLIDER_W);
  if (dir) {
    if (nudgeReady(millis())) {
      setDirtyU8(tint_pct, (uint8_t)constrain((int)tint_pct + dir * TINT_NUDGE_PCT, 0, 100));
      drawAdjustSliders();
    }
    return true;
  }
  if (sliderTrackHit(tx, PANEL_SLIDER_X, PANEL_SLIDER_W)) {
    setDirtyU8(tint_pct, (uint8_t)sliderPctFromTrack(tx, PANEL_SLIDER_X, PANEL_SLIDER_W));
    drawAdjustSliders();
    return true;
  }
  return false;
}

bool handleAdvancedAlignmentTouch(uint16_t tx, uint16_t ty) {
  if (tx >= PANEL_X && tx < PANEL_X + PANEL_W &&
      ty >= ADV_RESET_Y && ty < ADV_RESET_Y + 28) {
    btnReset();
    drawButtons();
    drawAdjustSliders();
    return true;
  }

  if (ty >= ADV_BRT_Y && ty < ADV_BRT_Y + 18 &&
      tx >= PANEL_SLIDER_X && tx < PANEL_SLIDER_X + PANEL_SLIDER_W) {
    const int gap = 2;
    int cell_w = (PANEL_SLIDER_W - gap * (BRT_NUM_CELLS - 1)) / BRT_NUM_CELLS;
    for (int i = 0; i < BRT_NUM_CELLS; i++) {
      int x = PANEL_SLIDER_X + i * (cell_w + gap);
      if (tx >= x && tx < x + cell_w) {
        int8_t new_brt = (int8_t)(i - 2);
        if (new_brt != cam_brightness) {
          cam_brightness = new_brt;
          brightness_apply_pending = true;
          markDirty();
        }
        drawAdjustSliders();
        return true;
      }
    }
  }

  if (tx < PANEL_SLIDER_X || tx >= PANEL_SLIDER_X + PANEL_SLIDER_W) return false;

  uint32_t now = millis();
  int dir = sliderNudgeDir(tx, PANEL_SLIDER_X, PANEL_SLIDER_W);
  bool track_hit = sliderTrackHit(tx, PANEL_SLIDER_X, PANEL_SLIDER_W);

  if (compactSliderRowHit(tx, ty, PANEL_SLIDER_X, ADV_LCD_BRT_Y, PANEL_SLIDER_W, 16)) {
    int v = lcd_brightness;
    if (dir && nudgeReady(now)) v += dir * LCD_BRIGHTNESS_STEP;
    else if (track_hit) v = sliderPctToLcdBrightness(sliderPctFromTrack(tx, PANEL_SLIDER_X, PANEL_SLIDER_W));
    else return false;
    setLcdBrightnessValue(v, true);
    drawAdjustSliders();
    return true;
  }

  if (compactSliderRowHit(tx, ty, PANEL_SLIDER_X, ADV_X_Y, PANEL_SLIDER_W, 16)) {
    int v = align_offset_x100;
    if (dir && nudgeReady(now)) v += dir * ALIGN_OFFSET_NUDGE100;
    else if (track_hit) v = pctToOffset100(sliderPctFromTrack(tx, PANEL_SLIDER_X, PANEL_SLIDER_W));
    else return false;
    setAlignmentOffsets(v, align_offset_y100, align_zoom_x_offset, align_zoom_y_offset, true);
    drawAdjustSliders();
    return true;
  }
  if (compactSliderRowHit(tx, ty, PANEL_SLIDER_X, ADV_Y_Y, PANEL_SLIDER_W, 16)) {
    int v = align_offset_y100;
    if (dir && nudgeReady(now)) v += dir * ALIGN_OFFSET_NUDGE100;
    else if (track_hit) v = pctToOffset100(sliderPctFromTrack(tx, PANEL_SLIDER_X, PANEL_SLIDER_W));
    else return false;
    setAlignmentOffsets(align_offset_x100, v, align_zoom_x_offset, align_zoom_y_offset, true);
    drawAdjustSliders();
    return true;
  }
  if (compactSliderRowHit(tx, ty, PANEL_SLIDER_X, ADV_ZX_Y, PANEL_SLIDER_W, 16)) {
    int v = align_zoom_x_offset;
    if (dir && nudgeReady(now)) v += dir * ALIGN_ZOOM_NUDGE_PCT;
    else if (track_hit) v = pctToZoomOffset(sliderPctFromTrack(tx, PANEL_SLIDER_X, PANEL_SLIDER_W));
    else return false;
    setAlignmentOffsets(align_offset_x100, align_offset_y100, v, align_zoom_y_offset, true);
    drawAdjustSliders();
    return true;
  }
  if (compactSliderRowHit(tx, ty, PANEL_SLIDER_X, ADV_ZY_Y, PANEL_SLIDER_W, 16)) {
    int v = align_zoom_y_offset;
    if (dir && nudgeReady(now)) v += dir * ALIGN_ZOOM_NUDGE_PCT;
    else if (track_hit) v = pctToZoomOffset(sliderPctFromTrack(tx, PANEL_SLIDER_X, PANEL_SLIDER_W));
    else return false;
    setAlignmentOffsets(align_offset_x100, align_offset_y100, align_zoom_x_offset, v, true);
    drawAdjustSliders();
    return true;
  }
  return false;
}

void handleTouch() {
  uint16_t tx, ty;
  if (!gt911_read_touch(&tx, &ty)) return;
  uint32_t now = millis();

  if (panel_tab == PANEL_TAB_PRESET && handlePresetTouch(tx, ty)) return;
  if (panel_tab == PANEL_TAB_ADVANCED && handleAdvancedAlignmentTouch(tx, ty)) return;
  if (panel_tab == PANEL_TAB_TUNE && handleTuneTouch(tx, ty)) return;

  // Manual range sliders.
  if (tx >= PANEL_SLIDER_X && tx < PANEL_SLIDER_X + PANEL_SLIDER_W) {
    int dir = sliderNudgeDir(tx, PANEL_SLIDER_X, PANEL_SLIDER_W);
    bool track_hit = sliderTrackHit(tx, PANEL_SLIDER_X, PANEL_SLIDER_W);

    if (panel_tab == PANEL_TAB_RANGE && range_mode == RANGE_MANUAL) {
      // Convert percent to temperature on the MANUAL_T_MIN..MAX scale.
      bool hit_lo = sliderRowHit(tx, ty, PANEL_SLIDER_X, X_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H);
      bool hit_hi = sliderRowHit(tx, ty, PANEL_SLIDER_X, Y_SLIDER_Y, PANEL_SLIDER_W, PANEL_SLIDER_H);
      if (hit_lo) {
        // Keep at least a 1 C span.
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

  if (tx >= IMG_X && tx < IMG_X + IMG_W &&
      ty >= IMG_Y && ty < IMG_Y + IMG_H) {
    if (now - last_touch_ms >= 120) {
      setScreenMarkerFromTouch(tx, ty);
      last_touch_ms = now;
    }
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
// any user-defined enum exists; using `int` dodges that trap.
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
// Render into a 16-row DRAM bounce buffer, then push swap565 pixels with
// setSwapBytes(false). This avoids direct PSRAM-to-LCD artifacts and keeps the
// LCD transfer path fast.
#define CHUNK_H 16
DRAM_ATTR uint16_t chunk_buf[CHUNK_H * IMG_W];

uint8_t therm_idx[768];
uint8_t therm_idx_prev[768];
uint8_t therm_idx_target[768];
float   therm_build_src[768];
bool     therm_idx_valid = false;
bool     therm_idx_cold = false;
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
  bool cold_palette = rangeUsesColdPalette(range_mode);
  bool same_target = therm_idx_valid &&
                     therm_idx_seq == seq &&
                     therm_idx_lo == lo &&
                     therm_idx_hi == hi &&
                     therm_idx_cold == cold_palette;

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
      float f = cold_palette ? (hi - t) / denom : (t - lo) / denom;
      // Clamp, catching NaN before palette indexing.
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
  therm_idx_cold = cold_palette;
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

// Map camera-pixel offset to thermal cell coordinate in 16.16 fixed point.
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

// Thermal grid fills the full 320x240 image area: 32x24 cells -> 10x10 px.
static constexpr int32_t TH_STEP_X_Q = (int32_t)(((int64_t)32 << 16) / IMG_W);
static constexpr int32_t TH_STEP_Y_Q = (int32_t)(((int64_t)24 << 16) / IMG_H);
static inline void thermal_steps(int32_t &step_x_q, int32_t &step_y_q) {
  step_x_q = TH_STEP_X_Q;
  step_y_q = TH_STEP_Y_Q;
}

static inline int clampPixelCoord(int v, int hi) {
  if (v < 0) return 0;
  if (v > hi) return hi;
  return v;
}

bool sampleThermalAtImagePoint(int x, int y, float *out) {
  if (!out) return false;
  x = clampPixelCoord(x, IMG_W - 1);
  y = clampPixelCoord(y, IMG_H - 1);

  int32_t step_x_q, step_y_q;
  thermal_steps(step_x_q, step_y_q);
  int tx0, tx1, ty0, ty1;
  uint16_t fx, fy;
  bool x_inside, y_inside;
  thermal_indices_fov(x - IMG_W / 2, step_x_q, 16, 31, tx0, tx1, fx, x_inside);
  thermal_indices_fov(y - IMG_H / 2, step_y_q, 12, 23, ty0, ty1, fy, y_inside);
  if (!x_inside || !y_inside) return false;

  float a, b, c, d;
  bool ready;
  portENTER_CRITICAL(&mlx_mux);
  ready = mlx_full_ready;
  a = mlx_temps_full[ty0 * 32 + (31 - tx0)];
  b = mlx_temps_full[ty0 * 32 + (31 - tx1)];
  c = mlx_temps_full[ty1 * 32 + (31 - tx0)];
  d = mlx_temps_full[ty1 * 32 + (31 - tx1)];
  portEXIT_CRITICAL(&mlx_mux);
  if (!ready) return false;

  float ifx = 256.0f - fx;
  float ify = 256.0f - fy;
  float top = (a * ifx + b * fx) / 256.0f;
  float bottom = (c * ifx + d * fx) / 256.0f;
  *out = (top * ify + bottom * fy) / 256.0f;
  return true;
}

void setScreenMarkerFromTouch(uint16_t tx, uint16_t ty) {
  screen_marker_x = clampPixelCoord((int)tx - IMG_X, IMG_W - 1);
  screen_marker_y = clampPixelCoord((int)ty - IMG_Y, IMG_H - 1);
  float temp;
  if (sampleThermalAtImagePoint(screen_marker_x, screen_marker_y, &temp)) {
    screen_marker_temp = temp;
    screen_marker_seq = thermal_frame_seq;
  } else {
    screen_marker_temp = NAN;
    screen_marker_seq = 0;
  }
  screen_marker_active = true;
}

void updateScreenMarkerTemp() {
  if (!screen_marker_active) return;
  uint32_t seq = thermal_frame_seq;
  if (seq == screen_marker_seq) return;
  float temp;
  if (sampleThermalAtImagePoint(screen_marker_x, screen_marker_y, &temp)) {
    screen_marker_temp = temp;
    screen_marker_seq = seq;
  }
}

// Camera crop parameters for zoom and parallax. Output parameters avoid
// Arduino's generated-prototype edge cases with local struct types.
static inline void cameraCropParamsFor(int zx, int zy, int px100, int py100,
                                       int32_t &src_x0_q, int32_t &src_y0_q,
                                       int32_t &step_x_q, int32_t &step_y_q)
{
  zx = clampZoomPct(zx);
  zy = clampZoomPct(zy);
  px100 = clampParallax100(px100);
  py100 = clampParallax100(py100);
  int cw = (IMG_W * 100 + zx / 2) / zx;
  int ch = (IMG_H * 100 + zy / 2) / zy;
  if (cw > IMG_W) cw = IMG_W;
  if (ch > IMG_H) ch = IMG_H;
  int64_t shift_x_q = ((int64_t)px100 * cw << 16) / (PARALLAX_SCALE * IMG_W);
  int64_t shift_y_q = ((int64_t)py100 * ch << 16) / (PARALLAX_SCALE * IMG_H);
  src_x0_q = (int32_t)(((int64_t)(IMG_W - cw) << 15) - shift_x_q);
  src_y0_q = (int32_t)(((int64_t)(IMG_H - ch) << 15) - shift_y_q);
  step_x_q = (int32_t)(((int64_t)cw << 16) / IMG_W);
  step_y_q = (int32_t)(((int64_t)ch << 16) / IMG_H);
}

static inline int floorQ16(int64_t q) {
  return q >= 0 ? (int)(q >> 16) : -(int)((-q + 65535) >> 16);
}

static inline int ceilQ16(int64_t q) {
  return q >= 0 ? (int)((q + 65535) >> 16) : -(int)((-q) >> 16);
}

static inline float q16ToFloat(int64_t q) {
  return (float)((double)q / 65536.0);
}

static inline void clampSourceRect(float &sx, float &sy, float &sw, float &sh,
                                   int crop_w, int crop_h) {
  if (sx < 0.0f) { sw += sx; sx = 0.0f; }
  if (sy < 0.0f) { sh += sy; sy = 0.0f; }
  if (sx + sw > crop_w) sw = crop_w - sx;
  if (sy + sh > crop_h) sh = crop_h - sy;
  if (sw < 1.0f) sw = crop_w;
  if (sh < 1.0f) sh = crop_h;
}

static inline void cameraPortalCropFor(int zx, int zy, int px100, int py100,
                                       int &crop_x, int &crop_y,
                                       int &crop_w, int &crop_h,
                                       float &src_x, float &src_y,
                                       float &src_w, float &src_h) {
  int32_t x0_q, y0_q, step_x_q, step_y_q;
  cameraCropParamsFor(zx, zy, px100, py100, x0_q, y0_q, step_x_q, step_y_q);
  int64_t x1_q = (int64_t)x0_q + (int64_t)step_x_q * IMG_W;
  int64_t y1_q = (int64_t)y0_q + (int64_t)step_y_q * IMG_H;

  crop_x = floorQ16(x0_q) - 1;
  crop_y = floorQ16(y0_q) - 1;
  int crop_x1 = ceilQ16(x1_q) + 1;
  int crop_y1 = ceilQ16(y1_q) + 1;

  if (crop_x < 0) crop_x = 0;
  if (crop_y < 0) crop_y = 0;
  if (crop_x >= IMG_W) crop_x = IMG_W - 1;
  if (crop_y >= IMG_H) crop_y = IMG_H - 1;
  if (crop_x1 > IMG_W) crop_x1 = IMG_W;
  if (crop_y1 > IMG_H) crop_y1 = IMG_H;
  if (crop_x1 <= crop_x) crop_x1 = crop_x + 1;
  if (crop_y1 <= crop_y) crop_y1 = crop_y + 1;

  crop_w = crop_x1 - crop_x;
  crop_h = crop_y1 - crop_y;
  src_x = q16ToFloat(x0_q) - crop_x;
  src_y = q16ToFloat(y0_q) - crop_y;
  src_w = q16ToFloat((int64_t)step_x_q * IMG_W);
  src_h = q16ToFloat((int64_t)step_y_q * IMG_H);
  clampSourceRect(src_x, src_y, src_w, src_h, crop_w, crop_h);
}

static inline void cameraCropParams(int32_t &src_x0_q, int32_t &src_y0_q,
                                    int32_t &step_x_q, int32_t &step_y_q)
{
  cameraCropParamsFor(zoom_x_pct, zoom_y_pct, parallax_x100, parallax_y100,
                      src_x0_q, src_y0_q, step_x_q, step_y_q);
}

static inline uint16_t cameraPixelWireAt(const uint16_t *cam, bool valid,
                                         int x, int y,
                                         int32_t sample_x0_q,
                                         int32_t sample_y0_q,
                                         int32_t step_x_q,
                                         int32_t step_y_q) {
  if (!valid || !cam) return 0;

  int sx = (int)((sample_x0_q + (int64_t)x * step_x_q) >> 16);
  int sy = (int)((sample_y0_q + (int64_t)y * step_y_q) >> 16);
  sx = clampPixelCoord(sx, IMG_W - 1);
  sy = clampPixelCoord(sy, IMG_H - 1);
  return cam[sy * IMG_W + sx];
}

static inline void drawCrosshairRow(uint16_t *row, int y) {
  const int cx = IMG_W / 2;
  const int cy = IMG_H / 2;
  if (y < cy - CROSS_R || y > cy + CROSS_R) return;

  if (y == cy || y == cy - CROSS_R || y == cy + CROSS_R) {
    for (int x = cx - CROSS_R; x <= cx + CROSS_R; x++) row[x] = TFT_WHITE;
  } else {
    row[cx - CROSS_R] = TFT_WHITE;
    row[cx] = TFT_WHITE;
    row[cx + CROSS_R] = TFT_WHITE;
  }
}

static inline void drawMarkerRow(uint16_t *row, int y) {
  if (!screen_marker_active) return;
  static constexpr int MARKER_R = 7;
  int dy = y - screen_marker_y;
  if (dy < -MARKER_R || dy > MARKER_R) return;
  for (int dx = -MARKER_R; dx <= MARKER_R; dx++) {
    int x = screen_marker_x + dx;
    if (x < 0 || x >= IMG_W) continue;
    int d2 = dx * dx + dy * dy;
    bool ring = d2 >= 34 && d2 <= 58;
    if (ring) row[x] = TFT_GREEN;
  }
}

static inline void drawOverlayRow(uint16_t *row, int y) {
  drawCrosshairRow(row, y);
  drawMarkerRow(row, y);
}

void drawCrossbar();
void renderThermalOnly();      // forward decl

void renderTinted() {
  if (!cam_ok || !cam_have_frame) { renderThermalOnly(); return; }

  float lo, hi; getThermalRange(lo, hi);
  buildThermalPaletteFrame(lo, hi);

  uint16_t *cam = (uint16_t*)cam_snapshot;
  uint8_t pct = tint_pct;
  int32_t step_x_q, step_y_q;
  thermal_steps(step_x_q, step_y_q);
  const int cx_center = IMG_W / 2;
  const int cy_center = IMG_H / 2;

  int32_t crop_x0_q, crop_y0_q, crop_step_x_q, crop_step_y_q;
  cameraCropParams(crop_x0_q, crop_y0_q, crop_step_x_q, crop_step_y_q);

  lcd.setSwapBytes(false);
  lcd.startWrite();
  for (int y0 = 0; y0 < IMG_H; y0 += CHUNK_H) {
    int chunk_h = (y0 + CHUNK_H <= IMG_H) ? CHUNK_H : (IMG_H - y0);
    for (int dy = 0; dy < chunk_h; dy++) {
      int y = y0 + dy;
      uint16_t *row = &chunk_buf[dy * IMG_W];
      int ty0, ty1; uint16_t fy; bool y_inside;
      thermal_indices_fov(y - cy_center, step_y_q, 12, 23, ty0, ty1, fy, y_inside);

      int src_y = (int)((crop_y0_q + (int64_t)y * crop_step_y_q) >> 16);
      src_y = clampPixelCoord(src_y, IMG_H - 1);
      uint16_t *cam_row = &cam[src_y * IMG_W];

      for (int x = 0; x < IMG_W; x++) {
        int src_x = (int)((crop_x0_q + (int64_t)x * crop_step_x_q) >> 16);
        uint16_t cbe = cam_row[clampPixelCoord(src_x, IMG_W - 1)];
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
            uint16_t cnat = swap565(cbe);
            uint8_t cr = ((cnat >> 11) & 0x1F) << 3;
            uint8_t cg = ((cnat >> 5)  & 0x3F) << 2;
            uint8_t cb =  (cnat        & 0x1F) << 3;
            uint16_t r = (uint16_t)cr + (uint16_t)palR[pidx] * pct / 100;
            uint16_t g = (uint16_t)cg + (uint16_t)palG[pidx] * pct / 100;
            uint16_t b = (uint16_t)cb + (uint16_t)palB[pidx] * pct / 100;
            if (r > 255) r = 255;
            if (g > 255) g = 255;
            if (b > 255) b = 255;
            out = swap565(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
          }
        }
        row[x] = out;
      }
      drawOverlayRow(row, y);
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
        row[x] = out;
      }
      drawOverlayRow(row, y);
    }
    lcd.pushImage(IMG_X, IMG_Y + y0, IMG_W, chunk_h, chunk_buf);
  }
  lcd.endWrite();
}

void renderCameraOnly() {
  if (!cam_have_frame) {
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
    return;
  }

  uint16_t *cam = (uint16_t*)cam_snapshot;
  if (panel_tab != PANEL_TAB_ADVANCED) {
    lcd.setSwapBytes(false);
    lcd.startWrite();
    for (int y0 = 0; y0 < IMG_H; y0 += CHUNK_H) {
      int chunk_h = (y0 + CHUNK_H <= IMG_H) ? CHUNK_H : (IMG_H - y0);
      for (int dy = 0; dy < chunk_h; dy++) {
        int y = y0 + dy;
        uint16_t *row = &chunk_buf[dy * IMG_W];
        memcpy(row, &cam[y * IMG_W], IMG_W * sizeof(uint16_t));
        drawOverlayRow(row, y);
      }
      lcd.pushImage(IMG_X, IMG_Y + y0, IMG_W, chunk_h, chunk_buf);
    }
    lcd.endWrite();
    return;
  }

  int32_t crop_x0_q, crop_y0_q, crop_step_x_q, crop_step_y_q;
  cameraCropParams(crop_x0_q, crop_y0_q, crop_step_x_q, crop_step_y_q);
  lcd.setSwapBytes(false);
  lcd.startWrite();
  for (int y0 = 0; y0 < IMG_H; y0 += CHUNK_H) {
    int chunk_h = (y0 + CHUNK_H <= IMG_H) ? CHUNK_H : (IMG_H - y0);
    for (int dy = 0; dy < chunk_h; dy++) {
      int y = y0 + dy;
      uint16_t *row = &chunk_buf[dy * IMG_W];
      int src_y = (int)((crop_y0_q + (int64_t)y * crop_step_y_q) >> 16);
      uint16_t *cam_row = &cam[clampPixelCoord(src_y, IMG_H - 1) * IMG_W];
      for (int x = 0; x < IMG_W; x++) {
        int src_x = (int)((crop_x0_q + (int64_t)x * crop_step_x_q) >> 16);
        row[x] = cam_row[clampPixelCoord(src_x, IMG_W - 1)];
      }
      drawOverlayRow(row, y);
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
WiFiServer stream_mjpeg_server(81);
WiFiClient stream_mjpeg_client;

uint8_t *freeze_cam_snapshot = nullptr;
float freeze_thermal_snapshot[768];
bool freeze_cam_valid = false;
bool freeze_thermal_valid = false;
bool share_ap_running = false;
bool share_routes_configured = false;
char share_ap_ssid[32] = "ThermalCam";
uint32_t freeze_capture_ms = 0;
float freeze_range_lo = 20.0f, freeze_range_hi = 30.0f;
RangeMode freeze_range_mode = RANGE_AUTO;
int freeze_zoom_x_pct = ALIGN_BASE_ZX;
int freeze_zoom_y_pct = ALIGN_BASE_ZY;
int freeze_parallax_x100 = 0, freeze_parallax_y100 = 0;
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
float stream_cam_encode_ms = 0.0f;
float stream_cam_send_ms = 0.0f;
float stream_cam_total_ms = 0.0f;
float stream_cam_jpeg_bytes = 0.0f;
float diag_grab_ms = 0.0f, diag_render_ms = 0.0f, diag_ui_ms = 0.0f;
uint32_t stream_cam_last_encode_ms = 0;
uint32_t stream_cam_last_send_ms = 0;
uint32_t stream_cam_last_total_ms = 0;
uint32_t stream_cam_last_jpeg_bytes = 0;
bool stream_stop_pending = false;
uint32_t stream_stop_deadline_ms = 0;
static constexpr uint32_t STREAM_STOP_DELAY_MS = 500;
bool stream_portal_transition_pending = false;
bool stream_cam_cache_valid = false;
uint8_t *stream_cam_jpeg = nullptr;
size_t stream_cam_jpeg_len = 0;
uint8_t *stream_cam_crop_buf = nullptr;
size_t stream_cam_crop_buf_len = 0;
uint16_t stream_cam_encode_w = IMG_W;
uint16_t stream_cam_encode_h = IMG_H;
bool stream_cam_pre_cropped = false;
float stream_cam_crop_sx = 0.0f;
float stream_cam_crop_sy = 0.0f;
float stream_cam_crop_sw = IMG_W;
float stream_cam_crop_sh = IMG_H;
uint32_t stream_cam_cache_ms = 0;
uint32_t stream_cam_cache_seq = 0;
uint32_t stream_cam_cache_count = 0;
uint32_t stream_cam_cache_timer_ms = 0;
float stream_cam_cache_fps = 0.0f;
uint32_t stream_next_cam_capture_ms = 0;
uint32_t stream_mjpeg_last_seq = 0;
uint32_t stream_mjpeg_clients = 0;
bool stream_mjpeg_running = false;
const char *stream_portal_stage = "idle";
static constexpr uint32_t STREAM_CAM_CAPTURE_INTERVAL_MS = 125;

static constexpr uint8_t STREAM_VIEW_OVERLAY = 0;
static constexpr uint8_t STREAM_VIEW_CAMERA  = 1;
static constexpr uint8_t STREAM_VIEW_THERMAL = 2;
// RGB565 snapshot -> browser JPEG quality. Lower quality cuts encode time and WiFi payload.

void setStreamMode(bool enabled);
extern float current_fps, cam_fps, mlx_fps;
extern uint32_t cam_count;
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

static inline uint32_t usToMsRounded(uint32_t us) {
  return (us + 500U) / 1000U;
}

void noteStreamCamTiming(uint32_t encode_us, uint32_t send_us,
                         uint32_t total_us, size_t jpeg_bytes) {
  stream_cam_last_encode_ms = usToMsRounded(encode_us);
  stream_cam_last_send_ms = usToMsRounded(send_us);
  stream_cam_last_total_ms = usToMsRounded(total_us);
  stream_cam_last_jpeg_bytes = (uint32_t)jpeg_bytes;

  const float alpha = 0.22f;
  float encode_ms = encode_us / 1000.0f;
  float send_ms = send_us / 1000.0f;
  float total_ms = total_us / 1000.0f;
  float bytes = (float)jpeg_bytes;
  if (stream_cam_jpeg_bytes <= 0.0f) {
    stream_cam_encode_ms = encode_ms;
    stream_cam_send_ms = send_ms;
    stream_cam_total_ms = total_ms;
    stream_cam_jpeg_bytes = bytes;
  } else {
    stream_cam_encode_ms += (encode_ms - stream_cam_encode_ms) * alpha;
    stream_cam_send_ms += (send_ms - stream_cam_send_ms) * alpha;
    stream_cam_total_ms += (total_ms - stream_cam_total_ms) * alpha;
    stream_cam_jpeg_bytes += (bytes - stream_cam_jpeg_bytes) * alpha;
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
  stream_cam_encode_ms = 0.0f;
  stream_cam_send_ms = 0.0f;
  stream_cam_total_ms = 0.0f;
  stream_cam_jpeg_bytes = 0.0f;
  stream_cam_last_encode_ms = 0;
  stream_cam_last_send_ms = 0;
  stream_cam_last_total_ms = 0;
  stream_cam_last_jpeg_bytes = 0;
  stream_cam_cache_count = 0;
  stream_cam_cache_timer_ms = now;
  stream_cam_cache_fps = 0.0f;
  stream_cam_cache_seq = 0;
  stream_cam_cache_ms = 0;
  stream_mjpeg_last_seq = 0;
  stream_mjpeg_clients = 0;
  stream_portal_stage = "reset";
}

void releaseStreamCameraCache() {
  if (stream_cam_jpeg) {
    free(stream_cam_jpeg);
    stream_cam_jpeg = nullptr;
  }
  if (stream_cam_crop_buf) {
    heap_caps_free(stream_cam_crop_buf);
    stream_cam_crop_buf = nullptr;
  }
  stream_cam_crop_buf_len = 0;
  stream_cam_jpeg_len = 0;
  stream_cam_encode_w = IMG_W;
  stream_cam_encode_h = IMG_H;
  stream_cam_pre_cropped = false;
  stream_cam_crop_sx = 0.0f;
  stream_cam_crop_sy = 0.0f;
  stream_cam_crop_sw = IMG_W;
  stream_cam_crop_sh = IMG_H;
  stream_cam_cache_valid = false;
  stream_cam_cache_ms = 0;
  stream_cam_cache_seq = 0;
}

bool ensureStreamCropBuffer(size_t bytes) {
  if (stream_cam_crop_buf && stream_cam_crop_buf_len >= bytes) return true;
  if (stream_cam_crop_buf) {
    heap_caps_free(stream_cam_crop_buf);
    stream_cam_crop_buf = nullptr;
    stream_cam_crop_buf_len = 0;
  }
  stream_cam_crop_buf = (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!stream_cam_crop_buf) {
    Serial.printf("Stream crop: PSRAM alloc failed (%u bytes)\n", (unsigned)bytes);
    return false;
  }
  stream_cam_crop_buf_len = bytes;
  return true;
}

bool updateStreamCameraCache(bool force) {
  if (!stream_mode || !cam_ok || stream_native_jpeg_active) return false;
  if (stream_view_mode == STREAM_VIEW_THERMAL) return stream_cam_cache_valid;
  uint32_t now = millis();
  if (!force && stream_next_cam_capture_ms &&
      (int32_t)(now - stream_next_cam_capture_ms) < 0) {
    return stream_cam_cache_valid;
  }
  stream_next_cam_capture_ms = now + STREAM_CAM_CAPTURE_INTERVAL_MS;

  uint32_t grab_t0 = micros();
  bool grabbed = grabCamera();
  noteLoopTiming(diag_grab_ms, micros() - grab_t0);
  if (grabbed) cam_count++;
  if (!cam_have_frame || !cam_snapshot) return false;

  uint8_t *jpg = nullptr;
  size_t jpg_len = 0;
  uint8_t jpeg_quality = stream_jpeg_quality;
  uint8_t *encode_src = cam_snapshot;
  size_t encode_len = cam_snapshot_len;
  int crop_x = 0, crop_y = 0, crop_w = IMG_W, crop_h = IMG_H;
  float crop_sx = 0.0f, crop_sy = 0.0f, crop_sw = IMG_W, crop_sh = IMG_H;
  cameraPortalCropFor(zoom_x_pct, zoom_y_pct, parallax_x100, parallax_y100,
                      crop_x, crop_y, crop_w, crop_h,
                      crop_sx, crop_sy, crop_sw, crop_sh);
  bool pre_crop = crop_x != 0 || crop_y != 0 || crop_w != IMG_W || crop_h != IMG_H;
  if (pre_crop) {
    size_t crop_len = (size_t)crop_w * crop_h * 2;
    if (ensureStreamCropBuffer(crop_len)) {
      uint16_t *dst = (uint16_t*)stream_cam_crop_buf;
      uint16_t *src = (uint16_t*)cam_snapshot;
      for (int y = 0; y < crop_h; y++) {
        memcpy(&dst[y * crop_w], &src[(crop_y + y) * IMG_W + crop_x],
               (size_t)crop_w * sizeof(uint16_t));
      }
      encode_src = stream_cam_crop_buf;
      encode_len = crop_len;
    } else {
      pre_crop = false;
    }
  }
  stream_cam_encode_w = pre_crop ? crop_w : IMG_W;
  stream_cam_encode_h = pre_crop ? crop_h : IMG_H;
  stream_cam_pre_cropped = pre_crop;
  stream_cam_crop_sx = pre_crop ? crop_sx : 0.0f;
  stream_cam_crop_sy = pre_crop ? crop_sy : 0.0f;
  stream_cam_crop_sw = pre_crop ? crop_sw : IMG_W;
  stream_cam_crop_sh = pre_crop ? crop_sh : IMG_H;

  uint32_t total_start_us = micros();
  uint32_t encode_start_us = micros();
  bool ok = fmt2jpg(encode_src, encode_len, stream_cam_encode_w, stream_cam_encode_h,
                    PIXFORMAT_RGB565, jpeg_quality, &jpg, &jpg_len);
  uint32_t encode_us = micros() - encode_start_us;
  if (!ok || !jpg || !jpg_len) {
    stream_jpeg_fail_count++;
    if (jpg) free(jpg);
    return false;
  }

  if (stream_cam_jpeg) free(stream_cam_jpeg);
  stream_cam_jpeg = jpg;
  stream_cam_jpeg_len = jpg_len;
  stream_cam_cache_valid = true;
  stream_cam_cache_ms = millis();
  stream_cam_cache_seq++;
  stream_cam_cache_count++;
  if (!stream_cam_cache_timer_ms) stream_cam_cache_timer_ms = now;
  if (now - stream_cam_cache_timer_ms >= 1000) {
    stream_cam_cache_fps = stream_cam_cache_count * 1000.0f /
                           (now - stream_cam_cache_timer_ms);
    stream_cam_cache_count = 0;
    stream_cam_cache_timer_ms = now;
  }
  noteStreamCamTiming(encode_us, 0, micros() - total_start_us, jpg_len);
  return true;
}

void startStreamMjpegServer() {
  if (stream_mjpeg_running) return;
  stream_mjpeg_server.begin();
  stream_mjpeg_running = true;
  stream_mjpeg_last_seq = 0;
  Serial.println("Stream MJPEG: http://192.168.4.1:81/stream");
}

void stopStreamMjpegServer() {
  if (stream_mjpeg_client) stream_mjpeg_client.stop();
  if (stream_mjpeg_running) stream_mjpeg_server.stop();
  stream_mjpeg_running = false;
  stream_mjpeg_last_seq = 0;
}

void handleStreamMjpegServer() {
  if (!stream_mode || !stream_mjpeg_running) return;
  WiFiClient incoming = stream_mjpeg_server.available();
  if (incoming) {
    if (stream_mjpeg_client) stream_mjpeg_client.stop();
    stream_mjpeg_client = incoming;
    stream_mjpeg_last_seq = 0;
    stream_mjpeg_clients++;
    uint32_t until = millis() + 25;
    while (stream_mjpeg_client.connected() && millis() < until) {
      while (stream_mjpeg_client.available()) stream_mjpeg_client.read();
      delay(0);
    }
    stream_mjpeg_client.print(
      "HTTP/1.1 200 OK\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "Cache-Control: no-store\r\n"
      "Connection: close\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n");
  }
  if (!stream_mjpeg_client || !stream_mjpeg_client.connected()) return;
  if (!stream_cam_cache_valid || !stream_cam_jpeg ||
      stream_mjpeg_last_seq == stream_cam_cache_seq) {
    return;
  }
  stream_mjpeg_client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                             (unsigned)stream_cam_jpeg_len);
  stream_mjpeg_client.write(stream_cam_jpeg, stream_cam_jpeg_len);
  stream_mjpeg_client.print("\r\n");
  stream_mjpeg_last_seq = stream_cam_cache_seq;
}

// Portal assets are split so the root page loads reliably over the AP.

static const char PORTAL_INDEX_HTML[] PROGMEM = R"PORTALINDEX(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Thermal Portal</title><link rel="stylesheet" href="/portal.css"></head><body><main>
<header><div><h1><span id="tCenter">--</span>C</h1><p>Scene <b id="tSpan">--</b>C - Range <b id="tRange">--</b>C - Marker <b id="markerTemp">--</b>C</p></div><b id="connState">loading</b></header>
<section class="topStats"><div>Cam <b id="statCam">--</b>fps</div><div>MLX <b id="statMlx">--</b>fps</div><div>Loop <b id="statLoop">--</b>fps</div></section>
<section class="grid"><div class="video"><canvas id="view" width="320" height="240"></canvas><canvas id="recCanvas" width="320" height="240" hidden></canvas><div class="actions recActions"><button id="recBtn">Record</button><label class="check"><input type="checkbox" id="recHud" checked> Record temp overlay</label><button id="stopPortal">Stop portal</button></div><p id="saveMsg">Recording is stored in browser memory until stopped.</p></div>
<div class="panel">
<label>View<select id="viewMode"><option value="0">Overlay</option><option value="1">Camera only</option><option value="2">Thermal only</option></select></label>
<label>Range<select id="rangeMode"><option value="0">AUTO</option><option value="1">INV</option><option value="2">AUT2</option><option value="3">AUT3</option><option value="4">DET</option><option value="5">HOT</option><option value="6">SKIN</option><option value="7">PCB</option><option value="8">HIGH</option><option value="9">WIDE</option><option value="10">COLD</option><option value="11">MAN</option></select></label>
<div class="field"><div>Distance <span id="distv"></span></div><div><div class="chips" id="distPresets"></div><div class="distCtl"><button id="distMinus">-5</button><input id="dist" type="range" min="5" max="500" step="1"><button id="distPlus">+5</button></div></div></div>
<p id="alignInfo">Alignment --</p>
<label>Tint <span id="tintv"></span><input id="tint" type="range" min="0" max="100" step="1"></label>
<label>Brightness <span id="brtv"></span><input id="brt" type="range" min="-2" max="2" step="1"></label>
<details id="manualBox"><summary>Manual range</summary>
<label>LO <span id="mlov"></span><input id="mlo" type="range" min="-20" max="120" step="0.1"></label>
<label>HI <span id="mhiv"></span><input id="mhi" type="range" min="-20" max="120" step="0.1"></label>
<p>Use HIGH or PCB for hotter scenes.</p>
</details>
<details id="alignBox"><summary>Advanced alignment</summary>
<label>X offset <span id="offxv"></span><input id="offx100" type="range" min="-4000" max="4000" step="25"></label>
<label>Y offset <span id="offyv"></span><input id="offy100" type="range" min="-4000" max="4000" step="25"></label>
<label>Zoom X offset <span id="offzxv"></span><input id="offzx" type="range" min="-50" max="50" step="1"></label>
<label>Zoom Y offset <span id="offzyv"></span><input id="offzy" type="range" min="-50" max="50" step="1"></label>
</details>
<details id="advBox"><summary>Advanced camera</summary>
<label>Contrast <span id="conv"></span><input id="con" type="range" min="-2" max="2" step="1"></label>
<label>Saturation <span id="satv"></span><input id="sat" type="range" min="-2" max="2" step="1"></label>
<label>Sharpness <span id="shpv"></span><input id="shp" type="range" min="-2" max="2" step="1"></label>
<label>Denoise <span id="denv"></span><input id="den" type="range" min="0" max="1" step="1"></label>
<label>JPEG Q<select id="jpgq"><option value="35">35 fastest</option><option value="45">45 fast</option><option value="55">55 balanced</option><option value="65">65 detail</option><option value="75">75 high</option></select></label>
</details>
<div class="actions"><button id="resetAlign">Reset 30 cm</button><button id="rotBtn">Rotate</button><button id="snap">Snapshot</button></div>
<label class="check"><input type="checkbox" id="crosshair" checked> Crosshair</label>
</div></section>
<details id="diagBox"><summary>Diagnostics</summary><section id="diag" class="diag"></section></details>
</main><script src="/portal.js"></script></body></html>
)PORTALINDEX";

static const char PORTAL_CSS[] PROGMEM = R"PORTALCSS(
body{margin:0;background:#0b0d0f;color:#e8edf2;font-family:system-ui,-apple-system,Segoe UI,sans-serif}main{max-width:1180px;margin:auto;padding:10px}header{display:flex;justify-content:space-between;gap:12px;align-items:end;flex-wrap:wrap}h1{font-size:34px;margin:0}p{color:#9aa8b3;margin:.25rem 0}.topStats{display:grid;grid-template-columns:repeat(3,minmax(90px,1fr));gap:8px;margin:8px 0}.topStats div{background:#10161b;border:1px solid #26343e;border-radius:8px;padding:8px;color:#cbd6dd}.topStats b{color:#9fe870;font-variant-numeric:tabular-nums}.grid{display:grid;grid-template-columns:minmax(280px,640px) minmax(290px,1fr);gap:10px}.video,.panel,details{background:#12171c;border:1px solid #28343d;border-radius:8px;padding:9px}.panel details{background:transparent;border:0;padding:0;margin:8px 0}canvas{width:100%;height:auto;background:#000;image-rendering:auto;border-radius:6px}label{display:grid;grid-template-columns:1fr auto;gap:6px 10px;align-items:center;margin:10px 0}.field{display:grid;grid-template-columns:96px 1fr;gap:8px;align-items:center;margin:8px 0}label span,.field span{justify-self:end;color:#9fe870;font-variant-numeric:tabular-nums}label input[type=range]{grid-column:1/-1;width:100%;min-width:0}label select{grid-column:1/-1;width:100%}input[type=range]{accent-color:#1683ff}button,select,input{font:inherit}button,select{background:#20303b;color:#fff;border:1px solid #425564;border-radius:6px;padding:7px}button{cursor:pointer}.actions{display:grid;grid-template-columns:repeat(auto-fit,minmax(112px,1fr));gap:8px;margin-top:8px}.recActions{align-items:center}.check{display:block;color:#d9e4eb}.chips{display:grid;grid-template-columns:repeat(5,1fr);gap:5px;margin-bottom:7px}.chips button{padding:6px 4px;font-size:13px}.chips button.active{background:#315d35;border-color:#9fe870;color:#fff}.distCtl{display:grid;grid-template-columns:48px 1fr 48px;gap:7px;align-items:center}.diag{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:6px;margin-top:10px}.diag div{background:#0d1115;border:1px solid #26323b;border-radius:6px;padding:6px;color:#c9d5dd}summary{cursor:pointer;color:#dbe6ed;font-weight:700}#connState{background:#1c252d;border:1px solid #384652;border-radius:6px;padding:5px 8px}.bad{background:#3a1b20!important;border-color:#8a2630!important;color:#ffd1d1!important}@media(max-width:760px){.grid{grid-template-columns:1fr}.field{grid-template-columns:86px 1fr}.topStats{grid-template-columns:1fr 1fr 1fr}.chips{grid-template-columns:repeat(3,1fr)}}
)PORTALCSS";

static const char PORTAL_JS[] PROGMEM = R"PORTALJS(
const W=320,H=240,$=id=>document.getElementById(id),canvas=$('view'),ctx=canvas.getContext('2d'),recCanvas=$('recCanvas'),recCtx=recCanvas.getContext('2d');
const scene=document.createElement('canvas');scene.width=W;scene.height=H;const sctx=scene.getContext('2d');const th=document.createElement('canvas');th.width=W;th.height=H;const thx=th.getContext('2d');
let S={view:0,range:0,px:0,py:0,px100:1313,py100:-670,offx100:0,offy100:0,offzx:0,offzy:0,zx:126,zy:126,basePx100:1313,basePy100:-670,baseZx:126,baseZy:126,alignDistanceCm:30,tint:70,mlo:22,mhi:38,brt:0,con:0,sat:0,shp:0,den:0,crosshair:1,lo:20,hi:30,tCenter:0,tMin:0,tMax:0,seq:0,camTransport:'--',camPreCropped:1,camStreamW:320,camStreamH:240,camCropSx:0,camCropSy:0,camCropSw:320,camCropSh:240},thermal=null,camImg=null,camUrl=null,marker=null,markerTemp=null,dirtyThermal=true,sceneDirty=true,rot=+(localStorage.thermalRotate||0),running=true,camBusy=false,thermBusy=false,stateBusy=false,diagBusy=false,rec=null,recStream=null,chunks=[],recUrl=null,recStarted=0,recTimer=0,sendTimer=0,pending={},camFetchMs=0,camDecodeMs=0,camBytes=0,camStatus='idle',recHud=localStorage.recHud==null?1:+localStorage.recHud;
const PRESETS=[['Macro',5],['Close',15],['Desk',30],['Room',100],['Far',500]],PARA_COEFF=35654,BASE_PX100=125,BASE_PY100=-670,BASE_ZX=126,BASE_ZY=126;
const PAL=Array.from({length:256},(_,i)=>{let j=i*180/255,R,G,B;if(j<30){R=0;G=0;B=20+120*j/30}else if(j<60){R=120*(j-30)/30;G=0;B=140-60*(j-30)/30}else if(j<90){R=120+135*(j-60)/30;G=0;B=80-70*(j-60)/30}else if(j<120){R=255;G=60*(j-90)/30;B=10-10*(j-90)/30}else if(j<150){R=255;G=60+175*(j-120)/30;B=0}else{R=255;G=235+20*(j-150)/30;B=255*(j-150)/30}return[R|0,G|0,B|0]});
const thImg=thx.createImageData(W,H),xMap=[],yMap=[];for(let x=0;x<W;x++){let tx=x*32/W,x0=clamp(Math.floor(tx),0,31),x1=clamp(x0+1,0,31),fx=tx-x0;xMap[x]=[31-x0,31-x1,fx]}for(let y=0;y<H;y++){let ty=y*24/H,y0=clamp(Math.floor(ty),0,23),y1=clamp(y0+1,0,23),fy=ty-y0;yMap[y]=[y0,y1,fy]}
let mjpegImg=new Image(),mjpegActive=false;mjpegImg.crossOrigin='anonymous';
function clamp(v,a,b){return Math.max(a,Math.min(b,v))}function fmt(v,d=1){return Number.isFinite(+v)?(+v).toFixed(d):'--'}function mmss(ms){let s=Math.floor((ms||0)/1000),m=Math.floor(s/60);return m+':'+String(s%60).padStart(2,'0')}function kb(v){return v?Math.round(v/1024)+'K':'--'}function bytes(v){return v?Math.round(v/102.4)/10+'K':'--'}function txt(id,v){let e=$(id);if(e)e.textContent=v}
function fetchTimeout(url,opt={},ms=1500){let c=new AbortController(),t=setTimeout(()=>c.abort(),ms);return fetch(url,Object.assign({},opt,{signal:c.signal})).finally(()=>clearTimeout(t))}function conn(t,b){let e=$('connState');e.textContent=t;e.classList.toggle('bad',!!b)}
function setCanvasRotation(){rot=((rot%4)+4)%4;let rw=rot%2?H:W,rh=rot%2?W:H;canvas.width=rw;canvas.height=rh;recCanvas.width=rw;recCanvas.height=rh;$('rotBtn').textContent='Rotate '+rot*90;localStorage.thermalRotate=rot;sceneDirty=true}
function drawRot(out,src){out.setTransform(1,0,0,1,0,0);out.clearRect(0,0,out.canvas.width,out.canvas.height);out.save();if(rot===1){out.translate(H,0);out.rotate(Math.PI/2)}else if(rot===2){out.translate(W,H);out.rotate(Math.PI)}else if(rot===3){out.translate(0,W);out.rotate(-Math.PI/2)}out.drawImage(src,0,0);out.restore()}
function viewToScene(x,y){let p=rot===1?{x:y,y:H-x}:rot===2?{x:W-x,y:H-y}:rot===3?{x:W-y,y:x}:{x,y};p.x=clamp(p.x,0,W-1);p.y=clamp(p.y,0,H-1);return p}
function rawTempAt(x,y,src=thermal){if(!src)return null;let tx=x*32/W,ty=y*24/H,x0=clamp(Math.floor(tx),0,31),y0=clamp(Math.floor(ty),0,23),x1=clamp(x0+1,0,31),y1=clamp(y0+1,0,23),fx=tx-x0,fy=ty-y0;let t=(r,c)=>src[r*32+(31-c)],a=t(y0,x0),b=t(y0,x1),c=t(y1,x0),d=t(y1,x1);return(a*(1-fx)+b*fx)*(1-fy)+(c*(1-fx)+d*fx)*fy}
function coldRange(){return +S.range===1||+S.range===3||S.rangeName==='INV'||S.rangeName==='AUT3'}function rebuildThermal(){if(!thermal)return;let d=thImg.data,lo=+S.lo||20,hi=+S.hi||30,den=(hi-lo)||1,cold=coldRange();for(let y=0;y<H;y++){let ym=yMap[y],y0=ym[0],y1=ym[1],fy=ym[2],ify=1-fy;for(let x=0;x<W;x++){let xm=xMap[x],x0=xm[0],x1=xm[1],fx=xm[2],ifx=1-fx,a=thermal[y0*32+x0],b=thermal[y0*32+x1],c=thermal[y1*32+x0],e=thermal[y1*32+x1],t=(a*ifx+b*fx)*ify+(c*ifx+e*fx)*fy,p=PAL[clamp(Math.round((cold?(hi-t):(t-lo))/den*255),0,255)],o=(y*W+x)*4;d[o]=p[0];d[o+1]=p[1];d[o+2]=p[2];d[o+3]=255}}thx.putImageData(thImg,0,0);dirtyThermal=false}
function drawCamera(c){let img=(mjpegActive&&mjpegImg.naturalWidth)?mjpegImg:camImg,iw=img&&(img.naturalWidth||img.width),ih=img&&(img.naturalHeight||img.height);if(!img||!iw||!ih)return;try{if(+S.camPreCropped){let sx=+S.camCropSx||0,sy=+S.camCropSy||0,sw=+S.camCropSw||iw,sh=+S.camCropSh||ih;sx=clamp(sx,0,iw-1);sy=clamp(sy,0,ih-1);sw=clamp(sw,1,iw-sx);sh=clamp(sh,1,ih-sy);c.drawImage(img,sx,sy,sw,sh,0,0,W,H);return}let zx=clamp(+S.zx||BASE_ZX,100,250),zy=clamp(+S.zy||BASE_ZY,100,250),cw=W*100/zx,ch=H*100/zy,px=(+S.px100||0)/100,py=(+S.py100||0)/100,sx=(W-cw)/2-px*cw/W,sy=(H-ch)/2-py*ch/H;sx=clamp(sx,0,W-cw);sy=clamp(sy,0,H-ch);c.drawImage(img,sx*iw/W,sy*ih/H,cw*iw/W,ch*ih/H,0,0,W,H)}catch(e){mjpegActive=false;camStatus='poll fallback';getCam()}}
function drawCross(c){if(!S.crosshair)return;c.save();c.strokeStyle='#fff';c.lineWidth=1;c.beginPath();c.moveTo(W/2-10,H/2);c.lineTo(W/2+10,H/2);c.moveTo(W/2,H/2-10);c.lineTo(W/2,H/2+10);c.rect(W/2-10,H/2-10,20,20);c.stroke();c.restore()}
function drawScene(){sctx.clearRect(0,0,W,H);sctx.fillStyle='#000';sctx.fillRect(0,0,W,H);if(+S.view!==2)drawCamera(sctx);if(+S.view!==1&&thermal){if(dirtyThermal)rebuildThermal();sctx.save();if(+S.view===0){sctx.globalCompositeOperation='lighter';sctx.globalAlpha=(+S.tint||0)/100}sctx.drawImage(th,0,0);sctx.restore()}if(marker){sctx.strokeStyle='#9fe870';sctx.beginPath();sctx.arc(marker.x,marker.y,6,0,Math.PI*2);sctx.stroke()}drawCross(sctx)}
function hudPanel(c,x,y,lines,anchor='left'){c.save();c.font='10px system-ui,-apple-system,Segoe UI,sans-serif';let pad=5,lh=12,w=0;for(let l of lines)w=Math.max(w,c.measureText(l).width);w+=pad*2;let h=lines.length*lh+pad*2;if(anchor.includes('right'))x-=w;if(anchor.includes('bottom'))y-=h;c.fillStyle='rgba(7,10,13,.68)';c.strokeStyle='rgba(159,232,112,.7)';c.lineWidth=1;c.fillRect(x,y,w,h);c.strokeRect(x+.5,y+.5,w-1,h-1);c.fillStyle='#eef6fb';for(let i=0;i<lines.length;i++)c.fillText(lines[i],x+pad,y+pad+9+i*lh);c.restore()}
function currentMarkerTemp(){if(marker&&thermal){let t=rawTempAt(marker.x,marker.y);if(t!=null&&Number.isFinite(+t))markerTemp=t}return marker?markerTemp:null}
function drawRecHud(c){if(!recHud)return;let h=c.canvas.height,m=currentMarkerTemp(),mt=m==null?'--':fmt(m),lines=[`Center ${fmt(S.tCenter)}C`,`Marker ${mt}C`,`Scene ${fmt(S.tMin)}-${fmt(S.tMax)}C`,`Palette ${fmt(S.lo)}-${fmt(S.hi)}C`];hudPanel(c,8,h-8,lines,'bottom')}
function render(){if(mjpegActive&&+S.view!==2)sceneDirty=true;if(sceneDirty||rec){drawScene();drawRot(ctx,scene);sceneDirty=false;if(rec){drawRot(recCtx,scene);drawRecHud(recCtx)}}requestAnimationFrame(render)}
function labels(){let px=(S.px100/100).toFixed(2);txt('tCenter',fmt(S.tCenter));txt('tSpan',`${fmt(S.tMin)}-${fmt(S.tMax)}`);txt('tRange',`${fmt(S.lo)}-${fmt(S.hi)}`);txt('markerTemp',markerTemp==null?'--':fmt(markerTemp));txt('statCam',fmt(S.camCacheFps??S.camFps));txt('statMlx',fmt(S.mlxFps));txt('statLoop',fmt(S.loopFps));txt('distv',(S.alignDistanceCm||30)+'cm');txt('alignInfo',`RGB shift ${px}px right - zoom ${S.zx}/${S.zy}% - selected distance plane only`);txt('tintv',S.tint+'%');txt('mlov',fmt(S.mlo)+'C');txt('mhiv',fmt(S.mhi)+'C');txt('brtv',S.brt);txt('conv',S.con);txt('satv',S.sat);txt('shpv',S.shp);txt('denv',S.den);txt('offxv',fmt((S.offx100||0)/100,2)+'px');txt('offyv',fmt((S.offy100||0)/100,2)+'px');txt('offzxv',(S.offzx||0)+'%');txt('offzyv',(S.offzy||0)+'%');document.querySelectorAll('#distPresets button').forEach(b=>b.classList.toggle('active',+b.dataset.cm===+S.alignDistanceCm));if(!$('diagBox').open)return;let items=[['Cam',S.camCacheFps??S.camFps,'fps'],['MLX',S.mlxFps,'fps'],['Loop',S.loopFps,'fps'],['Transport',mjpegActive?'mjpeg-cache':S.camTransport,''],['Stage',S.portalStage||'--',''],['Cache age',S.camCacheAgeMs,'ms'],['JPEG Q',S.jpegQuality,''],['Cam Enc',S.camEncodeMs,'ms'],['Cam Size',bytes(S.camJpegBytes||camBytes),''],['Fetch',fmt(camFetchMs),'ms'],['Decode',fmt(camDecodeMs),'ms'],['Cam Req',S.camReqFps,'fps'],['Therm Req',S.thermalReqFps,'fps'],['API',S.apiFps,'fps'],['Grab',fmt(S.grabMs),'ms'],['Render',fmt(S.renderMs),'ms'],['Heap',kb(S.heap),''],['PSRAM',kb(S.psram),''],['Seq',S.seq||'--',''],['Uptime',mmss(S.portalMs),''],['Reset',S.resetReason??'--',''],['ESP32',S.arduinoEsp32Version??'--',''],['MJPEG',S.mjpegRunning?'on':'off',''],['Cam',camStatus,'']];$('diag').innerHTML=items.map(i=>`<div>${i[0]} <b>${i[1]??'--'}</b>${i[2]}</div>`).join('')}
function manualBounds(){let hi=Math.max(120,+S.mhi||0,+S.tMax||0),max=clamp(Math.ceil((hi+10)/10)*10,120,300);for(let id of ['mlo','mhi'])if($(id)){$(id).min=-20;$(id).max=max}}
function sync(){manualBounds();for(let id of ['rangeMode','viewMode','tint','mlo','mhi','brt','con','sat','shp','den','jpgq','offx100','offy100','offzx','offzy'])if($(id))$(id).value=S[id==='rangeMode'?'range':id==='viewMode'?'view':id==='jpgq'?'jpegQuality':id];$('dist').value=S.alignDistanceCm||30;if($('manualBox')&&+S.range===11)$('manualBox').open=true;$('crosshair').checked=!!S.crosshair;$('recHud').checked=!!recHud;labels()}
function applyState(s){Object.assign(S,s);S.lo=s.rangeLo;S.hi=s.rangeHi;sync();dirtyThermal=true;sceneDirty=true;conn('online',false)}
async function getState(){if(stateBusy||!running)return;stateBusy=true;try{let r=await fetchTimeout('/api/state',{cache:'no-store'},1500);if(!r.ok)throw 0;applyState(await r.json())}catch(e){conn('state retry',true)}finally{stateBusy=false;setTimeout(getState,1000)}}
async function getDiag(){if(diagBusy||!running||!$('diagBox').open)return;diagBusy=true;try{let r=await fetchTimeout('/api/diag',{cache:'no-store'},1500);if(!r.ok)throw 0;Object.assign(S,await r.json());labels()}catch(e){}finally{diagBusy=false;if(running&&$('diagBox').open)setTimeout(getDiag,1000)}}
async function getThermal(){if(thermBusy||!running)return;thermBusy=true;try{let r=await fetchTimeout('/thermal.bin',{cache:'no-store'},1500);if(!r.ok)throw 0;let b=await r.arrayBuffer(),dv=new DataView(b),a=new Float32Array(768);for(let i=0;i<768;i++)a[i]=dv.getInt16(i*2,true)/100;thermal=a;let h=r.headers;S.seq=+(h.get('X-Frame-Seq')||S.seq);S.lo=+(h.get('X-Range-Lo')||S.lo);S.hi=+(h.get('X-Range-Hi')||S.hi);S.tCenter=+(h.get('X-Temp-Center')||S.tCenter);S.tMin=+(h.get('X-Temp-Min')||S.tMin);S.tMax=+(h.get('X-Temp-Max')||S.tMax);if(marker)markerTemp=rawTempAt(marker.x,marker.y);dirtyThermal=true;sceneDirty=true;labels()}catch(e){conn('thermal retry',true)}finally{thermBusy=false;setTimeout(getThermal,+S.view===1?750:(+S.view===2?125:250))}}
function loadImg(url){return new Promise((res,rej)=>{let i=new Image();i.onload=()=>res(i);i.onerror=rej;i.src=url})}
async function setCam(blob){let t=performance.now(),url=URL.createObjectURL(blob);try{let img=await loadImg(url);if(camUrl)URL.revokeObjectURL(camUrl);camUrl=url;camImg=img;camDecodeMs=performance.now()-t;camStatus='ok';sceneDirty=true}catch(e){URL.revokeObjectURL(url);camStatus='decode failed';throw e}}
async function getCam(){if(camBusy||!running||+S.view===2||mjpegActive)return;let t=performance.now();camBusy=true;try{let r=await fetchTimeout('/cam.jpg',{cache:'no-store'},1800);if(!r.ok)throw 0;let h=r.headers;S.camEncodeMs=+(h.get('X-Cam-Encode-Ms')||S.camEncodeMs);S.camSendMs=+(h.get('X-Cam-Send-Ms')||S.camSendMs);S.camTotalMs=+(h.get('X-Cam-Total-Ms')||S.camTotalMs);S.camJpegBytes=+(h.get('X-Cam-Bytes')||S.camJpegBytes);S.jpegQuality=+(h.get('X-JPEG-Quality')||S.jpegQuality);S.camPreCropped=+(h.get('X-Cam-Precropped')||S.camPreCropped);S.camStreamW=+(h.get('X-Cam-Width')||S.camStreamW);S.camStreamH=+(h.get('X-Cam-Height')||S.camStreamH);S.camCropSx=+(h.get('X-Cam-Crop-Sx')||S.camCropSx||0);S.camCropSy=+(h.get('X-Cam-Crop-Sy')||S.camCropSy||0);S.camCropSw=+(h.get('X-Cam-Crop-Sw')||S.camCropSw||S.camStreamW||W);S.camCropSh=+(h.get('X-Cam-Crop-Sh')||S.camCropSh||S.camStreamH||H);S.camTransport=h.get('X-Cam-Transport')||S.camTransport;let blob=await r.blob();camFetchMs=performance.now()-t;camBytes=blob.size;await setCam(blob);labels()}catch(e){camStatus='retry';conn('cam retry',true)}finally{camBusy=false;if(!mjpegActive)setTimeout(getCam,+S.view===1?125:160)}}
function post(o){Object.assign(pending,o);clearTimeout(sendTimer);sendTimer=setTimeout(()=>{let p=new URLSearchParams(pending);pending={};fetchTimeout('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p},1800).catch(()=>conn('control retry',true))},160)}
function recomputeLocalAlign(){let base=BASE_PX100+Math.round(PARA_COEFF/(+S.alignDistanceCm||30));S.basePx100=base;S.basePy100=BASE_PY100;S.baseZx=S.baseZx||BASE_ZX;S.baseZy=S.baseZy||BASE_ZY;S.px100=base+(+S.offx100||0);S.py100=BASE_PY100+(+S.offy100||0);S.zx=clamp((+S.baseZx||BASE_ZX)+(+S.offzx||0),100,250);S.zy=clamp((+S.baseZy||BASE_ZY)+(+S.offzy||0),100,250)}
function setDistance(v){v=clamp(Math.round(v),5,500);S.alignDistanceCm=v;recomputeLocalAlign();$('dist').value=v;post({dist:v});sceneDirty=true;labels()}
function setOffset(id,v){S[id]=+v;recomputeLocalAlign();post({[id]:v});sceneDirty=true;labels()}
for(let id of ['dist','tint','mlo','mhi','brt','con','sat','shp','den'])$(id).oninput=e=>{let v=+e.target.value;if(id==='dist'){setDistance(v)}else{S[id]=v;post({[id]:v});labels()}dirtyThermal=true;sceneDirty=true};
for(let id of ['offx100','offy100','offzx','offzy'])$(id).oninput=e=>setOffset(id,+e.target.value);
$('distMinus').onclick=()=>setDistance((+S.alignDistanceCm||30)-5);$('distPlus').onclick=()=>setDistance((+S.alignDistanceCm||30)+5);
$('distPresets').innerHTML=PRESETS.map(p=>`<button data-cm="${p[1]}">${p[0]}<br>${p[1]}</button>`).join('');$('distPresets').onclick=e=>{let b=e.target.closest('button');if(b)setDistance(+b.dataset.cm)};
$('viewMode').onchange=e=>{S.view=+e.target.value;sceneDirty=true;post({view:S.view});getCam();getThermal()};$('rangeMode').onchange=e=>{S.range=+e.target.value;dirtyThermal=true;sceneDirty=true;post({range:S.range})};$('jpgq').onchange=e=>{S.jpegQuality=+e.target.value;post({jpgq:S.jpegQuality})};$('crosshair').onchange=e=>{S.crosshair=e.target.checked?1:0;sceneDirty=true;post({crosshair:S.crosshair})};$('recHud').onchange=e=>{recHud=e.target.checked?1:0;localStorage.recHud=recHud};$('diagBox').ontoggle=()=>{labels();if($('diagBox').open)getDiag()};$('resetAlign').onclick=()=>{S.offx100=S.offy100=S.offzx=S.offzy=0;for(let id of ['offx100','offy100','offzx','offzy'])$(id).value=0;setDistance(30);post({reset_alignment:1})};$('rotBtn').onclick=()=>{rot=(rot+1)%4;setCanvasRotation()};$('snap').onclick=()=>{try{let a=document.createElement('a');a.download='thermal-frame.png';a.href=canvas.toDataURL('image/png');a.click()}catch(e){$('saveMsg').textContent='Snapshot unavailable with this camera transport.'}};canvas.onclick=e=>{let r=canvas.getBoundingClientRect(),p=viewToScene((e.clientX-r.left)*canvas.width/r.width,(e.clientY-r.top)*canvas.height/r.height);marker=p;markerTemp=rawTempAt(p.x,p.y);sceneDirty=true;labels()};
function recType(){return['video/mp4;codecs=avc1.42E01E','video/mp4;codecs=h264','video/mp4','video/webm;codecs=vp9','video/webm;codecs=vp8','video/webm'].find(t=>window.MediaRecorder&&MediaRecorder.isTypeSupported(t))||''}function stopRec(){if(rec&&rec.state!=='inactive')rec.stop()}$('recBtn').onclick=()=>{if(rec){stopRec();return}if(!recCanvas.captureStream||!window.MediaRecorder){$('saveMsg').textContent='Recording unavailable; use screen recording.';return}chunks=[];let type=recType();recStream=recCanvas.captureStream(15);try{rec=new MediaRecorder(recStream,type?{mimeType:type}:undefined)}catch(e){recStream.getTracks().forEach(t=>t.stop());recStream=null;$('saveMsg').textContent='Recording unavailable with this browser codec.';return}rec.ondataavailable=e=>{if(e.data.size)chunks.push(e.data)};rec.onstop=()=>{clearInterval(recTimer);if(recStream){recStream.getTracks().forEach(t=>t.stop());recStream=null}let mime=rec.mimeType||type||'video/webm',ext=mime.includes('mp4')?'mp4':'webm',blob=new Blob(chunks,{type:mime});if(recUrl)URL.revokeObjectURL(recUrl);recUrl=URL.createObjectURL(blob);$('saveMsg').innerHTML=`<a href="${recUrl}" download="thermal-stream.${ext}">Download recording</a>`;$('recBtn').textContent='Record';rec=null};rec.start();recStarted=Date.now();recTimer=setInterval(()=>{$('saveMsg').textContent='Recording '+mmss(Date.now()-recStarted)},500);$('recBtn').textContent='Stop'};
$('stopPortal').onclick=async()=>{$('saveMsg').textContent='Stopping portal...';try{let r=await fetchTimeout('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({stop_stream:1})},1200);running=false;$('saveMsg').textContent=r.ok?'Stop requested.':'Stop request failed; hold button 3s.'}catch(e){$('saveMsg').textContent='Stop failed; hold button 3s.'}};
function startMjpeg(){mjpegImg.onload=()=>{mjpegActive=true;camStatus='mjpeg';sceneDirty=true;labels()};mjpegImg.onerror=()=>{mjpegActive=false;camStatus='poll fallback';getCam()};mjpegImg.src=`http://${location.hostname}:81/stream?${Date.now()}`;setTimeout(()=>{if(!mjpegActive)getCam()},2500);setInterval(()=>{if(!mjpegActive&&mjpegImg.naturalWidth){mjpegActive=true;camStatus='mjpeg';labels()}if(mjpegActive)sceneDirty=true},150)}
setCanvasRotation();getState();getThermal();startMjpeg();render();
)PORTALJS";

static constexpr size_t PORTAL_INDEX_HTML_LEN = sizeof(PORTAL_INDEX_HTML) - 1;
static constexpr size_t PORTAL_CSS_LEN = sizeof(PORTAL_CSS) - 1;
static constexpr size_t PORTAL_JS_LEN = sizeof(PORTAL_JS) - 1;

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
  freeze_zoom_x_pct = zoom_x_pct;
  freeze_zoom_y_pct = zoom_y_pct;
  freeze_parallax_x100 = parallax_x100;
  freeze_parallax_y100 = parallax_y100;
  freeze_tint_pct = tint_pct;
  freeze_range_mode = range_mode;
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

void writeBmpHeader(WiFiClient &client, int bmp_w, int bmp_h) {
  uint8_t hdr[54] = {0};
  const uint32_t row_bytes = bmp_w * 3;
  const uint32_t img_size = row_bytes * bmp_h;
  hdr[0] = 'B'; hdr[1] = 'M';
  bmpPut32(hdr, 2, 54 + img_size);
  bmpPut32(hdr, 10, 54);
  bmpPut32(hdr, 14, 40);
  bmpPut32(hdr, 18, bmp_w);
  bmpPut32(hdr, 22, bmp_h);
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
                                             int32_t sample_x0_q,
                                             int32_t sample_y0_q,
                                             int32_t sample_step_x_q,
                                             int32_t sample_step_y_q) {
  uint16_t *cam = (uint16_t*)freeze_cam_snapshot;
  return cameraPixelWireAt(cam, freeze_cam_valid, x, y,
                           sample_x0_q, sample_y0_q,
                           sample_step_x_q, sample_step_y_q);
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
  return tempToPaletteIndexForMode(temp, freeze_range_lo, freeze_range_hi,
                                   rangeUsesColdPalette(freeze_range_mode));
}

void sendFreezeBmp(uint8_t mode, const char *filename) {
  if ((mode == SHARE_BMP_CAMERA && !freeze_cam_valid) ||
      (mode != SHARE_BMP_CAMERA && !freeze_thermal_valid)) {
    share_server.send(404, "text/plain", "No frozen frame available");
    return;
  }

  share_server.sendHeader("Content-Disposition",
                          String("inline; filename=\"") + filename + "\"");
  int bmp_w = IMG_W;
  int bmp_h = IMG_H;
  size_t row_bytes = (size_t)bmp_w * 3;
  share_server.setContentLength(54 + row_bytes * bmp_h);
  share_server.send(200, "image/bmp", "");
  WiFiClient client = share_server.client();
  writeBmpHeader(client, bmp_w, bmp_h);

  int32_t sample_x0_q, sample_y0_q;
  int32_t sample_step_x_q, sample_step_y_q;
  cameraCropParamsFor(freeze_zoom_x_pct, freeze_zoom_y_pct,
                      freeze_parallax_x100, freeze_parallax_y100,
                      sample_x0_q, sample_y0_q,
                      sample_step_x_q, sample_step_y_q);

  for (int y = bmp_h - 1; y >= 0; y--) {
    for (int x = 0; x < bmp_w; x++) {
      uint8_t r = 0, g = 0, b = 0;

      if (mode == SHARE_BMP_CAMERA || mode == SHARE_BMP_OVERLAY) {
        rgb565WireToRgb888(freezeCameraPixelWire(x, y, sample_x0_q, sample_y0_q,
                                                 sample_step_x_q, sample_step_y_q),
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
    client.write(share_bmp_line, row_bytes);
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
  html += RANGE_NAMES[range_mode];
  html += F(" ");
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
    share_server.send_P(200, PSTR("text/html"), PORTAL_INDEX_HTML,
                        PORTAL_INDEX_HTML_LEN);
  } else {
    handleShareIndex();
  }
}

void handlePortalCss() {
  share_server.sendHeader("Cache-Control", "no-store");
  share_server.send_P(200, PSTR("text/css"), PORTAL_CSS, PORTAL_CSS_LEN);
}

void handlePortalJs() {
  share_server.sendHeader("Cache-Control", "no-store");
  share_server.send_P(200, PSTR("application/javascript"), PORTAL_JS, PORTAL_JS_LEN);
}

void handleApiPing() {
  String json;
  json.reserve(220);
  json += F("{\"ok\":true,\"stream\":");
  json += stream_mode ? F("true") : F("false");
  json += F(",\"uptimeMs\":"); json += millis();
  json += F(",\"heap\":"); json += ESP.getFreeHeap();
  json += F(",\"psram\":"); json += ESP.getFreePsram();
  json += F(",\"indexBytes\":"); json += PORTAL_INDEX_HTML_LEN;
  json += F(",\"cssBytes\":"); json += PORTAL_CSS_LEN;
  json += F(",\"jsBytes\":"); json += PORTAL_JS_LEN;
  json += F("}");
  share_server.sendHeader("Cache-Control", "no-store");
  share_server.send(200, "application/json", json);
}

void addStreamJpegTimingHeaders(size_t jpg_len, uint32_t encode_us) {
  share_server.sendHeader("Cache-Control", "no-store");
  share_server.sendHeader("Access-Control-Allow-Origin", "*");
  share_server.sendHeader("X-Cam-Encode-Ms", String(encode_us / 1000.0f, 1));
  share_server.sendHeader("X-Cam-Send-Ms", String(stream_cam_send_ms, 1));
  share_server.sendHeader("X-Cam-Total-Ms", String(stream_cam_total_ms, 1));
  share_server.sendHeader("X-Cam-Bytes", String((unsigned)jpg_len));
  share_server.sendHeader("X-Cam-Precropped", stream_cam_pre_cropped ? "1" : "0");
  share_server.sendHeader("X-Cam-Width", String((unsigned)stream_cam_encode_w));
  share_server.sendHeader("X-Cam-Height", String((unsigned)stream_cam_encode_h));
  share_server.sendHeader("X-Cam-Crop-Sx", String(stream_cam_crop_sx, 3));
  share_server.sendHeader("X-Cam-Crop-Sy", String(stream_cam_crop_sy, 3));
  share_server.sendHeader("X-Cam-Crop-Sw", String(stream_cam_crop_sw, 3));
  share_server.sendHeader("X-Cam-Crop-Sh", String(stream_cam_crop_sh, 3));
  share_server.sendHeader("X-Cam-Seq", String((unsigned long)thermal_frame_seq));
  share_server.sendHeader("X-JPEG-Quality", String((int)stream_jpeg_quality));
  share_server.sendHeader("X-Cam-Cache-Age-Ms", String((unsigned long)(millis() - stream_cam_cache_ms)));
}

void handleStreamCameraJpg() {
  if (!stream_mode) {
    share_server.send(404, "text/plain", "Stream portal is not active");
    return;
  }

  noteStreamRequest(stream_cam_req_count, stream_cam_req_timer_ms,
                    stream_cam_req_fps);
  uint32_t total_start_us = micros();

  if (!stream_cam_cache_valid || !stream_cam_jpeg) {
    updateStreamCameraCache(true);
  }
  if (!stream_cam_cache_valid || !stream_cam_jpeg || !stream_cam_jpeg_len) {
    share_server.send(404, "text/plain", "No camera frame available");
    return;
  }

  uint32_t send_start_us = micros();
  addStreamJpegTimingHeaders(stream_cam_jpeg_len, stream_cam_last_encode_ms * 1000U);
  share_server.sendHeader("X-Cam-Transport", "rgb565-cache");
  share_server.setContentLength(stream_cam_jpeg_len);
  share_server.send(200, "image/jpeg", "");
  WiFiClient client = share_server.client();
  client.write(stream_cam_jpeg, stream_cam_jpeg_len);
  uint32_t send_us = micros() - send_start_us;
  noteStreamCamTiming(stream_cam_last_encode_ms * 1000U, send_us,
                      micros() - total_start_us, stream_cam_jpeg_len);
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
  share_server.sendHeader("Access-Control-Allow-Origin", "*");
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

static inline int argPx100Clamped(const char *px100_name, const char *px_name,
                                  int current) {
  int v = current;
  if (share_server.hasArg(px100_name)) {
    v = share_server.arg(px100_name).toInt();
  } else if (share_server.hasArg(px_name)) {
    float f = share_server.arg(px_name).toFloat();
    v = (int)(f >= 0.0f ? f * 100.0f + 0.5f : f * 100.0f - 0.5f);
  }
  return clampParallax100(v);
}

static inline int argZoomOffsetClamped(const char *name, int current) {
  if (!share_server.hasArg(name)) return current;
  return clampZoomOffsetPct(share_server.arg(name).toInt());
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
  if (share_server.hasArg("preset_cm")) {
    int cm = share_server.arg("preset_cm").toInt();
    if (applyAlignmentPresetCm(cm, false)) dirty = true;
  }
  bool has_distance_arg = share_server.hasArg("dist") || share_server.hasArg("distance_cm");
  if (has_distance_arg) {
    int cm = share_server.hasArg("dist") ? share_server.arg("dist").toInt()
                                         : share_server.arg("distance_cm").toInt();
    applyAlignmentDistanceCm(cm, false);
    dirty = true;
  }
  int offx100 = align_offset_x100;
  int offy100 = align_offset_y100;
  int offzx = align_zoom_x_offset;
  int offzy = align_zoom_y_offset;
  if (!has_distance_arg) {
    if (share_server.hasArg("offx100")) offx100 = argPx100Clamped("offx100", "offx", offx100);
    else if (share_server.hasArg("px100") || share_server.hasArg("px")) {
      offx100 = argPx100Clamped("px100", "px", parallax_x100) -
                baseParallaxX100ForDistance(align_distance_cm);
    }
    if (share_server.hasArg("offy100")) offy100 = argPx100Clamped("offy100", "offy", offy100);
    else if (share_server.hasArg("py100") || share_server.hasArg("py")) {
      offy100 = argPx100Clamped("py100", "py", parallax_y100) - baseParallaxY100();
    }
    if (share_server.hasArg("offzx")) offzx = argZoomOffsetClamped("offzx", offzx);
    else if (share_server.hasArg("zx")) offzx = argIntClamped("zx", zoom_x_pct, ZOOM_MIN, ZOOM_MAX) - baseZoomXPct();
    if (share_server.hasArg("offzy")) offzy = argZoomOffsetClamped("offzy", offzy);
    else if (share_server.hasArg("zy") || share_server.hasArg("zoom")) {
      int zarg = share_server.hasArg("zy") ? argIntClamped("zy", zoom_y_pct, ZOOM_MIN, ZOOM_MAX)
                                           : argIntClamped("zoom", zoom_y_pct, ZOOM_MIN, ZOOM_MAX);
      offzy = zarg - baseZoomYPct();
    }
  }
  int ti = argIntClamped("tint", tint_pct, 0, 100);
  int br = argIntClamped("brt", cam_brightness, -2, 2);
  int con = argIntClamped("con", cam_contrast, -2, 2);
  int sat = argIntClamped("sat", cam_saturation, -2, 2);
  int shp = argIntClamped("shp", cam_sharpness, -2, 2);
  int den = argIntClamped("den", cam_denoise, 0, 1);
  int jq = argIntClamped("jpgq", stream_jpeg_quality,
                         STREAM_JPEG_QUALITY_MIN, STREAM_JPEG_QUALITY_MAX);
  if (offx100 != align_offset_x100 || offy100 != align_offset_y100 ||
      offzx != align_zoom_x_offset || offzy != align_zoom_y_offset) {
    setAlignmentOffsets(offx100, offy100, offzx, offzy, false);
    dirty = true;
  }
  if (ti != tint_pct)   { tint_pct = (uint8_t)ti; dirty = true; }
  if (jq != stream_jpeg_quality) {
    stream_jpeg_quality = (uint8_t)jq;
    if (stream_native_jpeg_active) applyCameraJpegQuality();
    dirty = true;
  }
  if (br != cam_brightness) {
    cam_brightness = (int8_t)br;
    brightness_apply_pending = true;
    dirty = true;
  }
  if (con != cam_contrast) { cam_contrast = (int8_t)con; brightness_apply_pending = true; dirty = true; }
  if (sat != cam_saturation) { cam_saturation = (int8_t)sat; brightness_apply_pending = true; dirty = true; }
  if (shp != cam_sharpness) { cam_sharpness = (int8_t)shp; brightness_apply_pending = true; dirty = true; }
  if (den != cam_denoise) { cam_denoise = (int8_t)den; brightness_apply_pending = true; dirty = true; }
  if (share_server.hasArg("lenc")) {
    bool v = share_server.arg("lenc").toInt() != 0;
    if (v != cam_lenc) { cam_lenc = v; brightness_apply_pending = true; dirty = true; }
  }
  if (share_server.hasArg("gma")) {
    bool v = share_server.arg("gma").toInt() != 0;
    if (v != cam_raw_gma) { cam_raw_gma = v; brightness_apply_pending = true; dirty = true; }
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
    resetAlignmentCalibrated(false);
    dirty = true;
  }
  if (share_server.hasArg("stop_stream")) {
    stream_stop_pending = true;
    stream_stop_deadline_ms = millis() + STREAM_STOP_DELAY_MS;
    share_server.sendHeader("Cache-Control", "no-store");
    share_server.sendHeader("Connection", "close");
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
  json.reserve(1500);
  json += F("{\"stream\":");
  json += stream_mode ? F("true") : F("false");
  json += F(",\"ssid\":\""); json += share_ap_ssid; json += F("\"");
  json += F(",\"ip\":\""); json += WiFi.softAPIP().toString(); json += F("\"");
  json += F(",\"view\":"); json += stream_view_mode;
  json += F(",\"range\":"); json += (int)range_mode;
  json += F(",\"rangeName\":\""); json += RANGE_NAMES[range_mode]; json += F("\"");
  json += F(",\"px\":"); json += String(parallax_x100 / 100.0f, 2);
  json += F(",\"py\":"); json += String(parallax_y100 / 100.0f, 2);
  json += F(",\"px100\":"); json += parallax_x100;
  json += F(",\"py100\":"); json += parallax_y100;
  json += F(",\"offx100\":"); json += align_offset_x100;
  json += F(",\"offy100\":"); json += align_offset_y100;
  json += F(",\"offzx\":"); json += align_zoom_x_offset;
  json += F(",\"offzy\":"); json += align_zoom_y_offset;
  json += F(",\"zoom\":"); json += zoom_y_pct;
  json += F(",\"zx\":"); json += zoom_x_pct;
  json += F(",\"zy\":"); json += zoom_y_pct;
  json += F(",\"basePx100\":"); json += baseParallaxX100ForDistance(align_distance_cm);
  json += F(",\"basePy100\":"); json += baseParallaxY100();
  json += F(",\"baseZx\":"); json += baseZoomXPct();
  json += F(",\"baseZy\":"); json += baseZoomYPct();
  json += F(",\"alignPresetCm\":"); json += align_distance_cm;
  json += F(",\"alignDistanceCm\":"); json += align_distance_cm;
  json += F(",\"tint\":"); json += tint_pct;
  json += F(",\"mlo\":"); json += String(manual_lo, 1);
  json += F(",\"mhi\":"); json += String(manual_hi, 1);
  json += F(",\"brt\":"); json += (int)cam_brightness;
  json += F(",\"con\":"); json += (int)cam_contrast;
  json += F(",\"sat\":"); json += (int)cam_saturation;
  json += F(",\"shp\":"); json += (int)cam_sharpness;
  json += F(",\"den\":"); json += (int)cam_denoise;
  json += F(",\"lenc\":"); json += cam_lenc ? 1 : 0;
  json += F(",\"gma\":"); json += cam_raw_gma ? 1 : 0;
  json += F(",\"hud\":"); json += stream_hud_enabled ? 1 : 0;
  json += F(",\"crosshair\":"); json += stream_crosshair_enabled ? 1 : 0;
  json += F(",\"rangeLo\":"); json += String(lo, 2);
  json += F(",\"rangeHi\":"); json += String(hi, 2);
  json += F(",\"tCenter\":"); json += String(center, 2);
  json += F(",\"tMin\":"); json += String(mn, 2);
  json += F(",\"tMax\":"); json += String(mx, 2);
  json += F(",\"seq\":"); json += seq;
  json += F(",\"camFps\":"); json += String(cam_fps, 1);
  json += F(",\"mlxFps\":"); json += String(mlx_fps, 1);
  json += F(",\"loopFps\":"); json += String(current_fps, 1);
  json += F(",\"portalMs\":"); json += stream_started_ms ? (now - stream_started_ms) : 0;
  json += F(",\"camTransport\":\"");
  json += F("rgb565-cache\"");
  json += F(",\"jpegQuality\":"); json += (int)stream_jpeg_quality;
  json += F(",\"camPreCropped\":"); json += stream_cam_pre_cropped ? 1 : 0;
  json += F(",\"camStreamW\":"); json += stream_cam_encode_w;
  json += F(",\"camStreamH\":"); json += stream_cam_encode_h;
  json += F(",\"camCropSx\":"); json += String(stream_cam_crop_sx, 3);
  json += F(",\"camCropSy\":"); json += String(stream_cam_crop_sy, 3);
  json += F(",\"camCropSw\":"); json += String(stream_cam_crop_sw, 3);
  json += F(",\"camCropSh\":"); json += String(stream_cam_crop_sh, 3);
  json += F(",\"camCacheFps\":"); json += String(stream_cam_cache_fps, 1);
  json += F(",\"camCacheAgeMs\":"); json += stream_cam_cache_ms ? (now - stream_cam_cache_ms) : 0;
  json += F(",\"mjpegPort\":81");
  json += F(",\"portalStage\":\""); json += stream_portal_stage; json += F("\"");
  json += F(",\"camOk\":"); json += cam_ok ? 1 : 0;
  json += F(",\"camHaveFrame\":"); json += cam_have_frame ? 1 : 0;
  json += F("}");
  share_server.sendHeader("Cache-Control", "no-store");
  share_server.send(200, "application/json", json);
}

void handleStreamDiagnostics() {
  uint32_t now = millis();
  noteStreamRequest(stream_api_count, stream_api_timer_ms, stream_api_fps);
  updateCameraMemoryDiagnostics();

  String json;
  json.reserve(1800);
  json += F("{\"camReqFps\":"); json += String(stream_cam_req_fps, 1);
  json += F(",\"thermalReqFps\":"); json += String(stream_thermal_req_fps, 1);
  json += F(",\"apiFps\":"); json += String(stream_api_fps, 1);
  json += F(",\"grabMs\":"); json += String(diag_grab_ms, 1);
  json += F(",\"renderMs\":"); json += String(diag_render_ms, 1);
  json += F(",\"uiMs\":"); json += String(diag_ui_ms, 1);
  json += F(",\"portalMs\":"); json += stream_started_ms ? (now - stream_started_ms) : 0;
  json += F(",\"heap\":"); json += ESP.getFreeHeap();
  json += F(",\"psram\":"); json += ESP.getFreePsram();
  json += F(",\"indexBytes\":"); json += PORTAL_INDEX_HTML_LEN;
  json += F(",\"cssBytes\":"); json += PORTAL_CSS_LEN;
  json += F(",\"jsBytes\":"); json += PORTAL_JS_LEN;
  json += F(",\"camTransport\":\"");
  json += F("rgb565-cache\"");
  json += F(",\"jpegQuality\":"); json += (int)stream_jpeg_quality;
  json += F(",\"camPreCropped\":"); json += stream_cam_pre_cropped ? 1 : 0;
  json += F(",\"camStreamW\":"); json += stream_cam_encode_w;
  json += F(",\"camStreamH\":"); json += stream_cam_encode_h;
  json += F(",\"camCropSx\":"); json += String(stream_cam_crop_sx, 3);
  json += F(",\"camCropSy\":"); json += String(stream_cam_crop_sy, 3);
  json += F(",\"camCropSw\":"); json += String(stream_cam_crop_sw, 3);
  json += F(",\"camCropSh\":"); json += String(stream_cam_crop_sh, 3);
  json += F(",\"camEncodeMs\":"); json += String(stream_cam_encode_ms, 1);
  json += F(",\"camSendMs\":"); json += String(stream_cam_send_ms, 1);
  json += F(",\"camTotalMs\":"); json += String(stream_cam_total_ms, 1);
  json += F(",\"camJpegBytes\":"); json += (unsigned long)(stream_cam_jpeg_bytes + 0.5f);
  json += F(",\"camLastEncodeMs\":"); json += (unsigned long)stream_cam_last_encode_ms;
  json += F(",\"camLastSendMs\":"); json += (unsigned long)stream_cam_last_send_ms;
  json += F(",\"camLastTotalMs\":"); json += (unsigned long)stream_cam_last_total_ms;
  json += F(",\"camLastJpegBytes\":"); json += (unsigned long)stream_cam_last_jpeg_bytes;
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
  json += F(",\"camRgbFbCount\":"); json += CAMERA_RGB565_FB_COUNT;
  json += F(",\"camCacheFps\":"); json += String(stream_cam_cache_fps, 1);
  json += F(",\"camCacheAgeMs\":"); json += stream_cam_cache_ms ? (now - stream_cam_cache_ms) : 0;
  json += F(",\"camCacheSeq\":"); json += stream_cam_cache_seq;
  json += F(",\"mjpegRunning\":"); json += stream_mjpeg_running ? 1 : 0;
  json += F(",\"mjpegClients\":"); json += stream_mjpeg_clients;
  json += F(",\"portalStage\":\""); json += stream_portal_stage; json += F("\"");
  json += F(",\"resetReason\":"); json += (int)esp_reset_reason();
  json += F(",\"arduinoEsp32Version\":"); json += ESP_ARDUINO_VERSION;
  json += F(",\"safeReconfigure\":"); json += CAMERA_HAS_SAFE_RECONFIGURE ? 1 : 0;
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
  share_server.on("/portal.css", HTTP_GET, handlePortalCss);
  share_server.on("/portal.js", HTTP_GET, handlePortalJs);
  share_server.on("/api/ping", HTTP_GET, handleApiPing);
  share_server.on("/camera.bmp", [](){ sendFreezeBmp(SHARE_BMP_CAMERA, "camera.bmp"); });
  share_server.on("/thermal.bmp", [](){ sendFreezeBmp(SHARE_BMP_THERMAL, "thermal.bmp"); });
  share_server.on("/overlay.bmp", [](){ sendFreezeBmp(SHARE_BMP_OVERLAY, "overlay.bmp"); });
  share_server.on("/thermal.csv", handleThermalCsvRoute);
  share_server.on("/cam.jpg", HTTP_GET, handleStreamCameraJpg);
  share_server.on("/thermal.bin", HTTP_GET, handleStreamThermalBin);
  share_server.on("/api/state", HTTP_GET, handleStreamState);
  share_server.on("/api/diag", HTTP_GET, handleStreamDiagnostics);
  share_server.on("/api/control", HTTP_POST, handleStreamControl);
  share_server.on("/generate_204", [](){
    share_server.sendHeader("Location", "/", true);
    share_server.send(302, "text/plain", "");
  });
  share_server.on("/hotspot-detect.html", [](){
    share_server.sendHeader("Location", "/", true);
    share_server.send(302, "text/plain", "");
  });
  share_server.on("/connecttest.txt", [](){
    share_server.sendHeader("Location", "/", true);
    share_server.send(302, "text/plain", "");
  });
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
  int y = 206;
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
    stream_portal_stage = "enter";
    if (freeze_mode) {
      stopFreezeShareAP();
      releaseFreezeShareSnapshot();
      freeze_mode = false;
    }

    stream_mode = true;
    stream_native_jpeg_active = false;
    stream_stop_pending = false;
    stream_stop_deadline_ms = 0;
    stream_view_mode = STREAM_VIEW_OVERLAY;
    stream_started_ms = millis();
    resetStreamDiagnostics(stream_started_ms);
    stream_portal_stage = "ap";
    releaseStreamCameraCache();
    stream_next_cam_capture_ms = 0;
    if (!startFreezeShareAP()) {
      stopFreezeShareAP();
      stream_mode = false;
      stream_started_ms = 0;
      stream_portal_stage = "ap_fail";
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
    stream_portal_stage = "cache";
    updateStreamCameraCache(true);
    startStreamMjpegServer();
    stream_portal_stage = "online";
    drawStreamStatusScreen();
  } else {
    stream_portal_stage = "exit";
    stopStreamMjpegServer();
    stopFreezeShareAP();
    exitStreamCameraMode();
    stream_mode = false;
    stream_stop_pending = false;
    stream_stop_deadline_ms = 0;
    stream_started_ms = 0;
    releaseStreamCameraCache();
    stream_portal_stage = "idle";
    applyLcdBrightness();
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

static inline void noteLoopTiming(float &dst, uint32_t elapsed_us) {
  float ms = elapsed_us / 1000.0f;
  if (dst <= 0.0f) dst = ms;
  else dst += (ms - dst) * 0.20f;
}

void drawStatusBadge(int x, int y, int w, int h,
                     const char *text, uint16_t bg, uint16_t fg) {
  lcd.fillRoundRect(x, y, w, h, 3, bg);
  lcd.drawRoundRect(x, y, w, h, 3, TFT_DARKGREY);
  lcd.setTextSize(1);
  lcd.setTextColor(fg, bg);
  int tx = x + (w - (int)strlen(text) * 6) / 2;
  if (tx < x + 2) tx = x + 2;
  lcd.setCursor(tx, y + (h - 8) / 2);
  lcd.print(text);
}

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
  lcd.drawFastVLine(PANEL_X - 3, 2, 316, TFT_DARKGREY);
  drawButtons(); drawAdjustSliders();
}

void drawDynamicUI() {
  lcd.setTextColor(TFT_WHITE, TFT_BLACK); lcd.setTextSize(1);
  char buf[40];

  const char *event_text = nullptr;
  uint16_t event_bg = TFT_BLACK;
  if (share_ap_running) {
    event_text = "AP"; event_bg = TFT_RED;
  } else if (freeze_mode) {
    event_text = "FRZ"; event_bg = TFT_RED;
  } else if (millis() < save_indicator_until_ms) {
    event_text = "SAV"; event_bg = TFT_DARKGREEN;
  }
  drawStatusBadge(STATUS_BADGE_X, STATUS_MODE_Y, STATUS_BADGE_W, STATUS_BADGE_H,
                  MODE_NAMES[display_mode], MODE_BG[display_mode],
                  display_mode == MODE_TINT ? TFT_BLACK : TFT_WHITE);
  drawStatusBadge(STATUS_BADGE_X, STATUS_RANGE_Y, STATUS_BADGE_W, STATUS_BADGE_H,
                  RANGE_NAMES[range_mode], TFT_PURPLE, TFT_WHITE);
  lcd.fillRect(STATUS_EVENT_X, STATUS_EVENT_Y, STATUS_EVENT_W, STATUS_EVENT_H, TFT_BLACK);
  if (event_text) {
    drawStatusBadge(STATUS_EVENT_X, STATUS_EVENT_Y, STATUS_EVENT_W, STATUS_EVENT_H,
                    event_text, event_bg, TFT_WHITE);
  }

  // Centre temperature, or selected probe temperature once the image is tapped.
  bool marker_readout = screen_marker_active && !isnan(screen_marker_temp);
  const uint16_t temp_bg = 0x0841;
  lcd.fillRect(TEMP_CLEAR_X, TEMP_CLEAR_Y, TEMP_CLEAR_W, TEMP_CLEAR_H, TFT_BLACK);
  lcd.fillRoundRect(TEMP_CLEAR_X + 4, TEMP_CLEAR_Y + 1,
                    TEMP_CLEAR_W - 8, TEMP_CLEAR_H - 3, 6, temp_bg);
  lcd.setTextSize(3);
  if (marker_readout) {
    snprintf(buf, sizeof(buf), "M%.1fC", screen_marker_temp);
  } else {
    snprintf(buf, sizeof(buf), "%.1fC", t_center);
  }
  int tw = (int)strlen(buf) * 18;
  int tx = IMG_X + (IMG_W - tw) / 2;
  if (tx < TEMP_CLEAR_X) tx = TEMP_CLEAR_X;
  lcd.setTextColor(marker_readout ? TFT_YELLOW : TFT_WHITE, temp_bg);
  lcd.setCursor(tx, 5);
  lcd.print(buf);

  lcd.fillRect(FPS_X, FPS_Y, FPS_W, FPS_H, TFT_BLACK);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  snprintf(buf, sizeof(buf), "FPS %.1f", current_fps);
  lcd.setCursor(FPS_X, FPS_Y);
  lcd.print(buf);
}

// --------------------------------------------------------------- Storage --
// SD storage is disabled; freeze/WiFi export is the save path.
// ---------------------------------------------------------------- Setup --
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== FLIR v16 ===");
  Serial.printf("Clocks: CPU=%uMHz LCD_SPI=%luHz CAM_XCLK=16MHz RGB_FB=%u\n",
                getCpuFrequencyMhz(),
                (unsigned long)TFT_WRITE_FREQ_HZ,
                (unsigned)CAMERA_RGB565_FB_COUNT);
  
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
    Serial.println("CAMERA: init failed; continuing without camera. "
                   "TINT/CAM modes will fall back to thermal-only.");
  }

  Serial.println("[4/8] GT911 manual reset");
  gt911_init_manual();

  Serial.println("[5/8] LCD");
  lcd.init();
  lcd.setRotation(LCD_ROTATION);
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
  // Wait for one complete MLX frame before first render.
  {
    uint32_t mlx_t0 = millis();
    while (millis() - mlx_t0 < 2000) {
      if (readMLXSubpage() == MLX_READ_FULL) break;
      delay(1);
    }
    analyzeMLX();
  }
  last_mlx_ms = millis();

  Serial.println("[7/8] Palette + UI");
  buildPalette();
  loadSettings();
  applyLcdBrightness();
  layoutUi();
  if (brightness_apply_pending) { applyCameraBrightness(); brightness_apply_pending = false; }
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
    stream_portal_transition_pending = true;
  } else if (ev == BTN_SHORT && !stream_mode) {
    setFreezeMode(!freeze_mode);
  }
  if (stream_portal_transition_pending) {
    stream_portal_transition_pending = false;
    setStreamMode(!stream_mode);
  }
  handleFreezeShareAP();
  handleStreamMjpegServer();
  if (stream_stop_pending && stream_stop_deadline_ms &&
      (int32_t)(millis() - stream_stop_deadline_ms) >= 0) {
    setStreamMode(false);
  }

  if (!stream_mode) handleTouch();

  uint32_t now = millis();

  // Push a pending brightness change into the sensor. One SCCB write per
  // change, not per frame; touchscreen handler sets the flag, we apply it
  // at the next loop iteration.
  if (brightness_apply_pending) {
    applyCameraBrightness();
    brightness_apply_pending = false;
  }

  // MLX normally runs on its own core so foreground rendering doesn't pause
  // for I2C reads and MLX90640_CalculateTo(). This fallback only runs if the
  // task could not be created.
  if (!mlx_task_handle && !freeze_mode && !shouldThrottleMLXForScreenCameraOnly() &&
      now - last_mlx_ms >= MLX_POLL_MS) {
    last_mlx_ms = now;
    if (readMLXSubpage() == MLX_READ_FULL) analyzeMLX();
  }

  if (!freeze_mode && stream_mode) {
    updateStreamCameraCache(false);
  } else if (!freeze_mode) {
    uint32_t grab_t0 = micros();
    if (grabCamera()) cam_count++;
    noteLoopTiming(diag_grab_ms, micros() - grab_t0);
  }

  if (!stream_mode) updateScreenMarkerTemp();

  uint32_t render_t0 = micros();
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
  noteLoopTiming(diag_render_ms, micros() - render_t0);

  if (!stream_mode && now - last_ui_ms >= 200) {
    uint32_t ui_t0 = micros();
    drawDynamicUI();
    noteLoopTiming(diag_ui_ms, micros() - ui_t0);
    last_ui_ms = now;
  }

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
    Serial.printf("Loop=%.1f Cam=%.1f MLX=%.1f Grab=%.1fms Render=%.1fms UI=%.1fms Ctr=%.1fC [%.1f,%.1f] Mode=%s Rng=%s Tint=%d%% "
                  "ZX/Y=%d/%d%% PX100=%+d,%+d Dist=%dcm Btn=%d Frz=%d\n",
                  current_fps, cam_fps, mlx_fps,
                  diag_grab_ms, diag_render_ms, diag_ui_ms,
                  t_center, t_min, t_max,
                  MODE_NAMES[display_mode], RANGE_NAMES[range_mode], tint_pct,
                  zoom_x_pct, zoom_y_pct, parallax_x100, parallax_y100,
                  align_distance_cm,
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
