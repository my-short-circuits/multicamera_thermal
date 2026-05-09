# Multicamera Thermal

A handheld DIY thermal-imaging camera. Point it at something and a 3.5-inch touchscreen shows you where the heat is — overlaid on a regular camera view, or on its own as a pure heat map. Tap to tune the colour range, slide to line up the two cameras, press the side button to freeze a frame and pull it off the device over WiFi from any phone or laptop.

---

## Hardware

The sketch is configured for this hardware stack:

| Part | Role | Notes |
| --- | --- | --- |
| DFRobot FireBeetle 2 ESP32-S3-U (DFR0975-U, N16R8) | Main controller | ESP32-S3 with 16 MB flash and 8 MB OPI PSRAM. PSRAM is required for the camera frame ring and the freeze snapshot. |
| DFRobot Fermion 3.5-inch display (DFR0669 class) | LCD and touch UI | 480x320 ILI9488 SPI display with GT911 capacitive touch. Bundled GDI ribbon. |
| OV3660 (tested) or OV2640 (auto-detected via PID) | Visible camera | Captured as 320x240 RGB565 into PSRAM. The OV3660 is mounted with `vflip=1` because it is upside-down relative to the OV2640 it replaced; sensor type is detected at runtime in `initCamera()`. |
| Melexis MLX90640 (32x24, ~55x35 deg FOV) | Thermal sensor | Standard FOV variant assumed. Runs on a dedicated `Wire1` bus at 1 MHz. |
| Momentary pushbutton | Freeze / share control | Normally-open between GPIO10 and GND, using `INPUT_PULLUP`. Short press toggles freeze + WiFi share. |
| Optional SD card | Currently disabled | Historical SD bring-up code is retained but commented out — the freeze/WiFi server is the active save path. See "SD card status" below. |

## Pinout

The pinout is hard-coded in `multicamera_thermal.ino` and verified against the GDI ribbon pin map. Avoid GPIO12 entirely: it is the touch chip-select line on the GDI ribbon and changing it corrupts the touch controller.

### LCD and touch (shared SPI + I2C bus)

| Signal | GPIO |
| --- | ---: |
| TFT SCLK | 17 |
| TFT MOSI | 15 |
| TFT MISO | 16 |
| TFT DC | 3 |
| TFT RST | 38 |
| TFT CS | 18 |
| TFT backlight (PWM) | 21 |
| GT911 SDA | 1 |
| GT911 SCL | 2 |
| GT911 INT | 13 |

The LCD SPI bus runs at 50 MHz write / 16 MHz read. The published ILI9488 max is 20 MHz; 40 MHz is the broadly stable rate; 50 MHz worked on this panel without speckling. If you see random-coloured single-row glitches, drop `TFT_WRITE_FREQ_HZ` back toward 40 MHz.

### Visible camera (DVP parallel + SCCB)

| Signal | GPIO |
| --- | ---: |
| XCLK | 45 |
| SIOD (SCCB SDA) | 1 |
| SIOC (SCCB SCL) | 2 |
| D0 | 39 |
| D1 | 40 |
| D2 | 41 |
| D3 | 4 |
| D4 | 7 |
| D5 | 8 |
| D6 | 46 |
| D7 | 48 |
| VSYNC | 6 |
| HREF | 42 |
| PCLK | 5 |

The camera SCCB bus shares GPIO1/GPIO2 with GT911 and AXP313A on the main I2C bus (`Wire`, 100 kHz). `esp_camera_init()` is configured with `pin_sccb_*=-1` and `sccb_i2c_port=0` so it borrows the existing I2C driver — this avoids the duplicate-init problem documented in [esp32-camera issue #741](https://github.com/espressif/esp32-camera/issues/741).

XCLK is intentionally 16 MHz rather than 24 MHz: it gives the camera enough headroom on this MCU/PSRAM combination to keep `fb_count = 3` filled with `CAMERA_GRAB_LATEST` and avoid EV-VSYNC-OVF stalls.

### Thermal sensor and button

| Signal | GPIO |
| --- | ---: |
| MLX90640 SDA | 11 |
| MLX90640 SCL | 14 |
| Button | 10 |

The MLX90640 runs on `Wire1` at 1 MHz so its high-bandwidth bursts never stall the shared `Wire` bus. The included `MLX90640_I2C_Driver.cpp` is patched: every `Wire.` reference has been changed to `Wire1.`.


## Firmware Features

### Display modes

- `TINT`: visible camera with additive thermal palette overlay.
- `THERM`: thermal-only image.
- `CAM`: camera-only image.

The thermal grid stays mapped over the same 320x240 region in all modes. Camera zoom and parallax adjust the visible camera crop only — the thermal image does not move when you align the camera. This makes alignment a one-shot calibration: lock thermal in place, slide the camera until they overlap.

### Range modes

The side-panel range button cycles through the following presets. The `LO` / `HI` columns show the fixed palette mapping in degrees Celsius:

| Mode | Lo | Hi | Behaviour |
| --- | ---: | ---: | --- |
| `AUTO` | dynamic | dynamic | Auto range with 5%/5% percentile trim, 2.5 deg C minimum span, asymmetric IIR (fast expand / slow shrink). |
| `AUT2` | dynamic | dynamic | Subject-aware auto. Estimates ambient as the median, throws away pixels within 1.5 deg C of ambient, and uses only the leftover "interesting" pixels' min/max. Falls back to plain AUTO if there are too few subject pixels (uniform scene). |
| `SKIN` | 30.5 | 35.5 | Tight, centred on exposed-skin temperatures. Highest contrast for fingers/face. |
| `BODY` | 28.0 | 37.0 | Wider, covers cold extremities through warm core. Use for a clothed person. |
| `WARM` | 30.0 | 50.0 | Mugs, recently-used electronics, sun-warmed surfaces. |
| `HOT` | 40.0 | 90.0 | Stovetops, irons, soldering tips. |
| `VHOT` | 60.0 | 140.0 | Flame, exhausts. |
| `MAN` | settable | settable | Manual `LO`/`HI` from the RANGE tab sliders, range-clamped to 5..60 deg C with a minimum 1.0 deg C span. |

Selecting `MAN` automatically switches the side panel to the RANGE tab so the manual sliders are visible.

### Touch controls

The GT911 UI provides:

- **Top of side panel**: tab row to switch between `POS` (alignment) and `RANGE` (manual range) views.
- **POS tab**: `X` and `Y` parallax sliders, `ZOOM` slider, `TINT` slider, `ADJ` lock and `RST` (alignment reset). RST and the alignment sliders are dimmed when ADJ is locked, to prevent accidental touches.
- **RANGE tab**: `LO` and `HI` sliders. Each has `-` and `+` nudge buttons that move by 0.1 deg C while preserving a minimum 1.0 deg C span.
- **Top bar**: mode and range cycle buttons.
- **Bottom bar**: 5-step camera brightness control (-2..+2). Brightness is applied via the OV2640/OV3660 hardware register over SCCB, so it costs zero CPU per pixel — one I2C write when the value changes and the sensor's analogue front-end does the rest.

Settings persist in NVS via `Preferences` after a debounce period. Keys include `mlo` / `mhi` (manual range), `ptab` (last side-panel tab), `mode`, `rng`, alignment, etc.

### Thermal processing pipeline

 In order:

1. **MLX hardware**: chess-pattern subpages, 19-bit ADC resolution, refresh-rate code 0x05 (16 Hz subpage rate, ~8 complete frames/s).
2. **Fast subpage read** (`readMLXFrameDataFast`): bypasses the Melexis retry-on-new-subpage-arrival and accepts the read as-is. The retry is what keeps the stock library coherent at low refresh, but at higher rates it can burn most of the frame budget chasing newer data.
3. **Subpage publish gate**: a complete frame is only published once *both* chess subpages have been collected since the last publish (`mlx_subpage_mask`). Render code reads from `mlx_temps_full`, a separate buffer that is only updated at this gate. This is what keeps the chess interleave invisible at the data level.
4. **Bad/dead pixel repair**: two unit-specific cells get neighbour-averaged, then a generic L→R sweep replaces any out-of-range or NaN cell with its previous valid neighbour.
5. **Adaptive temporal IIR**: per-cell IIR with a low alpha for small frame-to-frame deltas (smoothing) and a high alpha for large deltas (motion catches up immediately). Default alphas: 0.45 still, 0.85 fast, 0.40 deg C delta crossover.
6. **Edge-aware spatial pass**: 4-neighbour weighted average that only includes neighbours within `MLX_SPATIAL_EDGE_C` of the centre cell. Default `0.0` disables it; raise toward 0.20 if cell-block noise becomes objectionable.
7. **Auto-range computation**: percentile-clipped (5/5%) min/max with the optional AUT2 subject filter applied on top, plus an asymmetric IIR that expands fast (so a hand entering frame is captured immediately) and shrinks slowly (so a hand briefly leaving frame doesn't crush the palette).
8. **Palette-frame snapshot**: per-cell temperatures map to palette indices in a separate `therm_idx[]` byte array. Bilinear interpolation is then done over these byte indices, not over RGB triples — three bilinear lookups per pixel become one.
9. **Render**: per-pixel byte-pre-swapped RGB565 written into a 16-row chunk buffer, then pushed to the LCD with `lcd.setSwapBytes(false)` so LovyanGFX uses the DMA fast path with no internal byte-swap copy.

### Freeze and WiFi export

Pressing the physical button toggles freeze mode.

When the frame is frozen:

- Camera and thermal acquisition stop updating the displayed frame.
- A WiFi access point starts. SSID format: `ThermalCam-XXXX` where XXXX is derived from the MAC.
- A captive DNS plus an HTTP server runs at `http://192.168.4.1/`.
- The landing page shows the frozen overlay and offers download links:
  - `overlay.bmp` — the rendered TINT-mode overlay.
  - `thermal.bmp` — thermal palette image without camera.
  - `camera.bmp` — visible camera frame.
  - `thermal.csv` — 32x24 raw thermal grid in degrees Celsius.

When the frame is unfrozen:

- HTTP server stops.
- Captive DNS stops.
- Soft-AP disconnects.
- WiFi peripheral is turned off.
- The frozen-frame PSRAM buffer is freed.

The AP is intentionally temporary and open. Use it only while saving a frame.

### SD card status

The repository contains historical SD-card bring-up and BMP-writing code (bit-banged CMD0 idle, two-phase mount via `esp_vfs_fat_sdspi_mount`, etc.), but the SD path is disabled in `setup()`. I couldn't get it to work

## Build Requirements

make sure you get this: DFRobot_AXP313A 1.0.1 and all the other libraries listed.

The Melexis MLX90640 API sources needed by the sketch are included in this repository:

- `MLX90640_API.cpp`, `MLX90640_API.h`
- `MLX90640_I2C_Driver.cpp` (patched for `Wire1`), `MLX90640_I2C_Driver.h`

### Arduino board settings

| Setting | Value |
| --- | --- |
| Board | DFRobot FireBeetle 2 ESP32-S3 |
| USB Mode | Hardware CDC and JTAG |
| USB CDC On Boot | Enabled |
| CPU Frequency | 240 MHz (WiFi) |
| Flash Size | 16 MB |
| Partition Scheme | 16M Flash (3MB APP / 9.9MB FATFS) |
| PSRAM | OPI PSRAM |



## Notes and Limitations


- Manual alignment depends on the camera lens, MLX90640 variant, and physical mounting. Reset is intentionally gated behind the ADJ lock so the alignment doesn't reset when you grab the device.
- The WiFi export AP is open while freeze mode is active. There is no password, on purpose, because the AP is short-lived.
- Battery display is hidden unless `BATTERY_ADC_PIN` is wired to an ADC GPIO and the divider is sized for the cell
