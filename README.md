# Multicamera Thermal

DIY ESP32-S3 visible plus thermal camera built around an OV2640-style parallel camera, a Melexis MLX90640 thermal array, and a 3.5 inch ILI9488 touch display.

This project is an embedded thermal/visible overlay camera. It renders a live RGB camera feed, a thermal-only view, or an additive thermal overlay, with touch controls for alignment, palette range, zoom, tint, and camera brightness. A physical button freezes the current frame and temporarily opens a WiFi access point so the frozen overlay, camera image, thermal image, and thermal CSV can be saved from a phone or computer.

No photos are included in this repository.

## Hardware

The sketch is configured for this hardware stack:

| Part | Role | Notes |
| --- | --- | --- |
| DFRobot FireBeetle 2 ESP32-S3 | Main controller | ESP32-S3 with PSRAM is required for camera frame buffering. |
| DFRobot Fermion 3.5 inch display, DFR0669 class | LCD and touch UI | 480x320 ILI9488 SPI display with GT911 capacitive touch. |
| OV2640-compatible parallel camera | Visible camera | Captured as 320x240 RGB565 into PSRAM. |
| Melexis MLX90640 | Thermal sensor | 32x24 thermal array, standard FOV assumptions in the sketch are 55 x 35 degrees. |
| DFRobot AXP313A | Power management | Initialized before display/camera bring-up. |
| Momentary pushbutton | Freeze/share control | Normally open button from GPIO10 to GND, using `INPUT_PULLUP`. |
| Optional SD card hardware | Currently disabled | The code contains disabled SD experiments, but WiFi frame export is the active save path. |

## Pinout

The pinout is hard-coded in `multicamera_thermal.ino`.

### LCD and touch

| Signal | GPIO |
| --- | ---: |
| TFT SCLK | 17 |
| TFT MOSI | 15 |
| TFT MISO | 16 |
| TFT DC | 3 |
| TFT RST | 38 |
| TFT CS | 18 |
| TFT backlight | 21 |
| GT911 SDA | 1 |
| GT911 SCL | 2 |
| GT911 INT | 13 |

GPIO12 is on the display ribbon as touch chip select on this hardware and should be avoided.

### Visible camera

| Signal | GPIO |
| --- | ---: |
| XCLK | 45 |
| SIOD | 1 |
| SIOC | 2 |
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

The camera SCCB bus shares GPIO1/GPIO2 with GT911 and AXP313A on the main I2C bus.

### Thermal sensor and button

| Signal | GPIO |
| --- | ---: |
| MLX90640 SDA | 11 |
| MLX90640 SCL | 14 |
| Button | 10 |

The MLX90640 runs on `Wire1` at 1 MHz. The included `MLX90640_I2C_Driver.cpp` is patched to use `Wire1` rather than the default `Wire` bus.

## Firmware Features

### Display modes

- `TINT`: visible camera with additive thermal palette overlay.
- `THERM`: thermal-only image.
- `CAM`: camera-only image.

The thermal image is mapped over the 320x240 display area. Camera zoom and parallax adjust only the visible camera crop so the thermal grid remains stable while the camera is aligned to it.

### Thermal processing

The MLX90640 path is tuned for small skin-temperature features such as fingers:

- MLX90640 chess mode.
- 19-bit ADC resolution.
- 16 subpages per second, giving about 8 complete thermal frames per second.
- Coherent RAM read validation so a subpage is not used if a new subpage arrives during the read.
- Full-frame publishing only after both chess subpages have been collected.
- EEPROM broken/outlier pixel repair.
- Global chess-subpage bias balancing.
- Adaptive temporal filtering with faster response for real movement.
- Optional edge-aware spatial pass, currently disabled by default to preserve small features.
- Percentile-clipped auto range to reduce the effect of outlier cells.

### Range modes

The side-panel range button cycles through:

- `AUTO`: smoothed auto range.
- `AUT2`: subject-aware auto range that tries to ignore ambient background when a thermal subject is present.
- `SKIN`: fixed exposed-skin range.
- `BODY`: fixed body/clothing range.
- `WARM`: mugs, electronics, and warm surfaces.
- `HOT`: stovetops and irons.
- `VHOT`: flame or exhaust-class temperatures.
- `MAN`: manual low/high range using the RANGE tab sliders.

Manual range uses `LO` and `HI` sliders. Each slider also has `-` and `+` nudge buttons; manual nudges move by 0.1 C and maintain a minimum 1.0 C span.

### Touch controls

The GT911 UI provides:

- Mode switching.
- Range switching.
- Position tab for X/Y parallax and camera zoom.
- Range tab for manual `LO`/`HI`.
- `ADJ` lock so alignment controls are not changed accidentally.
- `- [track] +` controls for tint, X, Y, zoom, manual low, and manual high.
- Five-step camera brightness control.
- Reset button for alignment values.

Settings are persisted with ESP32 `Preferences` after a debounce period.

### Freeze and WiFi export

Pressing the physical button toggles freeze mode.

When a frame is frozen:

- Camera and thermal acquisition stop updating the displayed frame.
- A WiFi access point starts with an SSID like `ThermalCam-ABCD`.
- The device serves a landing page at `http://192.168.4.1/`.
- The page shows the frozen overlay and links to:
  - `overlay.bmp`
  - `thermal.bmp`
  - `camera.bmp`
  - `thermal.csv`

When the frame is unfrozen:

- HTTP server stops.
- Captive DNS stops.
- The WiFi AP is disconnected.
- WiFi is turned off.
- The frozen camera PSRAM buffer is released.

The AP is intentionally temporary and open. Only use it while saving a frame.

### SD card status

The code contains historical SD-card bring-up and BMP-writing code, but the active save/export path is the freeze WiFi server. SD mounting is disabled in `setup()` because this hardware arrangement had LCD/SPI-bus interference. The WiFi export path avoids that bus conflict.

## Build Requirements

Tested locally with:

- Arduino CLI from Arduino IDE.
- ESP32 Arduino core 2.0.11.
- Board FQBN: `esp32:esp32:dfrobot_firebeetle2_esp32s3`.
- LovyanGFX 1.2.20.
- DFRobot_AXP313A 1.0.0.
- ESP32 bundled `WiFi`, `WebServer`, `DNSServer`, `Wire`, `SPI`, `Preferences`, and `FS`.

The Melexis MLX90640 API sources needed by the sketch are included in this repository:

- `MLX90640_API.cpp`
- `MLX90640_API.h`
- `MLX90640_I2C_Driver.cpp`
- `MLX90640_I2C_Driver.h`

## Arduino Board Settings

Use these board settings unless your hardware differs:

| Setting | Value |
| --- | --- |
| Board | DFRobot FireBeetle 2 ESP32-S3 |
| USB Mode | Hardware CDC and JTAG |
| USB CDC On Boot | Enabled |
| CPU Frequency | 240 MHz (WiFi) |
| Flash Size | 16 MB |
| Partition Scheme | 16M Flash (3MB APP/9.9MB FATFS) |
| PSRAM | OPI PSRAM |

Equivalent Arduino CLI verify command:

```powershell
arduino-cli compile `
  -b esp32:esp32:dfrobot_firebeetle2_esp32s3:CDCOnBoot=cdc,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=opi `
  --warnings all .
```

## Repository Contents

Required source files:

- `multicamera_thermal.ino`: main firmware, UI, rendering, freeze/share server, acquisition pipeline.
- `MLX90640_API.cpp` and `MLX90640_API.h`: Melexis thermal calculation API.
- `MLX90640_I2C_Driver.cpp` and `MLX90640_I2C_Driver.h`: I2C access layer patched for the dedicated `Wire1` MLX bus.

Ignored local-only artifacts include debug configs, SVD files, editor state, and build products.

## Notes and Limitations

- This is not a calibrated medical or safety instrument.
- Manual alignment depends on the camera lens, MLX90640 variant, and physical mounting.
- The WiFi export AP is open while freeze mode is active.
- Battery display is disabled unless `BATTERY_ADC_PIN` is wired and configured.
- Photos are intentionally not part of the initial repository.
