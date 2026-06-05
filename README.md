# Multicamera Thermal

Handheld ESP32-S3 thermal camera with a visible camera overlay, LCD controls,
freeze-frame WiFi export, and a browser portal for live preview and recording. For pictures/videos see: https://www.instagram.com/p/DY12QflxQjW/ (and others on that page)

## Hardware

| Part | Purpose |
| --- | --- |
| DFRobot FireBeetle 2 ESP32-S3-U, 16 MB flash / 8 MB OPI PSRAM | Main controller |
| DFRobot/Fermion 3.5 inch ILI9488 + GT911 display | LCD and touch UI |
| OV3660 or OV2640 camera | Visible-light camera |
| MLX90640 32x24 thermal sensor, standard FOV | Thermal data |
| 6×6×8mm Momentary Push Button with Cap| Freeze / portal control |
| ​Toggle Switch (S12F15 SS12F15VG4) | on and off|

PSRAM is required. SD card storage is not used; saving is done through WiFi
export or browser-side recording.

## Pins

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

LCD can be connected via included ribbon cables so you don't have to make a ton of connections manually (unless you use cheaper parts!)

### Visible camera

| Signal | GPIO |
| --- | ---: |
| XCLK | 45 |
| SIOD / SDA | 1 |
| SIOC / SCL | 2 |
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

Same deal - camera can be plugged in with ribbon cable. 
The camera runs QVGA RGB565 for the LCD path, with 16 MHz XCLK and two frame buffers. SCCB shares the main I2C bus on GPIO1/GPIO2.

### Thermal sensor and button

| Signal | GPIO |
| --- | ---: |
| MLX90640 SDA | 11 |
| MLX90640 SCL | 14 |
| MLX90640 SCL | 14 |
| MLX90640 VIN | 3.3v |
| MLX90640 GND | GND |
| MLX90640 PS | GND 

| Signal | GPIO |
| --- | ---: |
| Button | 10 |
| Button GND | GND |


The MLX90640 uses `Wire1` at 1 MHz. The included MLX90640 I2C driver is patched
to use `Wire1`.

## Firmware Features

- LCD modes: `TINT`, `THERM`, `CAM`.
- Range modes: `AUTO`, `INV`, `AUT2`, `AUT3`, `DET`, `HOT`, `SKIN`, `PCB`,
  `HIGH`, `WIDE`, `COLD`, `MAN`.
- Distance alignment presets: 5, 15, 30, 100, and 500 cm.
- Advanced alignment offsets: X/Y offset and X/Y zoom offset.
- Alignment reset returns to the current 30 cm calibration, with ADV sliders at
  zero for extra offsets.
- Tap the LCD image to place a temperature marker.
- Short button press freezes the frame and opens WiFi export.
- Hold the button for about 3 seconds to start or stop the live browser portal.

## LCD UI

The LCD keeps the live image visible and groups controls by page:

- Bottom buttons cycle current view and range.
- `ALIGN`: distance preset and distance slider.
- `TUNE`: tint control.
- `MAN`: manual low/high temperature range.
- `ADV`: alignment offsets, camera brightness, LCD brightness, and reset alignment.

Settings are saved to ESP32 NVS after a debounce.

## Browser Portal

Hold the button for about 3 seconds. Connect to the `ThermalCam-XXXX` WiFi
network and open:

```text
http://192.168.4.1/
```

The portal supports:

- Overlay, camera-only, and thermal-only views.
- Distance and alignment controls.
- Thermal range controls.
- Marker temperature readout.
- Browser-side recording with optional temperature overlay.

Recording is done in the browser with `canvas.captureStream()` and
`MediaRecorder`; the ESP32 does not store video. The recorded overlay only
includes center, marker, scene, and palette temperatures.

Main portal endpoints:

| Endpoint | Purpose |
| --- | --- |
| `/` | Portal UI |
| `/cam.jpg` | Latest camera JPEG, cropped before encoding when alignment zoom is active |
| `/thermal.bin` | 32x24 thermal grid as int16 centi-degrees C |
| `/api/state` | Live controls, temperatures, and basic FPS |
| `/api/diag` | Detailed diagnostics |
| `/api/control` | Control updates |
| `/thermal.csv` | Current thermal grid CSV |

## Freeze Export

Short-press the button to freeze. The device opens the same `ThermalCam-XXXX`
WiFi AP at `http://192.168.4.1/`.

Freeze export includes:

- `overlay.bmp`
- `thermal.bmp`
- `camera.bmp`
- `thermal.csv`
i.e., it lays out all three images (thermal, RGB, and combined)

Short-press again to exit freeze mode.


## Build

Known-good local build:

| Setting | Value |
| --- | --- |
| ESP32 board package | 2.0.11 |
| Board | DFRobot FireBeetle 2 ESP32-S3 |
| USB Mode | Hardware CDC and JTAG |
| USB CDC On Boot | Enabled |
| CPU Frequency | 240 MHz |
| Flash Size | 16 MB |
| Partition Scheme | 16M Flash, 3 MB app / 9.9 MB FATFS |
| PSRAM | OPI PSRAM |

Required Arduino libraries:

- LovyanGFX
- DFRobot_AXP313A (don't forget this! You may have to manually install it)

The MLX90640 API files are included in this repository.

## Notes

**- Arduino-ESP32 2.0.11 is the tested baseline.**
- Arduino-ESP32 3.x may work, but camera reconfiguration behavior has differed
  during testing.
- Alignment is plane-specific. A hand and the background will not align at the
  same time because they are at different distances. You need to adjust based on your build. 

## Wiring Chart
<img width="1051" height="967" alt="hybrid (3)" src="https://github.com/user-attachments/assets/f2235096-e8e3-47cd-85f9-6e66230412da" />
