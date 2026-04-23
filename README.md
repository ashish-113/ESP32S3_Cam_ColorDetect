# ESP32-S3-CAM: Live Stream + Red/Green/Black Color Detection

A small Arduino sketch for the Hiwonder / ThinkRobotics ESP32-S3-CAM AI Vision
Module that opens its own Wi-Fi hotspot, serves a live camera feed, and
reports real-time red / green / black detection percentages in the browser.
Works with the module as shipped (non-OV2640 sensor) because the JPEG encoding
is done in software.

---

## What the sketch does

- Boots the ESP32-S3 and initializes the on-module camera at `QVGA (320x240)`
  in `RGB565`.
- Creates an open Wi-Fi access point named `ESP32S3-CAM` (no password).
- Runs a tiny HTTP server on `http://192.168.4.1/` with three endpoints:
  - `/` - a single page with live video and a live color readout.
  - `/jpg` - captures one frame, runs color detection, returns it as JPEG.
  - `/status` - returns the latest detection as JSON
    (`red`, `green`, `black`, `dominant`, `confidence`).
- The browser page polls `/jpg` as fast as it can and `/status` every 300 ms,
  giving roughly 5-10 fps video plus live updating bars for each color.

The color detector converts each RGB565 pixel to HSV and classifies it as
`red`, `green`, `black`, or `other`. "Confidence" is the percentage of
sampled pixels belonging to the dominant class.

---

## Hardware

- Module: Hiwonder ESP32-S3-CAM AI Vision Module
  (ThinkRobotics SKU `HIWND-393`).
- SoC: ESP32-S3 (dual-core, 240 MHz).
- Memory: 8 MB external PSRAM (OPI), 4 MB flash (32 Mbit).
- Sensor: ships with a non-JPEG sensor. Common PIDs you may see on first boot:
  - `0x9b` - GC0308
  - `0x21` - GC2145
  - `0x26` - OV2640 (only this one can output JPEG in hardware)
  This is why the sketch uses `PIXFORMAT_RGB565` and `frame2jpg()` instead of
  `PIXFORMAT_JPEG`.
- Power: 5 V through the on-module USB Type-C port from a PC or 5 V adapter.

---

## What you need

- Arduino IDE 2.3.7 (tested).
- ESP32 board support package by Espressif Systems, v2.0.17 (tested).
  v3.x also works.
- A known-good USB-C **data** cable (charge-only cables will not enumerate
  a COM port).

---

## One-time IDE setup

1. `File` > `Preferences` > `Additional Boards Manager URLs`. Add:

   ```text
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```

2. `Tools` > `Board` > `Boards Manager...` > search `esp32` > install
   **esp32 by Espressif Systems**.

3. `Tools` > `Board` > `ESP32 Arduino` > select **ESP32S3 Dev Module**.

---

## Exact Tools menu settings (critical)

These are the settings we converged on through trial and error. Anything else
likely reproduces one of the errors in the Troubleshooting section below.

- Board: `ESP32S3 Dev Module`
- USB CDC On Boot: `Enabled`
- USB Mode: `Hardware CDC and JTAG`
- CPU Frequency: `240MHz (WiFi)`
- Flash Mode: `QIO 80MHz`
- Flash Size: `4MB (32Mb)`
- Partition Scheme: `Huge APP (3MB No OTA/1MB SPIFFS)`
- PSRAM: `OPI PSRAM` (must be set; otherwise camera init fails with
  `frame buffer malloc failed`)
- Upload Mode: `UART0 / Hardware CDC`
- Upload Speed: `921600`
- Core Debug Level: `None`
- Erase All Flash Before Sketch Upload: `Disabled`
  (enable for one upload if you just changed the PSRAM setting and it does
  not seem to take effect, then disable again)

---

## Upload procedure

1. Plug the module into the PC using the Type-C cable.
2. `Tools` > `Port` - pick the COM port that appears when the module is
   connected (it disappears when unplugged - that is the one).
3. If no new port appears: hold the `BOOT` button, tap `RESET`, release
   `BOOT`, then refresh the Port list.
4. Click Upload.
5. After upload, the module reboots into your sketch automatically.

---

## Usage

1. On your phone or laptop, open Wi-Fi settings. Within a few seconds you
   should see an open network called **`ESP32S3-CAM`**.
2. Connect to it. Ignore any "no internet" warning - this network is
   isolated and only for talking to the camera.
3. Open **`http://192.168.4.1/`** in any browser.
4. You should see:
   - Live video from the camera.
   - A `Dominant: <color> (<confidence>%)` line.
   - Three bars for Red, Green, and Black updating roughly 3 times a second.
5. Quick sanity checks:
   - Hold a bright red object in front of the lens - the Red bar jumps.
   - Swap it for green - Green bar jumps.
   - Cover the lens or aim at something very dark - Black bar jumps.

---

## How it works (short version)

- The camera runs in `RGB565` because the shipped sensor cannot produce JPEG.
- `XCLK` is set to `10 MHz`. The default 20 MHz causes `EV-VSYNC-OVF` and
  `EV-EOF-OVF` overflow errors on GC0308 / GC2145.
- Each request to `/jpg`:
  1. Calls `esp_camera_fb_get()` to grab a frame.
  2. Calls `analyzeRGB565()` which walks every 2nd pixel on every 2nd row,
     converts RGB565 -> RGB888 -> HSV, and counts red / green / black pixels.
  3. Calls `frame2jpg()` from `img_converters.h` to software-encode the
     frame as JPEG, then streams it to the client.
- Because handlers return quickly (instead of looping on one open socket),
  `/status` and `/jpg` can be served back to back - this is why the color
  bars actually update in the UI.
- The page uses plain `fetch()` + `new Image()` polling; no special client
  library required.

---

## Tuning color detection

All thresholds live in one function (`classify()` in the sketch):

```cpp
if (v < 55) return 3;                // black threshold (raise for dim rooms)
if (s < 85 || v < 60) return 0;      // saturation / brightness floors
if (h < 20 || h > 340) return 1;     // red hue window
if (h > 85  && h < 170) return 2;    // green hue window
```

Suggested tweaks:

- Warm / yellowish indoor light: try `s < 60 || v < 45` to be more
  permissive, and widen green to `h > 80 && h < 175`.
- Very bright scene where everything reads as "other": lower the saturation
  floor (`s < 70`).
- Scene with a lot of dark gray reading as black: raise the black
  threshold, for example `if (v < 40) return 3;`.

---

## Troubleshooting (things that actually went wrong while building this)

- `E (xxx) camera: JPEG format is not supported on this sensor`
  The sensor is not an OV2640. Fixed by capturing in `PIXFORMAT_RGB565` and
  encoding JPEG in software. Already done in this sketch.

- `cam_hal: cam_dma_config(301): frame buffer malloc failed`
  PSRAM is not being used. Set `Tools` > `PSRAM: OPI PSRAM` and re-upload.

- `cam_hal: EV-VSYNC-OVF` or `cam_hal: EV-EOF-OVF`
  `xclk_freq_hz` is too high for the shipped sensor. Keep it at `10000000`
  (10 MHz). 8 MHz also works if 10 still overflows.

- Nothing at all on Serial Monitor even though the board runs
  Need `USB CDC On Boot: Enabled` and `USB Mode: Hardware CDC and JTAG`,
  then re-upload. The USB mode is baked in at flash time.

- Live stream works but Red / Green / Black stay at 0%
  You are running an older version of the sketch that used an endless
  MJPEG `while(client.connected())` loop. That blocks the single-threaded
  `WebServer`, so `/status` is never served. Use the polling version
  (`/jpg` per request) that matches this README.

- Module does not enumerate a COM port
  Hold `BOOT`, tap `RESET`, release `BOOT`, then refresh `Tools` > `Port`.
  If still nothing, swap to a known-good USB-C data cable.

---

## Pin map reference

The camera GPIOs for this module, as used in the sketch:

```cpp
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   15
#define SIOD_GPIO_NUM    4   // I2C SDA to sensor
#define SIOC_GPIO_NUM    5   // I2C SCL to sensor
#define Y9_GPIO_NUM     16
#define Y8_GPIO_NUM     17
#define Y7_GPIO_NUM     18
#define Y6_GPIO_NUM     12
#define Y5_GPIO_NUM     10
#define Y4_GPIO_NUM      8
#define Y3_GPIO_NUM      9
#define Y2_GPIO_NUM     11
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM   13
```

---

## Credits / links

- Product page (ThinkRobotics):
  <https://thinkrobotics.com/products/esp32-s3-cam-ai-vision-module>
- Pin map source (NullLab / Hiwonder community repo):
  <https://github.com/nulllaborg/esp32s3-cam>
- Arduino core for ESP32:
  <https://github.com/espressif/arduino-esp32>
