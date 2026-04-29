# ESP32 / ESP32-S3 Robotics Sketches

Two self-contained Arduino sketches for two different boards that work
together as a small line-following ball-sorting robot:

- [`ESP32-S3-CAM/`](ESP32-S3-CAM/) - **ESP32-S3-CAM AI Vision Module**
  (Hiwonder / ThinkRobotics). Wi-Fi hotspot, live MJPEG stream,
  red / green / yellow color detection in the browser, and an **I2C slave**
  endpoint at address `0x52` that publishes the latest detection.
- [`ESP32_mcu/`](ESP32_mcu/) - **ESP32 Dev Module** driving the bot:
  two hobby DC motors via an L298N driver, a BFD1000 5-channel line +
  1-channel proximity sensor array, a hobby servo, and an **I2C master**
  that polls the camera. The bot follows a 3.5 cm black line on white;
  when the BFD1000 NEAR pin sees a ball it stops, asks the camera what
  colour it is, and either sweeps the servo (red or yellow) or drives on
  (green / no colour).

Each sketch lives in its own folder so Arduino IDE can open and compile it
without complaints.

## I2C wiring between the two boards

| Camera (ESP32-S3-CAM, slave) | Robot (ESP32 Dev Module, master) |
|---|---|
| `IIC.SDA` | `GPIO 21` (default `SDA`) |
| `IIC.SCL` | `GPIO 22` (default `SCL`) |
| `GND`     | `GND` (must be tied together)    |

Camera publishes 5 bytes from register `0x00`:

| Byte | Meaning |
|---|---|
| 0 | dominant: `0=none, 1=red, 2=green, 3=yellow` |
| 1 | red percentage (0..100) |
| 2 | green percentage (0..100) |
| 3 | yellow percentage (0..100) |
| 4 | confidence percentage (0..100) |

If your Hiwonder expansion board routes the IIC header to GPIOs other
than 1 / 2, change `I2C_SDA_PIN` / `I2C_SCL_PIN` near the top of
[`ESP32-S3-CAM/ESP32-S3-CAM.ino`](ESP32-S3-CAM/ESP32-S3-CAM.ino).

---

## Repository layout

```text
.
|-- README.md                    <- you are here
|-- .gitignore
|-- ESP32-S3-CAM/
|   `-- ESP32-S3-CAM.ino         <- camera sketch (target: ESP32S3 Dev Module)
`-- ESP32_mcu/
    `-- ESP32_mcu.ino            <- robot sketch (target: ESP32 Dev Module)
```

## What you need (both projects)

- Arduino IDE 2.3.7 (tested).
- ESP32 board support package by Espressif Systems, v2.0.17 (tested);
  v3.x also works.
- A USB-C **data** cable (charge-only cables will not enumerate a COM port).

No extra Arduino libraries are needed. The robot sketch drives the servo
through the ESP32 core's built-in LEDC PWM, so there is no `ESP32Servo`
library to install.

### One-time IDE setup

1. `File` > `Preferences` > `Additional Boards Manager URLs`. Add:

   ```text
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```

2. `Tools` > `Board` > `Boards Manager...` > search `esp32` > install
   **esp32 by Espressif Systems**.

---

# Project 1 - `ESP32-S3-CAM`: live stream + color detection

Folder: [`ESP32-S3-CAM/ESP32-S3-CAM.ino`](ESP32-S3-CAM/ESP32-S3-CAM.ino)

## What it does

- Boots the ESP32-S3 and initializes the on-module camera at `QVGA (320x240)`
  in `RGB565`.
- Creates an open Wi-Fi access point named `ESP32S3-CAM` (no password).
- Runs a tiny HTTP server on `http://192.168.4.1/` with three endpoints:
  - `/` - a single page with live video and a live color readout.
  - `/jpg` - captures one frame, runs color detection, returns it as JPEG.
  - `/status` - latest detection as JSON
    (`red`, `green`, `yellow`, `dominant`, `confidence`).
- Each pixel is converted RGB565 -> RGB888 -> HSV and classified as
  `red`, `green`, `yellow`, or `other`. "Confidence" is the percentage of
  sampled pixels in the dominant class.
- After every analyzed frame the camera also publishes the result on its
  IIC header as I2C **slave** at address `0x52` (5-byte response from
  register `0x00`, see top of this README).

## Hardware

- Module: Hiwonder ESP32-S3-CAM AI Vision Module (ThinkRobotics SKU `HIWND-393`).
- SoC: ESP32-S3 (dual-core, 240 MHz).
- Memory: 8 MB external PSRAM (OPI), 4 MB flash (32 Mbit).
- Sensor: ships with a non-JPEG sensor. Common PIDs you may see on first boot:
  - `0x9b` - GC0308
  - `0x21` - GC2145
  - `0x26` - OV2640 (only this one can output JPEG in hardware)
  This is why the sketch uses `PIXFORMAT_RGB565` and `frame2jpg()` instead of
  `PIXFORMAT_JPEG`.
- Power: 5 V via the on-module USB Type-C port.

## Tools menu (critical for the camera sketch)

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

## Usage

1. On your phone or laptop, open Wi-Fi settings and join the open network
   **`ESP32S3-CAM`** (ignore the "no internet" warning).
2. Open **`http://192.168.4.1/`** in any browser.
3. You should see live video plus live `Red`, `Green`, `Yellow` percentages
   and a `Dominant` readout that updates a few times per second.
4. Quick sanity checks: hold a red ball, then a green ball, then a yellow
   ball in front of the lens - each bar should jump in turn.

## Tuning color detection

All thresholds live in `classify()`:

```cpp
if (s < 85 || v < 70) return 0;      // saturation / brightness floors
if (h < 20 || h > 340) return 1;     // red    hue window  (~0 deg)
if (h > 40 && h < 70)  return 3;     // yellow hue window  (~60 deg)
if (h > 85 && h < 170) return 2;     // green  hue window  (~120 deg)
```

Common tweaks:

- Warm / yellowish indoor light: lower the saturation floor to `s < 65`,
  and tighten yellow to `h > 45 && h < 65` so wood / skin tones don't
  read as yellow.
- Bluish daylight: leave yellow as-is; if green looks dull, widen to
  `h > 80 && h < 175`.
- The detector deliberately has no "black" bucket. Dark / unlit pixels
  fall into the `other` class via the `s` and `v` floors.

## Pin map (camera module)

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

## Camera troubleshooting

- `JPEG format is not supported on this sensor` - non-OV2640 sensor; sketch
  uses `PIXFORMAT_RGB565` and software JPEG.
- `cam_dma_config(301): frame buffer malloc failed` - set
  `Tools > PSRAM: OPI PSRAM` and re-upload.
- `cam_hal: EV-VSYNC-OVF` / `EV-EOF-OVF` - `xclk_freq_hz` too high; the
  sketch uses `10 MHz`.
- Nothing on Serial Monitor even though it runs - need
  `USB CDC On Boot: Enabled` and `USB Mode: Hardware CDC and JTAG`,
  then re-upload.
- Stream works but Red / Green / Yellow stay at 0 - older sketch with an
  endless MJPEG loop blocking `/status`. The current sketch avoids this.
- Module not enumerating - hold `BOOT`, tap `RESET`, release `BOOT`, refresh
  `Tools > Port`.

---

# Project 2 - `ESP32_mcu`: differential-drive robot

Folder: [`ESP32_mcu/ESP32_mcu.ino`](ESP32_mcu/ESP32_mcu.ino)

## What it does

The robot is a small line-following ball-sorter:

1. **Boot motor self-test** (on by default): forward 600 ms, backward
   600 ms, left 600 ms, right 600 ms, then stop. Confirms each motor's
   direction and which side is left vs right.
2. **Line following on a 3.5 cm black line on white surface** (default
   `ENABLE_LINE_FOLLOW = true`): simple 5-channel bang-bang controller
   over the BFD1000.
3. **Ball detection via BFD1000 NEAR**: when NEAR fires the bot stops,
   force-polls the camera over I2C, and:
   - **red** or **yellow** ball -> sweep the servo
     (`SERVO_REST_ANGLE` -> `SERVO_ACTUATED_ANGLE`, hold, return).
   - **green** ball or no clear colour -> skip actuation, carry on.
   A `NEAR_COOLDOWN_MS` window (default 1.5 s) prevents the bot from
   re-triggering on the same ball while it drives past.
4. **Continuous sensor / camera monitor on Serial** (115200) every
   200 ms, e.g.
   `S1=1 S2=1 S3=0 S4=1 S5=1 NEAR=1 line=[..#..] obst=. | cam=red R=42 G=2 Y=3 conf=42`.

## Hardware

- Board: any standard ESP32 Dev Module (ESP32-WROOM-32, 30/38-pin board).
- Drive: two hobby DC motors driven by one L298N (each L298N has two
  H-bridges, so a single module is enough for two motors).
- Chassis: differential drive with a passive caster up front.
- Sensors: BFD1000 5-channel line sensor + 1-channel proximity (NEAR) pin.
- Actuator: hobby servo (SG90 / MG90S size) on `GPIO 23`.

Power: motors, sensors and servo from the battery; ESP32 from USB during
testing or from the L298N's onboard 5 V regulator output. **Tie all grounds
(ESP32 GND, L298N GND, BFD1000 GND, servo GND, battery negative) together** -
missing this is the most common reason "nothing reads" or "motors do nothing".
The servo's V+ should come from battery 5 V, **not** from the ESP32's 3V3
pin (a hobby servo can momentarily draw 500 mA which will brown the ESP32
out).

## Tools menu (robot sketch)

- Board: `ESP32 Dev Module`
- Upload Speed: `921600`
- CPU Frequency: `240MHz (WiFi/BT)`
- Flash Frequency: `80MHz`
- Flash Mode: `QIO`
- Flash Size: `4MB (32Mb)`
- Partition Scheme: `Default 4MB with spiffs`
- Core Debug Level: `None`

## Pin map (L298N + BFD1000)

L298N motor driver (one module driving both motors):

- `IN1` -> `GPIO13`, `IN2` -> `GPIO19`  (Motor A: left wheel)
- `IN3` -> `GPIO14`, `IN4` -> `GPIO27`  (Motor B: right wheel)
- L298N `5V` -> battery positive (or 5 V from regulator if jumpered)
- L298N `GND` -> battery negative AND ESP32 `GND`
- `ENA` and `ENB` jumpers: leave installed for full-speed digital control.
  Remove and wire to PWM-capable GPIOs if you want speed control.

BFD1000 sensor array (digital outputs, active level configurable in source):

- `S1`   -> `GPIO35` (input only on ESP32 - fine for inputs)
- `S2`   -> `GPIO34` (input only on ESP32 - fine for inputs)
- `S3`   -> `GPIO26`
- `S4`   -> `GPIO25`
- `S5`   -> `GPIO33`
- `NEAR` -> `GPIO32`
- `CLIP` -> not connected
- `Vcc`  -> battery positive (3.3 V or 5 V depending on board variant)
- `GND`  -> battery negative

Servo (hobby SG90 / MG90S):

- Signal (orange / yellow) -> `GPIO 23`
- V+      (red)            -> battery 5 V (NOT the ESP32 3V3 pin)
- GND     (brown / black)  -> common ground with ESP32 / L298N / battery

I2C link to the camera:

- `SDA` -> `GPIO 21` -> camera IIC SDA
- `SCL` -> `GPIO 22` -> camera IIC SCL
- shared `GND` (mandatory)

In the sketch, `IN1`/`IN2` drive the left motor and `IN3`/`IN4` drive the
right motor with the convention `dir = +1` forward, `-1` backward,
`0` brake.

## Direction / polarity adjustments

If the boot self-test shows something off, fix it once and forget it:

- **A wheel turns the wrong way** during the "forward" step - swap that
  motor's two leads at the L298N output terminals (or swap the matching
  `IN#` pin numbers in `ESP32_mcu.ino`).
- **"Left" actually turns right and vice versa** - your motors are wired
  to the swapped L298N channels. Swap the screw-terminal motor leads
  between the two L298N output pairs, or swap the `IN1/IN2` and `IN3/IN4`
  pin numbers in the source.
- **Sensor logic is inverted** (`#` shows on the white surface instead of
  on the black tape) - flip `LINE_ACTIVE_LEVEL` from `LOW` to `HIGH` near
  the top of the sketch. Same idea for `OBSTACLE_ACTIVE_LEVEL` if NEAR is
  inverted on your board.
- **Line follower steers the wrong way** - swap `S1`<->`S5` and
  `S2`<->`S4` in the pin defines (your sensor strip is mounted reversed
  relative to the chassis).

## Tunable behaviour constants

Near the top of [`ESP32_mcu/ESP32_mcu.ino`](ESP32_mcu/ESP32_mcu.ino):

```cpp
#define SERVO_PIN              23
#define SERVO_REST_ANGLE        0    // deg, idle position
#define SERVO_ACTUATED_ANGLE   90    // deg, "fire" position
#define SERVO_HOLD_MS         500    // hold at actuated angle
#define SERVO_RETURN_MS       250    // settle back at rest
#define NEAR_COOLDOWN_MS     1500    // ignore NEAR for this long after a sweep
```

## Quick test plan

1. Power the L298N from the battery; ESP32 from USB during bring-up.
   Tie all grounds together (battery, L298N, ESP32, BFD1000, servo, camera).
2. Lift the chassis so the wheels can spin freely.
3. Flash both boards (camera and robot). Open Serial Monitor on the robot
   side at 115200 and watch the boot self-test.
4. After the self-test, slide the bot over a black tape strip and wave a
   hand in front of NEAR - the printed `line=[#####]` and `obst=` columns
   should react. The `cam=` field should show `red` / `green` / `yellow`
   / `none` based on what the camera sees.
5. Place a coloured ball directly under the NEAR sensor:
   - red / yellow -> servo sweeps to `SERVO_ACTUATED_ANGLE` and back.
   - green / none -> servo stays put.
6. Put the bot on a 3.5 cm black line track and let it follow.

---

## Credits / references

- Hiwonder ESP32-S3-CAM product page (ThinkRobotics):
  <https://thinkrobotics.com/products/esp32-s3-cam-ai-vision-module>
- Hiwonder/NullLab ESP32-S3-CAM pin map source:
  <https://github.com/nulllaborg/esp32s3-cam>
- Arduino core for ESP32:
  <https://github.com/espressif/arduino-esp32>
