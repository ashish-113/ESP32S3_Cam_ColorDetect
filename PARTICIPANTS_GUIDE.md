# Participants Guidelines

Welcome! This document tells you everything you need to do **before**
the workshop so we can spend the session building, not installing.

## What we will build during the workshop

A small autonomous bot in two parts that talk to each other over I2C:

1. An **ESP32-S3 camera module** that streams live video over Wi-Fi and
   detects red / green / black objects in real time.
2. An **ESP32 dev module** driving a 2-wheel + caster robot through an
   L298N motor driver and a BFD1000 line/obstacle sensor array. The bot
   reads the color from the camera and reacts to it.

Both firmwares already exist in this repository - we will read, modify,
flash, and debug them together.

---

## Before you arrive: 60-second checklist

Tick each one off. Details for everything are in the sections below.

- [ ] Laptop fully charged + charger in the bag
- [ ] At least 5 GB free disk space
- [ ] You have **admin / sudo** rights on the laptop
- [ ] Arduino IDE 2.3.7 (or newer 2.x) installed
- [ ] ESP32 board package installed in Arduino IDE
- [ ] USB-Serial driver installed (CP210x / CH340)
- [ ] Both example sketches in this repo compile on your machine
- [ ] You know what `setup()` and `loop()` are in an Arduino sketch
- [ ] You can find the **Tools** menu, the **Board** selector, the
      **Port** selector and the **Serial Monitor** in Arduino IDE

---

## What we provide vs. what you bring

You bring:

- Your **laptop** (Windows / macOS / Linux all fine), fully charged,
  with at least one **USB-A** port or a USB-C-to-USB-A adapter / hub
- The laptop's **charger**
- A pen and a small notebook if you take notes
- Headphones are optional; phone for testing the camera Wi-Fi is useful

We provide at the venue:

- ESP32 Dev Module (ESP32-WROOM-32, 30/38 pin)
- ESP32-S3-CAM AI Vision Module (Hiwonder / ThinkRobotics)
- L298N motor driver, 2x hobby DC motors, caster, chassis
- BFD1000 line / obstacle sensor array
- Battery pack, jumper wires, screwdrivers
- USB-C and Micro-USB **data** cables (we will not lend charge-only cables)

---

## Laptop requirements

| Item              | Minimum                                  |
|-------------------|------------------------------------------|
| OS                | Windows 10/11, macOS 12+, Ubuntu 20.04+  |
| RAM               | 4 GB (8 GB comfortable)                  |
| Disk free         | 5 GB                                     |
| USB               | At least one USB-A; a USB hub is fine    |
| Privileges        | Admin / sudo (you must be able to install drivers and the IDE) |

If your laptop only has USB-C, bring a **USB-A to USB-C adapter or hub**
- the boards we hand out have USB-C and Micro-USB cables, but a few
peripherals expect USB-A on the laptop side.

---

## Software setup (do this before you arrive)

### 1. Install Arduino IDE 2.3.7

Download from <https://www.arduino.cc/en/software> and install with
default options. Launch it once after installing so it creates your
sketchbook folder.

### 2. Install the ESP32 board package

1. In Arduino IDE: `File` > `Preferences`.
2. In **Additional Boards Manager URLs**, paste:

   ```text
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```

   Click OK.
3. Open `Tools` > `Board` > `Boards Manager...`, search for `esp32`,
   and install **"esp32" by Espressif Systems**. Either the latest
   2.0.x release or 3.x is fine.
4. After install, you should see two new boards under
   `Tools > Board > esp32`:
   - `ESP32 Dev Module`
   - `ESP32S3 Dev Module`

### 3. Install USB-Serial drivers

The ESP32 dev module on the chassis uses a CP2102 or CH340 USB-serial
chip. Without the driver, no COM port will appear when you plug it in.

- **Windows / macOS, CP210x:** download from
  <https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers>
- **Windows / macOS, CH340:** download from
  <https://www.wch-ic.com/downloads/CH341SER_ZIP.html>
- **Linux:** both drivers are usually built in. If your user cannot open
  `/dev/ttyUSB0`, run once:

  ```bash
  sudo usermod -aG dialout $USER
  ```

  and log out / log back in.

The ESP32-S3 camera module does **not** need a separate driver: it
exposes USB CDC natively and shows up as a COM port automatically.

### 4. Arduino libraries: nothing extra to install

Everything we use in the workshop is bundled with the ESP32 board
package you just installed. Specifically:

- `WiFi.h`, `WebServer.h`, `Wire.h` (Arduino core)
- `esp_camera.h`, `img_converters.h`, `esp_log.h`
  (bundled with the ESP32 board package)

You do **not** need to install anything from `Library Manager`. If a
sketch fails to find one of these, your ESP32 board package install is
incomplete - reinstall it from `Boards Manager`.

---

## Verify your setup with the example sketches

Once the steps above are done, please make sure both example sketches
in this repository compile cleanly on your laptop. Compile only - you
don't need real hardware to verify the toolchain.

### Get the code

Either clone the repository:

```bash
git clone https://github.com/ashish-113/ESP32S3_Cam_ColorDetect.git
```

Or download it as a ZIP from
<https://github.com/ashish-113/ESP32S3_Cam_ColorDetect> and extract.

### Compile-test the camera sketch

1. Open `ESP32-S3-CAM/ESP32-S3-CAM.ino` in Arduino IDE.
2. Select these in the **Tools** menu:
   - Board: `ESP32S3 Dev Module`
   - USB CDC On Boot: `Enabled`
   - USB Mode: `Hardware CDC and JTAG`
   - Flash Size: `4MB (32Mb)`
   - Partition Scheme: `Huge APP (3MB No OTA/1MB SPIFFS)`
   - PSRAM: `OPI PSRAM`
3. Click **Verify** (the check-mark button, or `Sketch > Verify/Compile`).
4. After ~1 minute the bottom panel should say
   `Done compiling.` with no red errors.

### Compile-test the robot sketch

1. Open `ESP32_mcu/ESP32_mcu.ino` in Arduino IDE.
2. Select these in the **Tools** menu:
   - Board: `ESP32 Dev Module`
   - Flash Size: `4MB (32Mb)`
   - Partition Scheme: `Default 4MB with spiffs`
3. Click **Verify**.
4. Should compile in well under a minute and finish with `Done compiling.`

If either sketch fails to compile, fix it before the workshop - that's
the whole point of doing this in advance. The most common causes:

- ESP32 board package not installed or the wrong board selected.
- A library name not found (means board package is incomplete; reinstall).
- The `Tools > PSRAM: OPI PSRAM` setting is missing - select another
  board variant if your IDE doesn't show this option.

---

## Arduino IDE quick familiarity

You don't need to be an Arduino expert, but please know where the
following live so we don't waste time pointing at menus:

- **Sketch structure**: every Arduino sketch has two functions -
  `setup()` runs once at boot, `loop()` runs repeatedly forever after
  that. Pin defines, includes, and global variables sit at the top.
- **Verify** (check mark) compiles the sketch.
- **Upload** (right arrow) compiles and flashes the board.
- **Tools > Board** picks which chip you are flashing for.
- **Tools > Port** picks which USB COM device the IDE talks to. The
  port disappears when you unplug the board - that is how you identify
  the correct one.
- **Serial Monitor** (magnifier icon, top-right) opens a console to read
  `Serial.print(...)` output. Set the baud rate at the bottom-right to
  **115200** for both projects in this workshop.
- **Tabs**: each `.ino` in a sketch folder shows up as a tab. We
  intentionally keep one `.ino` per folder.

If any of those terms feel unfamiliar, work through the official
"Built-in Examples > 01.Basics > Blink" tutorial once on your laptop:

- `File > Examples > 01.Basics > Blink`
- Select an ESP32 board
- Verify (you don't need to actually upload)

That single example covers 90 % of the IDE skills we will use.

---

## A little C / C++ background helps

You will be reading and editing C-style code. You do not need to be
fluent. You will be comfortable if you know:

- Variable types: `int`, `uint8_t`, `float`, `bool`, `const char*`
- `if / else`, `for`, `while`
- Functions and how to call them
- Arrays and bracket indexing: `arr[2]`
- Basic boolean expressions: `&&`, `||`, `!`

We will explain anything specific to embedded code on the spot
(`volatile`, `PROGMEM`, ISR-safe code, bit shifts) - those are not
prerequisites.

---

## Optional pre-reading (recommended, not required)

If you want to walk in with a head start:

- The repo's main [README.md](README.md) - what each sketch does and
  the wiring diagram between them.
- ESP32 vs. ESP32-S3 - one is the chip on the bot brain, the other is
  on the camera. Same vendor, mostly compatible, slightly different
  pin numbering.
- I2C in 5 minutes -
  <https://learn.sparkfun.com/tutorials/i2c/all> (we will only use the
  master / slave roles and one register).

---

## Day-of expectations

- Arrive 15 minutes early so cable hand-out and Wi-Fi onboarding don't
  eat into build time.
- Keep your laptop on the venue Wi-Fi during the session - some firmware
  uses your phone to view the camera, and the laptop will need internet
  for any last-minute driver downloads.
- Don't move tables / chassis / batteries between desks; we'll have a
  spare-parts table.
- Ask questions early. A 30-second clarification beats a 30-minute dead
  end.

---

## If something is wrong on the day

If you arrive and your laptop refuses to compile or the COM port never
appears, that's fine - we have spare laptops at the venue with the
toolchain pre-installed. But please attempt steps 1-4 above ahead of
time so we use those spares for genuine emergencies.

See you at the workshop.
