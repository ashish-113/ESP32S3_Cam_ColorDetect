// ESP32_mcu.ino
// Differential-drive robot: ESP32 Dev Module + L298N motor driver (2 hobby DC motors)
// + BFD1000 5-channel line / 1-channel obstacle sensor array. Caster wheel up front.
//
// Tools menu:
//   Board:                "ESP32 Dev Module"
//   Upload Speed:         921600
//   CPU Frequency:        240MHz (WiFi/BT)
//   Flash Frequency:      80MHz
//   Flash Mode:           QIO
//   Flash Size:           4MB (32Mb)
//   Partition Scheme:     Default 4MB with spiffs
//   Core Debug Level:     None
//   Port:                 (the COM port that appears when the ESP32 is plugged in)

// =====================================================================
//                            PIN MAP
// =====================================================================
// L298N motor driver
//   IN1 -> GPIO13   IN2 -> GPIO19   (Motor A: left wheel)
//   IN3 -> GPIO14   IN4 -> GPIO27   (Motor B: right wheel)
//   5V  -> Battery + (through L298N onboard regulator if jumpered)
//   GND -> Battery -, AND tie to ESP32 GND (common ground is required!)
//   ENA / ENB jumpers: leave installed (full speed). Remove and wire to
//   PWM-capable GPIOs if you want speed control.
//
// BFD1000 sensor array (digital outputs, 5 line + 1 obstacle)
//   S1   -> GPIO35  (input only on ESP32 - fine for sensor input)
//   S2   -> GPIO34  (input only on ESP32 - fine for sensor input)
//   S3   -> GPIO26
//   S4   -> GPIO25
//   S5   -> GPIO33
//   NEAR -> GPIO32  (proximity / obstacle output)
//   CLIP -> not connected
//   Vcc  -> Battery + (typically 3.3 V or 5 V; check board)
//   GND  -> Battery -
//
// I2C link to ESP32-S3-CAM (camera is the I2C SLAVE at 0x52)
//   ESP32 SDA (GPIO21) -> camera IIC SDA
//   ESP32 SCL (GPIO22) -> camera IIC SCL
//   GND               <-> camera GND   (common ground required!)
// Camera publishes 5 bytes from register 0x00:
//   [0] dominant code: 0=none, 1=red, 2=green, 3=black
//   [1] red %      [2] green %    [3] black %    [4] confidence %
// =====================================================================

#include <Wire.h>

#define IN1 13
#define IN2 19
#define IN3 14
#define IN4 27

#define S1   35
#define S2   34
#define S3   26
#define S4   25
#define S5   33
#define NEAR 32

// BFD1000 typically pulls its output LOW when it sees the line (dark tape on
// light surface). If your board behaves the opposite way, change to HIGH.
#define LINE_ACTIVE_LEVEL  LOW

// NEAR pin polarity (most modules: LOW when an obstacle is close).
#define OBSTACLE_ACTIVE_LEVEL  LOW

// ---------- I2C link to camera ----------
#define CAM_I2C_ADDR    0x52
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22
#define I2C_FREQ_HZ     100000
#define CAM_POLL_MS     200

// ---------- behaviour switches ----------
const bool ENABLE_BOOT_MOTOR_TEST = true;   // drive a short sequence on power-up
const bool ENABLE_LINE_FOLLOW     = false;  // flip to true after the wiring is verified
const bool ENABLE_COLOR_REACTION  = false;  // act on color from the camera (red=stop, green=go)
const uint32_t SERIAL_PRINT_MS    = 200;

// =====================================================================
//                          MOTOR HELPERS
// =====================================================================
// dir: +1 = forward, -1 = backward, 0 = stop / brake.
// If a wheel turns the wrong way, swap that motor's two L298N outputs at
// the screw terminal (or swap the IN# pin numbers above).
void leftMotor(int dir) {
  digitalWrite(IN1, dir > 0 ? HIGH : LOW);
  digitalWrite(IN2, dir < 0 ? HIGH : LOW);
}

void rightMotor(int dir) {
  digitalWrite(IN3, dir > 0 ? HIGH : LOW);
  digitalWrite(IN4, dir < 0 ? HIGH : LOW);
}

void driveForward()  { leftMotor(+1); rightMotor(+1); }
void driveBackward() { leftMotor(-1); rightMotor(-1); }
void turnLeft()      { leftMotor(-1); rightMotor(+1); }
void turnRight()     { leftMotor(+1); rightMotor(-1); }
void stopMotors()    { leftMotor( 0); rightMotor( 0); }

// =====================================================================
//                         SENSOR HELPERS
// =====================================================================
struct SensorReadings {
  int s1, s2, s3, s4, s5;   // raw digitalRead for each line sensor
  int near_;                // raw digitalRead for the NEAR / proximity output
};

SensorReadings readSensors() {
  SensorReadings r;
  r.s1    = digitalRead(S1);
  r.s2    = digitalRead(S2);
  r.s3    = digitalRead(S3);
  r.s4    = digitalRead(S4);
  r.s5    = digitalRead(S5);
  r.near_ = digitalRead(NEAR);
  return r;
}

inline bool onLine(int v)        { return v == LINE_ACTIVE_LEVEL; }
inline bool obstacleClose(int v) { return v == OBSTACLE_ACTIVE_LEVEL; }

// =====================================================================
//                    CAMERA I2C CLIENT (read color)
// =====================================================================
struct CamColor {
  uint8_t dominant;     // 0=none, 1=red, 2=green, 3=black
  uint8_t red;          // %
  uint8_t green;        // %
  uint8_t black;        // %
  uint8_t confidence;   // %
  bool    valid;        // true if last poll succeeded
};

CamColor lastCam = {0, 0, 0, 0, 0, false};

const char* colorName(uint8_t code) {
  switch (code) {
    case 1: return "red";
    case 2: return "green";
    case 3: return "black";
    default: return "none";
  }
}

// Read the camera's "summary" register (0x00).
// Returns CamColor{ valid=false } when the camera is not responding (wrong
// address, broken wire, missing common ground, etc.) - keep an eye on the
// serial log: `cam=??` means the master got nothing back.
CamColor pollCamera() {
  CamColor c = {0, 0, 0, 0, 0, false};

  // Step 1: tell the slave which register we want.
  Wire.beginTransmission(CAM_I2C_ADDR);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission() != 0) return c;    // ack failed

  // Step 2: read 5 bytes of detection back.
  uint8_t got = Wire.requestFrom((uint8_t)CAM_I2C_ADDR, (uint8_t)5);
  if (got != 5) return c;

  c.dominant   = Wire.read();
  c.red        = Wire.read();
  c.green      = Wire.read();
  c.black      = Wire.read();
  c.confidence = Wire.read();
  c.valid      = true;
  return c;
}

// =====================================================================
//                              SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("== ESP32 MCU bot ==");

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  pinMode(S1, INPUT);   pinMode(S2, INPUT);
  pinMode(S3, INPUT);   pinMode(S4, INPUT);
  pinMode(S5, INPUT);   pinMode(NEAR, INPUT);

  // I2C master on default ESP32 pins (21=SDA, 22=SCL).
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, (uint32_t)I2C_FREQ_HZ);
  Serial.printf("I2C master ready, polling camera @0x%02X on SDA=%d SCL=%d\n",
                CAM_I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN);

  if (ENABLE_BOOT_MOTOR_TEST) {
    Serial.println("Motor self-test starting...");
    Serial.println(" -> forward 600 ms");
    driveForward();  delay(600); stopMotors(); delay(250);
    Serial.println(" -> backward 600 ms");
    driveBackward(); delay(600); stopMotors(); delay(250);
    Serial.println(" -> left  600 ms");
    turnLeft();      delay(600); stopMotors(); delay(250);
    Serial.println(" -> right 600 ms");
    turnRight();     delay(600); stopMotors(); delay(400);
    Serial.println("Motor self-test done.");
  }

  Serial.print("Line follow mode: ");
  Serial.println(ENABLE_LINE_FOLLOW ? "ENABLED" : "disabled (sensor-monitor only)");
  Serial.print("Color reaction:   ");
  Serial.println(ENABLE_COLOR_REACTION ? "ENABLED" : "disabled");
}

// =====================================================================
//                              LOOP
// =====================================================================
void loop() {
  SensorReadings s = readSensors();

  // ---- poll camera over I2C every CAM_POLL_MS ----
  static uint32_t lastPoll = 0;
  if (millis() - lastPoll > CAM_POLL_MS) {
    lastPoll = millis();
    lastCam = pollCamera();
  }

  // ---- periodic serial dump (sensors + camera color) ----
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > SERIAL_PRINT_MS) {
    lastPrint = millis();
    Serial.printf("S1=%d S2=%d S3=%d S4=%d S5=%d  NEAR=%d  line=[%c%c%c%c%c] obst=%c | cam=%s",
                  s.s1, s.s2, s.s3, s.s4, s.s5, s.near_,
                  onLine(s.s1) ? '#' : '.',
                  onLine(s.s2) ? '#' : '.',
                  onLine(s.s3) ? '#' : '.',
                  onLine(s.s4) ? '#' : '.',
                  onLine(s.s5) ? '#' : '.',
                  obstacleClose(s.near_) ? 'X' : '.',
                  lastCam.valid ? colorName(lastCam.dominant) : "??");
    if (lastCam.valid) {
      Serial.printf(" R=%u G=%u K=%u conf=%u\n",
                    lastCam.red, lastCam.green, lastCam.black, lastCam.confidence);
    } else {
      Serial.println();
    }
  }

  // ---- optional: react to color seen by the camera ----
  // Example mapping: red -> stop, green -> drive forward, anything else -> defer
  // to the line follower or stop. Tweak this block to taste.
  if (ENABLE_COLOR_REACTION && lastCam.valid && lastCam.confidence >= 5) {
    if (lastCam.dominant == 1) {        // red
      stopMotors();
      return;
    }
    if (lastCam.dominant == 2) {        // green
      driveForward();
      return;
    }
    // dominant == 3 (black) or 0 (none): fall through to line follower / stop.
  }

  if (!ENABLE_LINE_FOLLOW) {
    stopMotors();
    return;
  }

  // Stop if something is right in front of us.
  if (obstacleClose(s.near_)) {
    stopMotors();
    return;
  }

  // Simple 5-channel bang-bang line follower.
  // Assumes S1 is leftmost on the chassis and S5 is rightmost (swap macros if reversed).
  bool L1 = onLine(s.s1), L2 = onLine(s.s2), L3 = onLine(s.s3),
       L4 = onLine(s.s4), L5 = onLine(s.s5);

  if (L3 && !L1 && !L5) {
    driveForward();                  // line is centred
  } else if (L1 || L2) {
    turnLeft();                      // line drifted to the left
  } else if (L4 || L5) {
    turnRight();                     // line drifted to the right
  } else {
    stopMotors();                    // lost the line entirely
  }
}
