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
//   [0] dominant code: 0=none, 1=red, 2=green, 3=yellow
//   [1] red %      [2] green %    [3] yellow %    [4] confidence %
//
// Servo (signal pin only)
//   SIG -> GPIO23
//   V+  -> battery + (5 V); not from ESP32 3V3
//   GND -> battery -, common with ESP32 GND
// Behaviour: when the BFD1000 NEAR pin says a ball is in front of us we
// stop, ask the camera what colour it is, and sweep the servo for red or
// yellow. Green or no colour => no actuation.
// =====================================================================

#include <Wire.h>
#include <ESP32Servo.h>

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

// ---------- Servo (ESP32Servo library) ----------
#define SERVO_PIN              23
#define SERVO_REST_ANGLE        0    // deg, idle position
#define SERVO_ACTUATED_ANGLE   90    // deg, "fire" position
#define SERVO_HOLD_MS         500    // hold at actuated angle
#define SERVO_RETURN_MS       250    // settle back at rest
#define NEAR_COOLDOWN_MS     1500    // ignore NEAR for this long after a sweep

// ---------- behaviour switches ----------
const bool ENABLE_BOOT_MOTOR_TEST = true;   // drive a short sequence on power-up
const bool ENABLE_LINE_FOLLOW     = true;   // primary mode: follow the 3.5 cm black line
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
//                  SENSOR + CAMERA STATE (globals)
// =====================================================================
// NOTE: These helpers fill global state instead of returning user-defined
// structs by value. This avoids the Arduino IDE 2.x auto-prototype bug,
// where the IDE inserts function prototypes before struct definitions
// and then complains "'SensorReadings' does not name a type".

// Line / obstacle sensor snapshot.
int  sensS1 = 0, sensS2 = 0, sensS3 = 0, sensS4 = 0, sensS5 = 0;
int  sensNEAR = 0;

void readSensors() {
  sensS1   = digitalRead(S1);
  sensS2   = digitalRead(S2);
  sensS3   = digitalRead(S3);
  sensS4   = digitalRead(S4);
  sensS5   = digitalRead(S5);
  sensNEAR = digitalRead(NEAR);
}

inline bool onLine(int v)        { return v == LINE_ACTIVE_LEVEL; }
inline bool obstacleClose(int v) { return v == OBSTACLE_ACTIVE_LEVEL; }

// Latest result from the camera over I2C.
uint8_t camDominant   = 0;   // 0=none, 1=red, 2=green, 3=yellow
uint8_t camRed        = 0;   // %
uint8_t camGreen      = 0;   // %
uint8_t camYellow     = 0;   // %
uint8_t camConfidence = 0;   // %
bool    camValid      = false;

const char* colorName(uint8_t code) {
  switch (code) {
    case 1: return "red";
    case 2: return "green";
    case 3: return "yellow";
    default: return "none";
  }
}

void pollCamera() {
  camValid = false;
  Wire.beginTransmission(CAM_I2C_ADDR);
  Wire.write((uint8_t)0x00);                    // request "summary" register
  uint8_t tx = Wire.endTransmission();
  if (tx != 0) {
    // #region agent log
    // Slave didn't ACK or bus error. tx codes (Arduino ESP32 Wire):
    //   2 = NACK on address (no slave / wrong addr / wrong pins)
    //   3 = NACK on data
    //   4 = other error
    //   5 = timeout (often: no pull-ups / missing GND)
    // Hypotheses tested: H1, H2, H3, H4.
    static uint32_t lastTxErr = 0;
    if (millis() - lastTxErr > 2000) {
      lastTxErr = millis();
      Serial.printf("DBG|poll_tx_err|tx=%u\n", tx);
    }
    // #endregion
    return;
  }

  uint8_t got = Wire.requestFrom((uint8_t)CAM_I2C_ADDR, (uint8_t)5);
  if (got != 5) {
    // #region agent log
    // Address ACKed but slave returned wrong byte count.
    // Hypothesis tested: H5 (camera onI2CRequest broken / wrong pubBuf size).
    static uint32_t lastGotErr = 0;
    if (millis() - lastGotErr > 2000) {
      lastGotErr = millis();
      Serial.printf("DBG|poll_got_err|got=%u\n", got);
    }
    // #endregion
    return;
  }

  camDominant   = Wire.read();
  camRed        = Wire.read();
  camGreen      = Wire.read();
  camYellow     = Wire.read();
  camConfidence = Wire.read();
  camValid      = true;

  // #region agent log
  // First successful poll. Print once so we know the exact moment the link
  // came up and what the camera reported on that frame.
  static bool announcedOk = false;
  if (!announcedOk) {
    announcedOk = true;
    Serial.printf("DBG|poll_first_ok|dom=%u|R=%u|G=%u|Y=%u|conf=%u\n",
                  camDominant, camRed, camGreen, camYellow, camConfidence);
  }
  // #endregion
}

// =====================================================================
//                        SERVO (ESP32Servo)
// =====================================================================
Servo gServo;
uint32_t lastSweepEndMs = 0;   // when the last actuation cycle finished

void servoInit() {
  // The ESP32Servo library wants a periodHertz set before attach() so the
  // PWM timing matches a hobby servo (50 Hz, ~1-2 ms pulse).
  gServo.setPeriodHertz(50);
  gServo.attach(SERVO_PIN, 500, 2400);   // microseconds for 0..180 deg
  gServo.write(SERVO_REST_ANGLE);
}

void servoActuate() {
  gServo.write(SERVO_ACTUATED_ANGLE);
  delay(SERVO_HOLD_MS);
  gServo.write(SERVO_REST_ANGLE);
  delay(SERVO_RETURN_MS);
  lastSweepEndMs = millis();
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

  // #region agent log
  // Probe the I2C bus for any responding slave. Prints a hit per address that
  // ACKs, plus a summary line so we can tell apart "no bus" / "bus but wrong
  // address" / "camera at 0x52 OK".
  // Tests H1 (camera pins wrong), H2 (camera firmware not flashed), H3 (no
  // common ground), H4 (robot Wire.begin failed/wrong wires).
  Serial.println("DBG|i2c_scan_start");
  {
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      uint8_t err = Wire.endTransmission();
      if (err == 0) {
        Serial.printf("DBG|i2c_scan_hit|addr=0x%02X\n", addr);
        found++;
      }
    }
    Serial.printf("DBG|i2c_scan_done|found=%u|expected_cam=0x%02X\n",
                  found, CAM_I2C_ADDR);
  }
  // #endregion

  // Servo on GPIO 23, parked at rest.
  servoInit();
  Serial.printf("Servo on GPIO%d at %d deg (actuates to %d deg on red/yellow ball)\n",
                SERVO_PIN, SERVO_REST_ANGLE, SERVO_ACTUATED_ANGLE);

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
  Serial.println("Ball reaction:    NEAR + camera color (red/yellow -> servo)");
}

// =====================================================================
//                              LOOP
// =====================================================================
void loop() {
  readSensors();

  // ---- poll camera over I2C every CAM_POLL_MS ----
  static uint32_t lastPoll = 0;
  if (millis() - lastPoll > CAM_POLL_MS) {
    lastPoll = millis();
    pollCamera();
  }

  // ---- periodic serial dump (sensors + camera color) ----
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > SERIAL_PRINT_MS) {
    lastPrint = millis();
    Serial.printf("S1=%d S2=%d S3=%d S4=%d S5=%d  NEAR=%d  line=[%c%c%c%c%c] obst=%c | cam=%s",
                  sensS1, sensS2, sensS3, sensS4, sensS5, sensNEAR,
                  onLine(sensS1) ? '#' : '.',
                  onLine(sensS2) ? '#' : '.',
                  onLine(sensS3) ? '#' : '.',
                  onLine(sensS4) ? '#' : '.',
                  onLine(sensS5) ? '#' : '.',
                  obstacleClose(sensNEAR) ? 'X' : '.',
                  camValid ? colorName(camDominant) : "??");
    if (camValid) {
      Serial.printf(" R=%u G=%u Y=%u conf=%u\n",
                    camRed, camGreen, camYellow, camConfidence);
    } else {
      Serial.println();
    }
  }

  // ---- ball detected by NEAR? ----
  // When the BFD1000 NEAR pin is active the bot is parked in front of a
  // ball. Stop, ask the camera what colour it is *right now*, and decide.
  // A short cooldown after every action prevents re-triggering on the
  // same ball while we drive past it.
  if (obstacleClose(sensNEAR) &&
      millis() - lastSweepEndMs > NEAR_COOLDOWN_MS) {
    stopMotors();
    pollCamera();   // force-refresh so we act on the current frame
    if (camValid && (camDominant == 1 || camDominant == 3)) {
      Serial.printf("Ball: %s -> ACTUATE\n", colorName(camDominant));
      servoActuate();
    } else {
      Serial.printf("Ball: %s -> skip\n",
                    camValid ? colorName(camDominant) : "??");
      lastSweepEndMs = millis();   // arm the cooldown anyway
    }
    return;
  }

  // ---- line follow on the 3.5 cm black line ----
  if (!ENABLE_LINE_FOLLOW) {
    stopMotors();
    return;
  }

  // Simple 5-channel bang-bang line follower.
  // Assumes S1 is leftmost on the chassis and S5 is rightmost (swap macros if reversed).
  bool L1 = onLine(sensS1), L2 = onLine(sensS2), L3 = onLine(sensS3),
       L4 = onLine(sensS4), L5 = onLine(sensS5);

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
