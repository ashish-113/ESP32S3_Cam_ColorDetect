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
// =====================================================================

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

// ---------- behaviour switches ----------
const bool ENABLE_BOOT_MOTOR_TEST = true;   // drive a short sequence on power-up
const bool ENABLE_LINE_FOLLOW     = false;  // flip to true after the wiring is verified
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
}

// =====================================================================
//                              LOOP
// =====================================================================
void loop() {
  SensorReadings s = readSensors();

  // Periodic serial dump so you can verify wiring / orientation.
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > SERIAL_PRINT_MS) {
    lastPrint = millis();
    Serial.printf("S1=%d S2=%d S3=%d S4=%d S5=%d  NEAR=%d  line=[%c%c%c%c%c] obst=%c\n",
                  s.s1, s.s2, s.s3, s.s4, s.s5, s.near_,
                  onLine(s.s1) ? '#' : '.',
                  onLine(s.s2) ? '#' : '.',
                  onLine(s.s3) ? '#' : '.',
                  onLine(s.s4) ? '#' : '.',
                  onLine(s.s5) ? '#' : '.',
                  obstacleClose(s.near_) ? 'X' : '.');
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
