#include "RoboClaw.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>

//#include <Basicmicro.h>
//Basicmicro roboclaw(&Serial1, 10000);

#define TCAADDR 0x70

#define ROBOCLAW_ADDR 0x80

HardwareSerial RoboSerial(1);  // UART1
RoboClaw roboclaw(&RoboSerial, 10000);

const int LINACTSWITCH = D2;
const int BACKSWITCH = D3;

const long PPR = 103.8;
const int MM_PER_REV = 8;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_ICM20948 icm;

unsigned long last_time = 0;
unsigned long delta_time;
bool safetyTriggered;

// ── PID gains (loaded from RoboClaw) ──────────────────────────────────
float    KP, KI, KD;
uint32_t KiMax, DeadZone, PosMin, PosMax;

// ── PID state ─────────────────────────────────────────────────────────
float    integral   = 0.0f;
int32_t  lastError  = 0;
uint32_t lastTimeMs = 0;
int32_t  targetEnc  = 0;
bool     pidActive  = false;

// ── Tuning ────────────────────────────────────────────────────────────
const int32_t MAX_SPEED = 3000;
const int32_t MIN_SPEED = 50;

// ─────────────────────────────────────────────────────────────────────
void moveRelative(int32_t ticks) {
  uint8_t status;
  bool valid;
  int32_t current = roboclaw.ReadEncM1(ROBOCLAW_ADDR, &status, &valid);
  if (!valid) {
    Serial.println("Move aborted: encoder read failed");
    return;
  }

  targetEnc  = current + ticks;
  integral   = 0.0f;
  lastError  = 0;
  lastTimeMs = millis();
  pidActive  = true;

  Serial.print("Moving to encoder: ");
  Serial.println(targetEnc);
}

// ─────────────────────────────────────────────────────────────────────
void updatePID() {
  if (!pidActive) return;

  // ── Safety ────────────────────────────────────────────────────────
  if (digitalRead(LINACTSWITCH) == LOW || digitalRead(BACKSWITCH) == LOW) {
    roboclaw.SpeedM1(ROBOCLAW_ADDR, 0);
    pidActive = false;
    integral  = 0.0f;
    Serial.println("Safety triggered — stopped");
    return;
  }

  // ── Read encoder ──────────────────────────────────────────────────
  uint8_t status;
  bool valid;
  int32_t current = roboclaw.ReadEncM1(ROBOCLAW_ADDR, &status, &valid);
  if (!valid) return;

  // ── Time delta ────────────────────────────────────────────────────
  uint32_t now = millis();
  float dt = (now - lastTimeMs) / 1000.0f;
  if (dt <= 0.0f || dt < 0.001f) return;
  lastTimeMs = now;

  // ── PID math ──────────────────────────────────────────────────────
  int32_t error = targetEnc - current;

  integral += error * dt;
  integral  = constrain(integral, -(float)KiMax, (float)KiMax);

  float derivative = (float)(error - lastError) / dt;
  lastError = error;

  float output = (KP * error) + (KI * integral) + (KD * derivative);

  // ── Within deadzone: hold ─────────────────────────────────────────
  if (abs(error) <= (int32_t)DeadZone) {
    int32_t holdSpeed = (int32_t)constrain(output, -(float)MIN_SPEED, (float)MIN_SPEED);
    roboclaw.SpeedM1(ROBOCLAW_ADDR, holdSpeed);
    return;
  }

  // ── Outside deadzone: drive ───────────────────────────────────────
  int32_t speed = (int32_t)constrain(output, -(float)MAX_SPEED, (float)MAX_SPEED);

  if (speed > 0 && speed <  MIN_SPEED) speed =  MIN_SPEED;
  if (speed < 0 && speed > -MIN_SPEED) speed = -MIN_SPEED;

  roboclaw.SpeedM1(ROBOCLAW_ADDR, speed);
}

// ─────────────────────────────────────────────────────────────────────
// Returns true once motor is within deadzone of target
// ─────────────────────────────────────────────────────────────────────
bool atTarget() {
  uint8_t status;
  bool valid;
  int32_t current = roboclaw.ReadEncM1(ROBOCLAW_ADDR, &status, &valid);
  if (!valid) return false;
  return abs(targetEnc - current) <= (int32_t)DeadZone;
}

// ─────────────────────────────────────────────────────────────────────
// Runs PID loop until target reached or timeout (ms)
// ─────────────────────────────────────────────────────────────────────
void runUntilDone(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (!atTarget()) {
    if (millis() - start > timeoutMs) {
      Serial.println("WARNING: move timed out");
      roboclaw.SpeedM1(ROBOCLAW_ADDR, 0);
      pidActive = false;
      return;
    }
    updatePID();
  }
  Serial.println("Target reached");
}

int32_t getTicksFromMM(int mm) {
  return (int32_t) lround((PPR / MM_PER_REV) * mm);
}

void delayWithHoldingPID(int ms) {
  uint32_t holdStart = millis();
  while (millis() - holdStart < ms) {
    updatePID();
  }
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  pinMode(LINACTSWITCH, INPUT_PULLUP);
  pinMode(BACKSWITCH, INPUT_PULLUP);
  Serial.begin(38400);
  Serial.println("Waiting for motor controller power supply");
  delay(15000);

  RoboSerial.begin(
    38400,
    SERIAL_8N1,
    D8,  // RX  (optional)
    D9   // TX  (required)
  );

  Wire.begin();

  if (!icm.begin_I2C()) {
    Serial.println("ICM20948 not found");
    while (1)
      ;
  }
  Serial.println("ICM20948 OK");

  // Initialize sensors on each port
  for (uint8_t i = 0; i < 8; i++) {
    tcaselect(i);
    if (!lox.begin()) {
      Serial.print("Sensor not found on port ");
      Serial.println(i);
    } else {
      Serial.print("Sensor OK on port ");
      Serial.println(i);
    }
  }

  // ── Load PID gains ──────────────────────────────────────────────
  bool ok = roboclaw.ReadM1PositionPID(ROBOCLAW_ADDR, KP, KI, KD, KiMax, DeadZone, PosMin, PosMax);
  if (DeadZone == 0) DeadZone = 1;
  if (ok) {
    Serial.println("PID gains loaded:");
    Serial.print("  KP: ");       Serial.println(KP);
    Serial.print("  KI: ");       Serial.println(KI);
    Serial.print("  KD: ");       Serial.println(KD);
    Serial.print("  KiMax: ");    Serial.println(KiMax);
    Serial.print("  DeadZone: "); Serial.println(DeadZone);

    // Scale down gains for ESP32 loop rate vs RoboClaw internal rate
    KP = KP * 0.13f;
    KI = KI * 0.01f;
    KD = KD * 0.00f;

    Serial.println("Scaled gains (x0.1):");
    Serial.print("  KP: "); Serial.println(KP);
    Serial.print("  KI: "); Serial.println(KI);
    Serial.print("  KD: "); Serial.println(KD);
  } else {
    Serial.println("FAILED to load PID gains — halting");
    while (1);
  }


  safetyTriggered = digitalRead(BACKSWITCH) == LOW;
  while (!safetyTriggered) {
    Serial.println("Setting Up...");
    roboclaw.BackwardM1(ROBOCLAW_ADDR, 25);
    safetyTriggered = digitalRead(BACKSWITCH) == LOW;
  }
  roboclaw.BackwardM1(ROBOCLAW_ADDR, 0);
  Serial.println("Setup complete");
  roboclaw.ResetEncoders(ROBOCLAW_ADDR);
  delay(2000);
    // Drive forward slowly until switch releases
  while (digitalRead(BACKSWITCH) == LOW) {
    roboclaw.ForwardM1(ROBOCLAW_ADDR, 15);
  }
  roboclaw.ForwardM1(ROBOCLAW_ADDR, 0);
  delay(2000);
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  safetyTriggered = (digitalRead(LINACTSWITCH) == LOW || digitalRead(BACKSWITCH) == LOW);
  while (!safetyTriggered) {
    safetyTriggered = (digitalRead(LINACTSWITCH) == LOW || digitalRead(BACKSWITCH) == LOW);
    for (uint8_t i = 0; i < 8; i++) {
      if (i == 3 || i == 4 || i == 5) continue;
      tcaselect(i);

      lox.rangingTest(&measure, false);

      Serial.print("Port ");
      Serial.print(i);
      Serial.print(": ");

      if (measure.RangeStatus != 4) {
        Serial.print(measure.RangeMilliMeter);
        Serial.println(" mm");
      } else {
        Serial.println("Out of range");
      }
    }

    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);
    Serial.println("");
    Serial.print("Temperature ");
    Serial.print(temp.temperature);
    Serial.println(" deg C");

    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("Accel X: ");
    Serial.print(accel.acceleration.x);
    Serial.print("\tY: ");
    Serial.print(accel.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2 ");

    Serial.print("Mag X: ");
    Serial.print(mag.magnetic.x);
    Serial.print(" \tY: ");
    Serial.print(mag.magnetic.y);
    Serial.print(" \tZ: ");
    Serial.print(mag.magnetic.z);
    Serial.println(" uT");

    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("Gyro X: ");
    Serial.print(gyro.gyro.x);
    Serial.print(" \tY: ");
    Serial.println(gyro.gyro.y);
    Serial.println("");

    Serial.println("=== Moving FORWARD 7cm ===");
    moveRelative(getTicksFromMM(70));
    runUntilDone(10000); // 10 sec timeout

    delayWithHoldingPID(2000);

    Serial.println("=== Moving BACKWARD 7cm ===");
    moveRelative(getTicksFromMM(-70));
    runUntilDone(10000);

    unsigned long now = millis();
    delta_time = now - last_time;
    last_time = now;
    Serial.print("Timer: ");
    Serial.print(delta_time);
    Serial.println(" ms");
    Serial.println("----\n");
    delayWithHoldingPID(1000);
  }
  roboclaw.ForwardM1(ROBOCLAW_ADDR, 0);
  pidActive = false;
}
