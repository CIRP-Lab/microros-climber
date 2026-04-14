#include "RoboClaw.h"

#define address 0x80

const long PPR        = 103.8;
const int  MM_PER_REV = 8;
const int  MOVE_MM    = 70;  // 7 cm

HardwareSerial RoboSerial(1);
RoboClaw roboclaw(&RoboSerial, 10000);

const int LINACTSWITCH = D2;
const int BACKSWITCH   = D3;

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
  int32_t current = roboclaw.ReadEncM1(address, &status, &valid);
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
    roboclaw.SpeedM1(address, 0);
    pidActive = false;
    integral  = 0.0f;
    Serial.println("Safety triggered — stopped");
    return;
  }

  // ── Read encoder ──────────────────────────────────────────────────
  uint8_t status;
  bool valid;
  int32_t current = roboclaw.ReadEncM1(address, &status, &valid);
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
    roboclaw.SpeedM1(address, holdSpeed);
    return;
  }

  // ── Outside deadzone: drive ───────────────────────────────────────
  int32_t speed = (int32_t)constrain(output, -(float)MAX_SPEED, (float)MAX_SPEED);

  if (speed > 0 && speed <  MIN_SPEED) speed =  MIN_SPEED;
  if (speed < 0 && speed > -MIN_SPEED) speed = -MIN_SPEED;

  roboclaw.SpeedM1(address, speed);
}

// ─────────────────────────────────────────────────────────────────────
// Returns true once motor is within deadzone of target
// ─────────────────────────────────────────────────────────────────────
bool atTarget() {
  uint8_t status;
  bool valid;
  int32_t current = roboclaw.ReadEncM1(address, &status, &valid);
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
      roboclaw.SpeedM1(address, 0);
      pidActive = false;
      return;
    }
    updatePID();
  }
  Serial.println("Target reached");
}

// ─────────────────────────────────────────────────────────────────────
void setup() {
  pinMode(LINACTSWITCH, INPUT_PULLUP);
  pinMode(BACKSWITCH,   INPUT_PULLUP);

  Serial.begin(38400);
  delay(15000); // wait for RoboClaw to boot

  RoboSerial.begin(38400, SERIAL_8N1, D8, D9);
  delay(500);

  // ── Load PID gains ──────────────────────────────────────────────
  bool ok = roboclaw.ReadM1PositionPID(address, KP, KI, KD, KiMax, DeadZone, PosMin, PosMax);
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

  roboclaw.ResetEncoders(address);
  Serial.println("Encoders reset — starting test in 2 seconds");
  delay(2000);
}

// ─────────────────────────────────────────────────────────────────────
void loop() {
  int32_t ticks = (int32_t)lround((PPR / MM_PER_REV) * MOVE_MM);

  // ── Forward 7cm ───────────────────────────────────────────────────
  Serial.println("=== Moving FORWARD 7cm ===");
  moveRelative(ticks);
  runUntilDone(10000); // 10 sec timeout

  Serial.println("Holding for 5 seconds...");
  uint32_t holdStart = millis();
  while (millis() - holdStart < 5000) {
    updatePID(); // keep holding position during the delay
  }

  // ── Backward 7cm ──────────────────────────────────────────────────
  Serial.println("=== Moving BACKWARD 7cm ===");
  moveRelative(-ticks);
  runUntilDone(10000);

  Serial.println("Holding for 5 seconds...");
  holdStart = millis();
  while (millis() - holdStart < 5000) {
    updatePID();
  }
}