#include "RoboClaw.h"

#define ROBOCLAW_ADDR 0x80

HardwareSerial RoboSerial(1);  // UART1
RoboClaw roboclaw(&RoboSerial, 10000);

const int LINACTSWITCH = D2;
const int BACKSWITCH = D3;

const long PPR = 103.8;
const int MM_PER_REV = 8;

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
const int32_t MAX_SPEED = 10000;
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

bool backSwitchTriggered()   { return digitalRead(BACKSWITCH)   == LOW; }
bool linactSwitchTriggered() { return digitalRead(LINACTSWITCH) == LOW; }


void setup() {
  Serial.begin(38400);
  delay(500);

  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║   LINEAR ACTUATOR UNIT TEST           ║");
  Serial.println("╚══════════════════════════════════════╝");
  Serial.println("Waiting 15s for motor controller...");
  delay(15000);

  RoboSerial.begin(38400, SERIAL_8N1, D8, D9);

  // Sanity check: make sure we can talk to the RoboClaw
  uint16_t version;
  if (!roboclaw.ReadVersion(ROBOCLAW_ADDR, &version)) {
    Serial.println("[FAIL] Cannot communicate with RoboClaw — halting");
    state = TEST_FAILED;
    return;
  }

  Serial.println("[OK] RoboClaw detected");
  if (linactSwitchTriggered()) {
    Serial.println("[WARN] LINACTSWITCH already triggered at startup");
  }

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

}