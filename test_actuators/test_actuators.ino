// ═══════════════════════════════════════════════════════════════════
//  LINEAR ACTUATOR UNIT TEST
//  Test sequence:
//    1. Home  → retract until BACKSWITCH triggers
//    2. Extend → drive forward at constant speed until LINACTSWITCH triggers
//    3. Half   → retract to the midpoint encoder count, then stop
// ═══════════════════════════════════════════════════════════════════

#include "RoboClaw.h"

// ── Hardware ────────────────────────────────────────────────────────
#define ROBOCLAW_ADDR  0x80
#define LINACTSWITCH   D2   // fully-extended limit switch (LOW = triggered)
#define BACKSWITCH     D3   // fully-retracted limit switch (LOW = triggered)

HardwareSerial RoboSerial(1);
RoboClaw roboclaw(&RoboSerial, 10000);

// ── Test parameters ─────────────────────────────────────────────────
const uint8_t HOME_SPEED   = 50;   // slow retract during homing
const uint8_t EXTEND_SPEED = 64;   // constant speed for extension test
const uint8_t CREEP_SPEED  = 40;   // slow speed when approaching midpoint

// Encoder counts recorded during the test
int32_t encAtHome     = 0;
int32_t encAtFullExt  = 0;
int32_t encMidpoint   = 0;

// ── Test states ──────────────────────────────────────────────────────
enum TestState {
  TEST_IDLE,
  TEST_HOMING,
  TEST_EXTENDING,
  TEST_RETRACTING_TO_MID,
  TEST_DONE,
  TEST_FAILED
};

TestState state = TEST_IDLE;

// ── Helpers ──────────────────────────────────────────────────────────

bool backSwitchTriggered()   { return digitalRead(BACKSWITCH)   == LOW; }
bool linactSwitchTriggered() { return digitalRead(LINACTSWITCH) == LOW; }

int32_t readEncoder() {
  uint8_t status;
  bool valid;
  int32_t enc = roboclaw.ReadEncM1(ROBOCLAW_ADDR, &status, &valid);
  if (!valid) {
    Serial.println("  [WARN] Encoder read invalid");
    return 0;
  }
  return enc;
}

void stopMotor() {
  roboclaw.ForwardM1(ROBOCLAW_ADDR, 0);
}

void logState(const char* label) {
  Serial.print("\n══ ");
  Serial.print(label);
  Serial.println(" ══");
}

// ════════════════════════════════════════════════════════════════════
void setup() {
  pinMode(LINACTSWITCH, INPUT_PULLUP);
  pinMode(BACKSWITCH,   INPUT_PULLUP);

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

  Serial.println("\nStarting test sequence...\n");
  state = TEST_HOMING;
}

// ════════════════════════════════════════════════════════════════════
void loop() {
  switch (state) {

    // ── STEP 1: HOME ──────────────────────────────────────────────
    case TEST_HOMING: {
      logState("STEP 1: Homing (retracting to BACKSWITCH)");

      if (backSwitchTriggered()) {
        Serial.println("  Already at home position");
      } else {
        Serial.println("  Retracting...");
        while (!backSwitchTriggered()) {
          if (linactSwitchTriggered()) {
            Serial.println("[FAIL] LINACTSWITCH triggered during homing — halting");
            stopMotor();
            state = TEST_FAILED;
            return;
          }
          roboclaw.BackwardM1(ROBOCLAW_ADDR, HOME_SPEED);
        }
        stopMotor();
      }

      // Creep forward until switch releases
      Serial.println("  Creeping off BACKSWITCH...");
      while (backSwitchTriggered()) {
        roboclaw.ForwardM1(ROBOCLAW_ADDR, HOME_SPEED);
      }
      stopMotor();
      delay(100);

      roboclaw.ResetEncoders(ROBOCLAW_ADDR);
      encAtHome = 0;

      Serial.print("  [PASS] Homed. Encoder zeroed at: ");
      Serial.println(encAtHome);

      delay(500);
      state = TEST_EXTENDING;
      break;
    }

    // ── STEP 2: EXTEND ───────────────────────────────────────────
    case TEST_EXTENDING: {
      logState("STEP 2: Extending at constant speed until LINACTSWITCH");

      Serial.print("  Extending at speed ");
      Serial.print(EXTEND_SPEED);
      Serial.println("...");

      uint32_t startTime = millis();
      const uint32_t EXTEND_TIMEOUT_MS = 30000;

      while (!linactSwitchTriggered()) {
        if (backSwitchTriggered()) {
          Serial.println("[FAIL] BACKSWITCH triggered during extension — halting");
          stopMotor();
          state = TEST_FAILED;
          return;
        }
        if (millis() - startTime > EXTEND_TIMEOUT_MS) {
          Serial.println("[FAIL] Extension timed out — LINACTSWITCH never triggered");
          stopMotor();
          state = TEST_FAILED;
          return;
        }
        roboclaw.ForwardM1(ROBOCLAW_ADDR, EXTEND_SPEED);
      }
      stopMotor();

      encAtFullExt = readEncoder();
      encMidpoint  = encAtFullExt / 2;

      Serial.print("  [PASS] Fully extended. Encoder: ");
      Serial.println(encAtFullExt);
      Serial.print("  Midpoint target encoder: ");
      Serial.println(encMidpoint);

      delay(500);
      state = TEST_RETRACTING_TO_MID;
      break;
    }

    // ── STEP 3: RETRACT TO MIDPOINT ──────────────────────────────
    case TEST_RETRACTING_TO_MID: {
      logState("STEP 3: Retracting to midpoint");

      if (encAtFullExt == 0) {
        Serial.println("[FAIL] Full-extension encoder is 0 — cannot compute midpoint");
        state = TEST_FAILED;
        return;
      }

      const int32_t DEADBAND = 5;

      Serial.print("  Retracting to encoder ");
      Serial.print(encMidpoint);
      Serial.println("...");

      uint32_t startTime = millis();
      const uint32_t RETRACT_TIMEOUT_MS = 30000;

      while (true) {
        if (backSwitchTriggered()) {
          Serial.println("[FAIL] BACKSWITCH triggered before reaching midpoint");
          stopMotor();
          state = TEST_FAILED;
          return;
        }
        if (millis() - startTime > RETRACT_TIMEOUT_MS) {
          Serial.println("[FAIL] Midpoint retract timed out");
          stopMotor();
          state = TEST_FAILED;
          return;
        }

        int32_t current = readEncoder();
        int32_t error   = current - encMidpoint;

        if (abs(error) <= DEADBAND) {
          stopMotor();
          Serial.print("  [PASS] Reached midpoint. Final encoder: ");
          Serial.println(current);
          break;
        }

        // Slow down when close
        if (abs(error) < 50) {
          roboclaw.BackwardM1(ROBOCLAW_ADDR, CREEP_SPEED);
        } else {
          roboclaw.BackwardM1(ROBOCLAW_ADDR, HOME_SPEED);
        }
      }

      state = TEST_DONE;
      break;
    }

    // ── DONE ─────────────────────────────────────────────────────
    case TEST_DONE: {
      Serial.println("\n╔══════════════════════════════════════╗");
      Serial.println("║         TEST COMPLETE — PASS          ║");
      Serial.println("╚══════════════════════════════════════╝");
      Serial.print("  Home encoder:       "); Serial.println(encAtHome);
      Serial.print("  Full ext encoder:   "); Serial.println(encAtFullExt);
      Serial.print("  Midpoint encoder:   "); Serial.println(encMidpoint);
      Serial.print("  Full stroke ticks:  "); Serial.println(encAtFullExt - encAtHome);
      stopMotor();
      while (true) delay(1000);
    }

    // ── FAILED ───────────────────────────────────────────────────
    case TEST_FAILED: {
      Serial.println("\n╔══════════════════════════════════════╗");
      Serial.println("║         TEST FAILED — HALTED          ║");
      Serial.println("╚══════════════════════════════════════╝");
      stopMotor();
      while (true) delay(1000);
    }

    default:
      break;
  }
}