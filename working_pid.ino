#include "RoboClaw.h"

#define address 0x80

HardwareSerial RoboSerial(1);
RoboClaw roboclaw(&RoboSerial, 10000);

void setup() {
  Serial.begin(38400);
  Serial.println("Waiting for power to the controller...");
  delay(15000); // let Serial settle before printing

  RoboSerial.begin(
    38400,
    SERIAL_8N1,
    D8,   // RX
    D9    // TX
  );
  // DO NOT call roboclaw.begin() — RoboSerial is already initialized above

  Serial.println("Setup complete");
  roboclaw.begin(38400);

    // Try to read firmware version — simplest possible comms test
  char version[48];
  bool ok = roboclaw.ReadVersion(address, version);
  if (ok) {
    Serial.print("RoboClaw found! Version: ");
    Serial.println(version);
  } else {
    Serial.println("RoboClaw NOT responding — comms failure");
  }
}

void loop() {
  uint8_t status;
  bool valid;
  int32_t motor_1_count;

  // ── Read original encoder ──────────────────────────────────────────
  motor_1_count = roboclaw.ReadEncM1(address, &status, &valid);
  if (valid) {
    Serial.print("Original: ");
    Serial.println(motor_1_count);
  } else {
    Serial.println("Original: READ FAILED");
  }
  delay(2000);

  // ── Set encoder to 1000 then read back ────────────────────────────
  bool setOk = roboclaw.SetEncM1(address, 1000);
  Serial.print("SetEncM1 success: ");
  Serial.println(setOk ? "YES" : "NO");

  motor_1_count = roboclaw.ReadEncM1(address, &status, &valid);
  if (valid) {
    Serial.print("After set: ");
    Serial.println(motor_1_count);
  } else {
    Serial.println("After set: READ FAILED");
  }
  // delay(2000);

  // ── Run motor briefly and read speed ─────────────────────────────
  // roboclaw.ForwardM1(address, 25);
  // delay(500);

  uint8_t spd_status;
  bool spd_valid;
  int32_t motor_1_speed = roboclaw.ReadSpeedM1(address, &spd_status, &spd_valid);
  if (spd_valid) {
    Serial.print("Motor speed: ");
    Serial.println(motor_1_speed);
  } else {
    Serial.println("Speed: READ FAILED");
  }

  roboclaw.ForwardM1(address, 0);
  // delay(2000);

  // ── Reset encoders and verify ─────────────────────────────────────
  bool resetOk = roboclaw.ResetEncoders(address);
  Serial.print("ResetEncoders success: ");
  Serial.println(resetOk ? "YES" : "NO");

  motor_1_count = roboclaw.ReadEncM1(address, &status, &valid);
  if (valid) {
    Serial.print("After reset: ");
    Serial.println(motor_1_count);
  } else {
    Serial.println("After reset: READ FAILED");
  }
  // delay(2000);
}