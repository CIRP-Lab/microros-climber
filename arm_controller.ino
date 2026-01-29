#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>

#include "RoboClaw.h"

#define TCAADDR 0x70

#define ROBOCLAW_ADDR 0x80
HardwareSerial RoboSerial(1);   // UART1
RoboClaw roboclaw(&RoboSerial, 10000);

const int LINACTSWITCH = D2;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_ICM20948 icm;

unsigned long last_time = 0;
unsigned long delta_time;

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(38400);   // USB debug
  pinMode(LINACTSWITCH, INPUT_PULLUP);

  // RoboClaw UART
  RoboSerial.begin(
    38400,
    SERIAL_8N1,
    D8,   // RX  (optional)
    D9    // TX  (required)
  );

  roboclaw.begin(38400);
  Wire.begin();

  if (!icm.begin_I2C()) {
    Serial.println("ICM20948 not found");
    while (1);
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
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  // bool safetyTriggered = (digitalRead(LINACTSWITCH) == HIGH);
  // while (!safetyTriggered) {
  //   bool safetyTriggered = (digitalRead(LINACTSWITCH) == HIGH);
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
    Serial.print(gyro.gyro.y);

    unsigned long now = millis();
    delta_time = now - last_time;
    last_time = now;
    Serial.print("Timer: ");
    Serial.print(delta_time);
    Serial.println(" ms");
    Serial.println("----");

    // roboclaw.BackwardM1(ROBOCLAW_ADDR, 40);
    // delay(2000);
    roboclaw.ForwardM1(ROBOCLAW_ADDR, 100);
  // }
}  