#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>

//#include <Basicmicro.h>
//Basicmicro roboclaw(&Serial1, 10000);

#define TCAADDR 0x70

#define ROBOCLAW_ADDR 0x80

#include "RoboClaw.h"
HardwareSerial RoboSerial(1);   // UART1
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
bool valid;
uint8_t status;
    
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(38400);   // USB debug
  pinMode(LINACTSWITCH, INPUT_PULLUP);
  pinMode(BACKSWITCH, INPUT_PULLUP);

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
  roboclaw.SetM1MaxCurrent(ROBOCLAW_ADDR, 7000);
  roboclaw.SetM1PositionPID(
    ROBOCLAW_ADDR,
      1,   // kp
      0.5,   // ki
      0.25,  // kd
      2000,  // kiMax
      2,     // deadzone
      0,     // min
      127    // max
   );
  roboclaw.SetM1VelocityPID(ROBOCLAW_ADDR,1,0.5,0.25,44000);

  safetyTriggered = digitalRead(BACKSWITCH) == LOW;
  while (!safetyTriggered) {
    roboclaw.BackwardM1(ROBOCLAW_ADDR, 25);
    safetyTriggered = digitalRead(BACKSWITCH) == LOW;
  }
  roboclaw.BackwardM1(ROBOCLAW_ADDR, 0);
  Serial.println("Setup complete");
  roboclaw.ResetEncoders(ROBOCLAW_ADDR);
  int32_t enc = roboclaw.ReadEncM1(ROBOCLAW_ADDR, &status, &valid);
  roboclaw.SpeedAccelDistanceM1(ROBOCLAW_ADDR,100,100, 10, 1);
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

//     roboclaw.BackwardM1(ROBOCLAW_ADDR, 5000);
  // delay(2000);
//   roboclaw.ForwardM1(ROBOCLAW_ADDR, 500);

    int32_t enc = roboclaw.ReadEncM1(ROBOCLAW_ADDR, &status, &valid);
    if (valid) {
      Serial.print("encoder 1 parts moved: ");
      Serial.println(enc);
      Serial.print("Encoder status: \n");
      Serial.println(status);
    }
    else {
      Serial.println("Encoder reading failed\n");
    }

    int32_t encoders_to_adv = (int32_t) lround((PPR / MM_PER_REV) * 15);  
    Serial.print("encoder to adv: ");
    Serial.println(encoders_to_adv);
    
    roboclaw.SpeedAccelDistanceM1(ROBOCLAW_ADDR,100,100, 10, 1);

    unsigned long now = millis();
    delta_time = now - last_time;
    last_time = now;
    Serial.print("Timer: ");
    Serial.print(delta_time);
    Serial.println(" ms");
    Serial.println("----\n");
  }
  roboclaw.ForwardM1(ROBOCLAW_ADDR, 0);

}  
