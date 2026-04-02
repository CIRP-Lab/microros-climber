#include "RoboClaw.h"

#define address 0x80

HardwareSerial RoboSerial(1);   // UART1
RoboClaw roboclaw(&RoboSerial, 10000);

void setup() {
  // Start Serial1 on D8 (RX) and D9 (TX)
  // Format: Serial1.begin(baud, config, rxPin, txPin);
    RoboSerial.begin(
    38400,
    SERIAL_8N1,
    D8,   // RX  (optional)
    D9    // TX  (required)
  );

  
  roboclaw.begin(38400);
  Serial.begin(38400);
}

void loop() {
    // Read encoder channel 1
    int motor_1_count = roboclaw.ReadEncM1(address);
    Serial.print("Original:");
    Serial.print(motor_1_count);
    Serial.print("\n");
    
    delay(2000);

    // Set encoder
    roboclaw.SetEncM1(address, 10000);
    motor_1_count = roboclaw.ReadEncM1(address);
    Serial.print("After setting count:");
    Serial.print(motor_1_count);
    Serial.print("\n");

    delay(2000);

    // Start motor 1
//    roboclaw.ForwardM1(address, 64);
//    delay(500);
//    int motor_1_speed = roboclaw.ReadSpeedM1(address);
//    delay(500);
//    Serial.print("Motor speed:");
//    Serial.print(motor_1_speed);
//    Serial.print("\n");
//    roboclaw.ForwardM1(address,0);
//
//    delay(2000);

    // Reset encoders
    roboclaw.ResetEncoders(address);
    motor_1_count = roboclaw.ReadEncM1(address);
    Serial.print("After reset:");
    Serial.print(motor_1_count);
    Serial.print("\n");

    delay(2000);
    roboclaw.BackwardM1(address, 0);
    // Position the motor
    roboclaw.SpeedAccelDistanceM1(address, 10000, 2000, 10, 1);

    delay(2000);
}

//Very interesting, this code SpeedAccelDistanceM1 function works,
// But when I do the same on our robot arm setup function, the same 
// SpeecAccelDistance function doesn't work as well. 

// Even when I do BackwardM1 like I did in our original code to see
// if it was a buffer issue, it still works!!! 
