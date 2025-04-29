#include "Robot.h"
#include "Config.h"


Robot r2d2;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Robot setup complete");
  attachAllEncoderInterrupts();
}

void loop() {
  // r2d2.moveBackward();
  //  delay(2000);
  // r2d2.stopMovement();
  // delay(2000);
  r2d2.moveForward();
  delay(2000);
  r2d2.stopMovement();
  r2d2.displaySensorValues();
  r2d2.clearMovementData();
  delay(2000);
  r2d2.moveBackward();
  delay(2000);
  r2d2.stopMovement();
  r2d2.displaySensorValues();
  r2d2.clearMovementData();
  delay(2000);
}
