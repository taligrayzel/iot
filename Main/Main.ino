#include "Robot.h"

Robot myRobot;  // Use the default constructor

void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println("Robot setup complete");
}

void loop() {
  // Move forward
  myRobot.moveForward();
  delay(2000);  // Move forward for 2 seconds

  // Avoid obstacles by checking the distance
  myRobot.avoidObstacle();

  // If no obstacle, continue moving forward
  delay(1000);
}
