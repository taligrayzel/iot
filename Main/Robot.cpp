#include "Robot.h"
#include "Config.h"  // Include the combined config file

// Constructor to initialize motors and distance sensor
Robot::Robot()
    : motor1(IN_A_1, IN_A_2, ENC_A_1, ENC_A_2,PWM_A_1,PWM_A_2,GEAR_RATIO,TPR),
      motor2(IN_B_1, IN_B_2, ENC_B_1, ENC_B_2,PWM_B_1,PWM_B_2,GEAR_RATIO,TPR){
}

// Move forward by controlling both motors
void Robot::moveForward() {
    motor1.forward(170);
    motor2.forward(200);
}

// Move backward by controlling both motors
void Robot::moveBackward() {
    motor1.reverse(200);
    motor2.reverse(200);
}

// Stop both motors
void Robot::stopMovement() {
    motor1.stop();
    motor2.stop();
}

// Turn the robot left
void Robot::turnLeft() {
    motor1.reverse();
    motor2.forward();
}

// Turn the robot right
void Robot::turnRight() {
    motor1.forward();
    motor2.reverse();
}

// Read all sensors (simplified)
void Robot::readSensors() {
}

// Display sensor values to the serial monitor
void Robot::displaySensorValues() {
  Serial.print("motor1 movement deg: ");
  Serial.println(motor1.get_deg());
  Serial.print("motor2 movement deg: ");
  Serial.println(motor2.get_deg());
}
// Display sensor values to the serial monitor
void Robot::clearMovementData() {
  motor1.clear_movement();
  motor2.clear_movement();
}

// Get the distance from the distance sensor
long Robot::getDistance() {
    return 1;
    //distanceSensor.readDistance();
}

// Obstacle avoidance function based on distance sensor reading
void Robot::avoidObstacle() {
    long distance = getDistance();
    Serial.print("Distance: ");
    Serial.println(distance);
    // If an obstacle is detected within a range (e.g., 10 cm), stop the robot
    if (distance < 10) {
        stopMovement();
        Serial.println("Obstacle detected! Stopping robot.");
    }
}
