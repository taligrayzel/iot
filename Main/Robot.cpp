#include "Robot.h"
#include "Config.h"  // Include the combined config file

// Constructor to initialize motors and distance sensor
Robot::Robot()
    : motor1(IN_A_1, IN_A_2, ENC_A_1, ENC_A_2, GEAR_RATIO),
      motor2(IN_B_1, IN_B_2, ENC_B_1, ENC_B_2, GEAR_RATIO),
      distanceSensor(DIST_SENSOR_TRIG_PIN, DIST_SENSOR_ECHO_PIN) {
    pinMode(SENSOR_1_PIN, INPUT);
    pinMode(SENSOR_2_PIN, INPUT);
    pinMode(SENSOR_3_PIN, INPUT);
}

// Move forward by controlling both motors
void Robot::moveForward() {
    motor1.forward();
    motor2.forward();
}

// Move backward by controlling both motors
void Robot::moveBackward() {
    motor1.reverse();
    motor2.reverse();
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
    int sensor1Value = digitalRead(SENSOR_1_PIN);
    int sensor2Value = digitalRead(SENSOR_2_PIN);
    int sensor3Value = digitalRead(SENSOR_3_PIN);

    // Process sensor data (e.g., based on sensor input, make decisions)
    Serial.print("Sensor 1: ");
    Serial.println(sensor1Value);
    Serial.print("Sensor 2: ");
    Serial.println(sensor2Value);
    Serial.print("Sensor 3: ");
    Serial.println(sensor3Value);
}

// Display sensor values to the serial monitor
void Robot::displaySensorValues() {
    Serial.print("Sensor 1: ");
    Serial.println(digitalRead(SENSOR_1_PIN));
    Serial.print("Sensor 2: ");
    Serial.println(digitalRead(SENSOR_2_PIN));
    Serial.print("Sensor 3: ");
    Serial.println(digitalRead(SENSOR_3_PIN));
}

// Get the distance from the distance sensor
long Robot::getDistance() {
    return distanceSensor.readDistance();
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
