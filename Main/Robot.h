#ifndef ROBOT_H
#define ROBOT_H

#include "Config.h"  // Include the combined config file

class Robot {
private:
    Motor motor1;
    Motor motor2;
    DistanceSensor distanceSensor;

public:
    Robot();  // Default constructor
    
    void moveForward();
    void moveBackward();
    void stopMovement();
    void turnLeft();
    void turnRight();
    void readSensors();
    void displaySensorValues();
    long getDistance();
    void avoidObstacle();
};

#endif
