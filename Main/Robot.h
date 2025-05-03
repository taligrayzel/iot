#ifndef ROBOT_H
#define ROBOT_H

#include "Motor.h" 
#include "DistanceSensor.h"

class Robot {
private:
    Motor motor1;
    Motor motor2;
    //DistanceSensor distanceSensor;

public:
    Robot();  // Default constructor
    
    void moveForward();
    void moveBackward();
    void stopMovement();
    void clearMovementData();
    void turnLeft();
    void turnRight();
    void updateMovement();
    void readSensors();
    void displaySensorValues();
    long getDistance();
    void avoidObstacle();
    void startMovement(double target1, double target2);
    int slowCrawlTest();
    int jumpStartTest();


};

#endif
