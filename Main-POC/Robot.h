#ifndef ROBOT_H
#define ROBOT_H

#include "Motor.h" 
#include "DistanceSensor.h"
#include "DBdistanceSensor.h"



class Robot {
private:
    Motor motor1; //right
    Motor motor2; //left
    DistanceSensor leftDistanceSensor;
    DistanceSensor frontDistanceSensor;
    DistanceSensor rightDistanceSensor;
    DB distanceDB;

    //void setUpDistanceSensors();

public:
    Robot();  // Default constructor
    
    void debugDB(string x);
    void forwardDebug();
    void correctRobotToMiddle(float left_speed ,float right_speed);
    void connectToWifi();
    void connectIPForDebug(AsyncWebServer &server);
    void moveForward();
    //void moveBackward();
    void stopMovement();
    //void clearMovementData();
    void turnLeft();
    void turnRight();
    void followLeft();
    //void updateMovement();
    void beginForSensorsSetup();
    void setUpDistanceSensors();
    void readSensors(); //now with printing - todo later - print to data base
    void displaySensorValues();
    void beginForDB();
    long getDistance();
    void avoidObstacle();
    void startMovement(double target1, double target2);
    int slowCrawlTest();
    int jumpStartTest();


};

#endif
