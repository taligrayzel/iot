#ifndef ROBOT_H
#define ROBOT_H

#include "DistanceSensor.h"
#include "DBdistanceSensor.h"
#include "Wall.h"

class Robot {
private:
    DistanceSensor distanceSensor1;
    DistanceSensor distanceSensor2;
    DistanceSensor distanceSensor3;
    DB DBobject;
    Wall walls_state;

    void setUpDistanceSensors();

public:
    Robot();  // Default constructor
    
  
    void turnLeft();
    void turnRight();
    void beginForSensorsSetup();
    void readSensors(); //now with printing - todo later - print to data base
    void displaySensorValues();
    void beginForDB();
    long getDistance();
    void checkForWall();
  
    
};

#endif
