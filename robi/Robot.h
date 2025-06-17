#ifndef ROBOT_H
#define ROBOT_H

#include "DistanceSensor.h"
#include "MotorControlUnit.h"
#include "DataBase.h"
#include <Wire.h>
#include <WiFi.h>
#include "Config.h"

class Robot {
private:
    //DistanceSensor distanceSensor;
    MotorControlUnit mcu;
    DataBase db; 
    bool connectToWifi(); 

public:
    Robot();  // Default constructor
    
    void begin();
    void startMovement(MovementType move, float target);
    void clearMovementData();
    void updateMovement(float dt);
};

#endif
