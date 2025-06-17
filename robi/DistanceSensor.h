#ifndef DISTANCESENSOR_H
#define DISTANCESENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

class DistanceSensor {
private:
    Adafruit_VL53L0X lox;

public:
    DistanceSensor();
    bool begin();         // Initialize the sensor
    long readDistance();  // Read distance in centimeters (returns -1 if out of range)
};

#endif
