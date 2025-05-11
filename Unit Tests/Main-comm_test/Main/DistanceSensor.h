#ifndef DISTANCESENSOR_H
#define DISTANCESENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

class DistanceSensor {
public:
    uint8_t loxAddress;
    uint8_t loxSHT;
    int sensor;
    Adafruit_VL53L0X lox;
    VL53L0X_RangingMeasurementData_t measure;


    

public:
    DistanceSensor(uint8_t loxAddress, uint8_t loxSHT);
    int readSensor();

};

#endif
