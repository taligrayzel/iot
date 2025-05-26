#ifndef DISTANCESENSOR_H
#define DISTANCESENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#include <sstream>
#include <string>

#include <cstdio>   // snprintf
#include <cstdlib>  // size_t

class DistanceSensor {
public:
    uint8_t loxAddress;
    uint8_t loxSHT;
    int sensor;
    Adafruit_VL53L0X lox;
    VL53L0X_RangingMeasurementData_t measure;


    

public:
    DistanceSensor(uint8_t loxAddress, uint8_t loxSHT);
    float readSensor();
    //void setUpSensor();         // Initialize the sensor
    //long readDistance();  // Read distance in centimeters (returns -1 if out of range)
    //void setID();
    //void read_three_sensors();
};

#endif
