#ifndef DISTANCESENSOR_H
#define DISTANCESENSOR_H

#include <Arduino.h>

class DistanceSensor {
private:
    int trigPin;  // Trigger pin
    int echoPin;  // Echo pin

public:
    DistanceSensor(int trigPin, int echoPin);

    // Method to read the distance in centimeters
    long readDistance();
};

#endif
