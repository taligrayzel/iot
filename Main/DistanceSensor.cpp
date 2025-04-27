#include "DistanceSensor.h"

DistanceSensor::DistanceSensor(int trigPin, int echoPin) : trigPin(trigPin), echoPin(echoPin) {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

long DistanceSensor::readDistance() {
    // Send a pulse to trigger the sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure the time it takes for the pulse to return
    long duration = pulseIn(echoPin, HIGH);

    // Calculate distance based on the speed of sound (343 meters/second)
    long distance = (duration / 2) * 0.0344; // Distance in cm
    return distance;
}
