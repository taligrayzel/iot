#include "DistanceSensor.h"

DistanceSensor::DistanceSensor() {}

bool DistanceSensor::begin() {
    return lox.begin();
}

long DistanceSensor::readDistance() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // Perform distance measurement

    if (measure.RangeStatus != 4) {
        return measure.RangeMilliMeter / 10; // Convert mm to cm
    } else {
        return -1; // Out of range
    }
}
