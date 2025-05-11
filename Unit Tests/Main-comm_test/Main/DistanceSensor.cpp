#include "DistanceSensor.h"

DistanceSensor::DistanceSensor(uint8_t loxAddress, uint8_t loxSHT): loxAddress(loxAddress),loxSHT(loxSHT),lox(Adafruit_VL53L0X()) {
  //lox = Adafruit_VL53L0X();
  pinMode(loxSHT, OUTPUT);
  
}

int DistanceSensor::readSensor(){
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  // print sensor  reading
  if(measure.RangeStatus != 4) {     // if not out of range
    sensor = measure.RangeMilliMeter/10;    
    Serial.print(sensor);
    Serial.print("cm");  
    return sensor;  
  } else {
    Serial.print("Out of range");
    return -1;
  }
  Serial.println();

}

