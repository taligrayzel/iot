#include "DistanceSensor.h"

DistanceSensor::DistanceSensor(uint8_t loxAddress, uint8_t loxSHT): loxAddress(loxAddress),loxSHT(loxSHT),lox(Adafruit_VL53L0X()) {
  //lox = Adafruit_VL53L0X();
  pinMode(loxSHT, OUTPUT);
  
}

float DistanceSensor::readSensor(){
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  // print sensor  reading
  if(measure.RangeStatus != 4) {     // if not out of range
    sensor = measure.RangeMilliMeter; 
    // std::string str = std::to_string(sensor);
    // const char* ch = str.c_str();
    // WebSerial.print(ch);
    //Serial.println(sensor);
    return sensor;  
  } else {
    //WebSerial.print(" out of range ");
    //Serial.println(" out of range ");
    return -1;
  }
}

