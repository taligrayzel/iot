#include "Robot.h"
#include "Config.h"



Robot r2d2;


void setup() {
  Serial.println("start setup");
  Serial.begin(115200);

  Wire.begin(); 
  delay(1000);

  r2d2.beginForSensorsSetup();
  r2d2.beginForDB();
  Serial.println("finish setup");
}

void loop() {


  delay(1000);

  r2d2.readSensors();

}
