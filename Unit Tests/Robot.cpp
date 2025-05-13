#include "Robot.h"
#include "Config.h"  // Include the combined config file

// Constructor to initialize motors and distance sensor
Robot::Robot()
    : distanceSensor1(LOX1_ADDRESS, SHT_LOX1),
      distanceSensor2(LOX2_ADDRESS, SHT_LOX2),
      distanceSensor3(LOX3_ADDRESS, SHT_LOX3),
      DBobject()
      {

}

void Robot::beginForSensorsSetup(){

  digitalWrite(distanceSensor1.loxSHT, LOW);
  digitalWrite(distanceSensor2.loxSHT, LOW);
  digitalWrite(distanceSensor3.loxSHT, LOW);
  
  setUpDistanceSensors();
}

void Robot::beginForDB(){
  DBobject.connectToDB();
  DBobject.setTime();
  time_t now = time(nullptr);
  Serial.println("finished connecting to DB");

}

//set up all the distance sensors to work together
void Robot::setUpDistanceSensors(){
  Serial.println("Starting setUp for distance sensor...");
  // all reset
  digitalWrite(distanceSensor1.loxSHT, LOW);    
  digitalWrite(distanceSensor2.loxSHT, LOW);
  digitalWrite(distanceSensor3.loxSHT, LOW);
  delay(50);
  // all unreset
  digitalWrite(distanceSensor1.loxSHT, HIGH);
  digitalWrite(distanceSensor2.loxSHT, HIGH);
  digitalWrite(distanceSensor3.loxSHT, HIGH);
  delay(50);

  // activating LOX1 and reseting LOX2
  digitalWrite(distanceSensor1.loxSHT, HIGH);
  digitalWrite(distanceSensor2.loxSHT, LOW);
  digitalWrite(distanceSensor3.loxSHT, LOW);

  // initing LOX1
  if(!distanceSensor1.lox.begin(distanceSensor1.loxAddress)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(50);

  // activating LOX2
  digitalWrite(distanceSensor2.loxSHT, HIGH);
  digitalWrite(distanceSensor3.loxSHT, LOW);
  delay(50);

  //initing LOX2
  if(!distanceSensor2.lox.begin(distanceSensor2.loxAddress)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }

  // activating LOX3
  digitalWrite(distanceSensor3.loxSHT, HIGH);
  delay(50);

  //initing LOX3
  if(!distanceSensor3.lox.begin(distanceSensor3.loxAddress)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while(1);
  }

  Serial.println("successfuly finished setUpDistanceSensors...");

}


// Read all sensors (simplified)
void Robot::readSensors() {
  int left  = distanceSensor1.readSensor();
  int front = distanceSensor2.readSensor();
  int right = distanceSensor3.readSensor();
  
  walls_state.updateWalls(left, front, right);

  DBobject.uploadDistancesToDB(left, front, right);

  delay(50);
}

void Robot::checkForWall(){
  walls_state.checkForWall();
}








