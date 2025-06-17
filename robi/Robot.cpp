#include "HardwareSerial.h"
#include "Robot.h"

// Constructor to initialize motors and distance sensor
Robot::Robot(){
  Serial.println("Robot built");
}

bool Robot::connectToWifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return false;
  }
  Serial.println("CONNECTED TO WIFI");
  return true;
}

// Turn the robot right
void Robot::startMovement(MovementType move, float target) {
  Serial.println("MCU start movement!");
  switch(move){
    case MovementType::MANUAL_CONTROL:
      mcu.driveManual(target);
      break;
    case MovementType::TURNING_ANGLE:
      mcu.turnAngle(target);
      break;
    case MovementType::MOVING_DISTANCE:
      mcu.moveDistance(target);
      break;
    default:
      break;
  }
}

void Robot::clearMovementData(){
}

void Robot::updateMovement(float dt) {
  Serial.print("Robot Update Movement after :");
  Serial.println(dt);
  mcu.updateMovement(dt);
}

void Robot::begin(){
  mcu.begin();
  if (connectToWifi()) {
    if (db.begin()) {
      Serial.println("Robot fully initialized.");
    } else {
      Serial.println("Database failed to initialize.");
    }
  } else {
    Serial.println("Wi-Fi failed. Skipping DB initialization.");
  }
}