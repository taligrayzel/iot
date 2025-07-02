#include "Robot.h"
#include "DebugConfig.h"
#define DPRINT(x)    DPRINT_ROBOT(x)
#define DPRINTLN(x)  DPRINTLN_ROBOT(x)
#define LOG_FUNC(x) LOG_ROBOT_MSG(x)

// Constructor to initialize motors and distance sensor
Robot::Robot(){
  DPRINTLN("Robot built");
}

bool Robot::connectToWifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    DPRINT("WiFi Failed!\n");
    return false;
  }
  DPRINTLN("CONNECTED TO WIFI");
  return true;
}

// Turn the robot right
void Robot::startMovement(MovementType move, float target) {
  DPRINTLN("MCU start movement!");
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

bool Robot::isFinishedMovement(){
  return mcu.isMovementComplete();
}

void Robot::updateMovement(float dt) {
  DPRINT("Robot Update Movement after :");
  DPRINTLN(dt);
  DPRINTLN("Main loop on Core " + String(xPortGetCoreID()));

  mcu.updateMovement(dt);
}

void Robot::begin(){
  DPRINTLN("Robot begin...");
  DataBase* db_di = nullptr; 
  if (connectToWifi()) {
    if (db.begin()) {
      DPRINTLN("Database init COMPLETE.");
      db_di = &db;
    } else {
      DPRINTLN("Database failed to initialize.");
    }
  } else {
    DPRINTLN("Wi-Fi failed. Skipping DB initialization.");
  }
  mcu.begin(db_di);
  DPRINTLN("Robot begin...COMPLETE");
}