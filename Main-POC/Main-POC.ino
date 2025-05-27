#include "Robot.h"
#include "Config.h"

// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>
// #include <WebSerial.h>

Robot r2d2;
AsyncWebServer server(80);

int slowTestCounter = 10;
int test_result = 0;

unsigned long startTime = millis();


void setup() {
  Serial.println("start setup");
  Serial.begin(115200);

  Wire.begin(); 
  delay(1000);

  r2d2.beginForSensorsSetup();
  r2d2.connectToWifi();
  r2d2.connectIPForDebug(server);
  r2d2.beginForDB();
  //attachAllEncoderInterrupts();
  //r2d2.startMovement(3600,3600);
  //Serial.println("TARGET:,POS:,TIME:");
  Serial.println("finish setup");
}

void loop() {

  

  // r2d2.moveForward();
  // delay(1000);
  // r2d2.moveBackward();
  // delay(1000);
  // r2d2.stopMovement();
  delay(10);
  //WebSerial.println("Hello!");
 // r2d2.moveForward();

  //r2d2.turnLeft();

  
  r2d2.followLeft();
  // r2d2.forwardDebug();
  //rd2d.followLeftandRight();


  // if(slowTestCounter > 0){
  // int interresult = r2d2.jumpStartTest();
  // if(interresult > test_result)
  //   test_result = interresult;
  // slowTestCounter--;
  //}
}
