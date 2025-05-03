#include "Robot.h"
#include "Config.h"


Robot r2d2;
int slowTestCounter = 10;
 int test_result = 0;

unsigned long startTime = millis();


void setup() {
  Serial.begin(115200);
  delay(1000);
  attachAllEncoderInterrupts();
  r2d2.startMovement(3600,3600);
  Serial.println("TARGET:,POS:,TIME:");
}

void loop() {
  if(slowTestCounter > 0){
  int interresult = r2d2.jumpStartTest();
  if(interresult > test_result)
    test_result = interresult;
  slowTestCounter--;
  }
}
