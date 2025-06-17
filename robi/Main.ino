#include "Robot.h"
#include "Config.h"

Robot r2d2;

unsigned long time1 = 0;
float duration = 10.0f;  // duration of each phase in seconds
int phase = 0;           // 0 = straight linear ramp, 1 = sinusoidal
float startTime = 0;

void setup() {
  Serial.begin(115200);
  delay(5000);

  r2d2.begin();
  r2d2.clearMovementData();

  startTime = millis() / 1000.0f;
  phase = 0;

  Serial.println("Starting PID calibration - Straight movement");
  r2d2.startMovement(MovementType::MANUAL_CONTROL, 1.0f);  // initial target
  time1 = millis();
}

void loop() {
  unsigned long currMillis = millis();
  float currTime = currMillis / 1000.0f;
  float dt = (currMillis - time1) / 1000.0f;

  r2d2.updateMovement(dt);
  time1 = currMillis;

  float elapsed = currTime - startTime;

  float target = 0.0f;

  if (phase == 0) {
    // Linear ramp from 1.0 to -1.0 over 'duration' seconds
    target = 1.0f - 2.0f * (elapsed / duration);
    target = constrain(target, -1.0f, 1.0f);

    if (elapsed >= duration) {
      phase = 1;
      startTime = currTime;
      Serial.println("Starting PID calibration - Sinusoidal movement");
    }
  } else if (phase == 1) {
    // Sinusoidal target between -1 and 1 over 'duration' seconds
    float omega = 2.0f * 3.14159f / duration;
    target = sin(omega * elapsed);

    if (elapsed >= duration) {
      Serial.println("PID calibration complete");
      r2d2.startMovement(MovementType::MANUAL_CONTROL, 0.0f);  // stop motors
      while (true) {
        delay(1000);  // halt here
      }
    }
  }

  // Update motor PID target by restarting movement with new target
  r2d2.startMovement(MovementType::MANUAL_CONTROL, target);
  
  Serial.printf("Phase: %d, Target: %.3f\n", phase, target);
}
