// #include "Robot.h"
// #include "Config.h"

// Robot r2d2;

// unsigned long time1 = 0;
// float duration = 10.0f;  // duration per phase
// int phase = 0;
// float startTime = 0;

// void setup() {
//   Serial.begin(115200);
//   delay(5000);

//   r2d2.begin();
//   r2d2.clearMovementData();

//   startTime = millis() / 1000.0f;
//   phase = 0;

//   Serial.println("Starting PID calibration - Phase 0: Linear ramp");
//   r2d2.startMovement(MovementType::MANUAL_CONTROL, 0.0f);  // initial target
//   time1 = millis();
// }

// void loop() {
//   unsigned long currMillis = millis();
//   float currTime = currMillis / 1000.0f;
//   float dt = (currMillis - time1) / 1000.0f;

//   r2d2.updateMovement(dt);
//   time1 = currMillis;

//   float elapsed = currTime - startTime;
//   float target = 0.0f;

//   switch (phase) {
//     case 0:
//       // Phase 0: Linear ramp (1.0 to -1.0)
//       target = 1.0f - 2.0f * (elapsed / duration);
//       target = constrain(target, -1.0f, 1.0f);
//       if (elapsed >= duration) {
//         phase++;
//         startTime = currTime;
//         Serial.println("Starting PID calibration - Phase 1: Sinusoidal input");
//       }
//       break;

//     case 1:
//       // Phase 1: Sinusoidal input
//       target = sin((2.0f * PI / duration) * elapsed);
//       if (elapsed >= duration) {
//         phase++;
//         startTime = currTime;
//         Serial.println("Starting PID calibration - Phase 2: Step response");
//       }
//       break;

//     case 2:
//       // Phase 2: Step response (switch every 2 seconds)
//       target = (int(elapsed / 2) % 2 == 0) ? 1.0f : -1.0f;
//       if (elapsed >= duration) {
//         phase++;
//         startTime = currTime;
//         Serial.println("Starting PID calibration - Phase 3: Constant input");
//       }
//       break;

//     case 3:
//       // Phase 3: Constant low-speed input
//       target = 0.25f;
//       if (elapsed >= duration) {
//         Serial.println("PID calibration complete");
//         r2d2.startMovement(MovementType::MANUAL_CONTROL, 0.0f);  // stop motors
//         while (true) {
//           delay(1000);  // end loop
//         }
//       }
//       break;
//   }

//   r2d2.startMovement(MovementType::MANUAL_CONTROL, target);
//   Serial.printf("Phase: %d, Target: %.3f\n", phase, target);
  // unsigned long currMillis = millis();
  // float currTime = currMillis / 1000.0f;
  // float dt = (currMillis - time1) / 1000.0f;

  // r2d2.updateMovement(dt);
  // time1 = currMillis;
// }

/////////////////////////////////////////////////////////////////////////MOVE DISTANCE
// #include "Robot.h"
// #include "Config.h"

// Robot r2d2;

// unsigned long time1 = 0;
// int phase = 0; // Tracks which distance to run

// void setup() {
//   Serial.begin(115200);
//   delay(5000);

//   r2d2.begin();
//   r2d2.clearMovementData();

//   Serial.println("Starting position PID test - Phase 0: Move 2 meters forward");
//   r2d2.startMovement(MovementType::MOVING_DISTANCE, 2.0f);  // Move 2 meters forward

//   time1 = millis();
// }

// void loop() {
//   unsigned long currMillis = millis();
//   float dt = (currMillis - time1) / 1000.0f;
//   time1 = currMillis;

//   r2d2.updateMovement(dt);

//   if (r2d2.isFinishedMovement()) {
//     // Finished current move — decide what’s next
//     switch (phase) {
//       case 0:
//         Serial.println("Phase 0 complete. Starting Phase 1: Move 0.15 meters backward");
//         r2d2.startMovement(MovementType::MOVING_DISTANCE, -0.15f);
//         phase++;
//         break;

//       case 1:
//         Serial.println("Phase 1 complete. Starting Phase 2: Move 1 meter forward");
//         r2d2.startMovement(MovementType::MOVING_DISTANCE, 1.0f);
//         phase++;
//         break;

//       case 2:
//         Serial.println("Phase 2 complete. Position PID test finished. Stopping.");
//         r2d2.startMovement(MovementType::MANUAL_CONTROL, 0.0f);  // Stop motors
//         while (true) {
//           delay(1000); // stay idle
//         }
//         break;
//     }
//   }
// }

/////////////////////////////////////////////////////////////////////////MOVE ROTATION

// #include "Robot.h"
// #include "Config.h"

// Robot r2d2;

// unsigned long time1 = 0;
// int phase = 0; // Tracks which turn we are doing

// void setup() {
//   Serial.begin(115200);
//   delay(5000);

//   r2d2.begin();
//   r2d2.clearMovementData();

//   Serial.println("Starting rotation PID test - Phase 0: Rotate 90 degrees clockwise");
//   r2d2.startMovement(MovementType::TURNING_ANGLE, 360.0f);  // Positive = clockwise

//   time1 = millis();
// }

// void loop() {
//   unsigned long currMillis = millis();
//   float dt = (currMillis - time1) / 1000.0f;
//   time1 = currMillis;

//   r2d2.updateMovement(dt);

//   if (r2d2.isFinishedMovement()) {
//     // Done turning → pick next turn
//     switch (phase) {
//       case 0:
//         Serial.println("Phase 0 complete. Starting Phase 1: Rotate -45 degrees (counter-clockwise)");
//         r2d2.startMovement(MovementType::TURNING_ANGLE, -720.0f);  // Negative = counter-clockwise
//         phase++;
//         delay(1000); // stay idle
//         break;

//       case 1:
//         Serial.println("Phase 1 complete. Starting Phase 2: Rotate 180 degrees clockwise");
//         r2d2.startMovement(MovementType::TURNING_ANGLE, 180.0f);
//         phase++;
//         delay(1000); // stay idle
//         break;
//       case 2:
//         Serial.println("Phase 2 complete. Starting Phase 2: Rotate 180 degrees clockwise");
//         r2d2.startMovement(MovementType::TURNING_ANGLE, 90.0f);
//         phase++;
//         delay(1000); // stay idle
//         break;
//       case 3:
//         Serial.println("Phase 3 complete. Starting Phase 2: Rotate 180 degrees clockwise");
//         r2d2.startMovement(MovementType::TURNING_ANGLE, -45.0f);
//         phase++;
//         delay(1000); // stay idle
//         break;
//       case 4:
//         Serial.println("Phase 4 complete. Rotation PID test finished. Stopping.");
//         r2d2.startMovement(MovementType::MANUAL_CONTROL, 0.0f);  // Stop motors
//         while (true) {
//           delay(1000); // stay idle
//         }
//         break;
//     }
//   }
// }
#include "Robot.h"
#include "Config.h"

Robot r2d2;

unsigned long time1 = 0;
int phase = 0; // 0 = moving, 1 = turning

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Setup");
  r2d2.begin();
  r2d2.clearMovementData();

  Serial.println("Square path test: Phase 0 - Move forward 1 meter");
  r2d2.startMovement(MovementType::MOVING_DISTANCE, 1.0f); // forward 1 meter

  time1 = millis();
}

void loop() {
  unsigned long currMillis = millis();
  float dt = (currMillis - time1) / 1000.0f;
  time1 = currMillis;

  r2d2.updateMovement(dt);

  if (r2d2.isFinishedMovement()) {
    if (phase == 0) {
      // Finished moving forward — now turn right
      Serial.println("Finished forward move. Turning right 90 degrees...");
      r2d2.startMovement(MovementType::TURNING_ANGLE, 90.0f); // right turn
      phase = 1;
    } else if (phase == 1) {
      // Finished turning — move forward again
      Serial.println("Finished turn. Moving forward 1 meter...");
      r2d2.startMovement(MovementType::MOVING_DISTANCE, 1.0f); // next forward
      phase = 0;
    }
  }
}
