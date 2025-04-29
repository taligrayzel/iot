#include "motor_test_4.h"

#include <arduino.h> 


// Global encoder-to-motor pointer map
Motor* encoderMap[40] = { nullptr };


Motor::Motor(int in1, int in2, int enc1, int enc2, int gearR, int pwmChannel1, int pwmChannel2)
  : in1(in1), in2(in2), enc1(enc1), enc2(enc2), gearR(gearR),
    pwmChannel1(pwmChannel1), pwmChannel2(pwmChannel2) {


  encoderMap[enc1]= this;
  encoderMap[enc2]= this;

  pinMode(enc1, INPUT_PULLUP);
  pinMode(enc2, INPUT_PULLUP);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Setup LEDC PWM
  ledcSetup(pwmChannel1, 1000, 8); // 1kHz, 8-bit resolution
  ledcSetup(pwmChannel2, 1000, 8);

  ledcAttachPin(in1, pwmChannel1);
  ledcAttachPin(in2, pwmChannel2);

  lastEncoded = 0;
  encoderValue = 0;
}

void Motor::forward(int speed) {
  ledcWrite(pwmChannel1, speed);  // Motor forward
  ledcWrite(pwmChannel2, 0);
}

void Motor::reverse(int speed) {
  ledcWrite(pwmChannel1, 0);
  ledcWrite(pwmChannel2, speed);  // Motor reverse
}

void Motor::stop() {
  ledcWrite(pwmChannel1, 255);
  ledcWrite(pwmChannel2, 255);
}

void Motor::updateEncoder() {
  lastEncoded++;
}

double Motor::get_deg() {
  return (double)lastEncoded / gearR;
}

void Motor::clear_movement() {
  lastEncoded = 0;
  encoderValue = 0;
}


// Forward declare the global encoder map
extern class Motor* encoderMap[40];

#define EXPAND(x) x

// Interrupt Helpers
#define ENCODER_PINS \
  X(32)         \
  X(33)         \
  X(34)         \
  X(35)         

#define X(PIN) \
  void IRAM_ATTR encoderISR##PIN() { \
    if (encoderMap[EXPAND(PIN)]){\
      encoderMap[EXPAND(PIN)]->updateEncoder(); \
    } \
  }
  ENCODER_PINS
#undef X

void attachAllEncoderInterrupts() {
  #define X(PIN) attachInterrupt(digitalPinToInterrupt(PIN), encoderISR##PIN, CHANGE);
  ENCODER_PINS
  #undef X
}
