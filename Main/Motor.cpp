#include "Motor.h"
#include "MotorConfig.h"

// Global encoder-to-motor pointer map
Motor* encoderMap[40] = { nullptr };

// Constructor
Motor::Motor(int in1, int in2, int enc1, int enc2, int gearR)
  : in1(in1), in2(in2), enc1(enc1), enc2(enc2), gearR(gearR) {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enc1, INPUT_PULLUP);
  pinMode(enc2, INPUT_PULLUP);
  lastEncoded = 0;
  encoderValue = 0;
}

// Movement control
void Motor::forward() {
  Serial.print("Motor In Pins: ");
  Serial.print("Motor In1: ");
  Serial.print(in1);
  Serial.print(", In2: ");
  Serial.println(in2);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void Motor::reverse() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void Motor::stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

// Encoder
void Motor::updateEncoder() {
  lastEncoded++;
  // Optional: implement full quadrature decoding here
}

double Motor::get_deg() {
  return (double)lastEncoded / gearR;
}

void Motor::clear_movement() {
  lastEncoded = 0;
  encoderValue = 0;
}
]
#define EXPAND(x) x


// Interrupt Helpers
#define ENCODER_PINS \
  X(ENC_A_1)         \
  X(ENC_A_2)         \
  X(ENC_B_1)         \
  X(ENC_B_2)

#define X(PIN) \
  void IRAM_ATTR encoderISR##PIN() { \
    if (encoderMap[EXPAND(PIN)]) encoderMap[EXPAND(PIN)]->updateEncoder(); \
  }
ENCODER_PINS
#undef X

void attachAllEncoderInterrupts() {
  #define X(PIN) attachInterrupt(digitalPinToInterrupt(PIN), encoderISR##PIN, CHANGE);
  ENCODER_PINS
  #undef X
}
