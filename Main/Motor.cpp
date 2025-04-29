#include "Motor.h"
#include "MotorConfig.h"

// Global encoder-to-motor pointer map
Motor* encoderMap[40] = { nullptr };

const int8_t encoder_lookup_table[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};

Motor::Motor(int in1, int in2, int enc1, int enc2, int pwmChannel1, int pwmChannel2, int gearR, int tpr)
  : in1(in1), in2(in2), enc1(enc1), enc2(enc2), gearR(gearR), tpr(tpr),
    pwmChannel1(pwmChannel1), pwmChannel2(pwmChannel2) {


  encoderMap[enc1] = this;
  encoderMap[enc2] = this;

  pinMode(enc1, INPUT_PULLUP);
  pinMode(enc2, INPUT_PULLUP);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Setup LEDC PWM
  ledcSetup(pwmChannel1, 18000, 8); // 1kHz, 8-bit resolution
  ledcSetup(pwmChannel2, 18000, 8);

  ledcAttachPin(in1, pwmChannel1);
  ledcAttachPin(in2, pwmChannel2);

  lastEncoded = 0;
  encoderValue = 0;
}

void Motor::forward(int speed) {
  ledcWrite(pwmChannel1, speed);  // Motor forward
  ledcWrite(pwmChannel2, 0);
  Serial.print("Forward at speed: ");
  Serial.print(speed);
  Serial.print(", data: ");
  Serial.print("in1-");
  Serial.print(in1);
  Serial.print(" in2-");
  Serial.print(in2);
  Serial.print(" pwm1-");
  Serial.print(pwmChannel1);
  Serial.print(" pwm2-");
  Serial.println(pwmChannel2);
  Serial.print(" deg-");
  Serial.println(get_deg());
}

void Motor::reverse(int speed) {
  Serial.print("Reverse at speed: ");
  Serial.println(speed);
  ledcWrite(pwmChannel1, 0);
  ledcWrite(pwmChannel2, speed);  // Motor reverse
}

void Motor::stop() {
  ledcWrite(pwmChannel1, 255);
  ledcWrite(pwmChannel2, 255);
}

void Motor::updateEncoder() {
  // Read current A and B
  uint8_t msb = digitalRead(enc1);
  uint8_t lsb = digitalRead(enc2);
  uint8_t currEncoded = (msb << 1) | lsb;
  // Combine current A/B with previous state
  uint8_t state = (lastEncoded << 2) | currEncoded;

  // Update position from lookup table
  encoderValue += encoder_lookup_table[state];

  // Save last 2 bits (current A/B)
  lastEncoded = currEncoded;
}

double Motor::get_deg() {
  Serial.println("encoDerValue: ");
  Serial.println(encoderValue);
  float degrees_motor = (encoderValue / (float)tpr) * 90.0;
  Serial.println("degrees_motor: ");
  Serial.println(degrees_motor);
  return degrees_motor / gearR;
}

void Motor::clear_movement() {
  encoderValue = 0;
}
    
#define EXPAND(x) x

// Interrupt Helpers
#define ENCODER_PINS \
  X(ENC_A_1)         \
  X(ENC_A_2)         \
  X(ENC_B_1)         \
  X(ENC_B_2)     


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