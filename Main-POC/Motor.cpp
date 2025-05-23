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
// const int8_t encoder_lookup_table[16] = {
//     0,  0,  0,  0,   // 00 → 01 (forward), 00 → 10 (backward)
//     0,  0,  0,  1,   // 10 → 11 (forward), 01 → 00 (backward)
//     0,  0,  0,  -1,   // 11 → 10 (forward), 10 → 01 (backward)
//     0,  1,  -1,  0    // 01 → 11 (forward), 00 → 10 (backward)
// 270 FOR HALF THE DEGREES
// };

int mock_index = 0;
Motor::Motor(int in1, int in2, int enc1, int enc2, int pwmChannel1, int pwmChannel2, int gearR, int tpr)
  : in1(in1), in2(in2), enc1(enc1), enc2(enc2), gearR(gearR), tpr(tpr),
    pwmChannel1(pwmChannel1), pwmChannel2(pwmChannel2) {
  for(int i = 0; i < 16; i++){
    encoder_mock[i] = 0;
  }
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

  encoderMutex = portMUX_INITIALIZER_UNLOCKED;


  kp = 1;
  kd = 0;
  ki = 0;

  target = 0;
  pos = 0; 
  prevT = 0;
  eprev = 0;
  eintegral = 0;

}

void Motor::forward(int speed) {
  ledcWrite(pwmChannel1, 0);  // Motor forward
  ledcWrite(pwmChannel2, speed);
}

void Motor::reverse(int speed) {
  ledcWrite(pwmChannel1, speed);
  ledcWrite(pwmChannel2, 0);  // Motor reverse
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

  portENTER_CRITICAL(&encoderMutex);
  // Update position from lookup table
  encoderValue += encoder_lookup_table[state];

  portEXIT_CRITICAL(&encoderMutex);
  // Save last 2 bits (current A/B)
  lastEncoded = currEncoded;
}

double Motor::get_deg() {

  portENTER_CRITICAL(&encoderMutex);
  float degrees_motor = (encoderValue / (float)tpr) * 60.0;
  portEXIT_CRITICAL(&encoderMutex);
  return degrees_motor / gearR;
}

void Motor::clear_movement() {
  encoderValue = 0;
  target = 0;
  pos = 0; 
  prevT = 0;
  eprev = 0;
  eintegral = 0;
}

int applyDeadZoneCompensation(float output_pwm) {
  int pwm = (int)output_pwm;

  // if (pwm > 0 && pwm < 200) {
  //    pwm = 220;  // Set to minimum effective PWM
  //  } else if (pwm < 0 && pwm > -200) {
  //    pwm = -220; // For bidirectional motors (if using H-bridge)
  //  }

  // // return constrain(pwm, -255, 255);
  return pwm;
}

void Motor::set_movement(double target){
  this->target = target;
}


void Motor::updateMovement(){
  Serial.print(target);Serial.print(",");
  pos = get_deg();
  Serial.println(pos);
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
  double e = target - pos;
  // Serial.print("Error is: ");
  // Serial.println(e);
  // if(fabs(e) < 1.0e-3){
  //   stop();
  //   return;
  // }
  float dedt = (e-eprev)/(deltaT);    // derivative
  eintegral += e*deltaT;   // integral
  float u = kp*e + kd*dedt + ki*eintegral;  // control signal

  float pwm_output = applyDeadZoneCompensation(constrain((int)u, -255, 255));
  if(pwm_output < 0 ){
    // Serial.print("Movement backwards at speed: ");
    // Serial.println(fabs(pwm_output));
    reverse(fabs(pwm_output));
  }
  else{
    // Serial.print("Movement farward at speed: ");
    // Serial.println(fabs(pwm_output));
    forward(pwm_output);
  }
  eprev = e;
}
void Motor::printMock(){
  for(int i = 0; i < 16; i++){
    Serial.print(encoder_mock[i]);
    Serial.print(", ");
  }
  Serial.println();
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