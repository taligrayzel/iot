#include "Motor.h"

// Global encoder-to-motor pointer map
Motor* encoderMap[40] = { nullptr };

Motor::Motor(int in1, int in2, int enc1, int enc2, int pwmChannel1, int pwmChannel2, int gearR, float tpr)
  : in1(in1), in2(in2), enc1(enc1), enc2(enc2), gearR(gearR), tpr(tpr),
    pwmChannel1(pwmChannel1), pwmChannel2(pwmChannel2), positionPid(1.0f, 0.0f, 0.0f), speedPid(1.0f, 0.0f, 0.0f) {

  encoderHistoryPointer = 0;
  positionPid.setOutputLimits(-255.0f, 255.0f);
  speedPid.setOutputLimits(-1.0f, 1.0f);
  for(int i = 0; i < 16; i++){
    encoderHistory[i].timestamp = 0;
    encoderHistory[i].dir = 0;
    encoderHistory[i].pwm = 0;
    mock_state[i] = 0;
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

  updateNoSpeed(50);
  jumpStart = true;
  positionPid.reset();
  speedPid.reset();

  pwm = 0;
}

// Methods follow...
void Motor::tick() {
  // Ensure control actions are taken based on the current mode
  switch (currentDriveMode) {
      case DriveMode::POSITION_DRIVE:
          updatePositionControl(0.01f); // delta-time is set to a constant value (adjust if needed)
          break;
          
      case DriveMode::VELOCITY_DRIVE:
          updateSpeedControl(0.01f); // Same delta-time
          break;
          
      case DriveMode::IDLE:
          stop();
          break;
  }
  portENTER_CRITICAL(&encoderMutex);
  unsigned long prevT = encoderHistory[(encoderHistoryPointer + 15) % 16].timestamp;
  unsigned long currT = micros();
  if((currT - prevT) >= DEFAULT_SPEED_TIMEOUT){
    uint8_t failedPwm = pwm;
    encoderHistory[encoderHistoryPointer].timestamp = currT;
    encoderHistory[encoderHistoryPointer].dir = 0;
    encoderHistory[encoderHistoryPointer].pwm = pwm;
    encoderHistoryPointer = (encoderHistoryPointer + 1) % 16;
        // Update noSpeed and lowSpeed based on failed PWM
    if (failedPwm > noSpeed) {
      updateNoSpeed(failedPwm);
    }
    jumpStart = true;
  }else{
    if(pwm < lowSpeed)
      updateNoSpeed(noSpeed-5);
  }
  portEXIT_CRITICAL(&encoderMutex);
}



void Motor::setPositionTarget(float targetPosition) {
  clearPositionControl();
  currentDriveMode = DriveMode::POSITION_DRIVE;
  positionPid.setSetpoint(targetPosition);
}

void Motor::setSpeedTarget(float normalizedTargetRPM) {
  clearSpeedControl();
  currentDriveMode = DriveMode::VELOCITY_DRIVE;
  speedPid.setSetpoint(normalizedTargetRPM);
}

void Motor::enableJumpStart() {
  jumpStart = true;
}
void Motor::disableJumpStart() {
  jumpStart = false;
}
bool Motor::getJumpStart() {
  return jumpStart;
}

void Motor::move(bool dir, int speed) {
  int pwm1 = dir ?  pwmChannel1 : pwmChannel2;
  int pwm2 = dir ?  pwmChannel2 : pwmChannel1;
  if(jumpStart && speed < lowSpeed && speed > noSpeed){
    ledcWrite(pwm1, 250); 
    ledcWrite(pwm2, 0);
    delay(KICKSTART_DELAY);
    jumpStart = false;
  }
  ledcWrite(pwm1, speed);
  ledcWrite(pwm2, 0); 
  pwm = speed;
}

void Motor::forward(int speed) {
  move(FORWARD,speed);
}

void Motor::reverse(int speed) {
  move(REVERSE,speed);
}

void Motor::stop() {
  ledcWrite(pwmChannel1, 255);
  ledcWrite(pwmChannel2, 255);
}

void Motor::updateNoSpeed(uint8_t newNoSpeed) {  // Fix typo here
  noSpeed = max((uint8_t)0 , newNoSpeed);  // Correct the variable to newNoSpeed
  lowSpeed = min(noSpeed + 50, 255); 
}

void Motor::clearPositionControl() {
  positionPid.reset();        // Reset the position PID controller
}

void Motor::clearSpeedControl() {
  speedPid.reset();           // Reset the speed PID controller
}

void Motor::speedTimeOutInterrupt() {
  portENTER_CRITICAL(&encoderMutex);
  unsigned long prevT = encoderHistory[(encoderHistoryPointer + 15) % 16].timestamp;
  unsigned long currT = micros();
  if((currT - prevT) >= DEFAULT_SPEED_TIMEOUT){
    uint8_t failedPwm = pwm;
    encoderHistory[encoderHistoryPointer].timestamp = currT;
    encoderHistory[encoderHistoryPointer].dir = 0;
    encoderHistory[encoderHistoryPointer].pwm = pwm;
    encoderHistoryPointer = (encoderHistoryPointer + 1) % 16;
        // Update noSpeed and lowSpeed based on failed PWM
    if (failedPwm > noSpeed) {
      updateNoSpeed(failedPwm);
    }
    jumpStart = true;
  }else{
    if(pwm < lowSpeed)
      updateNoSpeed(noSpeed-5);
  }
  portEXIT_CRITICAL(&encoderMutex);
}

void Motor::updateEncoder() {
    int deltaTicks = (digitalRead(enc1) ^ digitalRead(enc2)) ? -1 : 1;
    //Serial.println()
    portENTER_CRITICAL(&encoderMutex);
    // Update position from lookup table
    encoderHistory[encoderHistoryPointer].timestamp = micros();
    encoderHistory[encoderHistoryPointer].dir = deltaTicks;
    encoderHistory[encoderHistoryPointer].pwm = pwm;
    encoderHistoryPointer = (encoderHistoryPointer + 1) % 16;
    encoderValue += deltaTicks;
    portEXIT_CRITICAL(&encoderMutex);
}

float Motor::getCurrentMaxRpm(){
  return currentMaxRpm;
}

double Motor::getSmoothedRPM(unsigned long timeWindowUs) {  // Default: 200 ms
    EncoderHistory encoderHistoryBuffer[16];
    size_t localPointer;

    unsigned long now = micros();  // Get current time

    portENTER_CRITICAL(&encoderMutex);
    for (int i = 0; i < 16; ++i) {
        encoderHistoryBuffer[i].timestamp = encoderHistory[i].timestamp;
        encoderHistoryBuffer[i].dir = encoderHistory[i].dir;
        encoderHistoryBuffer[i].pwm = encoderHistory[i].pwm;
    }
    localPointer = encoderHistoryPointer;
    portEXIT_CRITICAL(&encoderMutex);

    double totalRPM = 0;
    unsigned int totalPWM = 0;
    int validSamples = 0;
    // for (int s = 1; s < 16; ++s) {
    //     size_t i = (localPointer - s + 16) % 16;
    //     size_t j = (localPointer - s - 1 + 16) % 16;
    //     mock_state[i] = static_cast<double>(encoderHistoryBuffer[i].timestamp - encoderHistoryBuffer[j].timestamp);
    // }
    // Start from latest and go back in time
    for (int s = 1; s < 16; ++s) {
        size_t i = (localPointer - s + 16) % 16;
        size_t j = (localPointer - s - 1 + 16) % 16;

        double dt = static_cast<double>(encoderHistoryBuffer[i].timestamp - encoderHistoryBuffer[j].timestamp) /  1e6;
        double dTicks = encoderHistoryBuffer[i].dir;
        unsigned int histPwm = encoderHistoryBuffer[i].pwm;
        // Stop if this data is too old
        if ((now - encoderHistoryBuffer[i].timestamp) > timeWindowUs) break;

        if (dt > 1e-5) {
          double degrees_motor = (dTicks) / (2 * tpr * gearR);
          totalRPM += (degrees_motor*60) / dt;
          totalPWM +=  histPwm;
          ++validSamples;
        }else{
          encoderHistoryBuffer[j] = encoderHistoryBuffer[i];
        }
    }
    if(validSamples > 0){
      double res =  totalRPM/validSamples;
      double avgPWM = totalPWM/validSamples;
      if(avgPWM > 230){
        if(res > currentMaxRpm)
          currentMaxRpm = res;
        else
          currentMaxRpm = 0.95f * currentMaxRpm + 0.05f * res;
      }
      return res;
    }
    return 0;
}

double Motor::get_deg() {
  portENTER_CRITICAL(&encoderMutex);
  double degrees_motor = (encoderValue / (float)tpr) * 180;
  portEXIT_CRITICAL(&encoderMutex);
  return degrees_motor / gearR;
}

void Motor::clearHistory() {
  portENTER_CRITICAL(&encoderMutex);
  encoderValue = 0;
  for(int i = 0; i < 16; i++){
    encoderHistory[i].timestamp = 0;
    encoderHistory[i].dir = 0;
    mock_state[i] = 0;
  }
  portEXIT_CRITICAL(&encoderMutex);
}

void Motor::updatePositionControl(float dt) {
  double currentPosition = get_deg();  // Current position in degrees
  float controlSignal = positionPid.update(currentPosition, dt);  // PID output
  int pwm_output = (int)controlSignal;
  move(pwm_output > 0, abs(pwm_output));
}

void Motor::updateSpeedControl(float dt) {
  double curr_rpm = getSmoothedRPM(DEFAULT_SPEED_FRAME);
  float safeMaxRpm = fmax(currentMaxRpm, 1.0f); // Avoid divide by zero
  float normalizedCurrentRPM = curr_rpm / safeMaxRpm;
  float u  = speedPid.update(normalizedCurrentRPM , dt);
  int delta = (int)(u * 255.0f);
  move(pwm + delta > 0, abs(pwm+delta));
}

void Motor::printMock(){
  getSmoothedRPM();
  Serial.print("Motor Sampling Time Delta:");
  Serial.print("(");
  double totalTime = 0;
  for(int i = 0; i < 16; i++){
    Serial.print(mock_state[i]);
    totalTime+=mock_state[i];
    Serial.print(", ");
  }
  Serial.println("), ");
  Serial.print("Motor Sampling average Hz:");
  double averageTime = totalTime/16;
  Serial.println( 1e6 / averageTime);
}

#define EXPAND(x) x

// Interrupt Helpers
#define ENCODER_PINS \
  X(ENC_A_1)         \
  X(ENC_B_1)         
 // X(ENC_B_2)     
 // X(ENC_A_2)         \


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