#include "Motor.h"
// Motor.cpp
#include "DebugConfig.h"
#define DPRINT(x)    DPRINT_MOTOR(x)
#define DPRINTLN(x)  DPRINTLN_MOTOR(x)
#define LOG_FUNC(x) LOG_MOTOR_MSG(x)

// Global encoder-to-motor pointer map
Motor* encoderMap[40] = { nullptr };

Motor::Motor(String name, int in1, int in2, int enc1, int enc2, int pwmChannel1, int pwmChannel2, int gearR, float tpr, float pp, float pi, float pd, float sp, float si, float sd)
  : name(name), in1(in1), in2(in2), enc1(enc1), enc2(enc2), gearR(gearR), tpr(tpr),
    pwmChannel1(pwmChannel1), pwmChannel2(pwmChannel2),
    positionPid(pp, pi, pd , false, 0.01), speedPid(sp, si, sd, true) {

  encoderHistoryPointer = 0;

  positionPid.setOutputLimits(-1.0f, 1.0f);
  speedPid.setOutputLimits(-1.0f, 1.0f);
  speedPid.setIntegralLimits(-0.5f, 0.5f);
  positionPid.setIntegralLimits(-0.15f, 0.15f);  // avoid windup

  for(int i = 0; i < 16; i++){
    encoderHistory[i].timestamp = 0;
    encoderHistory[i].dir = 0;
    encoderHistory[i].pwm = 0;
    mock_state[i].timestamp = 0;
    mock_state[i].dir = 0;
    mock_state[i].pwm = 0;
  }

  encoderMap[enc1] = this;
  encoderMap[enc2] = this;

  pinMode(enc1, INPUT_PULLUP);
  pinMode(enc2, INPUT_PULLUP);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Setup LEDC PWM channels at 15kHz, 8-bit resolution
  ledcSetup(pwmChannel1, 15000, 8);
  ledcSetup(pwmChannel2, 15000, 8);

  ledcAttachPin(in1, pwmChannel1);
  ledcAttachPin(in2, pwmChannel2);

  encoderValue = 0;
  encoderMutex = portMUX_INITIALIZER_UNLOCKED;

  updateNoSpeed(50);
  jumpStart = true;
  positionPid.reset();
  speedPid.reset();
  pwm = 0;
  currentMaxRpm = 70;
}

void Motor::set_logger(DataBase* logger){
  db_logger = logger;
}

void Motor::tick(float dt) {
  DPRINT("Tick: ");

  switch (currentDriveMode) {
    case DriveMode::DRIVE:
      DPRINTLN(" Current motor mode: Drive ");
      updateSpeedControl(dt);
      break;

    case DriveMode::IDLE:
      DPRINTLN(" Current motor mode: Idle ");
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
    mock_state[encoderHistoryPointer].timestamp = currT;
    mock_state[encoderHistoryPointer].dir = 0;
    mock_state[encoderHistoryPointer].pwm = pwm;
    encoderHistoryPointer = (encoderHistoryPointer + 1) % 16;

    if (failedPwm > noSpeed) {
      updateNoSpeed(noSpeed + 7);
    }
    jumpStart = true;
  } else {
    if(pwm < noSpeed + 25)
      updateNoSpeed(max((int)30 , (int)noSpeed - 2));
  }
  portEXIT_CRITICAL(&encoderMutex);
}

float Motor::calculateNormalizedPosition(float currPosition){
  float res = currPosition - targetPosition;
  res /= 360.0f;
  if (res > 1.0f) res = 1.0f;
  if (res < -1.0f) res = -1.0f;
  return res;
}

void Motor::setPositionTarget(float targetPosition) {
  clearPositionControl();
  this->targetPosition = get_deg() + targetPosition;
}

void Motor::setSpeedTarget(float normalizedTargetRPM) {
  currentDriveMode = DriveMode::DRIVE;
  speedPid.setSetpoint(normalizedTargetRPM);
}

void Motor::enableJumpStart() {
  jumpStart = true;
}

void Motor::disableJumpStart() {
  jumpStart = false;
}

bool Motor::getJumpStart() const {
  return jumpStart;
}

void Motor::move(bool dir, int speed) {
  currentDriveMode = DriveMode::DRIVE;
  int pwm1 = dir ? pwmChannel1 : pwmChannel2;
  int pwm2 = dir ? pwmChannel2 : pwmChannel1;

  if(jumpStart && speed < lowSpeed){
    ledcWrite(pwm1, 250);
    ledcWrite(pwm2, 0);
    delay(KICKSTART_DELAY);
    jumpStart = false;
  }

  portENTER_CRITICAL(&encoderMutex);
  ledcWrite(pwm1, (uint32_t)speed);
  ledcWrite(pwm2, 0);
  pwm = speed;
  movementDir = dir;
  portEXIT_CRITICAL(&encoderMutex);
}

void Motor::stop() {
  ledcWrite(pwmChannel1, 0);
  ledcWrite(pwmChannel2, 0);
  currentDriveMode = DriveMode::IDLE;
}

void Motor::updateNoSpeed(uint8_t newNoSpeed) {
  noSpeed = max((uint8_t)0 , newNoSpeed);
  lowSpeed = min(noSpeed + 50, 255);
}

void Motor::clearPositionControl() {
  positionPid.reset();
  targetPosition = get_deg();
}

void Motor::clearSpeedControl() {
  speedPid.reset();
}

void Motor::updateEncoder() {
  portENTER_CRITICAL(&encoderMutex);
  int deltaTicks = movementDir ? 1 : -1;

  encoderHistory[encoderHistoryPointer].timestamp = micros();
  encoderHistory[encoderHistoryPointer].dir = deltaTicks;
  encoderHistory[encoderHistoryPointer].pwm = pwm;

  mock_state[encoderHistoryPointer].timestamp = encoderHistory[encoderHistoryPointer].timestamp;
  mock_state[encoderHistoryPointer].dir = deltaTicks;
  mock_state[encoderHistoryPointer].pwm = pwm;

  encoderHistoryPointer = (encoderHistoryPointer + 1) % 16;
  encoderValue += deltaTicks;
  portEXIT_CRITICAL(&encoderMutex);
}

float Motor::getCurrentMaxRpm() const {
  return currentMaxRpm;
}

double Motor::getSmoothedRPM(unsigned long timeWindowUs) {
  EncoderHistory encoderHistoryBuffer[16];
  size_t localPointer;

  unsigned long now = micros();

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

  for (int s = 1; s < 16; ++s) {
    size_t i = (localPointer - s + 16) % 16;
    size_t j = (localPointer - s - 1 + 16) % 16;

    double dt = static_cast<double>(encoderHistoryBuffer[i].timestamp - encoderHistoryBuffer[j].timestamp) / 1e6;
    double dTicks = encoderHistoryBuffer[i].dir;
    unsigned int histPwm = encoderHistoryBuffer[i].pwm;

    if ((now - encoderHistoryBuffer[i].timestamp) > timeWindowUs) break;

    if (dt > 1e-5) {
      double degrees_motor = (dTicks) / (2 * tpr * gearR);
      totalRPM += (degrees_motor * 60) / dt;
      totalPWM += histPwm;
      ++validSamples;
    } else {
      encoderHistoryBuffer[j] = encoderHistoryBuffer[i];
    }
  }

  if(validSamples > 0){
    double res = totalRPM / validSamples;
    double abs_res = fabs(res);
    double avgPWM = totalPWM / validSamples;
    if(abs_res > currentMaxRpm)
      currentMaxRpm = abs_res;
    else if(avgPWM > 230)
      currentMaxRpm = 0.95f * currentMaxRpm + 0.05f * abs_res;
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
    encoderHistory[i].pwm = 0;
    mock_state[i].pwm = 0;
    mock_state[i].timestamp = 0;
    mock_state[i].dir = 0;
  }
  portEXIT_CRITICAL(&encoderMutex);
}

bool Motor::updatePositionControl(float dt) {
  double currentPosition = get_deg();
  float normalizedPos = calculateNormalizedPosition(currentPosition);
  float velocitySetpoint = positionPid.update(normalizedPos, dt);

  DPRINT("----- Position Control ----- Real Target: ");
  DPRINT(targetPosition);
  DPRINT(" || Real Position: ");
  DPRINT(currentPosition);
  DPRINT(" || Normalized Position: ");
  DPRINT(normalizedPos);
  DPRINT(" || PID output: ");
  DPRINTLN(velocitySetpoint);

  if(positionPid.finished()){
    clearPositionControl();
    stop();
    return true;
  }

  speedPid.setSetpoint(velocitySetpoint);
  return false;
}

void Motor::updateSpeedControl(float dt) {
  double curr_rpm = getSmoothedRPM(DEFAULT_SPEED_FRAME);
  float safeMaxRpm = fmax(currentMaxRpm, 1.0f);

  logPID(speedPid.getSetPoint() * safeMaxRpm, curr_rpm, false);

  float normalizedCurrentRPM = curr_rpm / safeMaxRpm;
  float u = speedPid.update(normalizedCurrentRPM, dt);

  DPRINT("----- Speed Control ----- Target: ");
  DPRINT(speedPid.getSetPoint());
  DPRINT(" (");
  DPRINT(speedPid.getSetPoint() * safeMaxRpm);
  DPRINT(") || Current Max RPM: ");
  DPRINT(currentMaxRpm);
  DPRINT(" || Current RPM: ");
  DPRINT(curr_rpm);
  DPRINT(" (");
  DPRINT(normalizedCurrentRPM);
  DPRINT(") || PID output: ");
  DPRINT(u);

  int delta = (int)(u * 60.0f);
  DPRINT(" (");
  DPRINT(delta);
  DPRINTLN(")");
  portENTER_CRITICAL(&encoderMutex);
  int output_pwm = movementDir ? pwm + delta : -pwm + delta;
  portEXIT_CRITICAL(&encoderMutex);

  bool direction = output_pwm > 0;
  output_pwm = abs(output_pwm);
  output_pwm = output_pwm > 255 ? 255 : output_pwm;

  DPRINT(" || Command sent: ");
  DPRINT(output_pwm);
  DPRINT(" (");
  DPRINT(direction);
  DPRINTLN(")");

  move(direction, output_pwm);
}

void Motor::printMock() {
  getSmoothedRPM();
  DPRINT("Motor Sampling Time Delta: [");
  double totalTime = 0;
  for(int i = 0; i < 16; i++){
    DPRINT("(");
    DPRINT(mock_state[i].timestamp);
    DPRINT(",");
    DPRINT(mock_state[i].dir);
    DPRINT(",");
    DPRINT(mock_state[i].pwm);
    DPRINT("), ");
    totalTime += mock_state[i].timestamp;
  }
  DPRINTLN("]");

  double averageTime = totalTime / 16;
  DPRINT("Motor Sampling average Hz: ");
  DPRINTLN(1e6 / averageTime);
}

float Motor::getTargetSpeed() const {
  return speedPid.getSetPoint();
}

float Motor::getTargetPosition() const {
  return targetPosition;
}

DriveMode Motor::get_DriveMode() const {
  return currentDriveMode;
}

void Motor::setToMove() {
  currentDriveMode = DriveMode::DRIVE;
}

void Motor::logPID(float input, float output, bool positional) {
  LOG_FUNC(
    float pwm_w_dir = movementDir ? (float)pwm : -(float)pwm;
    const char* logType = positional ? "position" : "velocity";
    DPRINTLN(String("----- ") + (positional ? "Position" : "Speed") + " PID LOG -----");

    if (db_logger && db_logger->isReady()) {
      ILogRecord* rec = db_logger->createRecord("PID", logType, false);
      if (!rec) {
        DPRINTLN("WARNING: PID log skipped: record creation failed");
        return;
      }

      rec->set("motor", name)
         .set("input", input)
         .set("output", output)
         .set("pwm", pwm_w_dir)
         .set("error", positional ? positionPid.getPrevError() : speedPid.getPrevError())
         .set("integral", positional ? positionPid.getIntegError() : speedPid.getIntegError());

      if (!rec->commit(false)) {
        DPRINTLN("WARNING: PID log skipped: commit failed");
        delete rec;  // safe cleanup if commit failed
      }
    }
  );
}


// Interrupt Helpers and Attachments

#define EXPAND(x) x

#define ENCODER_PINS \
  X(ENC_A_1)         \
  X(ENC_B_1)

#define X(PIN) \
  void IRAM_ATTR encoderISR##PIN() { \
    if (encoderMap[EXPAND(PIN)]) { \
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
