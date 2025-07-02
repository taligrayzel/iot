#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, bool endless, float minimalErr)
    : kp(kp), ki(ki), kd(kd),
      setpoint(0.0f), integral(0.0f), prevError(0.0f), minimalErr(minimalErr),
      outputMin(-1e6f), outputMax(1e6f),
      integralMin(-1e6f), integralMax(1e6f),
      endless(endless), reached(false){
}

void PIDController::reset() {
    integral = 0.0f;
    prevError = 0.0f;
    reached = false;
}

bool PIDController::finished(){
  return !endless && reached;
}

void PIDController::setOutputLimits(float minOutput, float maxOutput) {
    outputMin = minOutput;
    outputMax = maxOutput;
}

void PIDController::setIntegralLimits(float minIntegral, float maxIntegral) {
    integralMin = minIntegral;
    integralMax = maxIntegral;
}

void PIDController::setSetpoint(float setpoint) {
    this->setpoint = setpoint;
}

float PIDController::update(float measured, float dt) {
    float error = setpoint - measured;

    // Proportional
    float P = kp * error;

    // Integral (accumulated with manual clamping)
    integral += error * dt;
    if (integral > integralMax) integral = integralMax;
    else if (integral < integralMin) integral = integralMin;
    float I = ki * integral;

    // Derivative
    float derivative = (dt > 0.0f) ? (error - prevError) / dt : 0.0f;
    float D = kd * derivative;

    prevError = error;

    // Compute PID output and manually clamp
    float output = P + I + D;
    if (output > outputMax) return outputMax;
    if (output < outputMin) return outputMin;
    reached = !endless && fabs(error) < minimalErr; 
    return output;
}

float PIDController::getSetPoint() const{
  return setpoint;
}

float PIDController::getPrevError() const{
  return prevError;
}
float PIDController::getIntegError() const{
  return integral;
}