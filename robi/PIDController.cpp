#include "PIDController.h"
#include <algorithm> // For std::clamp

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd),
      setpoint(0.0f), integral(0.0f), prevError(0.0f),
      outputMin(-1e6f), outputMax(1e6f),
      integralMin(-1e6f), integralMax(1e6f) {
}

void PIDController::reset() {
    integral = 0.0f;
    prevError = 0.0f;
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
    return output;
}

float PIDController::getSetPoint() const{
  return setpoint;
}

float PIDController::simulate(float measured, float dt) const{
    float error = setpoint - measured;

    // Proportional
    float P = kp * error;

    // Integral (accumulated with manual clamping)
    float integral_var = integral + error * dt;
    if (integral_var > integralMax) integral_var = integralMax;
    else if (integral_var < integralMin) integral_var = integralMin;
    float I = ki * integral_var;

    // Derivative
    float derivative = (dt > 0.0f) ? (error - prevError) / dt : 0.0f;
    float D = kd * derivative;

    // Compute PID output and manually clamp
    float output = P + I + D;
    if (output > outputMax) return outputMax;
    if (output < outputMin) return outputMin;
    return output;
}
