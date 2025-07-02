#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include <cmath>

class PIDController {
public:
    PIDController(float kp, float ki, float kd, bool endless, float minimalErr = 1.0f);

    void reset();

    void setOutputLimits(float minOutput, float maxOutput);
    void setIntegralLimits(float minIntegral, float maxIntegral);
    void setSetpoint(float setpoint);  
    
    float update(float measured, float dt);  
    bool finished();

    float getSetPoint() const;
    float getPrevError() const;
    float getIntegError() const;

private:
    float kp, ki, kd;        // PID constants
    float setpoint;          // Target value

    float integral;          // Integral term accumulator
    float prevError, minimalErr;         // Previous error (for derivative)
    
    float outputMin, outputMax;         // Output clamping range
    float integralMin, integralMax;     // Integral clamping range
    bool endless, reached;     // Integral clamping range
};

#endif // PIDCONTROLLER_H
