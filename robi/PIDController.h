#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    void reset();

    void setOutputLimits(float minOutput, float maxOutput);
    void setIntegralLimits(float minIntegral, float maxIntegral);
    void setSetpoint(float setpoint);  
    
    float update(float measured, float dt);  

    float simulate(float measured, float dt) const;  
    float getSetPoint() const;
private:
    float kp, ki, kd;        // PID constants
    float setpoint;          // Target value

    float integral;          // Integral term accumulator
    float prevError;         // Previous error (for derivative)
    
    float outputMin, outputMax;         // Output clamping range
    float integralMin, integralMax;     // Integral clamping range
};

#endif // PIDCONTROLLER_H
