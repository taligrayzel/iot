#pragma once
#include <Arduino.h>

#include "MotorConfig.h"
#include "PIDController.h"
#include "DataBase.h"


struct EncoderHistory{
  unsigned long timestamp;
  int8_t dir;
  uint8_t pwm;   
};


// Enum to define the drive modes of the motor
enum class DriveMode {
    IDLE,          // Motor is idle, not doing anything
    DRIVE // Motor is controlling position (e.g., degrees)
};


void attachAllEncoderInterrupts();

class Motor {
public:
  /**
   * Constructor
   * @param name Motor name
   * @param in1 GPIO for motor IN1 (PWM)
   * @param in2 GPIO for motor IN2 (PWM)
   * @param enc1 GPIO for encoder channel A
   * @param enc2 GPIO for encoder channel B
   * @param pwmChannel1 LEDC PWM channel for IN1
   * @param pwmChannel2 LEDC PWM channel for IN2
   * @param gearR Gear ratio
   * @param tpr Encoder tick per revolution
   */
  Motor(String name, int in1, int in2, int enc1, int enc2, int pwmChannel1, int pwmChannel2, int gearR, float tpr, float pp, float pi, float pd, float sp, float si, float sd);

  // Motor control functions
  void stop();
  void move(bool dir, int speed = 255);

  // Encoder handling functions
  void updateEncoder();  // Handle encoder updates

  void setPositionTarget(float targetPosition);
  void setSpeedTarget(float normalizedTargetRPM);

  // RPM calculation and control
  double getSmoothedRPM(unsigned long timeWindowUs = DEFAULT_SPEED_FRAME);
  float getCurrentMaxRpm() const;
  
  void clearHistory();

  // Speed control adjustments
  void updateNoSpeed(uint8_t newNoSpeed);  // Update noSpeed value

  // Getter for position in degrees
  double get_deg();
  float getTargetSpeed() const;
  float getTargetPosition() const;
  DriveMode get_DriveMode() const;

  // Debugging and testing
  void printMock();
  void tick(float dt);  // Handle updates to motor control in the current mode

  bool updatePositionControl(float dt);

  // Jump start handling
  void enableJumpStart();  // Enable jump start mode
  void disableJumpStart(); // Disable jump start mode
  bool getJumpStart() const;  // Check if jump start is enabled

  void set_logger(DataBase* logger);
  void setToMove();

private:
  void logPID(float input, float output, bool positional);
  // Reset control states
  void clearPositionControl();   // Reset position control
  void clearSpeedControl();      // Reset speed control
  float calculateNormalizedPosition(float currPosition);

  // Control updates
  void updateSpeedControl(float dt);

  // Motor pins and parameters
  int in1, in2;              // Motor driver control pins
  int enc1, enc2;            // Encoder pins
  int gearR;
  float tpr;                 // Gear ratio (for degrees)
  int pwmChannel1, pwmChannel2; // PWM channels
  String name;

  DataBase* db_logger;

  // PID Controllers for position and speed
  PIDController positionPid;
  PIDController speedPid;

  // Speed thresholds
  int lowSpeed;
  int noSpeed;

  bool jumpStart, movementDir;  // Flag for jump start mode

  // Encoder state tracking
  volatile long encoderValue;
  volatile size_t encoderHistoryPointer;
  volatile EncoderHistory encoderHistory[16];
  volatile EncoderHistory mock_state[16];

  // Movement parameters
  float currentMaxRpm;
  float targetPosition;
  uint8_t pwm;

  // Current motor drive mode (Position or Velocity control)
  DriveMode currentDriveMode = DriveMode::IDLE;  // Default is IDLE mode

  // Mutex for encoder state protection
  portMUX_TYPE encoderMutex;
};
