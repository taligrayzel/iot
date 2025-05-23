#pragma once
#include <Arduino.h>

void attachAllEncoderInterrupts();

class Motor {
public:
  /**
   * Constructor
   * @param in1 GPIO for motor IN1 (PWM)
   * @param in2 GPIO for motor IN2 (PWM)
   * @param enc1 GPIO for encoder channel A
   * @param enc2 GPIO for encoder channel B
   * @param pwmChannel1 LEDC PWM channel for IN1
   * @param pwmChannel2 LEDC PWM channel for IN2
   * @param gearR Gear ratio
   * @param tpr Encoder tick per revolution
   */
  Motor(int in1, int in2, int enc1, int enc2, int pwmChannel1, int pwmChannel2, int gearR, int tpr);

  // Motor controls
  void forward(int speed = 255);  // speed: 0–255
  void reverse(int speed = 255);
  void stop();

  // Encoder handling
  void updateEncoder();
  void updateMovement();
  double get_deg();
  void set_movement(double target);
  void clear_movement();
  void printMock();

private:
  int in1, in2;              // Motor driver control pins
  int enc1, enc2;            // Encoder pins
  int gearR, tpr;                 // Gear ratio (for degrees)
  int pwmChannel1, pwmChannel2; // PWM channels
  volatile long encoderValue;   // Optional use for more precise tracking
  volatile long lastEncoded;
  volatile int8_t encoder_mock[16];

  // movement params
  double target, pos; 
  float kp, kd, ki;
  long prevT;
  float eprev;
  float eintegral;

  portMUX_TYPE encoderMutex;
};
