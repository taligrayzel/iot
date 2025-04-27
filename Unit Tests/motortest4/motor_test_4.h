#pragma once
#include <Arduino.h>

// Forward declare the global encoder map
extern class Motor* encoderMap[40];

void attachAllEncoderInterrupts();

class Motor {
public:
  /**
   * Constructor
   * @param in1 GPIO for motor IN1 (PWM)
   * @param in2 GPIO for motor IN2 (PWM)
   * @param enc1 GPIO for encoder channel A
   * @param enc2 GPIO for encoder channel B
   * @param gearR Gear ratio for degree conversion
   * @param pwmChannel1 LEDC PWM channel for IN1
   * @param pwmChannel2 LEDC PWM channel for IN2
   */
  Motor(int in1, int in2, int enc1, int enc2, int gearR, int pwmChannel1, int pwmChannel2);

  // Motor controls
  void forward(int speed = 255);  // speed: 0–255
  void reverse(int speed = 255);
  void stop();

  // Encoder handling
  void updateEncoder();
  double get_deg();
  void clear_movement();

private:
  int in1, in2;              // Motor driver control pins
  int enc1, enc2;            // Encoder pins
  int gearR;                 // Gear ratio (for degrees)
  int pwmChannel1, pwmChannel2; // PWM channels
  volatile long encoderValue;   // Optional use for more precise tracking
  volatile long lastEncoded;
};