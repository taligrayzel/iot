#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
  const int in1, in2;
  const int enc1, enc2;
  const int gearR;
  volatile int lastEncoded;
  volatile long encoderValue;

public:
  Motor(int in1, int in2, int enc1, int enc2, int gearR);
  void forward();
  void reverse();
  void stop();
  void setSpeed(int speed); // Optional
  double get_deg();
  void clear_movement();
  void updateEncoder();
};

extern Motor* encoderMap[40]; // Global map of GPIO to motor for ISR use

void attachAllEncoderInterrupts();

#endif
