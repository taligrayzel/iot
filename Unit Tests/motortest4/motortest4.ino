#include "motor_test_4.h"

#define IN_B_1 18
#define IN_B_2 19
#define ENC_B_1 32
#define ENC_B_2 33
#define PWM_B_1 0
#define PWM_B_2 1


#define IN_A_1 14
#define IN_A_2 23
#define ENC_A_1 34
#define ENC_A_2 35
#define PWM_A_1 2
#define PWM_A_2 3

#define GEAR_RATIO 50



Motor mA(IN_A_1, IN_A_2, ENC_A_1, ENC_A_2, GEAR_RATIO, PWM_A_1, PWM_A_2);
Motor mB(IN_B_1, IN_B_2, ENC_B_1, ENC_B_2, GEAR_RATIO, PWM_B_1, PWM_B_2);

void setup() {
  Serial.begin(115200);
  delay(1000);
  attachAllEncoderInterrupts();
  Serial.println("Robot setup complete");
}

void loop() {
  
  mA.stop(); 
  mB.stop();
  delay(3000);
  mA.reverse(255);
  mB.reverse(255);
  delay(3000);
  mA.forward(255);
  mB.forward(255);
  Serial.print("mA deg: ");
  Serial.println(mA.get_deg());
  Serial.print("mb deg: ");
  Serial.println(mB.get_deg());
}