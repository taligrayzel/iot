#include "motor_test_4.h"

#define IN_A_1 18
#define IN_A_2 19
#define ENC_A_1 32
#define ENC_A_2 33
#define PWM_A_1 10
#define PWM_A_2 11

#define GEAR_RATIO 50



Motor m1(IN_A_1, IN_A_2, ENC_A_1, ENC_A_2, GEAR_RATIO, PWM_A_1, PWM_A_2);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Robot setup complete");
}

void loop() {
  
  m1.forward(255);
  delay(3000);
  m1.reverse(255);
  delay(3000);
  Serial.println(m1.get_deg());

}