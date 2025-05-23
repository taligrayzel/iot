#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H


// Motor A Pin Configuration (Hardware-Specific)
#define IN_A_1 12
#define IN_A_2 14
#define ENC_A_1 34
#define ENC_A_2 35
#define PWM_A_1 0
#define PWM_A_2 1


// Motor B Pin Configuration (Hardware-Specific)
#define IN_B_1 18
#define IN_B_2 19
#define ENC_B_1 32
#define ENC_B_2 33
#define PWM_B_1 2
#define PWM_B_2 3

// Miscellaneous
#define GEAR_RATIO 50
#define TPR 14

// Forward declare the global encoder map


//Motor A has correct degree for divisor of 60 degrees at calculation.
// jump starts for 250 initial 5 delay @ 110. 
 
#endif // MOTORCONFIG_H
