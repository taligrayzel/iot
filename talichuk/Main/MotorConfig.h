#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H


// Motor A Pin Configuration (Hardware-Specific)
#define IN_A_1 14
#define IN_A_2 12
#define ENC_A_1 34
#define ENC_A_2 35
#define PWM_A_1 0
#define PWM_A_2 1

#define A_PP 0.85
#define A_PI 0.02
#define A_PD 0.03
#define A_SP 0.608
#define A_SI 0.15
#define A_SD 0.00

// Motor B Pin Configuration (Hardware-Specific)
#define IN_B_1 19
#define IN_B_2 18
#define ENC_B_1 33
#define ENC_B_2 32
#define PWM_B_1 2
#define PWM_B_2 3

#define B_PP 0.85
#define B_PI 0.02
#define B_PD 0.03
#define B_SP 0.608
#define B_SI 0.15
#define B_SD 0.00

// Miscellaneous
#define GEAR_RATIO 100
#define TPR 10.56 // TPR + magic number


#define KICKSTART_DELAY 15
#define FORWARD 1
#define REVERSE 0


#define WHEEL_RADIUS 22.5 //in mm
#define WHEEL_BASE 85

#define DEFAULT_SPEED_TIMEOUT 40000
#define DEFAULT_SPEED_FRAME 20000 
// Forward declare the global encoder map

#define TICK_TO_CONTROL 5

//Motor A has correct degree for divisor of 60 degrees at calculation.
// jump starts for 250 initial 5 delay @ 110. 
 

#endif // MOTORCONFIG_H
