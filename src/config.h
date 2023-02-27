#ifndef CONFIG_HEADER
#define CONFIG_HEADER

#include <Arduino.h>

#define MOTOR_1_PWM PB4
#define MOTOR_1_DIR PB1
#define MOTOR_2_PWM PB5
#define MOTOR_2_DIR PB2
#define MOTOR_3_PWM PB10
#define MOTOR_3_DIR PB15
// /!\ THIS CODE WORKS ONLY IF YOU DO THE FOLLOWING : 
//remove R34 and R36
//Solder SB48 and SB49
#define ENCODER_1_A PC13
#define ENCODER_1_B PC14
#define ENCODER_2_A PC3
#define ENCODER_2_B PC2
#define ENCODER_3_A PC9 
#define ENCODER_3_B PC8

#define INCREMENT_TO_METRE 0.000375// 1.0/2666.66
#define VITESSE_CONSIGNE_TO_PWM_MOTOR 242.45

#define ANGLE_M1 0.0
#define ANGLE_M2 2*PI/3
#define ANGLE_M3 -2*PI/3
#define RAYON 0.115

#define MAX_ACCEL 2.0f //m.s^2

#define MAX_VITESSE 0.5f //m.s^-1
#define MAX_VITESSE_ROTATION 0.4f //rad.s^-1
#define DECELERATION_AVEC_DISTANCE 1.0f //m.s^-2

#endif // CONFIG_HEADER