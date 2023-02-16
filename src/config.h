#ifndef CONFIG_HEADER
#define CONFIG_HEADER

#include <Arduino.h>

#define MOTOR_1_PWM PB3 //D3
#define MOTOR_1_DIR PA10 //D2
#define MOTOR_2_PWM PB5 //D4
#define MOTOR_2_DIR PB4 //D5
#define MOTOR_3_PWM PA7 //D11
#define MOTOR_3_DIR PA6 //D12

#define ENCODER_1_A PA8
#define ENCODER_1_B PB10
#define ENCODER_2_A PA0
#define ENCODER_2_B PA1
#define ENCODER_3_A PC7 //D9
#define ENCODER_3_B PA9 //D8

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