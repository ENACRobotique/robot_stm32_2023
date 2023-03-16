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

#define ESC PA6
#define SERVO_1 PA7
#define SERVO_2 PA8
#define SERVO_3 PA9
#define SERVO_4 PB6
#define SERVO_5 PC7

#define AX12A_PIN PC4

#define CLK_AFFICHEUR PB11
#define DIO_AFFICHEUR PB12

#define INCREMENT_TO_METRE 0.0003447// 1.0/2666.66 
#define VITESSE_CONSIGNE_TO_PWM_MOTOR 242.45

#define ANGLE_M1 0.0
#define ANGLE_M2 2*PI/3
#define ANGLE_M3 -2*PI/3
#define RAYON 0.105 //old: 0.115

#define MAX_ACCEL 2.0f //m.s^2


#define SEUIL_PROCHE 0.3f //m
#define MAX_VITESSE 0.65f //m.s^-1
#define MAX_VITESSE_PROCHE 0.25f //m.s^-1
#define TOL_DIST 0.02 //m

#define SEUIL_PROCHE_ROTATION 0.2f //rad
#define TOL_THETA 0.01 //rad
#define COEF_DAMP_THETA 1.0f 
#define MAX_VITESSE_ROTATION 0.8f //rad.s^-1

#define INTERFACE_DRIVER 1
#define STEPPER_1_STP PA10
#define STEPPER_1_DIR PA0
#define STEPPER_2_STP PC1
#define STEPPER_2_DIR PC0
#define STEP_SPEED 200 // step.s^-1
#define STEP_ACC 30 //step.s^-2
#define CM_TO_STEP 1 // À DÉFINIR step.cm^-1

#define FIN_COURSE_1 PA1
#define FIN_COURSE_2 PA15

#endif // CONFIG_HEADER