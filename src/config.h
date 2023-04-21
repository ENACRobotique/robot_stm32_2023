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
#define ENCODER_1_A PC14
#define ENCODER_1_B PC13
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

#define DISPLAY_CLK PB11
#define DISPLAY_DATA PB12
#define COLOR PC6
#define POS_BUTTON PC5
#define TIRETTE PC15

#define INCREMENT_TO_METRE 0.0002605932
#define VITESSE_CONSIGNE_TO_PWM_MOTOR 242.45

#define ANGLE_M1 0.0
#define ANGLE_M2 2*PI/3
#define ANGLE_M3 -2*PI/3
// si le robot tourne trop -> baisser le rayon
#define RAYON 0.1314 //m

#define MAX_ACCEL 2.0f //m.s^2


#define SEUIL_PROCHE 0.2f //m
#define MAX_VITESSE 0.65f //m.s^-1
#define MAX_VITESSE_PROCHE 0.25f //m.s^-1
#define TOL_DIST 0.005f //m

#define SEUIL_PROCHE_ROTATION 0.2f //rad
#define TOL_THETA 0.01 //rad
#define COEF_DAMP_THETA 1.0f 
#define MAX_VITESSE_ROTATION 2.0f //rad.s^-1
#define MAX_VITESSE_ROTATION_PROCHE 0.2f //rad.s^-1

#define INTERFACE_DRIVER 1
#define STEPPER_1_STP PA10
#define STEPPER_1_DIR PA0
#define STEPPER_2_STP PC1
#define STEPPER_2_DIR PC0
#define STEP_MAX_SPEED 2000 // step.s^-1
#define STEP_SPEED 2000 // step.s^-1
#define STEP_ACC 2000 //step.s^-2
#define CM_TO_STEP 1 // À DÉFINIR step.cm^-1

#define FIN_COURSE_1 PA1
#define FIN_COURSE_2 PA15

#define DEG_TO_STEP 4.44f


#define ANGLE_PLAT -2.435257905307688

#endif // CONFIG_HEADER