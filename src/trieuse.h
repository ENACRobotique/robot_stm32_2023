#ifndef TRIEUSE_H
#define TRIEUSE_H

#include "config.h"
#include <AccelStepper.h>
#include "AX12A.h"
#include <Servo.h>
#include <AccelStepper.h>

//bras
typedef enum{
    IDLE_ELBOW,
    INWARDS = 145,
    OUTWARDS = 525
} elbowState_t;

typedef enum{
    INITIALISATION,
    IDLE_BRAS,
    UP=-10,
    DOWN=-2100,
}armState_t;

typedef enum{
    IDLE_MAIN,
    UNGRAB=200,
    GRAB=500
}handState_t;

//plateau
typedef enum 
{
    PLATE_INIT = -1,  // 
    ONE = 60,   //1
    TWO = 120,  //2
    THREE = 180,//3
    FOUR = 240, //4
    FIVE = 300, //5
    SIX = 360,  //6

}plate_pos;

//griffes
int openLpos = 2000;
int openRpos = 1000;
int closedLpos = 1100;
int closedRpos = 1830;

class ARM
{
    public :
        ARM(int PinFinCourse, int NumStepper, int idAX12Elbow, int idAX12Main,DynamixelSerial* AX12As); // + servo + ax12
        void init(HardwareSerial *serialDynamixel);
        void toggleBras(int choice = 2); // 0 pour en bas, 1 pour en haut, le reste pour 
                                         //l'endroit où le bras n'est pas.
        void toggleElbow(int choice = 2);// 0 pour l'intérieur, 1 pour l'extérieur, le 
                                         //reste por une valeur par défaut
        void toggleMain(int choice=2);   // 0 pour lâcher, 1 pour attraper, le 
                                         //reste por une valeur par défaut
        void update();

    private:
        AccelStepper lift_stepper;
        DynamixelSerial* AX12A;
        armState_t etatBras;
        elbowState_t etatAX;
        handState_t etatMain;
        int pinFinCourse;
        int pinSTP;
        int pinDIR;
        uint8_t ID_AX_Bras;
        uint8_t ID_AX_Main;

};

class PLATE
{
    public :
        PLATE(int num_stepper, int pin_zero); // stepper + fin de course
        void init();
        void update(int position); // rotation a sens unique
    
    private :
        AccelStepper _plate_stepper;
        int _pin_zero;      
        int _num_stepper;
        int _pin_STP;
        int _pin_DIR;
        int _position;

};

class CLAW
{
    public:
        CLAW(int pin_servo_gauche, int pin_servo_droite);
        void init();
        void update(bool state);
    
    private:
        Servo _Servo_Gauche;
        Servo _Servo_Droite;
        int _pin_servo_gauche;
        int _pin_servo_droite;
        bool _state;

};

#endif