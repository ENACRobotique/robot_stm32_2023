#ifndef ARM_H
#define ARM_H

#include "config.h"
#include <AccelStepper.h>
#include "AX12A.h"
#include <Servo.h>
#include <AccelStepper.h>
typedef enum{
    IDLE_ELBOW,
    INWARDS = 525,
    OUTWARDS = 830
} elbowState_t;

typedef enum{
    DOWN=-800,
    UP=-0,
    IDLE_BRAS,
    INITIALISATION,
}armState_t;

typedef enum{
    IDLE_MAIN,
    GRAB=1600,
    UNGRAB=2000
}handState_t;

class ARM
{
    public :
        ARM(int PinFinCourse, int NumStepper, int idAX12); // + servo + ax12
        void init(HardwareSerial serialDynamixel, int numServo);
        void toggleBras(int choice = 2); // 0 pour en bas, 1 pour en haut, le reste pour 
                                         //l'endroit où le bras n'est pas.
        void toggleElbow(int choice = 2);// 0 pour l'intérieur, 1 pour l'extérieur, le 
                                         //reste por une valeur par défaut
        void toggleMain(int choice=2);   // 0 pour lâcher, 1 pour attraper, le 
                                         //reste por une valeur par défaut
        void update();

    private:
        AccelStepper lift_stepper;
        DynamixelSerial AX12A;
        Servo actionMain;
        armState_t etatBras;
        elbowState_t etatAX;
        handState_t etatMain;
        int pinFinCourse;
        int pinSTP;
        int pinDIR;
        uint8_t ID_AX_Bras;

};


#endif