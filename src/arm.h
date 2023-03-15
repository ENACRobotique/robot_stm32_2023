#ifndef ARM_H
#define ARM_H

#include "config.h"
#include <AccelStepper.h>
#include "AX12A.h"
#include <Servo.h>
#include <AccelStepper.h>
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
    GRAB=200,
    UNGRAB=500
}handState_t;

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


#endif