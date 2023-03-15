#include "arm.h"


ARM::ARM(int PinFinCourse, int NumStepper, int idAX12Elbow, int idAX12Main, DynamixelSerial* AX12){
    this->ID_AX_Bras = idAX12Elbow;
    this->pinFinCourse = PinFinCourse;
    this->AX12A=AX12;
    this->ID_AX_Main=idAX12Main;
    this->etatBras = IDLE_BRAS;
    this->etatMain = IDLE_MAIN;
    this->etatAX = IDLE_ELBOW;
    switch (NumStepper)
    {
    case 1:
        this->pinDIR = STEPPER_1_DIR;
        this->pinSTP = STEPPER_1_STP;
        break; 

    case 2:
        this->pinDIR = STEPPER_2_DIR;
        this->pinSTP = STEPPER_2_STP;
        break;

    default:
        while(1);
    }
}

void ARM::init(HardwareSerial* serialDynamixel){
    this->lift_stepper = AccelStepper (INTERFACE_DRIVER, this->pinSTP, this->pinDIR);
    this->AX12A->init(serialDynamixel);
    this->etatBras = INITIALISATION;
    this->etatMain = UNGRAB;
    this->etatAX = OUTWARDS;
    this->AX12A->setCMargin(4,5,5);
    this->toggleElbow(1);
    this->toggleMain(0);
    this->lift_stepper.setMaxSpeed(2000);
    this->lift_stepper.setAcceleration(4000);
    this->lift_stepper.setSpeed(400);
    pinMode(this->pinFinCourse,INPUT_PULLUP);
}

void ARM::update(){
    switch (this->etatBras){
        case INITIALISATION:
            if (!digitalRead(this->pinFinCourse)){
                this->etatBras= IDLE_BRAS;
                this->etatAX=INWARDS;
                this->AX12A->move(this->ID_AX_Bras,(int)INWARDS);
                this->lift_stepper.setCurrentPosition(0);
                this->lift_stepper.setSpeed(0);
            }
            else{;
            this->lift_stepper.runSpeed();
            }
            break;
        case UP:
            this->lift_stepper.moveTo(UP);
            this->lift_stepper.run();
            break;
        case DOWN:
            this->lift_stepper.moveTo((long)DOWN);
            this->lift_stepper.run();
            break;
        case IDLE_BRAS:
            break;
    }
}
void ARM::toggleBras(int choice){
    switch (choice){
        case 0:
            this->lift_stepper.moveTo((long)DOWN);
            this->etatBras=DOWN;
            break;
        case 1:
            this->lift_stepper.moveTo(UP);
            this->etatBras=UP;
            break;
        default:
            this->toggleBras(((this->etatBras)==DOWN)?1:0);
    }
}
void ARM::toggleElbow(int choice){
    switch (choice){
        case 0:
            this->AX12A->move((unsigned char)(this->ID_AX_Bras),(int)INWARDS);
            this->etatAX=INWARDS;
            break;
        case 1:
            this->AX12A->move(this->ID_AX_Bras,(int)OUTWARDS);
            this->etatAX=OUTWARDS;
            break;
        default:
            toggleElbow(((this->etatAX)==OUTWARDS)?INWARDS:OUTWARDS);
            break;
    }
}
void ARM::toggleMain(int choice){
    switch(choice){
        case 0:
            this->AX12A->move(this->ID_AX_Main,UNGRAB);
            this->etatMain = UNGRAB;
            break;
        case 1:
            this->AX12A->move(this->ID_AX_Main,GRAB);
            this->etatMain = GRAB;
            break;
        default:
            toggleMain(((this->etatMain)==GRAB)?0:1);
    }
}
