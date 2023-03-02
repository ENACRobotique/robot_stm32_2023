#include "arm.h"


ARM::ARM(int PinFinCourse, int NumStepper, int idAX12, DynamixelSerial* AX12){
    this->ID_AX_Bras = idAX12;
    this->pinFinCourse = PinFinCourse;
    this->AX12A=AX12;
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
        while(1){};
    }
}

void ARM::init(HardwareSerial serialDynamixel, int numServo){
    this->lift_stepper = AccelStepper (INTERFACE_DRIVER, this->pinSTP, this->pinDIR);
    this->actionMain = Servo();
    int pinServo;
    switch (numServo){
        case 1:
            pinServo = SERVO_1;
            break;
        case 2:
            pinServo = SERVO_2;
            break;
        case 3:
            pinServo = SERVO_3;
            break;
        case 4:
            pinServo = SERVO_4;
            break;
        case 5:
            pinServo = SERVO_5;
            break;
        default:
            while(1){};
    }
    this->actionMain.attach(pinServo,2000);
    this->AX12A->init(&serialDynamixel);
    this->etatBras = INITIALISATION;
    this->etatMain = UNGRAB;
    this->etatAX = OUTWARDS;
    this->lift_stepper.setMaxSpeed(1000);
    this->lift_stepper.setAcceleration(2000);
    this->lift_stepper.setSpeed(500);
    pinMode(this->pinFinCourse,INPUT_PULLUP);
    this->AX12A->move(this->ID_AX_Bras,(int)OUTWARDS);
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
            else{
            this->lift_stepper.runSpeed();
            }
            break;
        case UP:
            this->lift_stepper.run();
            break;
        case DOWN:
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
            this->lift_stepper.moveTo((long)UP);
            this->etatBras=UP;
            break;
        default:
            this->lift_stepper.moveTo(((this->etatBras)==UP)?((long)DOWN):((long)UP));
            this->etatBras= ((this->etatBras)==UP)?DOWN:UP;
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
            this->AX12A->move(this->ID_AX_Bras,((this->etatAX)==OUTWARDS)?INWARDS:OUTWARDS);
            this->etatAX= ((this->etatAX)==OUTWARDS)?INWARDS:OUTWARDS;
            break;
    }
}
void ARM::toggleMain(int choice){
    switch(choice){
        case 0:
            this->actionMain.writeMicroseconds(UNGRAB);
            this->etatMain = UNGRAB;
            break;
        case 1:
            this->actionMain.writeMicroseconds(GRAB);
            this->etatMain = GRAB;
            break;
        default:
            this->actionMain.writeMicroseconds(((this->etatMain)==GRAB)?UNGRAB:GRAB);
            this->etatMain = ((this->etatMain)==GRAB)?UNGRAB:GRAB;
    }
}
