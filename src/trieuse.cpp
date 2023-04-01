#include "trieuse.h"

//############# BRAS #############

ARM::ARM(int PinFinCourse, int NumStepper, int idAX12Elbow, int idAX12Main, DynamixelSerial* AX12){
    this->ID_AX_Bras = idAX12Elbow;
    this->pinFinCourse = PinFinCourse;
    this->AX12A=AX12;
    this->ID_AX_Main = idAX12Main;
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
    this->lift_stepper.setMaxSpeed(STEP_MAX_SPEED);
    this->lift_stepper.setAcceleration(STEP_ACC);
    this->lift_stepper.setSpeed(STEP_SPEED);
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
            else{this->lift_stepper.runSpeed();}
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



//############# PLATEAU #############//
PLATE::PLATE(int num_stepper, int pin_zero)
{
    this->_num_stepper = num_stepper;
    this->_pin_zero = pin_zero;
    switch (num_stepper)
    {
    case 1:
        this->_pin_DIR = STEPPER_1_DIR;
        this->_pin_STP = STEPPER_1_STP;
        break; 

    case 2:
        this->_pin_DIR = STEPPER_2_DIR;
        this->_pin_STP = STEPPER_2_STP;
        break;

    default:
        while(1);
    }
}

void PLATE::init()
{
    this->_plate_stepper = AccelStepper (INTERFACE_DRIVER, this->_pin_STP, this->_pin_DIR);
    this->_plate_stepper.setMaxSpeed(STEP_MAX_SPEED);
    this->_plate_stepper.setAcceleration(STEP_ACC);
    this->_plate_stepper.setSpeed(STEP_SPEED);
    pinMode(this->_pin_zero,INPUT_PULLUP);
    this->_plate_stepper.runSpeed();
    

}

void PLATE::update(int position)
{
    switch(this->_position)
    {
        case INIT:
            if(!digitalRead(this->_pin_zero))
            {
                this->_plate_stepper.setSpeed(0);
                this->_plate_stepper.setCurrentPosition(0);
            }
            else {this->_plate_stepper.runSpeed();};
            
            break;

        default:
            this->_plate_stepper.moveTo(0);
            this->_plate_stepper.run();
        
        case 1:
            this->_plate_stepper.moveTo(ONE);
            this->_plate_stepper.run();
            break;
        case 2:
            this->_plate_stepper.moveTo(TWO);
            this->_plate_stepper.run();
            break;
        case 3:
            this->_plate_stepper.moveTo(THREE);
            this->_plate_stepper.run();
            break;
        case 4:
            this->_plate_stepper.moveTo(FOUR);
            this->_plate_stepper.run();
            break;
        case 5:
            this->_plate_stepper.moveTo(FIVE);
            this->_plate_stepper.run();
            break;
        case 6:
            this->_plate_stepper.moveTo(SIX);
            this->_plate_stepper.run();
            break;        
    }
}

//############# PINCES #############

CLAW::CLAW(int pin_servo_gauche, int pin_servo_droite)
{
    this->_pin_servo_gauche = pin_servo_gauche;
    this->_pin_servo_droite = pin_servo_droite;
}

void CLAW::init()
{
    this->_Servo_Gauche.attach(_pin_servo_gauche);
    this->_Servo_Droite.attach(_pin_servo_droite);
}



//############# TRIEUSE #############
