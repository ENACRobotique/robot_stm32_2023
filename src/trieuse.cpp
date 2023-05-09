#include "trieuse.h"
#include "comm.h"


long stepper_pos[6] = {60, 120, 180, 240, 300, 360};

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
    this->etatMain = UNGRAB_MAIN;
    this->etatAX = OUTWARDS;
    this->AX12A->setCMargin(4,5,5);
    this->toggleElbow(OUTWARDS_CHOICE);
    this->toggleMain(UNGRAB_CHOICE);
    this->lift_stepper.setMaxSpeed(STEP_MAX_SPEED);
    this->lift_stepper.setAcceleration(STEP_ACC);
    this->lift_stepper.setSpeed(STEP_SPEED);
    pinMode(this->pinFinCourse,INPUT_PULLUP);
}
armState_t ARM::getEtatBras(){return this->etatBras;}

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
        case GRAB_INTERNE_UP:
            this->lift_stepper.moveTo((long)GRAB_INTERNE_UP);
            this->lift_stepper.run();
            break;
        case GRAB_INTERNE_MIDDLE:
            this->lift_stepper.moveTo((long)GRAB_INTERNE_MIDDLE);
            this->lift_stepper.run();
            break;
        case GRAB_INTERNE_DOWN:
            this->lift_stepper.moveTo((long)GRAB_INTERNE_DOWN);
            this->lift_stepper.run();
            break;
        case GRAB_EXTERNE:
            this->lift_stepper.moveTo((long)GRAB_EXTERNE);
            this->lift_stepper.run();
            break;
        case IDLE_BRAS:
            break;
    }
}
void ARM::toggleBras(armChoice_t choice){
    switch (choice){

        case UP_CHOICE:
            this->lift_stepper.moveTo(UP);
            this->etatBras=UP;
            break;
        case GRAB_INTERNE_UP_CHOICE:
            this->lift_stepper.moveTo((long)GRAB_INTERNE_UP);
            this->etatBras= GRAB_INTERNE_UP;
            break;
        case GRAB_INTERNE_MIDDLE_CHOICE:
            this->lift_stepper.moveTo((long)GRAB_INTERNE_MIDDLE);
            this->etatBras= GRAB_INTERNE_MIDDLE;
            break;
        case GRAB_INTERNE_DOWN_CHOICE:
            this->lift_stepper.moveTo((long)GRAB_INTERNE_DOWN);
            this->etatBras= GRAB_INTERNE_DOWN;
            break;
        case GRAB_EXTERNE_CHOICE:
            this->lift_stepper.moveTo((long)GRAB_EXTERNE);
            this->etatBras= GRAB_EXTERNE;
            break;
        case DOWN_CHOICE:
            this->lift_stepper.moveTo((long)DOWN);
            this->etatBras=DOWN;
            break;
        default:
            this->toggleBras(((this->etatBras)==DOWN)?GRAB_INTERNE_UP_CHOICE:UP_CHOICE);
    }
}

bool ARM::IsBrasTargetReach()
{
    return this->lift_stepper.targetPosition() == this->lift_stepper.currentPosition();
}

void ARM::toggleElbow(elbowChoice_t choice){
    switch (choice){
        case INWARDS_CHOICE:
            this->AX12A->move((unsigned char)(this->ID_AX_Bras),(int)INWARDS);
            this->etatAX=INWARDS;
            break;
        case OUTWARDS_CHOICE:
            this->AX12A->move(this->ID_AX_Bras,(int)OUTWARDS);
            this->etatAX=OUTWARDS;
            break;
        default:
            toggleElbow(((this->etatAX)==OUTWARDS)?INWARDS_CHOICE:OUTWARDS_CHOICE);
            break;
    }
}
void ARM::toggleMain(handChoice_t choice){
    switch(choice){
        case UNGRAB_CHOICE:
            this->AX12A->move(this->ID_AX_Main,UNGRAB_MAIN);
            this->etatMain = UNGRAB_MAIN;
            break;
        case GRAB_CHOICE:
            this->AX12A->move(this->ID_AX_Main,GRAB_MAIN);
            this->etatMain = GRAB_MAIN;
            break;
        default:
            toggleMain(((this->etatMain)==GRAB_MAIN)?UNGRAB_CHOICE:GRAB_CHOICE);
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
int PLATE::isRunning(){
    return this->_plate_stepper.isRunning();
}

void PLATE::update(plate_pos position)
{
    switch(position)
    {
        case PLATE_INIT:
            if(!digitalRead(this->_pin_zero))
            {
                this->_plate_stepper.setSpeed(0);
                this->_plate_stepper.setCurrentPosition(0);
            }
            else {this->_plate_stepper.runSpeed();};
            
            break;
        case POS_ONE:
        case POS_TWO:
        case POS_THREE:
        case POS_FOUR:
        case POS_FIVE:
        case POS_SIX:
            long target = stepper_pos[position] * DEG_TO_STEP;
            while(abs(target - this->_plate_stepper.currentPosition()) > 180*DEG_TO_STEP) {
                if(target - this->_plate_stepper.currentPosition() > 0) {
                    target -= DEG_TO_STEP*360;
                } else {
                    target += DEG_TO_STEP*360;
                }
            }
            this->_plate_stepper.moveTo(target);
            
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

    Wire.begin();
    proximity_sensor.init();
    proximity_sensor.configureDefault();
    proximity_sensor.setTimeout(40);
}

void CLAW::update(claw_state state)
{

    switch(state)  // si l'état est 1 on ouvre les pinces 
    {
        case CLAW_OPEN :
            _Servo_Gauche.writeMicroseconds(OPEN_L_POS);
            _Servo_Droite.writeMicroseconds(OPEN_R_POS);
        break;

        case CLAW_CLOSED :
            _Servo_Gauche.writeMicroseconds(CLOSED_L_POS);
            _Servo_Droite.writeMicroseconds(CLOSED_R_POS); 
        break ;

        case CLAW_GRAB :
            _Servo_Gauche.writeMicroseconds(GRAB_L);
            _Servo_Droite.writeMicroseconds(GRAB_R);
        break ;
    }

}

int CLAW::check_presence()
{
    uint16_t range = proximity_sensor.readRangeSingleMillimeters();     //demande une lecture de distance au capteur
    if (proximity_sensor.timeoutOccurred()) {return -1;}     //retourne false si le Timeout est dépassé

    //return range <= DIST_DETECT_PALET;
    return range;
    //retourne true si un objet est à moins de DIST_DETECT_PALET en mm, retourne fasle sinon 
}


//############# TRIEUSE #############

void trieuseController2000::update(){
    switch(this->etatGeneral){
        case IDLE_TC:
            break;
        case INIT_TC:
            this->updateInitLoop();
            break;
        case GRAB_DISCS:
            this->updateGrabLoop();
            break;
        case DROP_DISC:
            this->updateDropLoop();
            break;
    }
}

void trieuseController2000::storeDiscsToPostion (uint8_t pos, int numAction){
    this->targetPos = PLATE_INIT;
    int posTab = 0xff;
    switch (pos){
        case 1:
            this->targetPos = POS_ONE;
            posTab = 0;
            break;
        case 3:
            this->targetPos =POS_THREE;
            posTab = 1;
            break;
        case 5:
            this->targetPos = POS_FIVE;
            posTab = 2;
            break;
        default :
            break;
    }

    if (this->targetPos == PLATE_INIT){
        char buffer[25];
        int n = sprintf(buffer,"Wrong Position : %d",pos)-1;
        radio.sendMessage(buffer,n);
    }
    else{
        this->_numeroAction = numAction;
        if (this->disqueEnStock[posTab]){
            char buffer[50];
            int n = sprintf(buffer,"Non empty position : %d remaining disks", this->disqueEnStock[posTab])-1;
            radio.sendMessage(buffer,n);
        }
        else{
            this->etatGeneral = GRAB_DISCS;
            this->etatMachineEtatGrab = IDLE_G;
            this->updateGrabLoop();
        }
    }
}

void trieuseController2000::getDiscFromPosition (uint8_t pos, int numAction){
    this->targetPos = PLATE_INIT;
    int posTab = 0xff;
    switch (pos){
        case 1:
            this->targetPos = POS_ONE;
            posTab = 0;
            break;
        case 3:
            this->targetPos =POS_THREE;
            posTab = 1;
            break;
        case 5:
            this->targetPos = POS_FIVE;
            posTab = 2;
            break;
        default :
            break;
    }
    if (this->targetPos == PLATE_INIT){
        char buffer[25];
        int n = sprintf(buffer,"Wrong Position : %d",pos)-1;
        radio.sendMessage(buffer,n);
    }
    else{
        this->_numeroAction = numAction;
        if (!this->disqueEnStock[posTab]){
            char buffer[50];
            radio.sendMessage("Empty position, can't get disk",30);
        }
        else{
            this->etatGeneral = DROP_DISC;
            this->etatMachineEtatDrop = IDLE_D;
            this->updateDropLoop();
        }
    
    }
}

trieuseController2000::trieuseController2000(CLAW claw,PLATE plate,ARM arm)
{
    this->_claw = claw;
    this->_plate = plate;
    this->_arm = arm;
}

void trieuseController2000::updateInitLoop()
{/*mettre le bras en haut et rentré avec main fermée*/
    this->_claw.update(CLAW_CLOSED);
    this->_plate.update(PLATE_INIT);
    this->_arm.toggleBras(UP_CHOICE);
    this->_arm.toggleElbow(INWARDS_CHOICE);
    this->_arm.toggleMain(GRAB_CHOICE);
    this->_arm.update();
    
}

void trieuseController2000::updateDropLoop()
{ // WIP DE DIEU DONC WAIT SVP 

    switch (this->etatMachineEtatDrop)
    {
    case IDLE_D:
        this->_arm.toggleBras(UP_CHOICE);
        this->_arm.update();    
        break;
    case INITIAL_ROTATION_D:
        this->_plate.update(targetPos);
        this->_arm.toggleMain(UNGRAB_CHOICE);
        this->_arm.toggleElbow(INWARDS_CHOICE);
        this->_arm.update();
        break;
    case DESCENTE_AVANT_GRAB_D:
        this->_arm.toggleBras(GRAB_INTERNE_DOWN_CHOICE);
        this->_arm.update();
        break;
    case GRABBING_D:
        this->_arm.toggleMain(GRAB_CHOICE);
        this->_arm.update();
        break;
    case ROTATION_2_LE_RETOUR_D:
        this->_arm.toggleElbow(OUTWARDS_CHOICE);
        this->_arm.update();
        break;
    case DESCENTE_AVANT_LACHER_D:
        this->_claw.update(CLAW_OPEN);
        this->_arm.toggleBras(DOWN_CHOICE);
        this->_arm.update();
        break;
    case UNGRABBING_D:
        this->_arm.toggleMain(UNGRAB_CHOICE);
        this->_arm.update();
        break;
    case RESET_D:
        this->_arm.toggleBras(UP_CHOICE);
        this->_arm.update();
        break;
    
    default:
        break;
    }

}

void trieuseController2000::updateGrabLoop(){}
// procédure de ramassage (exemple )
    //     if(!(arm.getEtatBras() == INITIALISATION))
    //     {

    //         digitalToggle(LED_BUILTIN);
    //         switch (procedure%6)
    //         {
    //         case 0 :
    //             arm.toggleBras(0);
    //             break;
    //         case 1 :
    //             arm.toggleBras(3);
    //             break;
    //         case 2 :
    //             arm.toggleMain(1);
    //             break;
    //         case 3 :
    //             arm.toggleBras(0);
    //             break;
    //         case 4 :
    //             arm.toggleBras(3);
    //             break;
    //         case 5 :
    //             arm.toggleMain(0);
    //             break;    
            
    //         default:
    //             break;
    //         }

    //         if (arm.IsBrasTargetReach()) {procedure++;};




Toboggan::Toboggan(int pin_servo_toboggan)
{
    this->_pin_servo_tobogan = pin_servo_toboggan;
}

void Toboggan::init()
{
    this->_Servo_toboggan.attach(_pin_servo_tobogan);
    switch_state(CLOSED_TOBOGGAN_STATE);
}

void Toboggan::switch_state(toboggan_state_t cible)
{
    switch(cible)
    {
        case OPEN_TOBOGGAN_STATE :
            _Servo_toboggan.writeMicroseconds(OPEN_TOBOGGAN_VAL); 
            break;

        case CLOSED_TOBOGGAN_STATE :
            _Servo_toboggan.writeMicroseconds(CLOSED_TOBOGGAN_VAL); 
            break ;

    }
}