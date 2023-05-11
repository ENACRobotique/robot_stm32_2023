#ifndef TRIEUSE_H
#define TRIEUSE_H

#include "config.h"
#include <AccelStepper.h>
#include "AX12A.h"
#include <Servo.h>
#include <Wire.h>
#include <VL6180X.h>

//bras
typedef enum{
    IDLE_ELBOW,
    INWARDS = 145,
    OUTWARDS = 525
} elbowState_t;

typedef enum{
    INWARDS_CHOICE = 0 ,
    OUTWARDS_CHOICE = 1,
    IDLE_ELBOW_CHOICE = 2
} elbowChoice_t;

typedef enum{
    INITIALISATION,
    IDLE_BRAS,
    UP=-10,
    GRAB_INTERNE_UP=-2200, 
    GRAB_INTERNE_MIDDLE=-3300,
    GRAB_INTERNE_DOWN=-4500,
    GRAB_EXTERNE=-8200,
    DOWN=-2100,
}armState_t;

typedef enum{
    UP_CHOICE = 0,
    GRAB_INTERNE_UP_CHOICE = 1, 
    GRAB_INTERNE_MIDDLE_CHOICE =2,
    GRAB_INTERNE_DOWN_CHOICE =3,
    GRAB_EXTERNE_CHOICE =4,
    DOWN_CHOICE =5,
}armChoice_t;

typedef enum{
    IDLE_MAIN,
    UNGRAB_MAIN=500,
    GRAB_MAIN=350
}handState_t;

typedef enum
{
    UNGRAB_CHOICE = 0,
    GRAB_CHOICE = 1,
    DEFAULT_CHOICE = 2,

}handChoice_t;

//plateau
typedef enum 
{
    PLATE_INIT = -1,
    POS_ONE = 1,
    POS_TWO,
    POS_THREE,
    POS_FOUR,
    POS_FIVE,
    POS_SIX,

}plate_pos;

// typedef enum 
// {
//     PLATE_INIT = -1,  // 
//     STEPPER_POS_ONE = 60,   //1
//     STEPPER_POS_TWO = 120,  //2
//     STEPPER_POS_THREE = 180,//3
//     STEPPER_POS_FOUR = 240, //4
//     STEPPER_POS_FIVE = 300, //5
//     STEPPER_POS_SIX = 360,  //6

// }plate_stepper_pos;

extern long stepper_pos[6];

//griffes
typedef enum 
{
    OPEN_L_POS = 1850,
    OPEN_R_POS = 1000,
    CLOSED_L_POS = 1100,
    CLOSED_R_POS = 1900,
    GRAB_L = 1500,
    GRAB_R = 1400,

}claw_pos;

typedef enum
{
    CLAW_CLOSED = 'c',
    CLAW_OPEN = 'o',
    CLAW_GRAB = 'g',
    CLAW_CHECK_PRESENCE ='C',
}claw_state;

//Bras
class ARM
{
    public :
        ARM();
        ARM(int PinFinCourse, int NumStepper, int idAX12Elbow, int idAX12Main,DynamixelSerial* AX12As); // + servo + ax12
        void init(HardwareSerial *serialDynamixel);
        void toggleBras(armChoice_t choice); // de 1 à 8 pour du plus haut au plus bas
                                         //l'endroit où le bras n'est pas.
        void toggleElbow(elbowChoice_t choice = IDLE_ELBOW_CHOICE);// 0 pour l'intérieur, 1 pour l'extérieur, le 
                                         //reste por une valeur par défaut
        void toggleMain(handChoice_t choice=DEFAULT_CHOICE);   // 0 pour lâcher, 1 pour attraper, le 
                                         //reste por une valeur par défaut
        void update();
        armState_t getEtatBras();
        bool IsBrasTargetReach();

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
        PLATE();
        PLATE(int num_stepper, int pin_zero); // stepper + fin de course
        void init();
        void update(plate_pos position); // rotation a sens unique
        int isRunning();
    
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
        CLAW();
        CLAW(int pin_servo_gauche, int pin_servo_droite);
        void init();
        void update(claw_state state);
        int check_presence();
    
    private:
        Servo _Servo_Gauche;
        Servo _Servo_Droite;
        int _pin_servo_gauche;
        int _pin_servo_droite;
        bool _state;
        VL6180X proximity_sensor;


};
extern CLAW pince;
//trieuseController2000
typedef enum {
    IDLE_TC,
    INIT_TC,
    GRAB_DISCS,
    DROP_DISC
}trieuseControllerDirectives_t;

typedef enum{
    IDLE_G,
    INITIAL_ROTATION_G,
    DESCENTE_AVANT_GRAB_G,
    GRABBING_G,
    REMONTEE_APRES_GRAB_G,
    ROTATION_2_ELECTRIC_BOOGALOO_G,
    DESCENTE_AVANT_LACHER_G,
    UNGRABBING_G,
    RETOUR_A_LA_CASE_DEPART_G,
}grabDiscsStates_t;

typedef enum{
    IDLE_D,
    INITIAL_ROTATION_D,
    DESCENTE_AVANT_GRAB_D,
    GRABBING_D,
    ROTATION_2_LE_RETOUR_D,
    DESCENTE_AVANT_LACHER_D,
    UNGRABBING_D,
    RESET_D,
}dropDiscState_t;
class trieuseController2000{
    public :
        trieuseController2000(CLAW claw,PLATE plate,ARM arm);
        void update();
        void storeDiscsToPostion (uint8_t pos, int numAction);
        void getDiscFromPosition (uint8_t pos, int numAction);
    private :
        void updateInitLoop();
        void updateDropLoop();
        void updateGrabLoop();
        int disqueEnStock[3]={0,0,0};
        int _numeroAction = 0 ;
        plate_pos targetPos;
        trieuseControllerDirectives_t etatGeneral=IDLE_TC;
        grabDiscsStates_t etatMachineEtatGrab;
        dropDiscState_t etatMachineEtatDrop;
        CLAW _claw;
        PLATE _plate;
        ARM _arm;
        

};


typedef enum{ // a changer 
    OPEN_TOBOGGAN_VAL = 1000,
    CLOSED_TOBOGGAN_VAL = 2000,
}toboggan_pos;
typedef enum{
    OPEN_TOBOGGAN_STATE = 's',//sorti
    CLOSED_TOBOGGAN_STATE = 'r',//rentré
}toboggan_state_t;


class Toboggan
{
    public :
        Toboggan(int pin_servo_tobogan);
        void init();
        void switch_state(toboggan_state_t open);

    private:
        int _pin_servo_tobogan;
        Servo _Servo_toboggan;
};

extern Toboggan toboggan;

#endif