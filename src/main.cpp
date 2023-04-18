#include <Arduino.h>
#include "encoder.h"
#include "config.h"
#include "../lib/metro.h"
#include "holo_control.h"
#include "motor_control.h"
#include "odometry.h"
#include "trieuse.h"
#include "AX12A.h"
#include "DisplayController.h"
#include "comm.h"

DisplayController afficheur = DisplayController();
DynamixelSerial AX12As;
Metro odom_refresh(10);
int bruhCounter=0;
int buttonPressed;
uint32_t lastPressedTimeStamp;
int positionDepart=1;
int colorIsGreen;
Metro odomSpamTimer(100);
Metro bruhcmd(1000);
Metro lazytimer(10000);
Encoder encoder1(ENCODER_1_A, ENCODER_1_B);
    //l'encodeur associé ne tourne pas dans le même sens que les autres, besoin d'un signe -
    //les autres tournent dans le sens trigo pour les valeurs positives
    //mais je crois qu'on a inversé les fils A & B, donc c'est bon
Encoder encoder2(ENCODER_2_A, ENCODER_2_B);
Encoder encoder3(ENCODER_3_A, ENCODER_3_B);

//MotorController(int mot_dir, int mot_pwm, bool reverse, float kp, float ki, float min, float max, int motor_number);
// MotorController motor1(MOTOR_1_DIR, MOTOR_1_PWM, false, 1.2, 0.5, -200.0, 200.0, 1);
// MotorController motor2(MOTOR_2_DIR, MOTOR_2_PWM, false, 1.2, 0.5, -200.0, 200.0, 2);
// MotorController motor3(MOTOR_3_DIR, MOTOR_3_PWM, false, 1.2, 0.5, -200.0, 200.0, 3);
MotorController motor1(MOTOR_1_DIR, MOTOR_1_PWM, false, 1.2, 0.0, -200.0, 200.0, 1);
MotorController motor2(MOTOR_2_DIR, MOTOR_2_PWM, false, 1.2, 0.0, -200.0, 200.0, 2);
MotorController motor3(MOTOR_3_DIR, MOTOR_3_PWM, false, 1.2, 0.0, -200.0, 200.0, 3);

Odometry odom(&encoder1, &encoder2, &encoder3);

HoloControl holo_control(&motor1, &motor2, &motor3, &odom);

Comm radio;

//système de tri
ARM arm(FIN_COURSE_2,2,5,4,&AX12As);
CLAW pince(SERVO_2, SERVO_1);
claw_state state[3] = {CLAW_CLOSED,CLAW_OPEN,CLAW_GRAB};
PLATE plateau(1,FIN_COURSE_1);
plate_pos cmd[6] ={POS_ONE, POS_TWO, POS_THREE, POS_FOUR, POS_FIVE, POS_SIX};


int disk;
bool main_state;
int procedure;
//loop (assiete_V_5 -> assiette_B_5 -> assiette_V_2 -> assiette B_2 -> assiette_V_5)
float x_pos_order[5]={-1.125,-1.125,-1.875,-1.875,-1.125};
float y_pos_order[5]={-0.225,-1.775,-1.775,-0.225,-0.225};
float teta_pos_order[5]={0.f,0.f,0.f,0.f,0.f};
int cmd_order;
float tgt_error=0.15;

int position = 0;
float tableau[] = {
    0.5,
    0.0,
    -0.5,
    0.0
};

void setup() {
    pinMode(TIRETTE, INPUT_PULLUP);
    pinMode(COLOR, INPUT_PULLUP);
    pinMode(POS_BUTTON,INPUT_PULLUP);
    afficheur.init();
    afficheur.setNbDisplayed(8001);
    pinMode(LED_BUILTIN,OUTPUT);
    Serial.begin(115200);
    Serial3.begin(500000);
    
    radio.sendMessage("Initialisation Bras",19);
    arm.init(&Serial3); // NE PAS METTRE ARM.UPDATE DANS LE SETUP !!! => ÇA PLANTE
    arm.toggleMain(0);
    plateau.init();
    plateau.update(PLATE_INIT);
    pince.init();
    pince.update(CLAW_CLOSED);
    disk = 0;
    procedure =0 ;
    main_state = 0;
    radio.sendMessage("Bras Initialisé",16);
    
    radio.sendMessage("Démarrage du robot bas niveau v0.2.0",37);
    encoder1.init();
    encoder2.init();
    encoder3.init();
    radio.sendMessage("Encodeurs initialisés",22);

    motor1.init();
    motor2.init();
    motor3.init();
    radio.sendMessage("Moteurs initialisés",20);

    holo_control.stop();
    radio.sendMessage("Robot à l'arrêt",17);

    odom.init();
    odom_refresh.reset();
    radio.sendMessage("Odométrie initialisée",23);
    
    //holo_control.set_ptarget(5, 0.f, 0);
    odom.set_x(x_pos_order[0]);
    odom.set_y(y_pos_order[0]);
    odom.set_theta(teta_pos_order[0]);
    cmd_order=0;
    

    
}

void loop() {
    if (digitalRead(TIRETTE)){//si la tirette est là
        colorIsGreen = digitalRead(COLOR);
        if (!digitalRead(POS_BUTTON)){
            buttonPressed = 1;
            lastPressedTimeStamp = millis();
        }
        else if (buttonPressed && (millis()-lastPressedTimeStamp)>10){
            buttonPressed=0;
            positionDepart %=5;
            positionDepart++;
            afficheur.setNbDisplayed((colorIsGreen?6000:8000)+positionDepart);
        }
    }
    if (odomSpamTimer.check()){
        radio.reportPosition();
        radio.reportSpeed();
    }

    //arm.update();

    // if (bruhcmd.check())
    // {

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
            
    //     };
    // }
    

    //plateau.update(cmd[pos%6]);


    // if (bruhcmd.check()){ 
    //     digitalToggle(LED_BUILTIN);
    //     if (bruhCounter==10){arm.toggleBras(0);}
    //     else if (bruhCounter==11){arm.toggleMain(1);}
    //     else if (bruhCounter==12){arm.toggleBras(1);}
    //     else if (bruhCounter==14){arm.toggleElbow(0);}
    //     else if (bruhCounter==15){arm.toggleMain(0);}
    //     else if (bruhCounter==16){arm.toggleElbow(1);bruhCounter=5;}
    //     bruhCounter++;
    // }
    //pince.update(CLAW_OPEN);
    

    
}
