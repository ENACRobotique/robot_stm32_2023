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


uint8_t hasMatchStarted = 0;
uint8_t startOfMatchReported = 0;
DisplayController afficheur = DisplayController();
DynamixelSerial AX12As;
Metro odom_refresh(40);
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

//MotorController(int mot_dir, int mot_pwm, bool reverse, double kp, double ki, double min, double max, int motor_number);
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
char hasMatchStated =0;
//loop (assiete_V_5 -> assiette_B_5 -> assiette_V_2 -> assiette B_2 -> assiette_V_5)
double x_pos_order[4]={1.125, 1.125, 1.875, 1.875};
double y_pos_order[4]={0, 2.5, 1.775, 0.225};
double teta_pos_order[4]={ANGLE_PLAT, ANGLE_PLAT, ANGLE_PLAT, ANGLE_PLAT};
int cmd_order;
double tgt_error=0.015;

int position = 0;
double tableau[] = {
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
    arm.toggleMain(UNGRAB_CHOICE);
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
    
    //holo_control.set_ptarget(0.5, 0.f, 0);
    // odom.set_x(x_pos_order[0]);
    // odom.set_y(y_pos_order[0]);
    // odom.set_theta(teta_pos_order[0]);
    // cmd_order=1;
    

    
}

void loop() {
    
    // if (1){//this code will be executed every time. 
    //     radio.update();
    //     if(startOfMatchReported){}//optimisation to keep loop shorter during match, probably not needed
    //     else if (digitalRead(TIRETTE)){//si la tirette est là (cas avant début de match)
    //         if (!digitalRead(POS_BUTTON)){
    //             buttonPressed = 1;
    //             lastPressedTimeStamp = millis();
    //         }
    //         else if ((buttonPressed && (millis()-lastPressedTimeStamp)>10) || (colorIsGreen != digitalRead(COLOR))){
    //             colorIsGreen = digitalRead(COLOR);
    //             buttonPressed=0;
    //             positionDepart %=5;
    //             positionDepart++;//Ordre de ces deux opérations délibéré pour avoir un résultat entre 1 et 5
    //             afficheur.setNbDisplayed((colorIsGreen?6000:8000)+positionDepart);// 6 = G et 8 = B, ici on parle le L3375P34K      (-;
    //         }
    //     }
    //     else if (hasMatchStarted) {
    //         radio.reportStart();
    //         startOfMatchReported=1;
    //     }
    //     else{//made that way to read pin twice before match start
    //         hasMatchStarted=1;
    //         startOfMatchReported=0;
    //     }
            
    // }

    radio.update();

    if (digitalRead(TIRETTE)){//si la tirette est là
        colorIsGreen = digitalRead(COLOR);
        if (!digitalRead(POS_BUTTON)){
            buttonPressed = 1;
            lastPressedTimeStamp = millis();
        }else if (buttonPressed && (millis()-lastPressedTimeStamp)>10){
            buttonPressed=0;
            positionDepart %=5;
            positionDepart++;
            afficheur.setNbDisplayed((colorIsGreen?6000:8000)+positionDepart);
        }
    }else if (!hasMatchStated){
        hasMatchStated =1;
        radio.reportStart();
    }

    if (odom_refresh.check()){//every 10ms
        odom.update();
        holo_control.update();
    }


    if (odomSpamTimer.check()){//every 100ms
        radio.reportPosition();
        radio.reportSpeed();
    }

    if (bruhcmd.check())//every second
    {   digitalToggle(LED_BUILTIN);
        //odom.print_odometry();
    }

    // if (lazytimer.check())//every 10s
    // {
    //     radio.sendMessage("Heart beat",10);
    //     holo_control.set_ptarget(x_pos_order[cmd_order],y_pos_order[cmd_order],teta_pos_order[cmd_order]);
    //     cmd_order = (cmd_order + 1) %4;
    //     afficheur.setNbDisplayed(cmd_order);

    // }

    
}
