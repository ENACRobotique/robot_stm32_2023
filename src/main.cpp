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
Metro bruhcmd(1000);
Metro lazytimer(5000);
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
armState_t disk_select[8]= 
{   ABOVE,
    GRAB_EXTERNE_1, 
    GRAB_EXTERNE_2,
    GRAB_EXTERNE_3,
    GRAB_INTERNE_1, 
    GRAB_INTERNE_2,
    GRAB_INTERNE_3,
    DOWN,
};
int disk;

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
    arm.init(&Serial3);
    arm.update();
    plateau.init();
    plateau.update(PLATE_INIT);
    // pince.init();
    // pince.update(CLAW_CLOSED);
    disk = 0;
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

    
}

void loop() {
    // if (digitalRead(TIRETTE)){//si la tirette est là
    //     colorIsGreen = digitalRead(COLOR);
    //     if (!digitalRead(POS_BUTTON)){
    //         buttonPressed = 1;
    //         lastPressedTimeStamp = millis();
    //     }
    //     else if (buttonPressed && (millis()-lastPressedTimeStamp)>10){
    //         buttonPressed=0;
    //         positionDepart %=5;
    //         positionDepart++;
    //         afficheur.setNbDisplayed((colorIsGreen?6000:8000)+positionDepart);
    //     }
    // }
    
    // if (bruhcmd.check()) {
    // position = (position + 1) % 4;
    // holo_control.set_vtarget_table(0.0, tableau[position], 0.0);
    // }

    
    if (lazytimer.check())
    {
        digitalToggle(LED_BUILTIN);
        arm.toggleBras(disk%8); 
        disk++;
        Serial.println(disk%8);

    }
    arm.update();

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
    
    // if (odom_refresh.check())
    // {
    //     odom.update();
    //     holo_control.update();
    //     // Serial.print( "(vx: " );
    //     // Serial.print(odom.get_vx() );
    //     // Serial.print( ", vy: " );
    //     // Serial.print(odom.get_vy() );
    //     // Serial.print( ") " );
    //     // Serial.print( "(x: " );
    //     // Serial.print(odom.get_x() );
    //     // Serial.print( ", y: " );
    //     // Serial.print(odom.get_y() );
    //     // Serial.print( ", theta: " );
    //     // Serial.print(odom.get_theta() );
    //     // Serial.println( ")" );
    //     // Serial.print(motor2.get_target_speed());
    //     // Serial.print(" ");
    //     // Serial.print(motor2.get_ramped_target_speed());
    //     // Serial.print(" ");
    //     // Serial.println(odom.get_v2speed());
    // }

    
}
