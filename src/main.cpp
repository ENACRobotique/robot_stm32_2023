#include <Arduino.h>
#include "encoder.h"
#include "config.h"
#include "../lib/metro.h"
#include "holo_control.h"
#include "motor_control.h"
#include "odometry.h"
#include "trieuse.h"
#include "AX12A.h"


DynamixelSerial AX12As;
Metro odom_refresh(10);
int bruhCounter=0;
Metro bruhcmd(1000);
Metro lazytimer(3000);
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


//ARM arm(FIN_COURSE_2,2,5,4,&AX12As);
PLATE plateau(1,FIN_COURSE_2);

int pos;
int position = 0;
float tableau[] = {
    0.5,
    0.0,
    -0.5,
    0.0
};

void setup() {
    pinMode(LED_BUILTIN,OUTPUT);
    Serial.begin(115200);
    Serial3.begin(500000);
    //arm.init(&Serial3);
    plateau.init();
    plateau.update(PLATE_INIT);

    Serial.println("Démarrage du robot bas niveau v0.2.0");
    encoder1.init();
    encoder2.init();
    encoder3.init();
    Serial.println("Encodeurs initialisés");

    motor1.init();
    motor2.init();
    motor3.init();
    Serial.println("Moteurs initialisés");

    holo_control.stop();
    Serial.println("Robot à l'arrêt");

    odom.init();
    odom_refresh.reset();
    Serial.println("Odométrie initialisée");
    
    holo_control.set_ptarget(2.f, 0.f, 90.f * DEG_TO_RAD);
    pos = 0;

}

void loop() {
    // if (bruhcmd.check()) {
    //     position = (position + 1) % 4;
    //     holo_control.set_vtarget_table(0.0, tableau[position], 0.0);
    // }
    //arm.update();
    plate_pos cmd[6] ={POS_ONE, POS_TWO, POS_THREE, POS_FOUR, POS_FIVE, POS_SIX};
    if (lazytimer.check())
    {
        digitalToggle(LED_BUILTIN);
        pos+=1;
    }
    plateau.update(cmd[pos%6]);


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
    // if (odom_refresh.check()){
    //     odom.update();
    //     holo_control.update();
    //     Serial.print( "(vx: " );
    //     Serial.print(odom.get_vx() );
    //     Serial.print( ", vy: " );
    //     Serial.print(odom.get_vy() );
    //     Serial.print( ") " );

    //     Serial.print( "(x: " );
    //     Serial.print(odom.get_x() );
    //     Serial.print( ", y: " );
    //     Serial.print(odom.get_y() );
    //     Serial.print( ", theta: " );
    //     Serial.print(odom.get_theta() );
    //     Serial.println( ")" );

    //     // Serial.print(motor2.get_target_speed());
    //     // Serial.print(" ");
    //     // Serial.print(motor2.get_ramped_target_speed());
    //     // Serial.print(" ");
    //     // Serial.println(odom.get_v2speed());
    // }

    
}
