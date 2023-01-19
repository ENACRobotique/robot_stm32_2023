#include <Arduino.h>
#include "utilities/logging.h"
#include "encoder.h"
#include "config.h"
#include "../lib/metro.h"
#include "holo_control.h"
#include "motor_control.h"
#include "odometry.h"

Metro odom_refresh(10);
Metro bruhcmd(2000);
Encoder encoder1(ENCODER_1_A, ENCODER_1_B);
    //l'encodeur associé ne tourne pas dans le même sens que les autres, besoin d'un signe -
    //les autres tournent dans le sens trigo pour les valeurs positives
    //mais je crois qu'on a inversé les fils A & B, donc c'est bon
Encoder encoder2(ENCODER_2_A, ENCODER_2_B);
Encoder encoder3(ENCODER_3_A, ENCODER_3_B);

//MotorController(int mot_dir, int mot_pwm, bool reverse, float kp, float ki, float min, float max, int motor_number);
MotorController motor1(MOTOR_1_DIR, MOTOR_1_PWM, false, 1.2, 0.5, -200.0, 200.0, 1);
MotorController motor2(MOTOR_2_DIR, MOTOR_2_PWM, false, 1.2, 0.5, -200.0, 200.0, 2);
MotorController motor3(MOTOR_3_DIR, MOTOR_3_PWM, false, 1.2, 0.5, -200.0, 200.0, 3);

HoloControl holo_control(&motor1, &motor2, &motor3);

Odometry odom(&encoder1, &encoder2, &encoder3);

int position = 0;
float tableau[] = {
    0.5,
    0.0,
    -0.5,
    0.0
};

void setup() {
    Logging::init(115200);
    // comm.init();
    Logging::info("Démarrage du robot bas niveau v0.1.0");
    // TODO: Logging::get()

    encoder1.init();
    encoder2.init();
    encoder3.init();
    Logging::info("Encodeurs initialisés");

    motor1.init();
    motor2.init();
    motor3.init();
    Logging::info("Moteurs initialisés");

    holo_control.stop();
    Logging::info("Robot arrêté");

    odom.init();
    odom_refresh.reset();
    Logging::info("Odometry initialisée");

    Logging::info("Init terminé");

}


void loop() {
    if (bruhcmd.check()) {
        position = (position + 1) % 4;
        holo_control.set_vtarget_holo(0.0, tableau[position], 0.0);
    }

    if (odom_refresh.check()){
        odom.update();
        holo_control.update(
           odom.get_v1speed(),
           odom.get_v2speed(),
           odom.get_v3speed()
        );

        /*Serial.print( "(vx: " );
        Serial.print(odom.get_vx() );
        Serial.print( ", vy: " );
        Serial.print(odom.get_vy() );
        Serial.print( ") " );

        Serial.print( "(x: " );
        Serial.print(odom.get_x() );
        Serial.print( ", y: " );
        Serial.print(odom.get_y() );
        Serial.print( ", theta: " );
        Serial.print(odom.get_theta() );
        Serial.println( ")" );*/

        Serial.print(motor2.get_target_speed());
        Serial.print(" ");
        Serial.print(motor2.get_ramped_target_speed());
        Serial.print(" ");
        Serial.println(odom.get_v2speed());
    }
}
