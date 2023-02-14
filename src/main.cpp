#include <Arduino.h>
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

Odometry odom(&encoder1, &encoder2, &encoder3);

HoloControl holo_control(&motor1, &motor2, &motor3, &odom);

int position = 0;
float tableau[] = {
    0.5,
    0.0,
    -0.5,
    0.0
};

void setup() {
    Serial.begin(115200);
    Serial.println("Démarrage du robot bas niveau v0.1.0");

    encoder1.init();
    encoder2.init();
    encoder3.init();
    Serial.println("Encodeurs initialisés");

    motor1.init();
    motor2.init();
    motor3.init();
    Serial.println("Moteurs initialisés");

    holo_control.stop();
    Serial.println("Robot arrêté");

    odom.init();
    odom_refresh.reset();
    Serial.println("Odometry initialisée");

    Serial.println("Init terminé");
}


void loop() {
    if (bruhcmd.check()) {
        position = (position + 1) % 4;
        holo_control.set_vtarget_holo(0.0, tableau[position], 0.0);
    }

    if (odom_refresh.check()){
        odom.update();
        holo_control.update();
    }
}
