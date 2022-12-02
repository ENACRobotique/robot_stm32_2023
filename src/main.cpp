#include <Arduino.h>
#include "utilities/logging.h"
#include "encoder.h"
#include "config.h"
#include "../lib/metro.h"
#include "holo_control.h"
#include "motor_control.h"

#define LOOP

Metro metro_move_interval = Metro(0);

Encoder encoder1(ENCODER_1_A, ENCODER_1_B);
    //l'encodeur associé ne tourne pas dans le même sens que les autres, besoin d'un signe -
    //les autres tournent dans le sens trigo pour les valeurs positives
    //mais je crois qu'on a inversé les fils A & B, donc c'est bon
Encoder encoder2(ENCODER_2_A, ENCODER_2_B);
Encoder encoder3(ENCODER_3_A, ENCODER_3_B);

MotorController motor1(MOTOR_1_DIR, MOTOR_1_PWM, false);
MotorController motor2(MOTOR_2_DIR, MOTOR_2_PWM, false);
MotorController motor3(MOTOR_3_DIR, MOTOR_3_PWM, false);

HoloControl holo_control(&motor1, &motor2, &motor3);

Eigen::Vector4i vtarget_list[6] = {
    Eigen::Vector4i(35, 0, 0, 1000),
    Eigen::Vector4i(0, 35, 0, 1000),
    Eigen::Vector4i(-35, 0, 0, 1000),
    Eigen::Vector4i(0, -35, 0, 1000),
    Eigen::Vector4i(0, 0, 300, 5420), //objectif, faire un 360°
    Eigen::Vector4i(0, 0, 0, 1000) //stop pour un moment avant de recommencer
};
int i = 0;


void setup() {
    Logging::init(115200);
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

    metro_move_interval.reset();

    Logging::info("Init terminé");
}

void loop() {
    #ifdef LOOP
    if (metro_move_interval.check()) {
        Eigen::Vector4i vtarget = vtarget_list[i];
        Logging::info("vtarget = (%d, %d, %d) for %d msecs", vtarget(0), vtarget(1), vtarget(2), vtarget(3));
        holo_control.set_vtarget_pwm(vtarget(0), vtarget(1), vtarget(2));
        metro_move_interval.interval(vtarget(3));
        metro_move_interval.reset();
        i = (i + 1) % 6;
    }
    #else
        motor1.send_motor_command_pwm(-30);
        motor2.send_motor_command_pwm(-30);
        motor3.send_motor_command_pwm(-30);
    #endif
}
