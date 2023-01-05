#include <Arduino.h>
#include "utilities/logging.h"
#include "encoder.h"
#include "config.h"
#include "../lib/metro.h"
#include "holo_control.h"
#include "motor_control.h"
#include "utilities/communication.h"

//#define LOOP
SerialRadio comm;
Metro metro_move_interval = Metro(500);

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

bool avancer = false;
bool reculer = false;
bool tourner_gauche = false;
bool tourner_droite = false;
bool translater_gauche = false;
bool translater_droite = false;

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
        if (Serial.available()){
            char c = Serial.read();
            switch (c){
                case 'a':
                    avancer = true;
                    reculer = false;
                    break;
                case 'r':
                    reculer = true;
                    avancer = false;
                    break;
                case 'h':
                    tourner_gauche = true;
                    tourner_droite = false;
                    break;
                case 'j':
                    tourner_droite = true;
                    tourner_gauche = false;
                    break;
                case 'g':
                    translater_gauche = true;
                    translater_droite = false;
                    break;
                case 'd':
                    translater_droite = true;
                    translater_gauche = false;
                    break;
                case 's':
                    avancer = false;
                    reculer = false;
                    tourner_gauche = false;
                    tourner_droite = false;
                    translater_gauche = false;
                    translater_droite = false;
                    holo_control.stop();
                    break;
                default:
                    break;
            }
        }
        if (metro_move_interval.check()){
            int vx = (avancer ? 35 : 0) + (reculer ? -35 : 0);
            int vy = (translater_gauche ? 35 : 0) + (translater_droite ? -35 : 0);
            int w = (tourner_gauche ? 300 : 0) + (tourner_droite ? -300 : 0);
            holo_control.set_vtarget_pwm(vx, vy, w);
            // avancer = false;
            // reculer = false;
            // tourner_gauche = false;
            // tourner_droite = false;
            // translater_gauche = false;
            // translater_droite = false;
        }
    #endif
}
