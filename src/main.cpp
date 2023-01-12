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
Encoder encoder1(ENCODER_1_A, ENCODER_1_B);
    //l'encodeur associé ne tourne pas dans le même sens que les autres, besoin d'un signe -
    //les autres tournent dans le sens trigo pour les valeurs positives
    //mais je crois qu'on a inversé les fils A & B, donc c'est bon
Encoder encoder2(ENCODER_2_A, ENCODER_2_B);
Encoder encoder3(ENCODER_3_A, ENCODER_3_B);

MotorController motor1(MOTOR_1_DIR, MOTOR_1_PWM, false, 0, 0, 0, 0, 1);
MotorController motor2(MOTOR_2_DIR, MOTOR_2_PWM, false, 0, 0, 0, 0, 2);
MotorController motor3(MOTOR_3_DIR, MOTOR_3_PWM, false, 0, 0, 0, 0, 3);

HoloControl holo_control(&motor1, &motor2, &motor3);


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

    Logging::info("Init terminé");
}

void loop() {
    
}
