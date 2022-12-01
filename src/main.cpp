#include <Arduino.h>
#include "utilities/logging.h"
#include "encoder.h"
#include "config.h"
#include "../lib/metro.h"
#include "holo_control.h"
#include "motor_control.h"

#define LED 13

Metro metro_log_loop = Metro(100);
Metro metro_control_loop = Metro(1000);
Encoder encoder1(ENCODER_1_A, ENCODER_1_B);
    //l'encodeur associé ne tourne pas dans le même sens que les autres, besoin d'un signe -
    //les autres tournent dans le sens trigo pour les valeurs positives
Encoder encoder2(ENCODER_2_A, ENCODER_2_B);
Encoder encoder3(ENCODER_3_A, ENCODER_3_B);

MotorController motor2(MOTOR_2_DIR, MOTOR_2_PWM, 0.1, 0.1, false);
double *motor2_tgt_speed_ptr = NULL;

double spd_list[4] = {0, 1, 0, -1};
int i = 0;

void setup() {
    pinMode(LED, OUTPUT);

    Logging::init(115200);
    Logging::info("Démarrage du robot bas niveau v0.1.0");
    // TODO: Logging::get()

    encoder1.init();
    encoder2.init();
    encoder3.init();
    Logging::info("Encodeurs initialisés");
    
    metro_log_loop.reset();

    motor2.init();
    Logging::info("Moteur 2 initialisé");
}

void loop() {
    if (metro_log_loop.check()){
        Logging::trace("Encoder 1 : %d", encoder1.get_value());
        Logging::trace("Encoder 2 : %d", encoder2.get_value());
        Logging::trace("Encoder 3 : %d", encoder3.get_value());
    }
    if (metro_control_loop.check()){
        motor2.send_motor_command(spd_list[(i++)%4]);
    }
}
