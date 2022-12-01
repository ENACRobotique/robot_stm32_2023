#include <Arduino.h>
#include "utilities/logging.h"
#include "encoder.h"
#include "config.h"
#include "../lib/metro.h"
#include "holo_control.h"
#include "motor_control.h"
#include "odometry.h"

#define LED 13

Metro metro_odom = Metro(100);
Metro metro_control_loop = Metro(1000);
Encoder encoder1(ENCODER_1_A, ENCODER_1_B);
    //l'encodeur associé ne tourne pas dans le même sens que les autres, besoin d'un signe -
    //les autres tournent dans le sens trigo pour les valeurs positives
Encoder encoder2(ENCODER_2_A, ENCODER_2_B);
Encoder encoder3(ENCODER_3_A, ENCODER_3_B);

Odometry odom(&encoder1, &encoder2, &encoder3);

MotorController motor2(MOTOR_2_DIR, MOTOR_2_PWM, 0.1, 0.1, &(odom._speed2), false);


void setup() {
    pinMode(LED, OUTPUT);

    Logging::init(115200);
    Logging::info("Démarrage du robot bas niveau v0.1.0");
    // TODO: Logging::get()

    encoder1.init();
    encoder2.init();
    encoder3.init();
    Logging::info("Encodeurs initialisés");
    
    metro_odom.reset();

    motor2.init();
    Logging::info("Moteur 2 initialisé");

    motor2.send_motor_command(0.1);

}

void loop() {
    // if (metro_odom.check()){
    //     odom.update();
    //     // Logging::trace("Encoder 1 : %d", encoder1.get_value());
    //     // Logging::trace("Encoder 2 : %d", encoder2.get_value());
    //     // Logging::trace("Encoder 3 : %d", encoder3.get_value());
    //     // Logging::trace("Speed 1 : %lf", odom._speed1);
    //     // Logging::trace("Speed 2 : %lf", odom._speed2);
    //     // Logging::trace("Speed 3 : %lf", odom._speed3);
    //     Serial.println(odom._speed1);
    //     Serial.println(odom._speed2);
    //     Serial.println(odom._speed3);
        
    // }

}
