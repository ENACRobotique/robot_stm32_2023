#include <Arduino.h>
#include "utilities/logging.h"
#include "encoder.h"
#include "config.h"
#include "../lib/metro.h"

#define LED 13

Metro metro_log_loop = Metro(100);
Encoder encoder1(ENCODER_1_A, ENCODER_1_B);
Encoder encoder2(ENCODER_2_A, ENCODER_2_B);
Encoder encoder3(ENCODER_3_A, ENCODER_3_B);

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
}

void loop() {
    if (metro_log_loop.check()){
        Logging::trace("Encoder 1 : %d", encoder1.get_value());
        Logging::trace("Encoder 2 : %d", encoder2.get_value());
        Logging::trace("Encoder 3 : %d", encoder3.get_value());
    }
}
