#include <Arduino.h>
#include "utilities/logging.h"
#include "utilities/timer.h"
#include "encoder.h"
#include "config.h"

#define LED 13

int i = 0;
Timer timer(1000);
Encoder encoder1(ENCODER_1_A, ENCODER_1_B);
Encoder encoder2(ENCODER_2_A, ENCODER_2_B);

//Arduino LED1 blink
void setup() {
    pinMode(LED, OUTPUT);

    Logging::init(115200);
    Logging::info("Démarrage du robot bas niveau v0.1.0");

    // TODO: Logging::get()

    encoder1.init();
    encoder2.init();

    Logging::info("Initialisation Encoder");

}

void loop() {
    // if (timer.finished()) {
    //     i++;
        
    //     digitalWrite(LED, !digitalRead(LED));
    //     timer.reset();

    //     Logging::debug("Fin de la boucle %d", i);
    // }
    Logging::trace("Encoder 1 : %d", encoder1.get_value());
    Logging::trace("Encoder 2 : %d", encoder2.get_value());
}
