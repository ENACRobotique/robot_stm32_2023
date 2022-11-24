#include <Arduino.h>
#include "utilities/logging.h"
#include "utilities/timer.h"

#define LED 13

int i = 0;
Timer timer(1000);

//Arduino LED1 blink
void setup() {
    pinMode(LED, OUTPUT);

    Logging::init(115200);
    Logging::info("Démarrage du robot bas niveau v0.1.0");

    // TODO: Logging::get()
}

void loop() {
    if (timer.finished()) {
        i++;
        
        digitalWrite(LED, !digitalRead(LED));
        timer.reset();

        Logging::debug("Fin de la boucle %d", i);
    }
}
