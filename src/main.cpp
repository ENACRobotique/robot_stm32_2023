#include <Arduino.h>
#include "utilities/logging.h"

#define LED 13

int i = 0;

//Arduino LED1 blink
void setup() {
    pinMode(LED, OUTPUT);

    Logging::init(115200);
    Logging::info("Démarrage du robot bas niveau v0.1.0");

    // TODO: Logging::get()
}

void loop() {
    i++;

    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
    delay(1000);

    Logging::debug("Fin de la boucle %d", i);
}
