#include "encoder.h"
#include "Arduino.h"

int Encoder::get_value() {
    //return counter;
    auto tmp = counter;
    counter = 0;
    return tmp;
}

void Encoder::init() {
    counter = 0;
    counterTotal =0;
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinA), [=]() {
        if (digitalRead(pinB) == HIGH) {
            counter++;
            counterTotal++;
        } else {
            counter--;
            counterTotal--;
        }
    }, RISING);
}