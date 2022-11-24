#include "encoder.h"
#include "Arduino.h"

int Encoder::get_value() {
    //return counter;
    auto tmp = counter;
    counter = 0;
    return tmp;
}

void Encoder::init() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinA), [=]() {
        if (digitalRead(pinB) == HIGH) {
            counter++;
        } else {
            counter--;
        }
    }, RISING);
}