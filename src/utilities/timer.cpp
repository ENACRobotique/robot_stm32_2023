#include "timer.h"

Timer::Timer(uint32_t period): _period(period), _start(0) {
    reset();
}

void Timer::reset() {
    _start = millis();
}

bool Timer::finished() const {
    return millis() - _start >= _period;
}