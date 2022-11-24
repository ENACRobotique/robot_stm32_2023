#ifndef TIMER_HEADER
#define TIMER_HEADER

#include <Arduino.h>

class Timer {
public:
    Timer(uint32_t period);

    void reset();
    bool finished() const;

private:
    uint32_t _period;
    uint32_t _start;
};

#endif // TIMER_HEADER