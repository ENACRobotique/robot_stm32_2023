#pragma once
#include <Arduino.h>

template <typename T>
T clamp(T inf, T sup, T x) {
    return min(sup, max(inf, x));
}

inline float distance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}