#pragma once

#include "AX12A.h"
//This is WIP. Please change these two values if it doesn't work as intended
typedef enum {
    RANGE_C = 200,
    DEPLOYE_C = 490//560 si Pb Sero Bleu réglé
} etatDeguisement;

class Deguisement{
    public:
        void init(DynamixelSerial* ax, int num);
        void retract();
        void deploie();
    private:
        DynamixelSerial* ax12;
        int numAX = 0;

};

extern Deguisement costume;