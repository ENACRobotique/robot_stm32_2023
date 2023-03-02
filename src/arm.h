#ifndef ARM_H
#define ARM_H

#include "config.h"
#include <AccelStepper.h>

class ARM
{
    public :
        ARM(AccelStepper *lift_stepper); // + servo + ax12
        void move_to(float dist); // dist en cm 
        void rotate(); // move ax12
        void grab(); //move servo

    private:
        AccelStepper *lift_stepper;
};


#endif