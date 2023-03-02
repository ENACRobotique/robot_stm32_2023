#include "arm.h"


ARM::ARM(AccelStepper *lift_stepper)
{
    lift_stepper->setCurrentPosition(0);
    lift_stepper->setMaxSpeed(STEP_SPEED);
    lift_stepper->setAcceleration(STEP_ACC);
}

void ARM::move_to(float dist)
{
    lift_stepper->moveTo(dist*CM_TO_STEP);
}
