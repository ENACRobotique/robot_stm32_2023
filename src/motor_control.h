#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorController {
    public:
        MotorController(int mot_dir, int mot_pwm, bool reverse);
        void init();
        void send_motor_command_pwm(int pwm);

    private:
        int pin_pwm;
        int pin_dir;
        bool reverse;

};

#endif // MOTOR_CONTROL_H