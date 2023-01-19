#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "pid.h"

class MotorController {
    public:
        MotorController(int mot_dir, int mot_pwm, bool reverse, float kp, float ki, float min, float max, int motor_number);
        void init();
        void set_target_speed(float speed_target);
        void update(float current_speed);
        void stop_and_reset_pid();
        void set_pid_coefs(float kp, float ki);
        void send_motor_command_pwm(int pwm);
        float get_target_speed(){return target_speed;};
        float get_ramped_target_speed(){return ramped_target_speed;};

    private:
        PID *pid;
        int pin_pwm;
        int pin_dir;
        bool reverse;
        float target_speed;
        float ramped_target_speed;
        int motor_number;
        uint32_t lastUpdate;

};

#endif // MOTOR_CONTROL_H