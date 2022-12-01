#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "../lib/Arduino-PID-Library-1.2.0/PID_v1.h"

#define MOTOR_SPEED_M_PER_S_TO_PWM 50
    //ceci est un coefficient nul, il faut en trouver un meilleur

class MotorController {
    public:
        MotorController(int mot_dir, int mot_pwm, double kp, double ki, bool reverse);
        void init();
        void set_pid_coefs(double kp, double ki);
        double * get_tgt_speed_ptr() {
            return &tgt_speed;
        }
        double * get_actual_speed_ptr() {
            return &actual_speed;
        }
        double * get_cmd_speed_ptr() {
            return &cmd_speed;
        }
        void calc_motor_command();
        void send_motor_command(double cmd_speed_loc);
        void update();

        void set_tgt_speed(double tgt_speed_loc) {
            tgt_speed = tgt_speed_loc;
        }


    private:
        PID pid;
        double tgt_speed = 0; //link to PID setpoint
        double actual_speed = 0; //link to PID input
        double cmd_speed = 0; //link to PID output
        int pin_pwm;
        int pin_dir;
        bool reverse;

};





#endif // MOTOR_CONTROL_H