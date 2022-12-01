#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "../lib/Arduino-PID-Library-1.2.0/PID_v1.h"

#define MOTOR_SPEED_M_PER_S_TO_PWM 318.8667520992843
    //ceci est un coefficient nul, il faut en trouver un meilleur
    //et c'est pas linéaire, cette valeur marche bien autour de PWM 10 soit, 0.03m/s

class MotorController {
    public:
        MotorController(int mot_dir, int mot_pwm, double kp, double ki, double *actual_speed_ptr, bool reverse);
        void init();
        void set_pid_coefs(double kp, double ki);
        double * get_tgt_speed_ptr() {
            return &tgt_speed;
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
        double cmd_speed = 0; //link to PID output
        int pin_pwm;
        int pin_dir;
        bool reverse;

};





#endif // MOTOR_CONTROL_H