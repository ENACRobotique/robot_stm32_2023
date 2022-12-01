#include "motor_control.h"
#include "utilities/logging.h"

MotorController::MotorController(int mot_dir, int mot_pwn, double kp, double ki, bool reverse) :
    pid(&actual_speed, &cmd_speed, &tgt_speed, kp, ki, 0, DIRECT, AUTOMATIC), 
    pin_pwm(mot_pwn), pin_dir(mot_dir), reverse(reverse) {

}

void MotorController::init() {
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_pwm, OUTPUT);
    actual_speed = 0;
    cmd_speed = 0;
    tgt_speed = 0;
}

void MotorController::set_pid_coefs(double kp, double ki) {
    pid.SetTunings(kp, ki, 0);
}

void MotorController::calc_motor_command() {
    pid.Compute();
}

void MotorController::send_motor_command(double cmd_speed_loc) {
    int pwm = static_cast<int>(cmd_speed_loc * MOTOR_SPEED_M_PER_S_TO_PWM);
    pwm = max(min(pwm, 255), -255); //clamp pwm between -255 and 255
    //Logging::debug("pwm: %d\n", pwm);
    if (pwm > 0) {
        digitalWrite(pin_dir, (reverse)?LOW:HIGH);
    } else {
        digitalWrite(pin_dir, (reverse)?HIGH:LOW);
    }
    analogWrite(pin_pwm, abs(pwm));
}

void MotorController::update() {
    calc_motor_command();
    send_motor_command(cmd_speed);
}