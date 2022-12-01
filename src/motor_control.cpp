#include "motor_control.h"
#include "utilities/logging.h"

MotorController::MotorController(int mot_dir, int mot_pwn, bool reverse) :
    pin_pwm(mot_pwn), pin_dir(mot_dir), reverse(reverse) {

}

void MotorController::init() {
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_pwm, OUTPUT);
    send_motor_command_pwm(0);
}

void MotorController::send_motor_command_pwm(int pwm) {
    pwm = max(min(pwm, 255), -255); //clamp pwm between -255 and 255
    //Logging::debug("pwm: %d\n", pwm);
    if (pwm > 0) {
        digitalWrite(pin_dir, (reverse)?LOW:HIGH);
    } else {
        digitalWrite(pin_dir, (reverse)?HIGH:LOW);
    }
    analogWrite(pin_pwm, abs(pwm));
}
