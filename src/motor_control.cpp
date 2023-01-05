#include "motor_control.h"
#include "utilities/logging.h"

MotorController::MotorController(int mot_dir, int mot_pwn, bool reverse, float kp, float ki, float min, float max, int motor_number) :
    pin_pwm(mot_pwn), pin_dir(mot_dir), reverse(reverse), motor_number(motor_number) {
    pid = new PID(kp, ki, 0.0, min, max);
}

void MotorController::init() {
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_pwm, OUTPUT);
    send_motor_command_pwm(0);
    pid->reset();
}

/** this function is deprecated
 * @deprecated
*/
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

void MotorController::set_target_speed(float target_speed){
    this->target_speed = target_speed;
    
}

void MotorController::update(){
    // get error, then update pid with error
}

void MotorController::stop_and_reset_pid(){

}

void MotorController::set_pid_coefs(float kp, float ki){

}