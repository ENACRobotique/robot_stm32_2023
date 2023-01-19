#include "motor_control.h"
#include "utilities/logging.h"
#include "config.h"

MotorController::MotorController(int mot_dir, int mot_pwn, bool reverse, float kp, float ki, float min, float max, int motor_number) :
    pin_pwm(mot_pwn), pin_dir(mot_dir), reverse(reverse), motor_number(motor_number) {
    pid = new PID(kp, ki, min, max);
}

void MotorController::init() {
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_pwm, OUTPUT);
    send_motor_command_pwm(0);
    lastUpdate = millis();
    pid->reset();
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

void MotorController::set_target_speed(float target_speed){
    this->target_speed = target_speed;
    lastUpdate = millis();
    pid->reset();
}

void MotorController::update(float current_speed){
    // get error, then update pid with error
    uint32_t currentMillis = millis();
    float dv = MAX_ACCEL*static_cast<float>(currentMillis - lastUpdate)/1000.0f;
    ramped_target_speed = clamp(ramped_target_speed - dv, ramped_target_speed + dv, target_speed);
    lastUpdate = currentMillis;
    float error = ramped_target_speed - current_speed;
    float speed_cmd = pid->update(error);
    
    int pwm_cmd = static_cast<int>(speed_cmd * VITESSE_CONSIGNE_TO_PWM_MOTOR);

    send_motor_command_pwm(pwm_cmd);
}

void MotorController::stop_and_reset_pid(){
    send_motor_command_pwm(0);
    lastUpdate = millis();
    pid->reset();
}

void MotorController::set_pid_coefs(float kp, float ki){
    pid->set_ki(ki);
    pid->set_kp(kp);
}