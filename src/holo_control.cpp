#include "holo_control.h"
#include "utilities/logging.h"
#include "motor_control.h"

HoloControl::HoloControl(MotorController *m1_, MotorController *m2_, MotorController *m3_) : 
    m1(m1_), m2(m2_), m3(m3_) {
}
void HoloControl::stop(){
    this->set_vtarget_pwm(0,0,0);
}

void HoloControl::set_vtarget_pwm(int pwm_x,int pwm_y, int pwm_theta){
    Eigen::Vector3d vtarget(pwm_x, pwm_y, RAYON*pwm_theta);
    Eigen::Vector3d motor_speeds = this->axis_to_motors * vtarget;
    m1->send_motor_command_pwm(static_cast<int>(motor_speeds(0)));
    m2->send_motor_command_pwm(static_cast<int>(motor_speeds(1)));
    m3->send_motor_command_pwm(static_cast<int>(motor_speeds(2)));
}
