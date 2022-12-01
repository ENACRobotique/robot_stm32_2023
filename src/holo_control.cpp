#include "holo_control.h"
#include "utilities/logging.h"
#include "motor_control.h"

HoloControl::HoloControl(MotorController *m1_, MotorController *m2_, MotorController *m3_) : 
    m1(m1_), m2(m2_), m3(m3_) {
}
void HoloControl::stop(){
    this->set_vtarget(0,0,0);
}

void HoloControl::set_vtarget(double vx, double vy, double vtheta){
    Eigen::Vector3d vtarget(vx, vy, RAYON*vtheta);
    Eigen::Vector3d motor_speeds = this->axis_to_motors * vtarget;
    m1->set_tgt_speed(motor_speeds(0));
    m2->set_tgt_speed(motor_speeds(1));
    m3->set_tgt_speed(motor_speeds(2));
}

void HoloControl::update(){
    //TODO: implement PID control
    m1->update();
    m2->update();
    m3->update();
}
