#include "holo_control.h"
#include "utilities/logging.h"
#include "motor_control.h"

HoloControl::HoloControl(MotorController *m1_, MotorController *m2_, MotorController *m3_) : 
    m1(m1_), m2(m2_), m3(m3_) {
}
void HoloControl::stop(){
    this->set_vtarget_raw(0.0,0.0,0.0);
}

void HoloControl::set_vtarget_raw(float v1, float v2, float v3){
    m1->set_target_speed(v1);
    m2->set_target_speed(v2);
    m3->set_target_speed(v3);
}

void HoloControl::set_vtarget_holo(float vx_robot, float vy_robot, float vtheta){
    Eigen::Vector3d vtarget(vx_robot, vy_robot, RAYON*vtheta);
    Eigen::Vector3d motor_speeds = this->axis_to_motors * vtarget;
    m1->set_target_speed(motor_speeds(0));
    m2->set_target_speed(motor_speeds(1));
    m3->set_target_speed(motor_speeds(2));
}

void HoloControl::update(float v1, float v2, float v3){
    m1->update(v1);
    m2->update(v2);
    m3->update(v3);
}