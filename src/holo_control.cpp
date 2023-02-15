#include "holo_control.h"
#include "motor_control.h"

HoloControl::HoloControl(MotorController *m1_, MotorController *m2_, MotorController *m3_, Odometry *odom_) : 
    m1(m1_), m2(m2_), m3(m3_), odom(odom_) {
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

void HoloControl::set_vtarget_global(float vx_table, float vy_table, float vtheta){
    // calc vx_robot, vy_robot accounting for robot orientation (sin(theta), cos(theta))
    float vx_robot =  vx_table * cos(odom->get_theta()) + vy_table * sin(odom->get_theta());
    float vy_robot = -vx_table * sin(odom->get_theta()) + vy_table * cos(odom->get_theta());
    this->set_vtarget_holo(vx_robot, vy_robot, vtheta);
}

void HoloControl::set_ptarget(float x, float y, float theta){
    // TODO
    // should have some sort of pid for vx, vy and vtheta
}

void HoloControl::update(){
    // Update target_speeds if target is position

    m1->update(odom->get_v1speed());
    m2->update(odom->get_v2speed());
    m3->update(odom->get_v3speed());
}
