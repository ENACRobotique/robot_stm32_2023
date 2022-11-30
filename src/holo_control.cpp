#include "holo_control.h"
#include "utilities/logging.h"

HoloControl::HoloControl(PID *pid_1, PID *pid_2, PID *pid_3){
    this->pid_1 = pid_1;
    this->pid_2 = pid_2;
    this->pid_3 = pid_3;
}

void HoloControl::stop(){
    this->set_vtarget(0,0,0);
}

void HoloControl::set_vtarget(double vx, double vy, double vtheta){
    Eigen::Vector3d vtarget(vx, vy, vtheta);
    Eigen::Vector3d motor_speeds = this->axis_to_motors * vtarget;
    double v1 = motor_speeds(0);
    double v2 = motor_speeds(1);
    double v3 = motor_speeds(2);
    //add code to set motor pid target
}

void HoloControl::update(){
    pid_1->Compute();
    pid_2->Compute();
    pid_3->Compute();
}