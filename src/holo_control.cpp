#include "holo_control.h"
#include "utilities/logging.h"

HoloControl::HoloControl(){
    //TODO: calibrate PIDs
    pid_1 = new PID(&cmd_v1, &setp_v1, &tgt_v1, 0.1, 0.1, 0.1, DIRECT, AUTOMATIC);
    pid_2 = new PID(&cmd_v2, &setp_v2, &tgt_v2, 0.1, 0.1, 0.1, DIRECT, AUTOMATIC);
    pid_3 = new PID(&cmd_v3, &setp_v3, &tgt_v3, 0.1, 0.1, 0.1, DIRECT, AUTOMATIC);

}

void HoloControl::stop(){
    this->set_vtarget(0,0,0);
}

void HoloControl::set_vtarget(double vx, double vy, double vtheta){
    Eigen::Vector3d vtarget(vx, vy, RAYON*vtheta);
    Eigen::Vector3d motor_speeds = this->axis_to_motors * vtarget;
    tgt_v1 = motor_speeds(0);
    tgt_v2 = motor_speeds(1);
    tgt_v3 = motor_speeds(2);
}

void HoloControl::update(){
    pid_1->Compute();
    pid_2->Compute();
    pid_3->Compute();
}

double *HoloControl::getcmd_v1_ptr(){
    return &cmd_v1;
}

double *HoloControl::getcmd_v2_ptr(){
    return &cmd_v2;
}

double *HoloControl::getcmd_v3_ptr(){
    return &cmd_v3;
}