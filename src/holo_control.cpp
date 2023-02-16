#include "holo_control.h"
#include "motor_control.h"

HoloControl::HoloControl(MotorController *m1_, MotorController *m2_, MotorController *m3_, Odometry *odom_) : 
    m1(m1_), m2(m2_), m3(m3_), odom(odom_) {
        cmd_mode = STOP;
        vx_table_tgt = 0.f;
        vy_table_tgt = 0.f;
        vtheta_tgt = 0.f;
}
void HoloControl::stop(){
    this->set_vtarget_raw(0.0,0.0,0.0);
    vx_table_tgt = 0.f;
    vy_table_tgt = 0.f;
    vtheta_tgt = 0.f;
    cmd_mode = STOP;
}

void HoloControl::set_vtarget_raw(float v1, float v2, float v3){
    m1->set_target_speed(v1);
    m2->set_target_speed(v2);
    m3->set_target_speed(v3);
    cmd_mode = VRAW;
}

void HoloControl::set_vtarget_holo(float vx_robot, float vy_robot, float vtheta){
    Eigen::Vector3d vtarget(vx_robot, vy_robot, RAYON*vtheta);
    Eigen::Vector3d motor_speeds = this->axis_to_motors * vtarget;
    set_vtarget_raw(motor_speeds(0), motor_speeds(1), motor_speeds(2));
    cmd_mode = VHOLO;
}

void HoloControl::set_vtarget_table(float vx_table, float vy_table, float vtheta){
    // calc vx_robot, vy_robot accounting for robot orientation (sin(theta), cos(theta))
    vx_table_tgt = vx_table;
    vy_table_tgt = vy_table;
    vtheta_tgt = vtheta;
    float mtheta = -(odom->get_theta());
    float cos_t = cos(mtheta);
    float sin_t = sin(mtheta);
    float vx_robot = cos_t * vx_table_tgt - sin_t * vy_table_tgt;
    float vy_robot = sin_t * vx_table_tgt + cos_t * vy_table_tgt;
    this->set_vtarget_holo(vx_robot, vy_robot, vtheta);
    cmd_mode = VTABLE;
}

void HoloControl::set_ptarget(float x, float y, float theta){
    // TODO
    // should have some sort of pid for vx, vy and vtheta
    cmd_mode = POSTABLE;
}

void HoloControl::update(){
    // Update target_speeds if target is position
    if (cmd_mode == VTABLE) {
        float mtheta = -(odom->get_theta());
        float cos_t = cos(mtheta);
        float sin_t = sin(mtheta);
        float vx_robot = cos_t * vx_table_tgt - sin_t * vy_table_tgt;
        float vy_robot = sin_t * vx_table_tgt + cos_t * vy_table_tgt;

        this->set_vtarget_holo(vx_robot, vy_robot, vtheta_tgt);
        cmd_mode = VTABLE; // seems stupid, but keep this line, otherwise cmd_mode gets erased to VHOLO
    }
 
    m1->update(odom->get_v1speed());
    m2->update(odom->get_v2speed());
    m3->update(odom->get_v3speed());
}
