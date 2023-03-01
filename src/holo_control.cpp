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

void HoloControl::recalc_vtargets_table_to_holo(){
    float mtheta = -(odom->get_theta());
    float cos_t = cos(mtheta);
    float sin_t = sin(mtheta);
    float vx_robot = cos_t * vx_table_tgt - sin_t * vy_table_tgt;
    float vy_robot = sin_t * vx_table_tgt + cos_t * vy_table_tgt;
    this->set_vtarget_holo(vx_robot, vy_robot, vtheta_tgt);
}

void HoloControl::set_vtarget_table(float vx_table, float vy_table, float vtheta){
    // calc vx_robot, vy_robot accounting for robot orientation (sin(theta), cos(theta))
    vx_table_tgt = vx_table;
    vy_table_tgt = vy_table;
    vtheta_tgt = vtheta;
    recalc_vtargets_table_to_holo();
    cmd_mode = VTABLE;
}

//this is WIP as heck, to be tested
void HoloControl::recalc_vtargets_position_tgt(){
    float current_x = odom->get_x();
    float current_y = odom->get_y();
    float current_theta = odom->get_theta();
    float dx = x_table_tgt - current_x;
    float dy = y_table_tgt - current_y;
    float dtheta = theta_tgt - current_theta;

    float vx_table = clamp(-dx * DECELERATION_AVEC_DISTANCE, dx * DECELERATION_AVEC_DISTANCE, ((dx>0)?1:-1) * MAX_VITESSE);
    float vy_table = clamp(-dy * DECELERATION_AVEC_DISTANCE, dy * DECELERATION_AVEC_DISTANCE, ((dy>0)?1:-1) * MAX_VITESSE);
    //the two speeds above should be normalised so that the total speed is <= MAX_VITESSE, but I'm lazy and tired so maybe later
    float vtheta = clamp(-dtheta * DECELERATION_AVEC_DISTANCE, dtheta * DECELERATION_AVEC_DISTANCE, ((dtheta>0)?1:-1) * MAX_VITESSE_ROTATION);
    this->set_vtarget_table(vx_table, vy_table, vtheta);
    cmd_mode = POSTABLE;
}

void HoloControl::set_ptarget(float x, float y, float theta){
    x_table_tgt = x;
    y_table_tgt = y;
    theta_tgt = theta;
    recalc_vtargets_position_tgt();
    cmd_mode = POSTABLE;
}

void HoloControl::update(){
    // Update target_speeds if target is position
    if (cmd_mode == VTABLE) {
        recalc_vtargets_table_to_holo();
        cmd_mode = VTABLE; // seems stupid, but keep this line, otherwise cmd_mode gets erased to VHOLO
    }
    else if (cmd_mode == POSTABLE) {
        // TO TEST
        recalc_vtargets_position_tgt();
        recalc_vtargets_table_to_holo();
        cmd_mode = POSTABLE; // seems stupid, but keep this line, otherwise cmd_mode gets erased to VHOLO
        //Sotp if closze to destination
        if (distance(x_table_tgt, y_table_tgt, odom->get_x(), odom->get_y()) < 0.02) {
            this->stop();
        }
    }
 
    m1->update(odom->get_v1speed());
    m2->update(odom->get_v2speed());
    m3->update(odom->get_v3speed());
}
