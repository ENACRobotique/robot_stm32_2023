#include "holo_control.h"
#include "motor_control.h"
#include "utilities/utils.h"

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

void HoloControl::set_vtarget_raw(double v1, double v2, double v3){
    m1->set_target_speed(v1);
    m2->set_target_speed(v2);
    m3->set_target_speed(v3);
    cmd_mode = VRAW;
}

void HoloControl::set_vtarget_holo(double vx_robot, double vy_robot, double vtheta){
    Eigen::Vector3d vtarget(vx_robot, vy_robot, RAYON*vtheta);
    Eigen::Vector3d motor_speeds = this->axis_to_motors * vtarget;
    set_vtarget_raw(motor_speeds(0), motor_speeds(1), motor_speeds(2));
    cmd_mode = VHOLO;
}

void HoloControl::recalc_vtargets_table_to_holo(){
    double mtheta = -(odom->get_theta());
    double cos_t = cos(mtheta);
    double sin_t = sin(mtheta);
    double vx_robot = cos_t * vx_table_tgt - sin_t * vy_table_tgt;
    double vy_robot = sin_t * vx_table_tgt + cos_t * vy_table_tgt;
    this->set_vtarget_holo(vx_robot, vy_robot, vtheta_tgt);
}

void HoloControl::set_vtarget_table(double vx_table, double vy_table, double vtheta){
    // calc vx_robot, vy_robot accounting for robot orientation (sin(theta), cos(theta))
    vx_table_tgt = vx_table;
    vy_table_tgt = vy_table;
    vtheta_tgt = vtheta;
    recalc_vtargets_table_to_holo();
    cmd_mode = VTABLE;
}

//this is WIP as heck, to be tested
void HoloControl::recalc_vtargets_position_tgt(){
    double current_x = odom->get_x();
    double current_y = odom->get_y();
    double current_theta = odom->get_theta();
    double dx = x_table_tgt - current_x;
    double dy = y_table_tgt - current_y;
    double dtheta = normalized(theta_tgt - current_theta);


    double dist = distance(x_table_tgt, y_table_tgt, current_x, current_y);

    double target_speed;
    if(dist > SEUIL_PROCHE) {
        target_speed = MAX_VITESSE*ratio_slow;
    } else if(dist < TOL_DIST) {
        target_speed = 0;
    } else {
        target_speed = MAX_VITESSE_PROCHE*ratio_slow;
    }

    // double vx_table = dx / dist * target_speed;
    // double vy_table = dy / dist * target_speed;
    double azimut = atan2(dy, dx);
    double vx_table = cos(azimut) * target_speed;
    double vy_table = sin(azimut) * target_speed;


    double vtheta;
    if(abs(dtheta)>TOL_THETA) {
        float maxSpeedAllowed = MAX_VITESSE_ROTATION*ratio_slow;
        if (abs(dtheta) <= SEUIL_PROCHE_ROTATION){
            maxSpeedAllowed = MAX_VITESSE_ROTATION_PROCHE*ratio_slow;
        }
        if(dtheta>0) {
             vtheta = maxSpeedAllowed;
        } else {
             vtheta = -maxSpeedAllowed;
        }
    } else {
        vtheta = 0.f;
    }
    

    this->set_vtarget_table(vx_table, vy_table, vtheta);
    cmd_mode = POSTABLE;
}


void HoloControl::set_ptarget(double x, double y, double theta){
    x_table_tgt = x;
    y_table_tgt = y;
    theta_tgt = normalized(theta);
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
        //Sotp if close to destination
        if (distance(x_table_tgt, y_table_tgt, odom->get_x(), odom->get_y()) < TOL_DIST && abs(fmod(theta_tgt,2*PI) - fmod(odom->get_theta(),2*PI)) < TOL_THETA) {
            this->stop();
        }
    }
 
    m1->update(odom->get_v1speed());
    m2->update(odom->get_v2speed());
    m3->update(odom->get_v3speed());
}


void HoloControl::set_ratio_slow(float ratio)
{
    this->ratio_slow = ratio;
}