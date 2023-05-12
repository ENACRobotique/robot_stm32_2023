#ifndef HOLO_CONTROL_H
#define HOLO_CONTROL_H

#include <ArduinoEigen.h>
#include "math.h"
#include "motor_control.h"
#include "config.h"
#include "odometry.h"

#define STOP 0
#define VRAW 1
#define VHOLO 2
#define VTABLE 3
#define POSTABLE 4

class HoloControl{
    public:
        HoloControl(MotorController *m1, MotorController *m2, MotorController *m3, Odometry *odom); // include MotorControl objects when they are implemented
        void stop(); // stop all motors (could be an alias for set_vtarget(0,0,0))
        void set_vtarget_raw(double v1, double v2, double v3);
        void set_vtarget_holo(double vx_robot, double vy_robot, double vtheta);

        void set_vtarget_table(double vx_table, double vy_table, double vtheta); // TO TEST
        void set_ptarget(double x, double y, double theta); // TODO

        void update();
        void set_ratio_slow(float ratio);

    private:
        const Eigen::Matrix3d axis_to_motors 
            {{-sin(ANGLE_M1), cos(ANGLE_M1), 1}, 
             {-sin(ANGLE_M2), cos(ANGLE_M2), 1}, 
             {-sin(ANGLE_M3), cos(ANGLE_M3), 1}};
        const Eigen::Matrix3d motors_to_axis = axis_to_motors.inverse();
        MotorController *m1, *m2, *m3;
        Odometry *odom;

        //add attributes to store target position, and associated parameters, plus PIDs
        int cmd_mode;
        double x_table_tgt, y_table_tgt, theta_tgt;
        double vx_table_tgt, vy_table_tgt, vtheta_tgt;
        bool target_reach;
        void recalc_vtargets_table_to_holo();
        void recalc_vtargets_position_tgt();
        float ratio_slow = 1.f;
};
extern HoloControl holo_control ;
#endif // HOLO_CONTROL_H
