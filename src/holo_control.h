#ifndef HOLO_CONTROL_H
#define HOLO_CONTROL_H

#include <ArduinoEigen.h>
#include "math.h"
#include "motor_control.h"
#include "config.h"
#include "odometry.h"

class HoloControl{
    public:
        HoloControl(MotorController *m1, MotorController *m2, MotorController *m3, Odometry *odom); // include MotorControl objects when they are implemented
        void stop(); // stop all motors (could be an alias for set_vtarget(0,0,0))
        void set_vtarget_raw(float v1, float v2, float v3);
        void set_vtarget_holo(float vx_robot, float vy_robot, float vtheta);

        void set_vtarget_global(float vx_table, float vy_table, float vtheta); // TO TEST
        void set_ptarget(float x, float y, float theta); // TODO

        void update();

    private:
        const Eigen::Matrix3d axis_to_motors 
            {{-sin(ANGLE_M1), cos(ANGLE_M1), 1}, 
             {-sin(ANGLE_M2), cos(ANGLE_M2), 1}, 
             {-sin(ANGLE_M3), cos(ANGLE_M3), 1}};
        const Eigen::Matrix3d motors_to_axis = axis_to_motors.inverse();
        MotorController *m1, *m2, *m3;
        Odometry *odom;

        //add attributes to store target position, and associated parameters, plus PIDs
        
};

#endif // HOLO_CONTROL_H