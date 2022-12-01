#ifndef HOLO_CONTROL_H
#define HOLO_CONTROL_H

#include <ArduinoEigen.h>
#include "../lib/Arduino-PID-Library-1.2.0/PID_v1.h"
#include "math.h"

#define ANGLE_M1 0.0
#define ANGLE_M2 2*PI/3
#define ANGLE_M3 4*PI/3
#define RAYON 1.0

class HoloControl{
    public:
        HoloControl(); // include MotorControl objects when they are implemented
        void stop(); // stop all motors (could be an alias for set_vtarget(0,0,0))
        void set_vtarget(double vx, double vy, double vtheta); // set open loop target velocities
        void update(); // update motor speeds

        double *getcmd_v1_ptr();
        double *getcmd_v2_ptr();
        double *getcmd_v3_ptr();
        
    private:
        const Eigen::Matrix3d axis_to_motors 
            {{-sin(ANGLE_M1), cos(ANGLE_M1), 1}, 
             {-sin(ANGLE_M2), cos(ANGLE_M2), 1}, 
             {-sin(ANGLE_M3), cos(ANGLE_M3), 1}};
        const Eigen::Matrix3d motors_to_axis = axis_to_motors.inverse();
        PID *pid_1, *pid_2, *pid_3; // PID controllers for each motor
        //variables used by PIDs
            //target speeds for motors
            double tgt_v1 = 0, tgt_v2 = 0, tgt_v3 = 0;
            //command speeds for motors
            double cmd_v1 = 0, cmd_v2 = 0, cmd_v3 = 0;
            //setpoints for motor PIDs
            double setp_v1 = 0, setp_v2 = 0, setp_v3 = 0;
};

#endif // HOLO_CONTROL_H