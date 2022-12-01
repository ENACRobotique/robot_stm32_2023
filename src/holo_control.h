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
        HoloControl(MotorController *m1, MotorController *m2, MotorController *m3); // include MotorControl objects when they are implemented
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
        //variables used by PIDs
        double *tgt_v1, *tgt_v2, *tgt_v3;
        MotorController *m1, *m2, *m3;
};

#endif // HOLO_CONTROL_H