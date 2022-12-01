#ifndef HOLO_CONTROL_H
#define HOLO_CONTROL_H

#include <ArduinoEigen.h>
#include "../lib/Arduino-PID-Library-1.2.0/PID_v1.h"
#include "math.h"
#include "motor_control.h"

#define ANGLE_M1 0.0
#define ANGLE_M2 2*PI/3
#define ANGLE_M3 4*PI/3
#define RAYON 0.115

class HoloControl{
    public:
        HoloControl(MotorController *m1, MotorController *m2, MotorController *m3); // include MotorControl objects when they are implemented
        void stop(); // stop all motors (could be an alias for set_vtarget(0,0,0))
        void set_vtarget(double vx, double vy, double vtheta); // set open loop target velocities
        void update(); // update motor speeds

        
    private:
        const Eigen::Matrix3d axis_to_motors 
            {{-sin(ANGLE_M1), cos(ANGLE_M1), 1}, 
             {-sin(ANGLE_M2), cos(ANGLE_M2), 1}, 
             {-sin(ANGLE_M3), cos(ANGLE_M3), 1}};
        const Eigen::Matrix3d motors_to_axis = axis_to_motors.inverse();
        MotorController *m1, *m2, *m3;
};

#endif // HOLO_CONTROL_H