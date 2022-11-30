#ifndef HOLO_CONTROL_H
#define HOLO_CONTROL_H

#include <ArduinoEigen.h>
#include "../lib/Arduino-PID-Library-1.2.0/PID_v1.h"

#define CPHI cos(2*PI/3)
#define SPHI sin(2*PI/3)

class HoloControl{
    public:
        HoloControl(PID *pid_1, PID *pid_2, PID *pid_3); // include MotorControl objects when they are implemented
        void stop(); // stop all motors (could be an alias for set_vtarget(0,0,0))
        void set_vtarget(double vx, double vy, double vtheta); // set open loop target velocities
        //void set_target(double x, double y, double theta); // set closed loop target position
        void update(); // update motor speeds 
        
    private:
        const Eigen::Matrix3d axis_to_motors = (Eigen::Matrix3d() << 0, CPHI, 1-2*CPHI*CPHI, 2, SPHI, 2*SPHI*CPHI, 1/3, 1/3, 1/3).finished();
        PID *pid_1, *pid_2, *pid_3; // PID controllers for each motor
        //PID pid_x, pid_y, pid_theta; // PID controllers for closed loop control
};

#endif // HOLO_CONTROL_H