#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "encoder.h"
#include "config.h"
#include <ArduinoEigen.h>
#include "math.h"
#include "Arduino.h"

class Odometry{
    public:
        Odometry(Encoder *e1, Encoder *e2, Encoder *e3):e1(e1),e2(e2),e3(e3){
            v1 = 0.f;
            v2 = 0.f;
            v3 = 0.f;
            vx_robot = 0.f;
            vy_robot = 0.f;
            vtheta = 0.f;
            vx = 0.f;
            vy = 0.f;
            x = 0.f;
            y = 0.f;
            theta = 0.f;
            lastMillis = millis();
        };
        void init(){
            v1 = 0.f;
            v2 = 0.f;
            v3 = 0.f;
            vx_robot = 0.f;
            vy_robot = 0.f;
            vtheta = 0.f;
            vx = 0.f;
            vy = 0.f;
            lastMillis = millis();
        }
        void update();

        double get_v1speed(){return v1;};
        double get_v2speed(){return v2;};
        double get_v3speed(){return v3;};

        double get_vx_robot(){return vx_robot;};
        double get_vy_robot(){return vy_robot;};
        double get_vtheta(){return vtheta;};

        double get_vx(){return vx;};
        double get_vy(){return vy;};

        double get_x(){return x;};
        double get_y(){return y;};
        double get_theta(){return fmod(theta,2*PI);};

        void set_x(double x){this->x = x;};
        void set_y(double y){this->y = y;};
        void set_theta(double theta){this->theta = theta;};

        void print_odometry();

    
    private:
        double v1, v2, v3;
        double vx_robot, vy_robot, vtheta;
        double vx, vy;
        double x, y, theta;
        uint32_t lastMillis;

        Encoder *e1, *e2, *e3;
        const Eigen::Matrix3d axis_to_motors 
            {{-sin(ANGLE_M1), cos(ANGLE_M1), 1}, 
             {-sin(ANGLE_M2), cos(ANGLE_M2), 1}, 
             {-sin(ANGLE_M3), cos(ANGLE_M3), 1}};
        const Eigen::Matrix3d motors_to_axis = axis_to_motors.inverse();
};

extern Odometry odom;
#endif //ODOMETRY_H