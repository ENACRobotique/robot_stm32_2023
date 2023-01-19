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
            v1 = 0;
            v2 = 0;
            v3 = 0;
            vx_robot = 0;
            vy_robot = 0;
            vtheta = 0;
            vx = 0;
            vy = 0;
            x = 0;
            y = 0;
            theta = 0;
            lastMillis = millis();
        };
        void init(){
            v1 = 0;
            v2 = 0;
            v3 = 0;
            vx_robot = 0;
            vy_robot = 0;
            vtheta = 0;
            vx = 0;
            vy = 0;
            lastMillis = millis();
        }
        void update();

        float get_v1speed(){return v1;};
        float get_v2speed(){return v2;};
        float get_v3speed(){return v3;};

        float get_vx_robot(){return vx_robot;};
        float get_vy_robot(){return vy_robot;};
        float get_vtheta(){return vtheta;};

        float get_vx(){return vx;};
        float get_vy(){return vy;};

        float get_x(){return x;};
        float get_y(){return y;};
        float get_theta(){return theta;};

        void set_x(float x){this->x = x;};
        void set_y(float y){this->y = y;};
        void set_theta(float theta){this->theta = theta;};

    
    private:
        float v1, v2, v3;
        float vx_robot, vy_robot, vtheta;
        float vx, vy;
        float x, y, theta;
        uint32_t lastMillis;

        Encoder *e1, *e2, *e3;
        const Eigen::Matrix3d axis_to_motors 
            {{-sin(ANGLE_M1), cos(ANGLE_M1), 1}, 
             {-sin(ANGLE_M2), cos(ANGLE_M2), 1}, 
             {-sin(ANGLE_M3), cos(ANGLE_M3), 1}};
        const Eigen::Matrix3d motors_to_axis = axis_to_motors.inverse();
};


#endif //ODOMETRY_H