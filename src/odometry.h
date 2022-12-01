#ifndef ODOMETRY_H
#define ODOMETRY_H 
#include "encoder.h"
#include <Arduino.h>
#define INCR_PER_TURN 100.0
#define RADIUS 0.05

class Odometry 
{
    public:
        Odometry(Encoder *encoder1,Encoder *encoder2,Encoder *encoder3);

        void update();
        int _speed1;
        int _speed2;
        int _speed3;
        
        


    private:
        
        int _raw_val1;
        int _raw_val2;
        int _raw_val3;

        unsigned long _time_previous;
        unsigned long _time_current;

        float _dt_coef;
        

        Encoder *_encoder1;
        Encoder *_encoder2;
        Encoder *_encoder3;

        void compute_speed();
        void compute_position();

};

//void setup {}

#endif