#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "encoder.h"
#include <Arduino.h>
#define INCR_TO_M 0.0003750937734433608
    //trouver mieux

class Odometry 
{
    public:
        Odometry(Encoder *encoder1,Encoder *encoder2,Encoder *encoder3);

        void update();
        double _speed1;
        double _speed2;
        double _speed3;


        int _raw_val1;
        int _raw_val2;
        int _raw_val3;
        
    private:

        unsigned long _time_previous;
        unsigned long _time_current;

        double _dt_coef;
        

        Encoder *_encoder1;
        Encoder *_encoder2;
        Encoder *_encoder3;

        void compute_speed();
        void compute_position();

};

//void setup {}

#endif