#include "odometry.h"

Odometry::Odometry(Encoder *encoder1,Encoder *encoder2,Encoder *encoder3)
{
    _encoder1=encoder1;
    _encoder2=encoder2;
    _encoder3=encoder3;
    
    _time_previous=millis();
    _time_current=millis();
}

void Odometry::update()
{
    _time_previous = _time_current;

    _raw_val1 = _encoder1->get_value();
    _raw_val2 = _encoder2->get_value();
    _raw_val3 = _encoder3->get_value();

    _time_current = millis();

    _dt_coef = (_time_current - _time_previous)/(1000.0*INCR_TO_M);
 
    compute_speed();
    compute_position();

}

void Odometry::compute_speed()
{
    _speed1 = _raw_val1/_dt_coef; // en m/s
    _speed2 = _raw_val2/_dt_coef;
    _speed3 = _raw_val3/_dt_coef;
    
}

void Odometry::compute_position()
{
    //calculs matriciels
}
