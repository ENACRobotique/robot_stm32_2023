#pragma once
#include <Arduino.h>
#include "utilities/utils.h"

const double PROP_MAX_INTEGRAL=0.5;
class PID {
public:
    /**
     * Instantiate PID
    */
    PID(double kp, double ki, double min, double max):
        kp(kp), ki(ki), min(min), max(max) { this->kd = 0; }

    /**
     * Update PID calculations
     * @param error difference between target speed and current speed
     * @returns speed command
    */
    double update(double error) {
        uint32_t now = millis();
        double dt = (now-prev_time)/1000.0;
        prev_time = now;
        double delta_error = (error - prev_error)/dt;
        prev_error = error;
        integral += error*dt;
        if (ki>0.00001){
            integral = clamp((min*PROP_MAX_INTEGRAL)/ki, (max*PROP_MAX_INTEGRAL)/ki, integral);
            // clamp intergral pour que ki*integral ne dépasse pas x% de max.
        }
        double output = kp * error + ki * integral + kd * delta_error;
        return clamp(min, max, output);
    }

    /**
     * Resets PID
    */
    void reset() {
        integral = 0.0;
        prev_error = 0.0;
    }

    void set_kp(double kp) { this->kp = kp; }
    void set_ki(double ki){ this->ki = ki; }
    // void set_kd(double kd);
    // void set_min(double min);
    // void set_max(double max);
    // double get_kp();
    // double get_ki();
    // double get_kd();
    // double get_min();
    // double get_max();
private:
    double kp;
    double ki;
    double kd;
    double min;
    double max;
    double integral;
    double prev_error;
    uint32_t prev_time;
};