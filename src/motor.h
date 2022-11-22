#pragma once 

#include "Arduino.h"
#include "config.h"

class MotorControl {
public:
	void update();
	void init();
	void set_PID (char c, int kp, int ki);

	double get_cons_speed_1(){
		return cons_speed_1;
	}

	double get_cons_speed_2(){
		return cons_speed_2;
	}

	double get_cons_speed_3(){
		return cons_speed_3;
	}
	void cmd_mot(double cmd_speed_1, double cmd_speed_2, double cmd_speed_3);
	void send_mot_signal(int cmde1, int cmde2, int cmde3);
	void set_cons(double cmd_speed_1, double cmd_speed_2, double cmd_speed_3);
	void stop();
private:
	const int CMD_TIMEOUT = 1500;
	unsigned long last_cmd = millis();
	double goal_speed;
	double goal_omega;
	double cons_speed;
	double cons_omega;
};

extern MotorControl motor;