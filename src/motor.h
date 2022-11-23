#pragma once 

#include "Arduino.h"
#include "config.h"

class MotorControl {
public:
	void init();
	void update(); //Boucle d'asservissement
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

	void set_cons_goal(double cmd_speed_1, double cmd_speed_2, double cmd_speed_3); //Set speed goal to reach at the end of the servo loop (Asservissement) in m/s

	void cmd_mot(double cmd_speed_1, double cmd_speed_2, double cmd_speed_3); //Speed of each wheel in m/s to send now to the motor
	void send_mot_signal(int cmde1, int cmde2, int cmde3); //PWM(0-255) command to send to each motor
	void stop(); 
private:
	const int CMD_TIMEOUT = 1500; //Stop sending motor command after x milliseconds to avoid loosing control of robot
	unsigned long last_cmd = millis();

	// In m/s : 
	double goal_speed_1;
	double goal_speed_2;
	double goal_speed_3;
	double cons_speed_1;
	double cons_speed_2;
	double cons_speed_3;
};

extern MotorControl motor;