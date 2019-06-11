#pragma once
#ifndef DEPTHPID_H
#define DEPTHPID_H

#include "MiniPID.h"

class DepthPIDController
{
private:
	//PID object
	MiniPID depthControl;
	//PID tuning constants
	float kp;
	float ki;
	float kd;
	//pid output limits
	const int OUT_LIMIT = 1;
	//pid target closeness constant
	const float MARGIN = 0.5;
	//pid targets
	float target;
	//reversal bools just in case
	bool depthFlip;

public:
	//Constructors
	DepthPIDController();
	DepthPIDController(float newKp, float newKi, float newKd);
	DepthPIDController(float newKp, float newKi, float newKd, float newTarget);
	//LOOP FUNCTIONS
	float getMotorValues(float input);
	bool atDepthTarget(float input);
	//SPECIALTY FUNCTIONS
	//get tuning constants
	void getPIDTunings(float& getKp, float& getKi, float& getKd);
	//set tuning constants (p, i, d)
	void setPIDTunings(float newKp, float newKi, float newKd);
	//set pid target
	void setTarget(float newTarget);
	//get current pid target
	float getTarget();
	//flip pid control directions
	void flipPID();
};

#endif