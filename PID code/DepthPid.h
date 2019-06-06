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
	const int MARGIN = 0.5;
	//pid targets
	float target;
	//reversal bools just in case
	bool depthFlip;

public:
	//Constructors
	DepthPIDController() : depthControl(0, 0, 0), kp(0), ki(0), kd(0), target(0), depthFlip(false);
	DepthPIDController(float newKp, float newKi, float newKd) : depthControl(newKp, newKi, newKd), kp(newKp), ki(newKi), kd(newKd), target(0), depthFlip(false);
	DepthPIDController(float newKp, float newKi, float newKd, float newTarget) : depthControl(newKp, newKi, newKd), kp(newKp), ki(newKi), kd(newKd), target(newTarget), depthFlip(false);
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