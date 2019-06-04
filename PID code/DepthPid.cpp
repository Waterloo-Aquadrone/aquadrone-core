#include <cstdlib>
#include <cmath>
#include <iostream>
#include <vector>

#include "MiniPID.cpp"

using namespace std;

class DepthPIDController
{
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
	DepthPIDController() : depthControl(0, 0, 0), kp(0), ki(0), kd(0), target(0), depthFlip(false)
	{
		depthControl.setSetpoint(target);
		depthControl.setOutputLimits(OUT_LIMIT);
	}

	DepthPIDController(float newKp, float newKi, float newKd) : depthControl(newKp, newKi, newKd), kp(newKp), ki(newKi), kd(newKd), target(0), depthFlip(false)
	{
		depthControl.setSetpoint(target);
		depthControl.setOutputLimits(OUT_LIMIT);
	}

	DepthPIDController(float newKp, float newKi, float newKd, float newTarget) : depthControl(newKp, newKi, newKd), kp(newKp), ki(newKi), kd(newKd), target(newTarget), depthFlip(false)
	{
		depthControl.setSetpoint(target);
		depthControl.setOutputLimits(OUT_LIMIT);
	}

	//LOOP FUNCTIONS
	float getMotorValues(float input)
	{
		return depthControl.getOutput(input);
	}

	bool atDepthTarget(float input)
	{
		return fabs(input - target) <= MARGIN;
	}

	//SPECIALTY FUNCTIONS
	//get tuning constants
	vector<float> getPIDTunings()
	{
		vector<float> result;
		
		result.push_back(kp);
		result.push_back(ki);
		result.push_back(kd);
		return result;
	}

	//set tuning constants (p, i, d)
	void setPIDTunings(vector<float> tunings)
	{
		kp = tunings[0];
		ki = tunings[1];
		kd = tunings[2];
		depthControl.setPID(kp, ki, kd);
	}

	//set pid target
	void setTarget(float newTarget)
	{
		target = newTarget;
		depthControl.setSetpoint(newTarget);
	}

	//get current pid target
	float getTarget()
	{
		return target;
	}

	//flip pid control directions
	void flipPID()
	{
		depthFlip = !depthFlip;
		depthControl.setDirection(depthFlip);
	}
};