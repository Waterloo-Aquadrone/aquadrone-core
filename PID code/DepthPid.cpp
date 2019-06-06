#include "DepthPid.h"
#include <cmath>
	//Constructors
	DepthPIDController::DepthPIDController() : depthControl(0, 0, 0), kp(0), ki(0), kd(0), target(0), depthFlip(false)
	{
		depthControl.setSetpoint(target);
		depthControl.setOutputLimits(OUT_LIMIT);
	}

	DepthPIDController::DepthPIDController(float newKp, float newKi, float newKd) : depthControl(newKp, newKi, newKd), kp(newKp), ki(newKi), kd(newKd), target(0), depthFlip(false)
	{
		depthControl.setSetpoint(target);
		depthControl.setOutputLimits(OUT_LIMIT);
	}

	DepthPIDController::DepthPIDController(float newKp, float newKi, float newKd, float newTarget) : depthControl(newKp, newKi, newKd), kp(newKp), ki(newKi), kd(newKd), target(newTarget), depthFlip(false)
	{
		depthControl.setSetpoint(target);
		depthControl.setOutputLimits(OUT_LIMIT);
	}

	//LOOP FUNCTIONS
	float DepthPIDController::getMotorValues(float input)
	{
		return depthControl.getOutput(input);
	}

	bool DepthPIDController::atDepthTarget(float input)
	{
		return fabs(input - target) <= MARGIN;
	}

	//SPECIALTY FUNCTIONS
	//get tuning constants
	void DepthPIDController::getPIDTunings(float& getKp, float& getKi, float& getKd)
	{
		getKp = kp;
		getKi = ki;
		getKd = kd;
	}

	//set tuning constants (p, i, d)
	void DepthPIDController::setPIDTunings(float newKp, float newKi, float newKd)
	{
		kp = newKp;
		ki = newKi;
		kd = newKd;
		depthControl.setPID(kp, ki, kd);
	}

	//set pid target
	void DepthPIDController::setTarget(float newTarget)
	{
		target = newTarget;
		depthControl.setSetpoint(newTarget);
	}

	//get current pid target
	float DepthPIDController::getTarget()
	{
		return target;
	}

	//flip pid control directions
	void DepthPIDController::flipPID()
	{
		depthFlip = !depthFlip;
		depthControl.setDirection(depthFlip);
	}
};