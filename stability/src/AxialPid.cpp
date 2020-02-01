#include "AxialPid.h"
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <cmath>

#include <aquadrone_msgs/SubState.h>

RotPIDController::RotPIDController(float newRKp, float newRKi, float newRKd, float newPKp, float newPKi, float newPKd, float newYKp, float newYKi, float newYKd, float rTarget, float pTarget, float yTarget) :
	//initializing PID constants & targets
	rollKp(newRKp), rollKi(newRKi), rollKd(newRKd), pitchKp(newPKp), pitchKi(newPKi), pitchKd(newPKd), yawKp(newYKp), yawKi(newYKi), yawKd(newYKd), rollTarget(rTarget), pitchTarget(pTarget), yawTarget(yTarget),
	//initializing PID controllers
	rollControl(rollKp, rollKi, rollKd), pitchControl(pitchKp, pitchKi, pitchKd), yawControl(yawKp, yawKi, yawKd)
{
	OUT_LIMIT = 1;
	MARGIN = 0.5;
	//setting output limits
	rollControl.setOutputLimits(OUT_LIMIT);
	pitchControl.setOutputLimits(OUT_LIMIT);
	yawControl.setOutputLimits(OUT_LIMIT);	
	//setting pid targets
	rollControl.setSetpoint(0);
	pitchControl.setSetpoint(0);
	yawControl.setSetpoint(0);
	rollFlip = pitchFlip = yawFlip = false;

}

RotPIDController::RotPIDController(float newRKp, float newRKi, float newRKd, float newPKp, float newPKi, float newPKd, float newYKp, float newYKi, float newYKd) :
	//initializing PID constants & targets
	rollKp(newRKp), rollKi(newRKi), rollKd(newRKd), pitchKp(newPKp), pitchKi(newPKi), pitchKd(newPKd), yawKp(newYKp), yawKi(newYKi), yawKd(newYKd), rollTarget(0), pitchTarget(0), yawTarget(0),
	//setting PID controllers
	rollControl(rollKp, rollKi, rollKd), pitchControl(pitchKp, pitchKi, pitchKd), yawControl(yawKp, yawKi, yawKd)
{
	OUT_LIMIT = 1;
	MARGIN = 0.5;
	//setting pid limits
	rollControl.setOutputLimits(OUT_LIMIT);
	pitchControl.setOutputLimits(OUT_LIMIT);
	yawControl.setOutputLimits(OUT_LIMIT);

	//setting pid targets
	rollControl.setSetpoint(0);
	pitchControl.setSetpoint(0);
	yawControl.setSetpoint(0);

	rollFlip = pitchFlip = yawFlip = false;
}

RotPIDController::RotPIDController():
	//initializing PID constants and targets
	rollKp(0), rollKi(0), rollKd(0), pitchKp(0), pitchKi(0), pitchKd(0), yawKp(0), yawKi(0), yawKd(0), rollTarget(0), pitchTarget(0), yawTarget(0),
	//initializing PID controllers
	rollControl(rollKp, rollKi, rollKd), pitchControl(pitchKp, pitchKi, pitchKd), yawControl(yawKp, yawKi, yawKd)
{
	OUT_LIMIT = 1;
	MARGIN = 0.5;
	//setting pid limits
	rollControl.setOutputLimits(OUT_LIMIT);
	pitchControl.setOutputLimits(OUT_LIMIT);
	yawControl.setOutputLimits(OUT_LIMIT);

	//setting pid targets
	rollControl.setSetpoint(0);
	pitchControl.setSetpoint(0);
	yawControl.setSetpoint(0);

	rollFlip = pitchFlip = yawFlip = false;
	std::cout<<"a"<<std::endl;
}

	
//returns all pid outputs in a vector in this order (roll, pitch, yaw)
void RotPIDController::getCommandValues(float rollValue, float pitchValue, float yawValue, float& rollOut, float& pitchOut, float& yawOut)
{
	rollVal = rollValue;
	pitchVal = pitchValue;
	yawVal = yawValue;
	rollOut = rollControl.getOutput(rollValue - rollTarget);
	pitchOut = pitchControl.getOutput(pitchValue - pitchTarget);
	yawOut = yawControl.getOutput(yawValue - yawTarget);
}

void RotPIDController::runPID(float&pReturn, float&rReturn, float&yReturn)
{
	rReturn = rollControl.getOutput(rollVal - rollTarget);
	pReturn = pitchControl.getOutput(pitchVal - pitchTarget);
	yReturn = yawControl.getOutput(yawVal-yawTarget);

}


//returns just roll pid output
float RotPIDController::getRollValue(float rollValue)
{
	rollVal = rollValue;
	return rollControl.getOutput(rollValue - rollTarget);
}

//returns just pitch pid output
float RotPIDController::getPitchValue(float pitchValue)
{
	pitchVal = pitchValue;
	return pitchControl.getOutput(pitchValue - pitchTarget);
}

//returns just yaw pid output
float RotPIDController::getYawValue(float yawValue)
{
	yawVal = yawValue;
	return yawControl.getOutput(yawValue - yawTarget);
}

//returns whether the current roll position is at the target (within a certain margin)
bool RotPIDController::atRollTarget(float rollValue)
{
	rollVal = rollValue;
	return (fabs(rollVal - rollTarget) <= MARGIN);
}

//returns whether the current pitch position is at the target (within a certain margin)
bool RotPIDController::atPitchTarget(float pitchValue)
{
	pitchVal = pitchValue;
	return (fabs(pitchVal - pitchTarget) <= MARGIN);
}

//returns whether the current yaw position is at the target (within a certain margin)
bool RotPIDController::atYawTarget(float yawValue)
{
	yawVal = yawValue;
	return (fabs(yawVal - yawTarget) <= MARGIN);
}

//returns whether the current rotation is at the target (within a certain margin)
bool RotPIDController::atTarget(float rollValue, float pitchValue, float yawValue)
{
	rollVal = rollValue;
	pitchVal = pitchValue;
	yawVal = yawValue;
	return atRollTarget(rollValue) && atPitchTarget(pitchValue) && atYawTarget(yawValue);
}



//flips pid control (for if the robot is moving away from target instead of towards it)
void RotPIDController::flipRoll()
{
	rollFlip = !rollFlip;
	rollControl.setDirection(rollFlip);
}

void RotPIDController::flipPitch()
{
	pitchFlip = !pitchFlip;
	pitchControl.setDirection(pitchFlip);
}

void RotPIDController::flipYaw()
{
	yawFlip = !yawFlip;
	yawControl.setDirection(yawFlip);
}

//set pid tunings
void RotPIDController::setRollPID(float kp, float ki, float kd)
{
	rollKp = kp;
	rollKi = ki;
	rollKd = kd;

	rollControl.setPID(kp, ki, kd);
}
void RotPIDController::setPitchPID(float kp, float ki, float kd)
{
	pitchKp = kp;
	pitchKi = ki;
	pitchKd = kd;

	pitchControl.setPID(kp, ki, kd);
}
void RotPIDController::setYawPID(float kp, float ki, float kd)
{
	yawKp = kp;
	yawKi = ki;
	yawKd = kd;

	yawControl.setPID(kp, ki, kd);
}

//set pid target values
void RotPIDController::setRollTarget(float newTarget)
{
	rollTarget = newTarget;
	//rollControl.setSetpoint(newTarget);
}

void RotPIDController::setPitchTarget(float newTarget)
{
	pitchTarget = newTarget;
	//pitchControl.setSetpoint(newTarget);
}

void RotPIDController::setYawTarget(float newTarget)
{
	yawTarget = newTarget;
	//yawControl.setSetpoint(newTarget);
}

void RotPIDController::setTargets(geometry_msgs::Vector3 input)
{
	pitchTarget = input.y;
	rollTarget = input.x;
	yawTarget = input.z;
}

void RotPIDController::setInputs(aquadrone_msgs::SubState input)
{
  	pitchVal = input.orientation_RPY.y;
  	yawVal = input.orientation_RPY.z;
  	rollVal = input.orientation_RPY.x;	
}

