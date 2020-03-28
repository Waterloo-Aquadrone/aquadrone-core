#pragma once

#ifndef AXIALPID_H
#define AXIALPID_H
#include "MiniPID.h"
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

#include <aquadrone_msgs/SubState.h>

class RotPIDController {
private:
	//pid control objects
	MiniPID rollControl;
	MiniPID pitchControl;
	MiniPID yawControl;

	//pid tuning (kp, ki, kd)
	float rollKp, rollKi, rollKd;
	float pitchKp, pitchKi, pitchKd;
	float yawKp, yawKi, yawKd;

	//pid output limits
	double OUT_LIMIT_R;
	double OUT_LIMIT_P;
	double OUT_LIMIT_Y;

	//pid target closeness constant
	int MARGIN;

	//pid targets
	float rollTarget;
	float pitchTarget;
	float yawTarget;

	//latest sensor input
	float rollVal;
	float pitchVal;
	float yawVal;

	//pid reverses (just in case something is messed up)
	bool rollFlip;
	bool pitchFlip;
	bool yawFlip;

	//ros things (WILL NEED TO MANUALLY SET UP USING startRosLoop FUNCTION)
	ros::Subscriber target;
	ros::Subscriber sensor;

public:
	//constructors (WILL NOT SET UP THE ROS)
	RotPIDController(float newRKp, float newRKi, float newRKd, float newPKp, float newPKi, float newPKd, float newYKp, float newYKi, float newYKd, float rTarget, float pTarget, float yTarget);
		
	RotPIDController(float newRKp, float newRKi, float newRKd, float newPKp, float newPKi, float newPKd, float newYKp, float newYKi, float newYKd);

	RotPIDController();

	//LOOP FUNCTIONS (TO BE CALLED IN A LOOP THING)

	//returns all pid outputs in a vector in this order (roll, pitch, yaw)
	void getCommandValues(float rollValue, float pitchValue, float yawValue, float& rollOut, float& pitchOut, float& yawOut);

	//runs pid without returning values
	void runPID(float&, float&, float&);

	//returns just roll pid output
	float getRollValue(float rollValue);

	//returns just pitch pid output
	float getPitchValue(float pitchValue);

	//returns just yaw pid output
	float getYawValue(float yawValue);

	//returns whether the current roll position is at the target (within a certain margin)
	bool atRollTarget(float rollValue);

	//returns whether the current pitch position is at the target (within a certain margin)
	bool atPitchTarget(float pitchValue);

	//returns whether the current yaw position is at the target (within a certain margin)
	bool atYawTarget(float yawValue);

	//returns whether the current rotation is at the target (within a certain margin)
	bool atTarget(float rollValue, float pitchValue, float yawValue);

	//SPECIALTY FUNCTIONS (called not in a loop)

	//flips pid control (for if the robot is moving away from target instead of towards it)
	void flipRoll();

	void flipPitch();

	void flipYaw();

	//set pid tunings
	void setRollPID(float kp, float ki, float kd, double limit);
	void setPitchPID(float kp, float ki, float kd, double limit);
	void setYawPID(float kp, float ki, float kd, double limit);

	//set pid target values
	void setRollTarget(float newTarget);

	void setPitchTarget(float newTarget);

	void setYawTarget(float newTarget);

	//***NEW*** startRosLoop will return a nodehandle pointer and subscribers
	void startRosLoop(int, char**/*,ros::Subscriber*&, ros::Subscriber*&*/);

	void setTargets(geometry_msgs::Vector3);

	void setInputs(aquadrone_msgs::SubState);
	
};

	

#endif
