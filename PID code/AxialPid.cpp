#include <cstdlib>
#include <cmath>
#include <iostream>
#include <vector>

#include "MiniPID.cpp"

using namespace std;

class RotPIDController {
	//pid control objects
	MiniPID rollControl;
	MiniPID pitchControl;
	MiniPID yawControl;

	//pid tuning (kp, ki, kd)
	vector<double> rollTune = { 0,0,0 };
	vector<double> pitchTune = { 0,0,0 };
	vector<double> yawTune = { 0,0,0 };

	//pid output limits
	const int OUT_LIMIT = 1;

	//pid target closeness constant
	const int MARGIN = 0.5;

	//pid targets
	float rollTarget;
	float pitchTarget;
	float yawTarget;

	//pid reverses (just in case something is messed up)
	bool rollFlip;
	bool pitchFlip;
	bool yawFlip;

public:
	//constructors
	RotPIDController(vector<double> newRoll, vector<double> newPitch, vector<double> newYaw, float rTarget, float pTarget, float yTarget) : 
		//initializing the PIDs for roll, pitch, yaw
		rollControl(MiniPID(newRoll[0], newRoll[1], newRoll[2])),	
		pitchControl(MiniPID(newPitch[0], newPitch[1], newPitch[2])),
		yawControl(MiniPID(newYaw[0], newYaw[1], newYaw[2])),
		//setting target rotations
		rollTarget(rTarget), pitchTarget(pTarget), yawTarget(yTarget)
	{
		//initializing the tuning arrays
		rollTune = newRoll;
		pitchTune = newPitch;
		yawTune = newYaw;

		//setting pid limits
		rollControl.setOutputLimits(OUT_LIMIT);
		pitchControl.setOutputLimits(OUT_LIMIT);
		yawControl.setOutputLimits(OUT_LIMIT);

		//setting pid targets
		rollControl.setSetpoint(rTarget);
		pitchControl.setSetpoint(pTarget);
		yawControl.setSetpoint(yTarget);

		rollFlip = pitchFlip = yawFlip = false;
	}

	RotPIDController(vector<double> newRoll, vector<double> newPitch, vector<double> newYaw) :
		//initializing the PIDs for roll, pitch, yaw
		rollControl(MiniPID(newRoll[0], newRoll[1], newRoll[2])),
		pitchControl(MiniPID(newPitch[0], newPitch[1], newPitch[2])),
		yawControl(MiniPID(newYaw[0], newYaw[1], newYaw[2])),
		//setting target rotations
		rollTarget(0), pitchTarget(0), yawTarget(0)
	{
		//initializing the tuning arrays
		rollTune = newRoll;
		pitchTune = newPitch;
		yawTune = newYaw;

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

	RotPIDController():
		//initializing the PIDs for roll, pitch, yaw
		rollControl({ 0,0,0 }),
		pitchControl({ 0,0,0 }),
		yawControl({ 0,0,0 }),
		//setting target rotations
		rollTarget(0), pitchTarget(0), yawTarget(0)
	{
		//initializing the tuning arrays
		rollTune = pitchTune = yawTune = { 0,0,0 };

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

	//LOOP FUNCTIONS (TO BE CALLED IN A LOOP THING)
	
	//returns all pid outputs in a vector in this order (roll, pitch, yaw)
	vector<float> getMotorValues(float rollValue, float pitchValue, float yawValue)
	{
		vector<float> result;
		result.push_back(rollControl.getOutput(rollValue));
		result.push_back(pitchControl.getOutput(pitchValue));
		result.push_back(yawControl.getOutput(yawValue));
		return result;
	}

	//returns just roll pid output
	float getRollValue(float rollValue)
	{
		return rollControl.getOutput(rollValue);
	}

	//returns just pitch pid output
	float getPitchValue(float pitchValue)
	{
		return pitchControl.getOutput(pitchValue);
	}

	//returns just yaw pid output
	float getYawValue(float yawValue)
	{
		return yawControl.getOutput(yawValue);
	}

	//returns whether the current roll position is at the target (within a certain margin)
	bool atRollTarget(float rollValue)
	{
		return (fabs(rollValue - rollTarget) <= MARGIN);
	}

	//returns whether the current pitch position is at the target (within a certain margin)
	bool atPitchTarget(float pitchValue)
	{
		return (fabs(pitchValue - pitchTarget) <= MARGIN);
	}

	//returns whether the current yaw position is at the target (within a certain margin)
	bool atYawTarget(float yawValue)
	{
		return (fabs(yawValue - yawTarget) <= MARGIN);
	}

	//returns whether the current rotation is at the target (within a certain margin)
	bool atTarget(float rollValue, float pitchValue, float yawValue)
	{
		return atRollTarget(rollValue) && atPitchTarget(pitchValue) && atYawTarget(yawValue);
	}

	//SPECIALTY FUNCTIONS (called not in a loop)

	//flips pid control (for if the robot is moving away from target instead of towards it)
	void flipRoll()
	{
		rollFlip = !rollFlip;
		rollControl.setDirection(rollFlip);
	}

	void flipPitch()
	{
		pitchFlip = !pitchFlip;
		pitchControl.setDirection(pitchFlip);
	}

	void flipYaw()
	{
		yawFlip = !yawFlip;
		yawControl.setDirection(yawFlip);
	}

	//set pid tunings
	void setRollPID(float kp, float ki, float kd)
	{
		rollTune[0] = kp;
		rollTune[1] = ki;
		rollTune[2] = kd;

		rollControl.setPID(kp, ki, kd);
	}
	void setPitchPID(float kp, float ki, float kd)
	{
		pitchTune[0] = kp;
		pitchTune[1] = ki;
		pitchTune[2] = kd;

		pitchControl.setPID(kp, ki, kd);
	}
	void setYawPID(float kp, float ki, float kd)
	{
		yawTune[0] = kp;
		yawTune[1] = ki;
		yawTune[2] = kd;

		yawControl.setPID(kp, ki, kd);
	}

	//set pid target values
	void setRollTarget(float newTarget)
	{
		rollTarget = newTarget;
		rollControl.setSetpoint(newTarget);
	}

	void setPitchTarget(float newTarget)
	{
		pitchTarget = newTarget;
		pitchControl.setSetpoint(newTarget);
	}

	void setYawTarget(float newTarget)
	{
		yawTarget = newTarget;
		yawControl.setSetpoint(newTarget);
	}
};