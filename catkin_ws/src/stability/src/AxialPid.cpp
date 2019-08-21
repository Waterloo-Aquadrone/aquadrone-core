#include "AxialPid.h"
#include </opt/ros/kinetic/include/std_msgs/Float64.h>
#include </opt/ros/kinetic/include/geometry_msgs/Vector3.h>
#include </opt/ros/kinetic/include/sensor_msgs/Imu.h>
#include </opt/ros/kinetic/include/ros/ros.h>
#include </opt/ros/kinetic/include/geometry_msgs/Quaternion.h>
#include <cmath>

RotPIDController::RotPIDController(float newRKp, float newRKi, float newRKd, float newPKp, float newPKi, float newPKd, float newYKp, float newYKi, float newYKd, float rTarget, float pTarget, float yTarget) :
	//initializing PID constants & targets
	rollKp(newRKp), rollKi(newRKi), rollKd(newRKd), pitchKp(newPKp), pitchKi(newPKi), pitchKd(newPKd), yawKp(newYKp), yawKi(newYKi), yawKd(newYKd), rollTarget(rTarget), pitchTarget(pTarget), yawTarget(yTarget),
	//initializing PID controllers
	rollControl(rollKp, rollKi, rollKd), pitchControl(pitchKp, pitchKi, pitchKd), yawControl(yawKp, yawKi, yawKd)
{
	OUT_LIMIT = 5;
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
	OUT_LIMIT = 5;
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
	OUT_LIMIT = 5;
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

//LOOP FUNCTIONS (TO BE CALLED IN A LOOP THING)
	
//returns all pid outputs in a vector in this order (roll, pitch, yaw)
void RotPIDController::getMotorValues(float rollValue, float pitchValue, float yawValue, float& rollOut, float& pitchOut, float& yawOut)
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
	//std::cout<<rReturn<<" "<<pReturn<<" "<<yReturn<<" "<<std::endl;
	//std::cout<<"runningPID"<<std::endl;
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

//SPECIALTY FUNCTIONS (called not in a loop)

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
	//std::cout<<"f"<<std::endl;
	pitchTarget = input.y;
	rollTarget = input.x;
	yawTarget = input.z;
	//std::cout<<"hi"<<std::endl;
}

void RotPIDController::setInputs(sensor_msgs::Imu input)
{
  	EulerAngles angles = ToEulerAngles(input.orientation);
  	pitchVal = angles.pitch;
  	yawVal = angles.yaw;
  	rollVal = angles.roll;
	//std::cout<<pitchVal<<" "<< yawVal<< " " << rollVal <<std::endl;
}

//has become obsolete, not being used
void RotPIDController::startRosLoop(int argc, char** argv/*, ros::Subscriber*&subTarget, ros::Subscriber*&subSensor*/)
{
	//ros::init(argc, argv, "pidLoop");
	//ros::NodeHandle n;
	//previous code
			//move subscriber things to stability_node
	//ros::Subscriber target = n.subscribe("target_topic", 50, &RotPIDController::setTargets, this);
	//ros::Subscriber sensor = n.subscribe("aquadrone_v2/out/imu", 50, &RotPIDController::setInputs, this);
	//experimental loop stuff
	//subTarget = &target;
	//subSensor = &sensor;
	//return &n;

	//new loop code
	//target = node.subscribe("target_topic", 50, &RotPIDController::setTargets, this);
	//sensor = node.subscribe("aquadrone_v2/out/imu", 50, &RotPIDController::setInputs, this);
	/*
	ros::Rate r(5);
	while(ros::ok())
	{
		runPID(rollVal, pitchVal, yawVal);
		r.sleep();
	}
	*/
}

void RotPIDController::runSubs(int argc, char** argv)
{
	ros::init(argc, argv, "pidLoop");
	ros::NodeHandle n;
	target = n.subscribe("target_topic", 50, &RotPIDController::setTargets, this);
	sensor = n.subscribe("/aquadrone_v2/out/imu", 50, &RotPIDController::setInputs, this);	
	ros::spinOnce();
}


//copied from wikipedia
RotPIDController::EulerAngles RotPIDController::ToEulerAngles(geometry_msgs::Quaternion q)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}
