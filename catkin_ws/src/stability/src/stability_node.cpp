#include </opt/ros/kinetic/include/ros/ros.h>
#include </opt/ros/kinetic/include/geometry_msgs/Wrench.h>
#include </opt/ros/kinetic/include/sensor_msgs/Imu.h>
#include </opt/ros/kinetic/include/std_msgs/Float64.h>
#include <cmath>

#include "MiniPID.h"
#include "AxialPid.h"
#include "DepthPid.h"

/*
void stabilityMotorLoop(sensor_msgs::Imu input)
{
  EulerAngles angles = ToEulerAngles(input.orientation);
  double pitch = angles.pitch;
  double yaw = angles.yaw;
  double roll = angles.roll;
  //giving the data to rotctrl
  //rotCtrl.setPitchTarget(pitchSet.data);
  //rotCtrl.setRollTarget(rollSet.data);
  //rotCtrl.setYawTarget(yawSet.data);
  float pMotor = 0;
  float yMotor = 0;
  float rMotor = 0;

  rotCtrl.getMotorValues(roll, pitch, yaw, rMotor, pMotor, yMotor);
  //making the publisher
  ros::NodeHandle n;

  ros::Publisher motorPub = n.advertise<geometry_msgs::Vector3>("motorStability", 100);

  geometry_msgs::Vector3 returnVal;
  returnVal.x = rMotor;
  returnVal.y = pMotor;
  returnVal.z = yMotor;

  motorPub.publish(returnVal);
  ros::spinOnce();
}
*/

int main(int argc, char **argv)
{
    RotPIDController rotCtrl;
    rotCtrl.setPitchPID(1,1,1);
    rotCtrl.setRollPID(1,1,1);
    rotCtrl.setYawPID(1,1,1);
    //experimental loop stuff
    //ros::Subscriber* subSensor;
    //ros::Subscriber* subTarget;

    //startRosLoop has become sort of obselete
    //ros::NodeHandle* subs = rotCtrl.startRosLoop(argc, argv);

  std::cout<<"b"<<std::endl;

  ros::init(argc, argv, "stability");

  ros::NodeHandle n;

  ros::Publisher motorPub = n.advertise<geometry_msgs::Wrench>("motorStability", 100);
  //ros::Subscriber target = n.subscribe("target_topic", 50, &RotPIDController::setTargets, &rotCtrl);
ros::Subscriber sensor = n.subscribe("aquadrone_v2/out/imu", 50, &RotPIDController::setInputs, &rotCtrl);
  std::cout<<"c"<<std::endl;
  

  ros::Rate r(5);
  while(ros::ok())
	{
        float pMotor = 0;
        float yMotor = 0;
        float rMotor = 0;
        //rotCtrl.runSubs(argc, argv); //new looping subscriber code
        geometry_msgs::Vector3 target;
        target.x = 0;
        target.y = 0;
        target.z = 0;
        rotCtrl.setTargets(target);
		    rotCtrl.runPID(pMotor, rMotor, yMotor);
        //std::cout<<pMotor<<" "<<rMotor<<" "<<yMotor<<" "<<std::endl;
        geometry_msgs::Wrench returnVal;

        returnVal.torque.x = double(rMotor);
        returnVal.torque.y = double(pMotor);
        returnVal.torque.z = double(yMotor);
        std::cout<<returnVal.torque<<std::endl;
        //experimental loop stuff
        //ros::Subscriber target = *subTarget;
        //ros::Subscriber sensor = *subSensor;
        //subscribers
        
        //ros::Subscriber target = n.subscribe("target_topic", 50, &RotPIDController::setTargets, &rotCtrl);
        
        motorPub.publish(returnVal);
        ros::spinOnce();
		r.sleep();
	}

  return 0;
}

/*
//copied from wikipedia
struct EulerAngles
{
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(geometry_msgs::Quaternion q)
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
*/
