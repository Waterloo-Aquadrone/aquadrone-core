#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <cmath>

#include "AxialPid.h"

int main(int argc, char **argv)
{
    RotPIDController rotCtrl;
    rotCtrl.setPitchPID(0.1,0.0001,5);
    rotCtrl.setRollPID(0.01,0.0001,5);
    rotCtrl.setYawPID(0.1,0.0001,5);

  std::cout<<"b"<<std::endl;

  ros::init(argc, argv, "stability");

  ros::NodeHandle n;

  ros::Publisher motorPub = n.advertise<geometry_msgs::Wrench>("motorStability", 100);
  ros::Subscriber target = n.subscribe("orientation_Target", 50, &RotPIDController::setTargets, &rotCtrl);
  ros::Subscriber sensor = n.subscribe("aquadrone/out/imu", 50, &RotPIDController::setInputs, &rotCtrl);
  std::cout<<"c"<<std::endl;
  

  ros::Rate r(10);
  while(ros::ok())
	{
        float pMotor = 0;
        float yMotor = 0;
        float rMotor = 0;

        geometry_msgs::Vector3 target;
        /*
        target.x = 0.5;
        target.y = 0.5;
        target.z = 0.5;
        rotCtrl.setTargets(target);
        */
		    rotCtrl.runPID(pMotor, rMotor, yMotor);

        geometry_msgs::Wrench returnVal;

        returnVal.torque.x = double(rMotor);
        returnVal.torque.y = double(pMotor);
        returnVal.torque.z = double(yMotor);
        std::cout<<returnVal.torque<<std::endl;
        
        motorPub.publish(returnVal);
        ros::spinOnce();
		r.sleep();
	}

  return 0;
}


