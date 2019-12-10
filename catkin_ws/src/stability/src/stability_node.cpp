#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <cmath>

#include "AxialPid.h"

int main(int argc, char **argv)
{
    RotPIDController rotCtrl;

    double Kp = 0.1;
    double Ki = 0.00001;
    double Kd = 1;
    rotCtrl.setPitchPID(Kp, Ki, Kd);
    rotCtrl.setRollPID(Kp, Ki, Kd);
    rotCtrl.setYawPID(Kp*5, Ki, Kd);

  std::cout<<"b"<<std::endl;

  ros::init(argc, argv, "stability");

  ros::NodeHandle n;

  ros::Publisher motorPub = n.advertise<geometry_msgs::Wrench>("motorStability", 5);
  ros::Subscriber target = n.subscribe("orientation_target", 5, &RotPIDController::setTargets, &rotCtrl);
  ros::Subscriber sensor = n.subscribe("aquadrone/out/imu", 5, &RotPIDController::setInputs, &rotCtrl);
  std::cout<<"c"<<std::endl;
  
  geometry_msgs::Vector3 or_target;
  or_target.x = 0.0;
  or_target.y = 0.0;
  or_target.z = 0.0;
  rotCtrl.setTargets(or_target);

  ros::Rate r(10);
  while(ros::ok())
	{
        float pMotor = 0;
        float yMotor = 0;
        float rMotor = 0;

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


