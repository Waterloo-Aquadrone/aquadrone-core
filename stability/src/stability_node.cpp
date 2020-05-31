#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <cmath>

#include "AxialPid.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stability");

  ros::NodeHandle n;


  RotPIDController rotCtrl;

  double Kp_def = 0.1;
  double Ki_def = 0.00001;
  double Kd_def = 1;
  double limit_def = 1;

  double Kp_p; n.param("/stability/pitch/Kp", Kp_p,  Kp_def);
  double Ki_p; n.param("/stability/pitch/Ki", Ki_p,  Ki_def);
  double Kd_p; n.param("/stability/pitch/Kd", Kd_p,  Kd_def);
  double limit_p; n.param("/stability/pitch/limit", limit_p, limit_def);

  double Kp_r; n.param("/stability/roll/Kp", Kp_r, Kp_def);
  double Ki_r; n.param("/stability/roll/Ki", Ki_r, Ki_def);
  double Kd_r; n.param("/stability/roll/Kd", Kd_r, Kd_def);
  double limit_r; n.param("/stability/roll/limit", limit_r, limit_def);

  double Kp_y; n.param("/stability/yaw/Kp", Kp_y, Kp_def);
  double Ki_y; n.param("/stability/yaw/Ki", Ki_y, Ki_def);
  double Kd_y; n.param("/stability/yaw/Kd", Kd_y, Kd_def);
  double limit_y; n.param("/stability/yaw/limit", limit_y, limit_def);

  rotCtrl.setPitchPID(Kp_p, Ki_p, Kd_p, limit_p);
  rotCtrl.setRollPID(Kp_r, Ki_r, Kd_r, limit_r);
  rotCtrl.setYawPID(Kp_y, Ki_y, Kd_y, limit_y);

  ROS_INFO_STREAM("Starting stability node");
  ROS_INFO_STREAM("Kp's (r,p,y):" << Kp_r << ", " << Kp_p << ", " << Kp_y);

  

  ros::Publisher motorPub = n.advertise<geometry_msgs::Wrench>("motorStability", 5);
  ros::Subscriber target = n.subscribe("orientation_target", 5, &RotPIDController::setTargets, &rotCtrl);
  ros::Subscriber sensor = n.subscribe("state_estimation", 5, &RotPIDController::setInputs, &rotCtrl);
  
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
        //std::cout<<returnVal.torque<<std::endl;
        
        motorPub.publish(returnVal);
        ros::spinOnce();
		r.sleep();
	}

  return 0;
}


