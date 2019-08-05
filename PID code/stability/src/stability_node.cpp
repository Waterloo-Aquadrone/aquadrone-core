#include </opt/ros/kinetic/include/ros/ros.h>
#include </opt/ros/kinetic/include/geometry_msgs/Wrench.h>
#include </opt/ros/kinetic/include/sensor_msgs/Imu.h>
#include </opt/ros/kinetic/include/std_msgs/Float64.h>
#include <cmath>

#include <MiniPID.h>
#include <AxialPid.h>
#include <DepthPid.h>

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
    rotCtrl.startRosLoop(argc, argv);

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "stability");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  ros::Publisher motorPub = n.advertise<geometry_msgs::Wrench>("motorStability", 100);

  ros::Rate r(5);
    while(ros::ok())
	{
        float pMotor = 0;
        float yMotor = 0;
        float rMotor = 0;
		rotCtrl.runPID(pMotor, rMotor, yMotor);

        geometry_msgs::Wrench returnVal;
        returnVal.torque.x = rMotor;
        returnVal.torque.y = pMotor;
        returnVal.torque.z = yMotor;

        motorPub.publish(returnVal);

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