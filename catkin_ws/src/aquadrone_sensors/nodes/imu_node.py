#!/usr/bin/env python

import rospy
from aquadrone_msgs.msg import AquadroneIMU
from sensor_msgs.msg import Imu
from Adafruit_BNO005 import BNO005

#print(dir(BNO005))
bno = BNO005.BNO055()

rospy.init_node('aquadrone_IMU')
pubUW = rospy.Publisher('/aquadrone_v2/out/SensorUW', aquadroneIMU)
pubIMU = rospy.Publisher('/aquadrone_v2/out/imu', Imu)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
  msgUW = aquadroneIMU()
  msgUW.euler = bno.read_euler()
  msgUW.gyroscope = bno.read_gyroscope()
  msgUW.accelerometer = bno.read_accelerometer()
  msgUW.linear_acceleration = bno.read_linear_acceleration()
  msgUW.quaternion = bno.read_quaternion()
  msgIMU = Imu()
  msgIMU.orientation = bno.read_quaternion()
  msgIMU.angular_velocity = bno.read_gyroscope()
  msgIMU.linear_acceleration = bno.read_linear_acceleration()
  pubUW.publish(msgUW)
  pubIMU.publish(msgIMU)
  rate.sleep()
Col