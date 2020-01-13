#!/usr/bin/env python

import rospy
from aquadrone_msgs.msg import AquadroneIMU
from sensor_msgs.msg import Imu
from Adafruit_BNO005 import BNO005

#print(dir(BNO005))
bno = BNO005.BNO055()

rospy.init_node('aquadrone_IMU')
pubUW = rospy.Publisher('/aquadrone/out/SensorUW', AquadroneIMU)
pubIMU = rospy.Publisher('/aquadrone/out/imu', Imu)
rate = rospy.Rate(50)  # The BNO005 outputs all IMU data at 100Hz


while not rospy.is_shutdown():
  msgUW = AquadroneIMU()
  msgUW.euler = bno.read_euler()
  msgUW.gyroscope = bno.read_gyroscope()
  msgUW.accelerometer = bno.read_accelerometer()
  msgUW.linear_acceleration = bno.read_linear_acceleration()
  msgUW.quaternion = bno.read_quaternion()

  msgIMU = Imu()
  # TODO: add variance data based on BNO005 datasheet
  msgIMU.orientation = bno.read_quaternion()
  msgIMU.angular_velocity = bno.read_gyroscope()
  msgIMU.linear_acceleration = bno.read_linear_acceleration()

  pubUW.publish(msgUW)
  pubIMU.publish(msgIMU)
  rate.sleep()
