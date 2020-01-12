#!/usr/bin/env python

import rospy
from sensor_msgs.msg import FluidPressure
from ms5837_python import ms5837

#-------
port = 1
#-------

pub = rospy.Publisher('/aquadrone/out/pressure', FluidPressure, queue_size=1)
rospy.init_node('depth_sensor')
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    sensor = ms5837.MS5837_30BA(port)

    if not sensor.init():
        rospy.core.logerr("unable to init ms5837, depth sensor")
    else:
        try:
            while not rospy.is_shutdown():
                if sensor.read():
                    msg = FluidPressure()
                    msg.fluid_pressure = sensor.pressure()# mbar
                    pub.publish(msg)
                else:
                    rospy.core.logwarn("unable to read ms5837, depth sensor")
                rate.sleep()
        except IOError as ioerror:
            print(ioerror)

    rate.sleep()


