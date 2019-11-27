#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Float32

rospy.init_node('fake_news')

pub = rospy.Publisher('/aquadrone/fake/out', Float32, queue_size=1)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    msg = Float32(0.5 + random.random() * 0.2)
    pub.publish(msg)
    rate.sleep()
    
