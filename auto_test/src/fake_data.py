#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Float32

rospy.init_node('fake_news')

pub = rospy.Publisher('/aquadrone/fake/out', Float32, queue_size=1)

rate = rospy.Rate(10)

value = 0.5

def fake_input(msg):
    print("RECIEVED @%s" % rospy.rostime.get_time())
    global value
    value = msg.data
    print("   value: " + str(value))

sub = rospy.Subscriber('/aquadrone/fake/in', Float32, callback=fake_input)

print('ONLINE')

while not rospy.is_shutdown():
    msg = Float32(value + (random.random()-.5) * .4)
    pub.publish(msg)
    rate.sleep()
    
