#!/usr/bin/env python

# - - - IMPORTS:
# Basic:
import matplotlib.pyplot as plot
import sys
import rospy
# Specific:
from std_msgs import *
# - - - IMPORTS.

# CORE VARIABLES:
TIME_START = None

# CORE VARIABLES.


def observe(data_series, data_name):
    if 'time' in DATA[data_series].keys():
        DATA[data_series]['time'].append(rospy.get_time() - TIME_START)
    DATA[data_series][data_name].append(msg.pose[1].position.z)


# - - - TEST SPECIFICATIONS:
# Data Series:
DATA = {
    'depth:tf': {'time': [], 'metres': [] },
    'depth:ctrl': {'time': [], 'metres': [] },
    }
# Subscribers:
SUB = {
    'depth:tf': rospy.Subsriber('', Float32, lambda msg: observe(data_series)),
    }
# Publishers:
PUB = {
    'depth:ctrl': rospy.Publisher('', Float32),
    }
# - - - TEST SPECIFICATIONS.


rospy.init_node('test_controller') #initialize node

print('\n')

rospy.sleep(2)  # delay for startup (specifically gazebo)

START_TIME = rospy.get_time()  # t = 0


# - - - MAIN:



print('plotting results...')
clt.plot()

print("Ready to exit...")

# - - - MAIN.

    

