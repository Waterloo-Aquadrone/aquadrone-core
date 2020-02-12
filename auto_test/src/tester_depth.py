#!/usr/bin/env python

# - - - IMPORTS:
# Basic:
import matplotlib.pyplot as plot
import sys
import rospy
from math import *
# Specific:
from std_msgs import *
# - - - IMPORTS.

# CORE VARIABLES:
TIME_START = None
SUB = {}  # Subscribers
# CORE VARIABLES.


def input_data(data_series, data_name, data):
    if 'time' in DATA[data_series].keys():
        DATA[data_series]['time'].append(rospy.get_time() - TIME_START)
    DATA[data_series][data_name].append(data)

def observe(topic, msg_type, data_series, data_name, data_from_msg_fnc)
    SUB[data_series] = rospy.Subsriber(topic, msg_type, lambda msg: input_data(data_series, data_name, data_from_msg_fnc(msg)))


# - - - TEST SPECIFICATIONS:
# Data Series:
DATA = {
    'depth:tf': {'time': [], 'metres': [] },
    'depth:ctrl': {'time': [], 'metres': [] },
    }
# Oberservations:
observe('/aquadrone/fake/news', Float32, 'depth:tf', 'metres', lambda msg: abs(msg.pose[1].position.z))
# Publishers:
PUB = {
    'depth:ctrl': rospy.Publisher('/aquadrone/no/ctrl', Float32),
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

    

