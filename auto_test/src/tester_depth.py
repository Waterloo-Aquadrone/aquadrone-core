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

def observe(topic, msg_type, data_series, data_name, data_from_msg_fnc):
    SUB[data_series] = rospy.Subsriber(topic, msg_type, lambda msg: input_data(data_series, data_name, data_from_msg_fnc(msg)))

def publish(publiser_name, msg):
    PUB[publiser_name].publish(msg)


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

# ARGUMENTS:
ARGS = {}
for arg in sys.argv[1:]:
    sep = arg.partition(":=")
    ARGS[sep[0]] = sep[2]
# Excepted Arguments: duration

rospy.init_node('test_controller') #initialize node

print('\n')

rospy.sleep(2)  # delay for startup (specifically gazebo)


# - - - MAIN:


#TODO fixed control signal interval

print('\n=======TEST=======')

for key, sub in SUB.items():
    print('[OBSERVING] ' + sub.topic)

for key, pub in PUB.items():
    print('[CONTROL] ' + pub.topic)

START_TIME = rospy.get_time()  # t = 0

print('---BEGINING TEST\n')

duration = ARGS['duration']

rate = rospy.Rate(10)
while not rospy.is_shutdown() and rospy.rostime.get_time() < t0 + duration:  # TEST CONTROL LOOP

    publish('depth:ctrl', Float32(10.2))

    rate.sleep()

print('\n---ENDING TEST')

print('plotting results...')

colours = ['g','r','c','m' ,'y']
for obs in self.obsrs:
    series = obs.data_series
    # allocates colors to each series
    if len(colours) == 0:
        colour = 'k'
    else:
        colour = colours.pop(0)
    plot.plot(series.x_values, series.y_values, '.-' + colour, label='OBS:'+series.ylabel)

if self.cmdr is not None:
    series = self.cmdr.data_series
    plot.step(series.x_values, series.y_values, 'o--b', where='post', label='CTRL:'+series.ylabel)
    plot.xlabel(series.xlabel)

plot.legend(title="LEGEND", fontsize='x-small')
plot.show()

print("Ready to exit...")

# - - - MAIN.

    

