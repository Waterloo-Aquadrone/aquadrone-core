#!/usr/bin/env python

# - - - IMPORTS:
# Basic:
import matplotlib.pyplot as plot
import sys
import rospy
from math import *
# Specific:
from std_msgs.msg import *
# - - - IMPORTS.

# CORE VARIABLES:
TIME_START = None
SUB = {}  # Subscribers
# CORE VARIABLES.


def input_data(data_series, data_name, data):
    if TIME_START is None:
        # if test has not started do not accept data
        return
    if '_time' in DATA[data_series].keys():
        DATA[data_series]['_time'].append(rospy.get_time() - TIME_START)
    DATA[data_series][data_name].append(data)

def observe(topic, msg_type, data_series, data_name, data_from_msg_fnc):
    SUB[data_series] = rospy.Subscriber(topic, msg_type, lambda msg: input_data(data_series, data_name, data_from_msg_fnc(msg)))

def publish(publiser_name, msg):
    PUB[publiser_name].publish(msg)

def group_data(data_dict):
    ind = None
    dep = []
    control = []
    for series, values in data_dict.items():
        if series.startswith('_'):
            ind = series.strip('_'), values
        elif series.endswith('ctrl'):
            control.append((series, values))
        else:
            dep.append((series, values))
    return ind, dep, control

# - - - TEST SPECIFICATIONS:
# Data Series:
DATA = {
    'depth:tf': {'_time': [], 'metres': [] },
    'depth:ctrl': {'_time': [], 'metres': [] },
    }
# Oberservations:
observe('/aquadrone/fake/out', Float32, 'depth:tf', 'metres', lambda msg: abs(msg.data))
# Publishers:
PUB = {
    'depth:ctrl': rospy.Publisher('/aquadrone/no/ctrl', Float32, queue_size=10),
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


TIME_START = rospy.get_time()  # t = 0

print('---BEGINING TEST\n')

duration = float(ARGS['duration'])

rate = rospy.Rate(10)
while not rospy.is_shutdown() and rospy.rostime.get_time() < TIME_START + duration:  # TEST CONTROL LOOP

    publish('depth:ctrl', Float32(10.2))

    rate.sleep()

print('\n---ENDING TEST')

#print(SUB)
#print(DATA)
#print(PUB)

print('plotting results...')

colours = ['g','r','c','m' ,'y']
for name, data in DATA.items():
    ind, dep, control = group_data(data)
    for series, values in dep:
        # allocates colors to each series
        if len(colours) == 0:
            colour = 'k'
        else:
            colour = colours.pop(0)
            
        plot.plot(ind[1], values, '.-' + colour, label=series)
   
for series, values in control:
    # allocates colors to each series
    if len(colours) == 0:
        colour = 'k'
    else:
        colour = colours.pop(0)
        
    plot.plot(ind[1], values, '.-' + colour, label=series)
    plot.step(ind[1], values, 'o--b', where='post', label='CTRL:'+series)

#plot.xlabel(series.xlabel)


plot.legend(title="LEGEND", fontsize='x-small')
plot.show()

print("Ready to exit...")

# - - - MAIN.

    

