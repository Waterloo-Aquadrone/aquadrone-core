#!/usr/bin/env python
#roslaunch auto_test basic_tester.launch testctl:=tester_depth duration:=20

# - - - IMPORTS:
# Basic:
import matplotlib.pyplot as plot
import sys
import rospy
import sched
from numpy import interp
# Specific:
from std_msgs.msg import *
from gazebo_msgs.msg import *
# - - - IMPORTS.

# CORE VARIABLES:
TIME_START = None
DURATION = 10

ARGS = {}

DATA = {}
# DATA[<data_set>][<variable>]
# {'data_set': {'variable': [], ...}, ...}
# e.g.: DATA['depth:ctrl']['_time'].append(rospy.get_time() - TIME_START)
# '_' at the begining indicates that the variable is independent

SUB = {}  # Subscribers
PUB = {} # Publishers
# CORE VARIABLES.

# SECONDARY VARIABLES:
COLOURS = ['g','r','c','m' ,'y']
# SECONDARY VARIABLES.


# Creates a data set(s) with variables in var_names.
def create_data(data_set_names, var_names):
    if isinstance(data_set_names, list):
        for data_set in data_set_names:
            create_data(data_set, var_names)
        return
    data = {}
    for var in var_names:
        data[var] = []
    DATA[data_set_names] = data


# Takes data and stores it in DATA. Indexed by data set and variable name.
# Automatically adds independant variable values such as time.
# e.g. append_value('depth', 'metres', 1.75) which also does append_value('depth', '_time', rospy.get_time() - TIME_START)
def append_value(data_set_name, var_name, value):
    if TIME_START is None:
        # if test has not started do not accept data
        return
    if '_time' is not var_name and '_time' in DATA[data_set_name].keys(): # automatically adds time value
        append_value(data_set_name, '_time', rospy.get_time() - TIME_START)
    try:
        DATA[data_set_name][var_name].append(value)
    except:
        create_data(data_set_name, [var_name,])
        DATA[data_set_name][var_name].append(value)


# Sets up a subscriber that simply observes data, dumping it into DATA using append_value.
def observe(data_set_name, var_name, topic, msg_type, value_from_msg_fnc):
    SUB[data_set_name] = rospy.Subscriber(topic, msg_type, lambda msg: append_value(data_set_name, var_name, value_from_msg_fnc(msg)))


# Publishes a control message & appends the appropriate data
def publish(data_set_name, var_name, msg, num_value):
    if data_set_name not in PUB:
        print("ERROR: PUB does not contain data set:" + data_set_name)
        return
    PUB[data_set_name].publish(msg)
    append_value(data_set_name, var_name, num_value)


# Plots observational data from DATA as a line of unique colour.
def plot_data(data_set_name, x_var_name, y_var_name):
    colour = 'k'if len(COLOURS) == 0 else COLOURS.pop(0)
    plot.plot(DATA[data_set_name][x_var_name], DATA[data_set_name][y_var_name], '.-' + colour, label=data_set_name)


# Plots observational data from DATA as a stepped line of unique colour.
def plot_control_data(data_set_name, x_var_name, y_var_name):
    colour = 'k'if len(COLOURS) == 0 else COLOURS.pop(0)
    plot.step(DATA[data_set_name][x_var_name], DATA[data_set_name][y_var_name], 'o--b', where='post', label=data_set_name)


# Encapsulation of the print function so it can be passed as a event action (function argument).
def fprint(text):
    print(text)


# - - - TEST SETUP:
# Data Sets:
depth_vars = ['_time', 'metres']
create_data('depth', depth_vars)
create_data('depth:ctrl', depth_vars)
# Oberservations:
#observe adds a subcriber to SUB[data_set_name] with the callback function that automatically appends the data passing it through the lambda function.
observe('depth', 'metres', '/gazebo/model_states', ModelStates, lambda msg: -msg.pose[1].position.z)
# Publishers:
PUB['depth:ctrl'] = rospy.Publisher('/depth_control/goal_depth', Float64, queue_size=10)
# - - - TEST SETUP.

# - - - ARGUMENTS:
for arg in sys.argv[1:]:
    sep = arg.partition(":=")
    ARGS[sep[0]] = sep[2]
# Excepted Arguments: duration

try:
    DURATION = float(ARGS['duration'])
except Exception:
    print("TEST: Defaulted to 10s duration.")
    DURATION = 10

# - - - ARGUMENTS.

# - - - MAIN:

rospy.init_node('test_controller') #initialize node

print('\n')
rospy.sleep(2)  # delay for startup
print('\n=======TEST=======')

ctl = sched.scheduler(rospy.get_time, rospy.sleep)

# CONTROL EVENTS:
t = 0
while t < DURATION:
    depth = 2 + .2 * t
    ctl.enter(t, 1, publish, ('depth:ctrl', 'metres', Float64(depth), depth))
    t += 1/.2
# CONTROL EVENTS.

# Completion report events
for i in range(1, 5):
    ctl.enter(i / 4.0 * DURATION, 1, fprint, ('-TEST: %d%% COMPLETE' % (i / 4.0 * 100),))

print('---TEST: BEGINING\n')

TIME_START = rospy.get_time()
ctl.run()
while rospy.get_time() < TIME_START + DURATION:
    rospy.sleep(.005)

print('\n---TEST: ENDED')

#print(SUB)
#print(DATA)
#print(PUB)

# - - - MAIN.

# - - - PLOTTING:

print('---TEST: PLOTTING RESULTS...')

# Plots:
plot_data('depth', '_time', 'metres')
plot_control_data('depth:ctrl', '_time', 'metres')

# Labels:
plot.xlabel('time (s)')
plot.ylabel('metres (m)')

# Configuration:
plot.legend(title="LEGEND", fontsize='x-small')

plot.show()

# - - - PLOTTING.

print("---TEST: READY TO EXIT...")



    

