#!/usr/bin/env python

import matplotlib.pyplot as plot
import sys
import rospy


def access(access_point, path):
    attr = getattr(access_point, path[0])
    if len(path) > 1:
        return access(attr, path[1:])
    else:
        return attr


class MessageFinder:
    def __init__(self, topic, msg_attr=None):
        self.topic = topic
        self.Msg = None
        self.type = None
        self.msg_attr = msg_attr

        topics_list = rospy.get_published_topics()
        for name, typ in topics_list:
            if topic == name:
                self.type = typ
        
        if not self.type == None:

            type_path = self.type.split('/')
            type_path.insert(-1,'msg')

            msg_class_path = reduce(lambda x,y: x+'.'+y, type_path[:-1])

            exec("import " + msg_class_path)
            print("imported " + msg_class_path)

            msg_module = sys.modules[type_path[0]]
            self.Msg = access(msg_module, type_path[1:])
        else:
            print('message not found')
    
    def create_subscriber(self, callback_fnc, callback_fnc_args=None):
        return rospy.Subscriber(self.topic, self.Msg, callback=callback_fnc, callback_args=callback_fnc_args)

    def create_publisher(self):
        return rospy.Publisher(self.topic, self.Msg)

    def extract_data(self, msg):
        return getattr(msg, self.msg_attr)


class DataSeries:
    def __init__(self, x_axis_name='x', y_axis_name='y'):
        self.x_values = []
        self.y_values = []
        self.xlabel = x_axis_name
        self.ylabel = y_axis_name
       
    def __iadd__(self, data):
        self.x_values.append(data[0])
        self.y_values.append(data[1])
        return self


class DataCollector:
    def __init__(self, timeout, commander, obersevers):
        self.timeout = timeout
        self.obsrs = obersevers
        self.cmdr =commander

    def run_test(self):
        print('=======TEST=======')
        if self.cmdr is None:
            print('[CONTROL] None')
            print('\b#')
        else:
            print('[CONTROL]' + self.cmdr.msg.topic)
        for obs in self.obsrs:
            print('[OBSERVING]' + obs.msg.topic)
        
        rate = rospy.Rate(2)
        print('---BEGINING TEST')
        t2 = rospy.rostime.get_time() + self.timeout
            
        for obs in self.obsrs:
             obs.active = True
        
        while not rospy.is_shutdown() and rospy.rostime.get_time() < t2:
            print('[time]' + str(rospy.rostime.get_time()) + 's')
            rate.sleep()
        
        for obs in self.obsrs:
             obs.active = False
        print('---ENDING TEST')
        
        print('plotting results...')
        self.plot()
        
        
    def plot(self):
        series = self.obsrs[0].data_series
        plot.plot(series.x_values, series.y_values)
        plot.ylabel(series.ylabel)
        plot.xlabel(series.xlabel)
        plot.show()


#gets data from subscriber
class Observer:
    def __init__(self, topic, msg_attr):
        self.data_series = DataSeries('time', topic.split('/')[-1])
       
        self.msg = MessageFinder(topic, msg_attr)

        if self.msg.Msg == None:
            print("cannot observe unpublished topic: '" + topic + "'")
            self.sub = None
        else:
            self.sub = self.msg.create_subscriber(self.read)
            self.active = False

        
    def read(self, data):
        if not self.active:
            return
        if self.data_series.xlabel == 'time':
            x = rospy.rostime.get_time()
        else:
            x = rospy.rostime.get_time()# because there is no other option for independent variables
        self.data_series += [x, self.msg.extract_data(data)]


class LinearControl:
    '''
    y2 |          *
       |
    y0 |*
     0 ---------------
       0
       |<-------->\
           deltax
    '''
    def __init__(self, y0, y1, deltax, rate):
        self.y0 = y0
        self.y1 = y1
        self.deltax = deltax
        self.rate = rospy.Rate(rate)

    def interplorate(self, x):
        return (y1 - y0) / deltax * x + y0



#publishes control signals
class Commander:
    def __init__(self, topic, msg_attr):
        self.pub = MessageFinder(topic, )
        self.reportto = data_series
        self.data_series = DataSeries()

    def pub_commands():
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg =
            
            self.pub.publish(msg)            
            rate.sleep()


rospy.init_node('test_controller')

print("==============")
for x, y in rospy.get_published_topics():
    print(x, y)
print("==============")

#MessageFinder test code
msg = MessageFinder('/rosout', '')
if not repr(msg.Msg) == "<class 'rosgraph_msgs.msg._Log.Log'>":
    print('PROBLEM:', msg.Msg, "!= <class 'rosgraph_msgs.msg._Log.Log'>")

obs = Observer('/aquadrone/fake/out', 'data')

clt = DataCollector(5, None, [obs,])

clt.run_test()

# main
'''
obs = Observer('aquadrone/sensors/out/pressure', 'fluid_pressure')
com = Commander('')

collect = DataCollector()

collect.run_test()
'''

    

