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
            raise ValueError('message not found: %s' % (topic,))
    
    def create_subscriber(self, callback_fnc, callback_fnc_args=None):
        return rospy.Subscriber(self.topic, self.Msg, callback=callback_fnc, callback_args=callback_fnc_args)

    def create_publisher(self):
        return rospy.Publisher(self.topic, self.Msg)

    def extract_data(self, msg):
        return getattr(msg, self.msg_attr)

    def insert_data(self, msg, value):
        return setattr(msg, self.msg_attr, value)


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
    
    def __str__(self):
        text = ""
        for i in range(len(self.x_values)):
            text += "(%f,%f) " % (self.x_values[i], self.y_values[i])
        return text


class DataCollector:
    def __init__(self, timeout, commander, observers):
        self.timeout = timeout
        self.obsrs = observers# list of Observers
        self.cmdr =commander# single Commander
        self.cmdr.duration = timeout

    def run_test(self, t0='on call'):
        print('=======TEST=======')
        if self.cmdr is None:
            print('[CONTROL] None')
        else:
            print('[CONTROL]' + self.cmdr.msgf.topic)
        for obs in self.obsrs:
            print('[OBSERVING]' + obs.msgf.topic)
        
        rate = rospy.Rate(2)
        
        if t0 == 'on call':
            t0 = rospy.rostime.get_time()
        
        print('---BEGINING TEST')
            
        for obs in self.obsrs:
            obs.t0 = t0
            obs.active = True

        self.cmdr.t0 = t0
        self.cmdr.pub_commands()
        '''
        while not rospy.is_shutdown() and rospy.rostime.get_time() < t2:
            print('[time]' + str(rospy.rostime.get_time()) + 's')
            rate.sleep()'''
        
        for obs in self.obsrs:
             obs.active = False

        print('---ENDING TEST')
        
        print('plotting results...')
        self.plot()
        
        
    def plot(self):
        colours = ['g','r','c','m' ,'y']
        for obs in self.obsrs:
            series = obs.data_series
            if len(colours) == 0:
                colour = 'k'
            else:
                colour = colours.pop(0)
            plot.plot(series.x_values, series.y_values, '.-' + colour, label='OBS:'+series.ylabel)
        series = self.cmdr.data_series
        plot.step(series.x_values, series.y_values, 'o--b', where='post', label='CTRL:'+series.ylabel)
        plot.xlabel(series.xlabel)
        plot.legend(title="LEGEND", fontsize='x-small')
        plot.show()


#gets data from subscriber
class Observer:
    def __init__(self, topic, msg_attr):
        self.data_series = DataSeries('time', topic)
       
        self.msgf = MessageFinder(topic, msg_attr)

        if self.msgf.Msg == None:
            print("cannot observe unpublished topic: '" + topic + "'")
            self.sub = None
        else:
            self.sub = self.msgf.create_subscriber(self.read)
            self.active = False

        
    def read(self, data):
        if not self.active:
            return
        if self.t0 is None:
            print("t0 is None")
            self.t0 = 0

        if self.data_series.xlabel == 'time':
            x = rospy.rostime.get_time() - self.t0
        else:
            x = rospy.rostime.get_time() - self.t0# because there is no other option for independent variables      
        
        self.data_series += [x, self.msgf.extract_data(data)]


class LinearControl:
    '''
    y2 |          *
       |
    y0 |*
     0 ---------------
       0
       |<-------->|
           deltax
    '''
    def __init__(self, duration, y0, y1=None):
        self.y0 = y0
        if y1 is None:
            self.y1 = y0#automatically a constant value
        else:
            self.y1 = y1
        self.duration = duration
        #dt is delta time or duration

    def interplorate(self, x):
        return (self.y1 - self.y0) / self.duration * x + self.y0



#publishes control signals
class Commander:
    def __init__(self, topic, msg_attr, duration, y0, y1=None, update_rate=0):
        self.msgf = MessageFinder(topic, msg_attr)
        self.pub = rospy.Publisher(topic, self.msgf.Msg, queue_size=10)
        self.data_series = DataSeries('time', topic)
        self.linCtrl = LinearControl(duration, y0, y1)
        self.rate = update_rate

    def pub_commands(self):
        if self.t0 is None:
            print("t0 is None")
            self.t0 = 0

        rospy_rate = rospy.Rate(10)
        if self.rate > 0:
            rospy_rate = rospy.Rate(self.rate)            

        t2 = self.linCtrl.duration + self.t0

        while not rospy.is_shutdown() and rospy.rostime.get_time() < t2:
            msg = self.msgf.Msg()
            self.msgf.insert_data(msg, self.create_control_data()[1])
            self.pub.publish(msg)

            if self.rate > 0:
                rospy_rate.sleep()
            else:
                while not rospy.is_shutdown() and rospy.rostime.get_time() < t2:
                    rospy_rate.sleep()
        
        data = self.create_control_data()#adding a point at the end allows plot to interplorate across test range
        print('@%s:{:.2f} %s<<{:.2f}'.format(data[0], data[1]) % (self.data_series.xlabel, self.data_series.ylabel))
    
    def create_control_data(self):
        data = [None,None]
        data[0] = rospy.rostime.get_time() - self.t0
        data[1] = self.linCtrl.interplorate(data[0])
        self.data_series += data
        return data


rospy.init_node('test_controller')

print("==============")
for x, y in rospy.get_published_topics():
    print(x, y)
print("==============")

from std_msgs.msg import Float32
pubx = rospy.Publisher('/aquadrone/fake/in', Float32, queue_size=1)
#pubx.publish(Float32(0.6))
rospy.Rate(10).sleep()

#MessageFinder test code
msg = MessageFinder('/rosout', '')
if not repr(msg.Msg) == "<class 'rosgraph_msgs.msg._Log.Log'>":
    print('PROBLEM:', msg.Msg, "!= <class 'rosgraph_msgs.msg._Log.Log'>")

obs = Observer('/aquadrone/fake/out', 'data')

cmdr = Commander('/aquadrone/fake/in', 'data', 5, y0=10,y1=20, update_rate=1)

clt = DataCollector(1, cmdr, [obs,])

clt.run_test()

# main
'''
obs = Observer('aquadrone/sensors/out/pressure', 'fluid_pressure')
com = Commander('')

collect = DataCollector()

collect.run_test()
'''

    

