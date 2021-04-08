#!/usr/bin/env python3.8

import rospy
import sys
import smbus
from random import random
from sensor_msgs.msg import BatteryState
from MCP342x import MCP342x


RESOLUTION = 18
NUM_CELLS = 4
R1 = [10.0e3, 47.0e3, 86.6e3, 127.0e3]
R2 = 28.7e3

VOLTAGE_100 = 4.0
VOLTAGE_0 = 3.2


class MCP342x_Test:
    def __init__(self, addr, channel, resolution, scale_factor):
        self.ident = (addr, channel, resolution)
        self.scale = scale_factor

    def configure(self):
        print(str(rospy.get_time()) + " " + str(self.ident) + " MCP3424-SIM CONFIGURED")
        
    def convert_and_read(self):
        voltage = 14.8 / 4 * (self.ident[1] + 1) * 0.75 + (random() - 0.5) * 0.4
        print(str(rospy.get_time()) + " " + str(self.ident) + " " + "{:.2f}".format(voltage) + " MCP3424-SIM CONVERT_AND_READ")
        return voltage


class BatteryPack:
    def __init__(self, bus, addr, topic):
        self.pub = rospy.Publisher(topic, BatteryState, queue_size=10)
        self.volts_raw = [0.0] * NUM_CELLS
        self.volt_sensors = []
        for j in range(4):
            if bus is None:
                adr_sensor = MCP342x_Test(addr, j, RESOLUTION, R2 / (R1[j]+R2) )
            else:
                adr_sensor = MCP342x(bus, addr, j, resolution=RESOLUTION, scale_factor= R2 / (R1[j]+R2) )
            self.volt_sensors.append(adr_sensor)
        
        for s in self.volt_sensors:
            s.configure()
        
    def read(self):
        self.volts_raw = [s.convert_and_read() for s in self.volt_sensors]
        
    def voltage(self):
        try:
            return self.volts_raw[3]
        except IndexError:
            return 0.0
    
    def cell_voltages(self):
        result = []
        ref = 0
        for v in self.volts_raw:
            result.append(v - ref)
            ref = v
        return result
    
    def publish(self):
        msg = BatteryState()
    
        msg.voltage = self.voltage()
        msg.cell_voltage = self.cell_voltages()
        msg.percentage = (msg.voltage - VOLTAGE_0 * NUM_CELLS) / ((VOLTAGE_100 - VOLTAGE_0) * NUM_CELLS) # (CURR - MIN) / RANGE
        msg.percentage = max(min(msg.percentage, 1.0), 0.0) # Clamp to range [0, 1]
        
        self.pub.publish(msg)
    
    def read_and_publish(self):
        self.read()
        self.publish()
        

if __name__ == '__main__':
    #http://docs.ros.org/en/api/sensor_msgs/html/msg/BatteryState.html
    rospy.init_node("battery_sensor")

    try:
        bus = smbus.SMBus(1)
    except IOError as e:
        rospy.core.logerr("unable to init MCP3424, battery voltage sensor")
        bus = None

    pack1 = BatteryPack(bus, 110, 'battery_1_state')
    pack2 = BatteryPack(bus, 101, 'battery_2_state')

    if isinstance(pack1.volt_sensors[0], MCP342x_Test) or isinstance(pack2.volt_sensors[0], MCP342x_Test):
        rospy.core.logwarn("MCP3424, battery voltage sensors in test mode")

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        
        pack1.read_and_publish()
        pack2.read_and_publish()
        
        rate.sleep()
