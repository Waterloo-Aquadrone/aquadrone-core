import rospy
from aquadrone_pneumatics.msg import command, status
from aquadrone_pneumatics.srv import PneumaticsStatus
from threading import Timer
import sys, select, termios, tty
import RPi.GPIO as GPIO

# { key: [pin, time before closing] }
key_to_pin = {
    "j": [5, 5.0],
    "k": [6, 0.05],
    "l": [7, 0.05],
}

class Pneumatics():
    def __init__(self):
        rospy.init_node('pneumatics_server', log_level=rospy.DEBUG)
        s = rospy.Service('pneumatics_status', PneumaticsStatus, self.handle_pneumatics_status)
        self.claw_open = False
        self.l_torpedo_open = False
        self.r_torpedo_open = False

    def handle_pneumatics_status(self, req):
        msg = PneumaticsStatus()
        msg.claw_open = self.claw_open
        msg.l_torpedo_open = self.l_torpedo_open
        msg.r_torpedo_open = self.r_torpedo_open
        return msg

    # https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
    def getKey(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def useKeyAction(self, key):
        if key in key_to_pin.keys():
            rospy.sleep(5)
            GPIO.output(key_to_pin[key][0], GPIO.HIGH)
            rospy.sleep(key_to_pin[key][1])
            GPIO.output(key_to_pin[key][0], GPIO.LOW)

if __name__ == "main":
    pneumatics_contoller = Pneumatics()
    while (1):
        key = pneumatics_contoller.getKey()
        pneumatics_contoller.useKeyAction(key)
