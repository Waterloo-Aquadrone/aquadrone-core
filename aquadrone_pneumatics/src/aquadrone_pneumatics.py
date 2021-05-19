import rospy
from aquadrone_pneumatics.msg import command, status
from aquadrone_pneumatics.srv import PneumaticsCommands, PneumaticsCommandsResponse
import RPi.GPIO as GPIO


command_to_pin = {
    "l_torpedo": [6, 0.05],
    "r_torpedo": [7, 0.05],
    "claw_open": [5, 5.0],
    "claw_close": [5, 5.0],
}

class Pneumatics():
    def __init__(self):
        rospy.init_node('pneumatics_server', log_level=rospy.DEBUG)
        s = rospy.Service('pneumatics_commands', PneumaticsCommands, self.handle_pneumatics_commands)
        self.claw_open = False
        self.l_torpedo_fired = False
        self.r_torpedo_fired = False

    def run(self):
        rospy.spin()

    def handle_pneumatics_status(self):
        msg = PneumaticsCommandsResponse()
        msg.claw_open = self.claw_open
        msg.l_torpedo_open = self.l_torpedo_fired
        msg.r_torpedo_open = self.r_torpedo_fired
        return msg

    def fire_torpedo(self, msg):
        GPIO.output(command_to_pin[msg][0], GPIO.HIGH)
        rospy.sleep(command_to_pin[msg][1])
        GPIO.output(command_to_pin[msg][0], GPIO.LOW)

    def handle_pneumatics_command(self, req):
        msg = req.command
        if msg == "claw_open":
            GPIO.output(command_to_pin[msg][0], GPIO.HIGH)
            self.claw_open = True
        elif msg == "claw_close":
            GPIO.output(command_to_pin[msg][0], GPIO.LOW)
            self.claw_open = False
        elif msg == "l_torpedo" and not self.l_torpedo_fired:
            self.fire_torpedo(msg)
            self.l_torpedo_fired = True
        elif msg == "r_torpedo" and not self.r_torpedo_fired:
            self.fire_torpedo(msg)
            self.r_torpedo_fired = True

        return self.handle_pneumatics_status()
