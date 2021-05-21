import rospy
from aquadrone_pneumatics.srv import PneumaticsCommand, PneumaticsCommandResponse


class PneumaticsController:
    TORPEDO_FIRE_TIME = 0.05  # seconds

    def __init__(self, real):
        rospy.Service('pneumatics_commands', PneumaticsCommand, self.handle_pneumatics_command)
        self.claw_open = True
        self.left_torpedo_fired = False
        self.right_torpedo_fired = False
        self.real = real

        if self.real:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            self.claw_pin = rospy.get_param('/GPIO_pins/claw')
            self.left_torpedo_pin = rospy.get_param('/GPIO_pins/left_torpedo')
            self.right_torpedo_pin = rospy.get_param('/GPIO_pins/right_torpedo')

    @staticmethod
    def run():
        rospy.spin()

    def generate_pneumatics_status(self):
        msg = PneumaticsCommandResponse()
        msg.claw_open = self.claw_open
        msg.left_torpedo_fired = self.left_torpedo_fired
        msg.right_torpedo_fired = self.right_torpedo_fired
        return msg

    def fire_torpedo(self, torpedo_pin):
        self.GPIO.output(torpedo_pin, self.GPIO.HIGH)
        rospy.sleep(PneumaticsController.TORPEDO_FIRE_TIME)
        self.GPIO.output(torpedo_pin, self.GPIO.LOW)

    def handle_pneumatics_command(self, req):
        command = req.command
        if command == "open_claw":
            if self.real:
                self.GPIO.output(self.claw_pin, self.GPIO.HIGH)
            self.claw_open = True
        elif command == "close_claw":
            if self.real:
                self.GPIO.output(self.claw_pin, self.GPIO.LOW)
            self.claw_open = False
        elif command == "fire_left_torpedo":
            if self.real and not self.left_torpedo_fired:
                self.fire_torpedo(self.left_torpedo_pin)
            self.left_torpedo_fired = True
        elif command == "fire_right_torpedo":
            if self.real and not self.right_torpedo_fired:
                self.fire_torpedo(self.right_torpedo_pin)
            self.right_torpedo_fired = True

        return self.generate_pneumatics_status()
