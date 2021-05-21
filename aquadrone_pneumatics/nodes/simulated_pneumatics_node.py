import rospy
from aquadrone_pneumatics.pneumatics_controller import PneumaticsController


if __name__ == "main":
    rospy.init_node('pneumatics_controller', log_level=rospy.DEBUG)

    pneumatics_controller = PneumaticsController(real=False)
    pneumatics_controller.run()
