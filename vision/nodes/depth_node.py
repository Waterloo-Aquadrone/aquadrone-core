import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2

# TODO:
# read from two topics at the same times
# how to calculate depths
# Depth message type

class DepthNode():
    def __init__(self):
        self.name = 'Depth Node'
        rospy.init_node(self.name)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('vision/depths', Depths, queue_size=1)
        rospy.Subscriber('vision/centers', Image, self.find_depths, queue_size=1)


    def find_depths(self, centers):
        pass


if __name__ == '__main__':
    depth_node = DepthNode()
    rospy.spin()
