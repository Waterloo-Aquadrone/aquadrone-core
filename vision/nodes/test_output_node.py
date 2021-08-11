#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import aquadrone_msgs.BoundingBox as BoundingBox
import aquadrone_msgs.BoundingBoxes as BoundingBoxes

class TestOutputNode():
    def __init__(self):
        self.name = 'Test Output Node'
        rospy.init_node(self.name, log_level=rospy.DEBUG)
        self.bridge = CvBridge()

        # to test publish_test_data_node
        # rospy.Subscriber('vision/test', Image, self.show_image)

        # to test image_preprocessing_node
        rospy.Subscriber('vision/preprocess', Image, self.show_image)


    def show_image(self, image):
        rospy.logdebug('Received image')
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        cv2.imshow('image', img)
        cv2.waitKey(5000)


if __name__ == '__main__':
    test_output_node = TestOutputNode()
    rospy.spin()
