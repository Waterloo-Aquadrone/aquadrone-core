#!/usr/bin/env python3
import rospy
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from aquadrone_msgs.msg import Frame # remove when object detection node simulator is no longer needed

class TestInputNode():
    def __init__(self):
        self.name = 'test_input_node'
        self.image_path = rospy.get_param("/test_data_path")
        self.image_ext = '.jpg'
        self.image_names = ['dog', 'eagle', 'giraffe', 'horses', 'kite', 'person']
        self.bridge = CvBridge()

        # uncomment next line when object detection simulator node is no longer needed
        # self.pub = rospy.Publisher('vision/test', Image, queue_size=1)

        # delete next line when object detection simulator node is no longer needed
        self.pub = rospy.Publisher('vision/test', Frame, queue_size=1)

        rospy.init_node(self.name, log_level=rospy.DEBUG)
        rospy.logdebug('Wait for  the other nodes to start')
        rospy.sleep(5)
        self.read_and_publish_images()


    def read_and_publish_images(self):
        for img in self.image_names:
            if rospy.is_shutdown(): break
            full_path = self.image_path + img + self.image_ext
            image = cv2.imread(full_path)
            if image is None:
                warning_msg = 'Could not read image {}'.format(img)
                rospy.logdebug(warning_msg)
            else:
                ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')

                # uncomment next line when object detection simulator node is no longer needed
                # self.pub.publish(ros_image)

                # delete next 4 lines when object detection simulator node is no longer needed
                frame_msg = Frame()
                frame_msg.frame = ros_image
                frame_msg.name = img
                self.pub.publish(frame_msg)

                rospy.logdebug('Sucessfully published {}'.format(img))
            rospy.sleep(5)


if __name__ == '__main__':
    publish_test_data_node = TestInputNode()
    rospy.spin()
