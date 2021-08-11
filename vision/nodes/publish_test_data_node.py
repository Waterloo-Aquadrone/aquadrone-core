#!/usr/bin/env python3
import rospy
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class PublishTestDataNode():
    def __init__(self):
        self.name = 'Publish Test Data Node'
        self.image_path = './testdata/'
        self.image_ext = '.jpg'
        self.image_names = ['dog', 'eagle', 'giraffe', 'horses', 'kite', 'person']
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('vision/test', Image, queue_size=1)
        rospy.init_node(self.name, log_level=rospy.DEBUG)
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
                self.pub.publish(ros_image)
                rospy.logdebug('Sucessfully published {}'.format(img))
            rospy.sleep(5)


if __name__ == '__main__':
    publish_test_data_node = PublishTestDataNode()
    rospy.spin()
