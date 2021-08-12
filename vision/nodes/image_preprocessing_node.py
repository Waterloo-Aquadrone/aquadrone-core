#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# TODO:
# calibrate blur_threshold
# apply preprocessing to image, possibly:
# Illumination correction such as CLAHE
# Smoothing such as blur

class ImagePreprocessingNode():
    def __init__(self):
        self.name = 'image_preprocessing_node'
        self.blur_threshold = 100
        rospy.init_node(self.name, log_level=rospy.DEBUG)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('vision/preprocess', Image, queue_size=1)
        rospy.Subscriber('vision/test', Image, self.process_image, queue_size=1) # for test
        rospy.Subscriber('/zed/zed_node/rgb/image_rect_color', Image, self.process_image, queue_size=1)


    def process_image(self, image):
        try:
            frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            focus_measure = cv2.Laplacian(frame_gray, cv2.CV_64F).var()
            if focus_measure > self.blur_threshold:
                # put preprocessing here
                image_preprocess = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.pub.publish(image_preprocess)
            else:
                pass

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    img_preprocessing_node = ImagePreprocessingNode()
    rospy.spin()
