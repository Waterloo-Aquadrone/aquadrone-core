#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from aquadrone_msgs.msg import Frame # remove when object detection node simulator is no longer needed

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

        # uncomment next 2 lines when object detection simulator node is no longer needed
        # self.pub = rospy.Publisher('vision/preprocess', Image, queue_size=1)
        # rospy.Subscriber('vision/test', Image, self.process_image, queue_size=1) # for test

        # delete next 2 lines when object detection simulator node is no longer needed
        self.pub = rospy.Publisher('vision/preprocess', Frame, queue_size=1)
        rospy.Subscriber('vision/test', Frame, self.process_image, queue_size=1) # for test

        rospy.Subscriber('/zed/zed_node/rgb/image_rect_color', Image, self.process_image, queue_size=1)


    def process_image(self, image):
        try:
            # uncomment next line when object detection simulator node is no longer needed
            # frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

            # delete next line when object detection simulator node is no longer needed
            frame = self.bridge.imgmsg_to_cv2(image.frame, desired_encoding='bgr8')

            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            focus_measure = cv2.Laplacian(frame_gray, cv2.CV_64F).var()
            if focus_measure > self.blur_threshold:
                # put preprocessing here
                image_preprocess = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

                # uncomment next line when object detection simulator node is no longer needed
                # self.pub.publish(image_preprocess)

                # delete next 4 lines when object detection simulator node is no longer needed
                frame_msg = Frame()
                frame_msg.frame = image_preprocess
                frame_msg.name = image.name
                self.pub.publish(frame_msg)
            else:
                pass

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    img_preprocessing_node = ImagePreprocessingNode()
    rospy.spin()
