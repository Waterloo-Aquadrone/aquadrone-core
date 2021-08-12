#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from aquadrone_msgs.msg import BoundingBox, BoundingBoxes
from aquadrone_msgs.msg import Center, Centers

class TestOutputNode():
    def __init__(self):
        self.name = 'test_output_node'
        self.classes_file_path = './testdata/coco.names'
        rospy.init_node(self.name, log_level=rospy.DEBUG)
        self.bridge = CvBridge()
        with open(self.classes_file_path, 'r') as classes_file:
            self.classes = [cls.strip() for cls in classes_file.readlines()]

        # uncomment to test publish_test_data_node (comment other topics)
        # rospy.Subscriber('vision/test', Image, self.show_image)

        # uncomment to test image_preprocessing_node (comment other topics)
        # rospy.Subscriber('vision/preprocess', Image, self.show_image)

        # uncomment to test object detection node (comment other topics)
        rospy.Subscriber('vision/bounding_boxes', BoundingBoxes, self.show_bboxes)

        # uncomment to test image_postprocessing_node (comment other topics)
        # rospy.Subscriber('vision/centers', Centers, self.show_centers)


    def show_image(self, image):
        rospy.logdebug('Received image')
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        cv2.imshow('image', img)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()


    def show_bboxes(self, bboxes):
        frame = self.bridge.imgmsg_to_cv2(bboxes.frame, desired_encoding='bgr8')

        for bbox in bboxes.boxes:
            id = bbox.class_id
            x = bbox.x_center
            y = bbox.y_center
            w = bbox.width
            h = bbox.height
            color = list(np.random.random(size=3) * 256)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            text = "{}".format(self.classes[id])
            cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        cv2.imshow('image', frame)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()


    def show_centers(self, centers):
        pass


if __name__ == '__main__':
    test_output_node = TestOutputNode()
    rospy.spin()
