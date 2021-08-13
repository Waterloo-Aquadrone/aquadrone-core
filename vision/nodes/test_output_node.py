#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from aquadrone_msgs.msg import BoundingBox, BoundingBoxes
from aquadrone_msgs.msg import Center, Centers

# TODO:
# Test actual object detection node once training weights and cfg file are ready

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
        # rospy.Subscriber('vision/bounding_boxes', BoundingBoxes, self.show_bboxes)

        # uncomment to test image_postprocessing_node (comment other topics)
        rospy.Subscriber('vision/centers', Centers, self.show_centers)


    def show_image(self, image):
        rospy.logdebug('Received image')
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        cv2.imshow('image', img)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()


    def show_bboxes(self, bboxes):
        frame = self.bridge.imgmsg_to_cv2(bboxes.frame, desired_encoding='bgr8')
        frame_width = frame.shape[1]
        frame_height = frame.shape[0]
        for bbox in bboxes.boxes:
            id = bbox.class_id
            x = int(bbox.x_center*frame_width)
            y = int(bbox.y_center*frame_height)
            w = int(bbox.width*frame_width)
            h = int(bbox.height*frame_height)
            rospy.logdebug('Id is {} with type {}'.format(id, type(id)))
            rospy.logdebug('X is {} with type {}'.format(x, type(x)))
            rospy.logdebug('Y is {} with type {}'.format(y, type(y)))
            rospy.logdebug('W is {} with type {}'.format(w, type(w)))
            rospy.logdebug('H is {} with type {}'.format(h, type(h)))
            color = list(np.random.random(size=3) * 256)
            cv2.rectangle(frame, (int(x - w/2), int(y - h/2)), (int(x + w/2), int(y + h/2)), color, 2)
            text = "{}".format(self.classes[id])
            cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        cv2.imshow('image', frame)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()


    def show_centers(self, centers):
        frame = self.bridge.imgmsg_to_cv2(centers.frame, desired_encoding='bgr8')
        for center in centers.centers:
            color = list(np.random.random(size=3) * 256)
            cv2.circle(frame, (int(center.x_center), int(center.y_center)), 0, color, thickness=-1)
            text = "{}".format(self.classes[center.class_id])
            cv2.putText(frame, text, (int(center.x_center), int(center.y_center - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        cv2.imshow('image', frame)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    test_output_node = TestOutputNode()
    rospy.spin()
