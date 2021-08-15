#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from aquadrone_msgs.msg import BoundingBox, BoundingBoxes
from aquadrone_msgs.msg import Center, Centers
from cv_bridge import CvBridge, CvBridgeError
from center_imports import *

# TODO:
# add center strategies for each class
# handle case where no object can be detected


class ImagePostprocessingNode():
    def __init__(self):
        self.name = 'image_post_processing_node'
        rospy.init_node(self.name, log_level=rospy.DEBUG)
        self.bridge = CvBridge()
        self.contour_strategy = ContourCenterStrategyClass()
        self.colour_strategy = ColourCenterStrategyClass()
        self.pub = rospy.Publisher('vision/centers', Centers, queue_size=1)
        rospy.Subscriber('vision/bounding_boxes', BoundingBoxes, self.process_bounding_boxes, queue_size=1)


    def process_bounding_boxes(self, bbox_msg):
        self.centers_msg = Centers()
        bboxes = bbox_msg.boxes
        if len(bboxes) == 0: self.handle_no_objects()
        else:
            self.frame = self.bridge.imgmsg_to_cv2(bbox_msg.frame, desired_encoding='bgr8')
            self.frame_height = self.frame.shape[0]
            self.frame_width = self.frame.shape[1]
            # rospy.logdebug('##################### Next Image ########################')
            # rospy.logdebug('Frame dimensions are {} by {}'.format(self.frame_width, self.frame_height))
            for bbox in bboxes: self.centers_msg.centers.append(self.get_center_of_object(bbox))
            self.centers_msg.frame = bbox_msg.frame
            self.pub.publish(self.centers_msg)


    def get_center_of_object(self, bbox):
        center_cropped = None
        id = bbox.class_id
        x = int(bbox.x_center*self.frame_width)
        y = int(bbox.y_center*self.frame_height)
        w = int(bbox.width*self.frame_width)
        h = int(bbox.height*self.frame_height)

        bottom_left = (int(x - w/2), int(y - h/2))
        # rospy.logdebug('x: {}, y: {}, w: {}, h: {}, id: {}'.format(x, y, w, h, id))
        # rospy.logdebug('Bottom left corner would be {}'.format(bottom_left))
        # rospy.logdebug('Leftmost point: {} and topmost point: {} '.format(bottom_left[0] + w, bottom_left[1] + h))
        # rospy.logdebug('------------------------------')
        cropped_frame = self.frame[bottom_left[1]:bottom_left[1] + h, bottom_left[0]:bottom_left[0] + w]

        # replace with actual classes once training is complete

        if id == 1: # bicycle
            center_cropped = self.contour_strategy.findCenter(cropped_frame)

        elif id == 2: # car
            center_cropped = self.contour_strategy.findCenter(cropped_frame)

        elif id == 16: # dog
            center_cropped = self.contour_strategy.findCenter(cropped_frame)

        elif id == 14: # bird
            center_cropped = self.contour_strategy.findCenter(cropped_frame)

        elif id == 23: # giraffe
            center_cropped = self.contour_strategy.findCenter(cropped_frame)

        elif id == 22: # zebra
            center_cropped = self.contour_strategy.findCenter(cropped_frame)

        elif id == 17: # horse
            center_cropped = self.contour_strategy.findCenter(cropped_frame)

        elif id == 0: # person
            center_cropped = self.contour_strategy.findCenter(cropped_frame)

        elif id == 33: # kite
            center_cropped = self.contour_strategy.findCenter(cropped_frame)

        else:
            pass

        center = Center()
        center.x_center = bottom_left[0] + center_cropped[0]
        center.y_center = bottom_left[1] + center_cropped[1]
        center.class_id = id
        return center


    def handle_no_objects(self):
        pass


if __name__ == '__main__':
    image_postprocessing_node = ImagePostprocessingNode()
    rospy.spin()
