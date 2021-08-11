import rospy
import sensor_msgs.Image as Image
import aquadrone_msgs.BoundingBox as BoundingBox
import aquadrone_msgs.BoundingBoxes as BoundingBoxes
import aquadrone_msgs.Center as Center
import aquadrone_msgs.Centers as Centers
import cv2
from cv_bridge import CvBridge, CvBridgeError
from center_imports import *

# TODO:
# add center strategies for each class


class ImagePostprocessingNode():
    def __init__(self):
        self.name = 'Post Processing'
        rospy.init_node(self.name)
        self.centers = []
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('vision/centers', Centers, queue_size=1)
        rospy.Subscriber('vision/bounding_boxes', BoundingBoxes, self.process_bounding_boxes, queue_size=1)


    def process_bounding_boxes(self, bbox_msg):
        bboxes = bbox_msg.boxes
        if len(bboxes) == 0: self.handle_no_objects()
        else:
            self.frame = self.bridge.imgmsg_to_cv2(bbox_msg.frame, desired_encoding='passthrough')
            for bbox in bboxes: self.centers.append(self.get_center_of_object(bbox))
            centers_msg = Centers()
            centers_msg.frame = bbox_msg.frame
            centers_msg.centers = self.centers
            self.pub.publish(centers_msg)


    def get_center_of_object(self, bbox):
        top_left_corner = (bbox.y_center + bbox.height/2, bbox.x_center + bbox.width/2)
        cropped_frame = self.frame[top_left_corner[0]:top_left_corner[0]+bbox.height,
                                top_left_corner[1]:top_left_corner[1]+bbox.width]

        center_cropped = None # add center strategies based on class ids

        

        center = Center()
        center.x_center = top_left_corner[1] + center_cropped[0]
        center.y_center = top_left_corner[0] + center_cropped[1]
        center.class_id = bbox.class_id
        return center


    def handle_no_objects(self):
        pass


if __name__ == '__main__':
    image_postprocessing_node = ImagePostprocessingNode()
    rospy.spin()
