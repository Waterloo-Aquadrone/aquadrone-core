#!/usr/bin/env python3
import rospy
import sensor_msgs.Image as Image
import aquadrone_msgs.BoundingBox as BoundingBox
import aquadrone_msgs.BoundingBoxes as BoundingBoxes
from aquadrone_msgs.msg import Frame

class ObjectDetectionSimulatorNode():
    def __init__(self):
        self.name = 'object_detection_simulator_node'
        self.bbox_msg = BoundingBoxes()
        self.count = 0
        self.init_node(self.name, log_level=rospy.DEBUG)
        self.pub = rospy.Publisher('vision/bounding_boxes', BoundingBoxes, queue_size=1)
        rospy.Subscriber('vision/preprocess', Frame, self.publish_bounding_boxes, queue_size=1)


    def publish_bounding_boxes(self, frame_msg):
        self.bbox_msg.frame = frame_msg.frame
        self.bbox_msg.boxes = []
        bbox_file_path = './testdata/{}.txt'.format(frame_msg.name)
        with open(self.bbox_file_path, 'r') as bbox_files:
            self.bbox_line = [box.strip() for box in bbox_files.readlines()]
            for bbox in bbox_line:
                features = bbox.split(' ')
                temp_bbox_msg = BoundingBox()
                temp_bbox_msg.x_center = features[1]
                temp_bbox_msg.y_center = features[2]
                temp_bbox_msg.width = features[3]
                temp_bbox_msg.height = features[4]
                temp_bbox_msg.class_id = features[0]
                self.bbox_msg.boxes.append(temp_bbox_msg)

        self.pub.publish(self.bbox_msg)


if __name__ == '__main__':
    object_detection_simulator_node = ObjectDetectionSimulatorNode()
    rospy.spin()
