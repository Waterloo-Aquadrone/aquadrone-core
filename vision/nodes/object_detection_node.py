import rospy
import sensor_msgs.Image as Image
import aquadrone_msgs.BoundingBox as BoundingBox
import aquadrone_msgs.BoundingBoxes as BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError
import cv2

# TODO:
# Add path to classes, config, and weights file once training is done
# Find out what the scale parameter is for and calibrate it
# double check that output of cvbridge is an rgb Image
# calibrate confidence and nms_threshold
# object detection code -> https://opencv-tutorial.readthedocs.io/en/latest/yolo/yolo.html

class ObjectDetectionNode():
    def __init__(self):
        self.name = 'Object Detection'
        self.classes_file_path = ''
        self.config_file_path = ''
        self.weights_file_path = ''
        self.classes = None
        self.class_ids = []
        self.confidences = []
        self.bounding_boxes = []
        self.select_bounding_boxes = []
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4 # non-maximum suppression
        rospy.init_node(self.name)
        self.bridge = CvBridge()
        self.bbox_msg = BoundingBoxes()
        self.pub = rospy.Publisher('vision/bounding_boxes', Image, queue_size=1)
        rospy.Subscriber('vision/preprocess', Image, self.detect_objects, queue_size=1)


    def detect_objects(self, image):
        try:
            self.bbox_msg.frame = image
            frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') # should I imread?
            self.frame_dim = (lambda w, h : (w, h))(frame.shape[0], frame.shape[1])
            self.scale = 1.00
            self.setup_yolo_net(frame)
            self.run_yolo_net()
            self.pub.publish(self.bbox_msg)

        except CvBridgeError as e:
            print(e)


    def setup_yolo_net(self, frame):
        with open(self.classes_file_path, 'r') as classes_file:
            self.classes = [class.strip() for class in classes_file.readlines()]

        self.net = cv2.dnn.readNetFromDarknet(self.config_file_path, self.weights_file_path)

        self.frame_blob = cv2.dnn.blobFromImage(frame, self.scale, (416,416), (0,0,0), True, crop=False)
        self.net.setInput(self.frame_blob)


    def run_yolo_net(self):
        outputs = self.net.forward(self.get_output_layers())

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                # select class id with maximum score (confidence) - what the object most likely is
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > self.confidence_threshold:
                    bounding_box = detection[:4] * np.array([w, h, w, h])
                    (center_x, center_y, width, height) = bounding_box.astype("int")
                    x = int(center_x - (width / 2))
                    y = int(center_y - (height / 2))
                    bounding_box = [x, y, int(width), int(height)]
                    self.bounding_boxes.append(bounding_box)
                    self.confidences.append(float(confidence))
                    self.class_ids.append(class_id)


        indices = cv2.dnn.NMSBoxes(self.bounding_boxes, self.confidences,
                                   self.confidence_threshold, self.nms_threshold)

        for i in indices.flatten():
            temp_bbox_msg = BoundingBox()
            temp_bbox_msg.x_center = self.bounding_boxes[i][0]
            temp_bbox_msg.y_center = self.bounding_boxes[i][1]
            temp_bbox_msg.width = self.bounding_boxes[i][2]
            temp_bbox_msg.height = self.bounding_boxes[i][3]
            temp_bbox_msg.class_id = self.class_ids[i]
            self.select_bounding_boxes.append(temp_bbox_msg)

        self.bbox_msg.boxes = self.select_bounding_boxes


    def get_out_layers(self):
        layers = self.net.getLayerNames()
        out_layers = [layers[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        return out_layers


if __name__ == '__main__':
    object_detection_node = ObjectDetectionNode()
    rospy.spin()
