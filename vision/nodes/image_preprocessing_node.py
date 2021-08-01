import rospy
import sensor_msgs.Image as Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# TODO:
# calibrate blur_threshold
# apply preprocessing to image, possibly:
# Illumination correction such as CLAHE
# Smoothing such as blur

class ImagePreprocessingNode():
    def __init(self):
        self.name = 'Image Preprocessing'
        self.blur_threshold = 100
        rospy.init_node(self.name)
        self.bridge = CvBridge()
        rospy.Subscriber('/zed/zed_node/rgb/image_rect_color', Image, self.process_image, queue_size=1)
        self.pub = rospy.Publisher('image/preprocess', Image, queue_size=1)


    def process_image(self, image):
        try:
            frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            focus_measure = cv2.Laplacian(frame_gray, cv2.CV_64F).var()
            if focus_measure > self.blur_threshold:
                # put preprocessing here
                image_preprocess = self.bridge.cv2_to_imgmsg(frame, encoding='passthrough')
                self.pub.publish(image_preprocess)
            else:
                pass # how to skip this frame and wait for the other?

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    img_preprocessing_node = ImagePreprocessingNode()
    rospy.spin()
